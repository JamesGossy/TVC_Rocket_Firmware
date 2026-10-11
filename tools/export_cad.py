"""Dump the Fusion design TVC_Rocket_Gimbal_v3 to plain files for the sim (READ ONLY).

Run it inside Fusion (Scripts and Add-Ins, or the MCP "script" tool). It never changes the
design: it only reads geometry, joints, materials and mass properties at whatever pose the
design is in (the neutral pose: gimbal 0, keeper ring 0, legs stowed, claws closed).
Do NOT save the design afterwards.

    exec(open(r"C:\\...\\TVS_rocket_simulation\\tools\\export_cad.py").read())

Writes vehicle/raw/ next to this repo:
    cad_dump.json   occurrences, bodies (material, mass, CG, inertia), joints, rigid groups, features
    meshes/*.bin    world-pose triangle meshes, one per body (float32 xyz nodes, then uint32 indices)
    done.txt        written last, so a caller that timed out can poll for it

tools/build_vehicle.py turns that into vehicle/vehicle.json and vehicle/meshes/*.glb.
Fusion's API units are cm, kg and kg cm^2: the dump keeps them, build_vehicle.py converts.
"""
import adsk.core, adsk.fusion, json, os, time, traceback
from array import array

REPO = r"C:\Users\JamesGoss\Documents\Projects\TVS_rocket_simulation"
RAW = os.path.join(REPO, "vehicle", "raw")
MESH_DIR = os.path.join(RAW, "meshes")
# Fusion's own mesh quality preset (the surfaceTolerance property had no effect): coarse enough that the glb files
# stay small for the web viewer. Set to adsk.fusion.TriangleMeshQualityOptions.NormalQualityTriangleMesh for finer.
MESH_QUALITY = adsk.fusion.TriangleMeshQualityOptions.NormalQualityTriangleMesh
# Small hardware whose faces we don't need (we only want pins, bores, studs, lugs, ...)
SKIP_FACES = ("M2x4 button head", "M2 heat-set insert", "M3 hex nut")


def p3(p):
    return [p.x, p.y, p.z] if p is not None else None


def param(v):
    """A motion link ratio is a ModelParameter: keep its expression and value."""
    try:
        return {"expression": v.expression, "value": v.value}
    except Exception:
        return str(v)


def tensor(props):
    """(xx, yy, zz, xy, yz, xz) in kg cm^2 about the component origin; older APIs prepend a bool."""
    r = props.getXYZMomentsOfInertia()
    if isinstance(r[0], bool):
        r = r[1:]
    return list(r)


def dump_faces(body):
    """Cylinders, spheres and cones of a body (world coordinates), with a convex/concave flag."""
    out = []
    for f in body.faces:
        g = f.geometry
        kind = None
        if isinstance(g, adsk.core.Cylinder):
            kind = "cyl"
        elif isinstance(g, adsk.core.Sphere):
            kind = "sphere"
        elif isinstance(g, adsk.core.Cone):
            kind = "cone"
        if kind is None:
            continue
        d = {"kind": kind, "origin": p3(g.origin), "radius": g.radius,
             "area": f.area, "bbox": [p3(f.boundingBox.minPoint), p3(f.boundingBox.maxPoint)]}
        if kind != "sphere":
            d["axis"] = p3(g.axis)
        try:  # outward normal pointing away from the axis/centre = a pin or ball, otherwise a bore
            pt = f.pointOnFace
            ok, n = f.evaluator.getNormalAtPoint(pt)
            o = g.origin
            rad = [pt.x - o.x, pt.y - o.y, pt.z - o.z]
            if kind != "sphere":
                ax = g.axis
                s = rad[0] * ax.x + rad[1] * ax.y + rad[2] * ax.z
                rad = [rad[0] - s * ax.x, rad[1] - s * ax.y, rad[2] - s * ax.z]
            d["convex"] = (n.x * rad[0] + n.y * rad[1] + n.z * rad[2]) > 0
        except Exception:
            d["convex"] = None
        out.append(d)
    return out


def dump_joint(j, kind):
    d = {"kind": kind, "name": j.name}
    try:
        jm = j.jointMotion
        d["type"] = jm.jointType  # 0 rigid, 1 revolute, 2 slider, 3 cylindrical, 4 pin-slot, 5 planar, 6 ball
        try:
            d["axis"] = p3(jm.rotationAxisVector)
        except Exception:
            pass
        try:
            d["value"] = jm.rotationValue
            lim = jm.rotationLimits
            d["limits"] = [lim.minimumValue, lim.maximumValue, lim.isMinimumValueEnabled, lim.isMaximumValueEnabled]
        except Exception:
            pass
    except Exception as e:
        d["error"] = str(e)
    try:
        g1 = j.geometryOrOriginOne if kind == "joint" else j.geometry
        d["origin"] = p3(g1.origin)
        if kind == "joint":
            d["origin2"] = p3(j.geometryOrOriginTwo.origin)
    except Exception as e:
        d["origin_error"] = str(e)
    for name in ("occurrenceOne", "occurrenceTwo"):
        try:
            d[name] = getattr(j, name).name
        except Exception:
            d[name] = None
    try:
        d["motion_links"] = [{"name": m.name, "valueOne": param(m.valueOne), "valueTwo": param(m.valueTwo),
                              "reversed": m.isReversed, "jointOne": m.jointOne.name, "jointTwo": m.jointTwo.name}
                             for m in j.motionLinks]
    except Exception:
        pass
    return d


def run(context):
    t0 = time.time()
    res = {}
    try:
        os.makedirs(MESH_DIR, exist_ok=True)
        for old in ("done.txt",):
            if os.path.exists(os.path.join(RAW, old)):
                os.remove(os.path.join(RAW, old))
        app = adsk.core.Application.get()
        design = adsk.fusion.Design.cast(app.activeProduct)
        root = design.rootComponent
        data_file = app.activeDocument.dataFile
        res["source"] = {"document": app.activeDocument.name, "version": data_file.versionNumber,
                         "comment": data_file.description, "units": "cm / kg / kg cm^2 (Fusion API units)"}
        res["parameters"] = {p.name: p.expression for p in design.userParameters}

        occs, bodies = [], []
        for oi, occ in enumerate(root.allOccurrences):
            occs.append({"idx": oi, "name": occ.name, "component": occ.component.name,
                         "transform": occ.transform2.asArray(), "grounded": occ.isGrounded,
                         "visible": occ.isLightBulbOn, "bodies": occ.bRepBodies.count})
            for bi in range(occ.bRepBodies.count):
                proxy = occ.bRepBodies.item(bi)
                native = occ.component.bRepBodies.item(bi)
                b = {"occ": oi, "body": native.name, "material": native.material.name if native.material else None,
                     "visible": proxy.isVisible}
                pp = native.getPhysicalProperties(adsk.fusion.CalculationAccuracy.HighCalculationAccuracy)
                b.update({"mass": pp.mass, "volume": pp.volume, "cg": p3(pp.centerOfMass), "inertia_origin": tensor(pp)})
                # Proxy properties are only a cross-check (they go stale if joints are driven in the same script)
                prox = proxy.getPhysicalProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy)
                b["proxy_cg_world"] = p3(prox.centerOfMass)
                b["proxy_mass"] = prox.mass
                bb = proxy.boundingBox
                b["bbox_world"] = [p3(bb.minPoint), p3(bb.maxPoint)]
                if not occ.name.startswith(SKIP_FACES):
                    b["faces"] = dump_faces(proxy)
                if proxy.isVisible and occ.isLightBulbOn:
                    calc = proxy.meshManager.createMeshCalculator()
                    calc.setQuality(MESH_QUALITY)
                    mesh = calc.calculate()
                    nodes = array("f", mesh.nodeCoordinatesAsDouble)
                    idx = array("I", mesh.nodeIndices)
                    fn = "m_%03d_%d.bin" % (oi, bi)
                    with open(os.path.join(MESH_DIR, fn), "wb") as fh:
                        nodes.tofile(fh)
                        idx.tofile(fh)
                    b["mesh"] = {"file": fn, "nodes": len(nodes) // 3, "indices": len(idx)}
                bodies.append(b)
        res["occurrences"] = occs
        res["bodies"] = bodies
        res["joints"] = [dump_joint(j, "joint") for j in root.joints] + \
                        [dump_joint(j, "asbuilt") for j in root.asBuiltJoints]
        res["rigid_groups"] = [{"name": g.name, "members": [o.name for o in g.occurrences]} for g in root.rigidGroups]
        res["seconds"] = time.time() - t0
    except Exception:
        res["error"] = traceback.format_exc()
    with open(os.path.join(RAW, "cad_dump.json"), "w") as fh:
        json.dump(res, fh)
    with open(os.path.join(RAW, "done.txt"), "w") as fh:
        fh.write("error" if "error" in res else "ok")


def stop(context):
    pass
