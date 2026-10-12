#!/usr/bin/env python3
"""Turn vehicle/raw/ (written by tools/export_cad.py inside Fusion) into the files the sim and viewer use:

    vehicle/vehicle.json        links (mass, CG, inertia, parts), joints, linkage points, contact points,
                                blocker table, keeper and leg data, clearances, nose profile, self-check
    vehicle/meshes/<link>.glb   one mesh file per link, in that link's frame

    python tools/build_vehicle.py [--no-meshes]     (the meshes need the trimesh and fast-simplification packages)

Frames. Fusion world: origin at the gimbal pivot, +Z toward the nose, mm. Sim body frame: x out the nose,
measured up from the bottom of the rocket (aft end of the ascent motor), metres.
    body (x, y, z) = ((Z_cad + 134 mm) / 1000, X_cad / 1000, Y_cad / 1000)
That is a cyclic permutation of the axes, so it is right handed and keeps the sense of every rotation:
a Fusion rotation about +Y is a sim rotation about +z, about +X is about +y, about +Z is about +x.

Link frame: origin at the link's joint origin, axes parallel to the body frame in the neutral pose.
Everything in vehicle.json (CG, joint origins, points) is given in the link frame unless it says "body".
The neutral pose is gimbal 0, blocker claws closed, keeper ring 0, legs stowed.
"""
import json, math, os, sys, time
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RAW = os.path.join(ROOT, "vehicle", "raw")
OUT = os.path.join(ROOT, "vehicle")
Z_BOTTOM_MM = -134.0  # aft end of the ascent motor in Fusion world Z: body x = 0 there
M = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], float)  # CAD (X, Y, Z) -> body (Z, X, Y)


def P(p_cm):
    """Fusion point (cm) -> body frame point (m)."""
    p = np.asarray(p_cm, float) * 10.0
    return np.array([p[2] - Z_BOTTOM_MM, p[0], p[1]]) / 1000.0


def V(v):
    """Fusion direction -> body frame direction."""
    return M @ np.asarray(v, float)


def skew(a):
    return np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])


def rot(axis, angle):
    """Rotation matrix about a unit axis, right handed."""
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    K = skew(a)
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * (K @ K)


def tensor_shift(m, d):
    """Parallel-axis term for mass m displaced by d."""
    return m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))


def combine(parts):
    """Mass, CG and inertia about that CG of a list of {mass, cg, I} (I about the part's own CG)."""
    m = sum(p["mass"] for p in parts)
    cg = sum(p["mass"] * p["cg"] for p in parts) / m
    I = sum(p["I"] + tensor_shift(p["mass"], p["cg"] - cg) for p in parts)
    return m, cg, I


# ---------------------------------------------------------------------------------------------------------------
# 1. Read the Fusion dump and get every body's mass, CG and inertia in the body frame
# ---------------------------------------------------------------------------------------------------------------
def load_dump():
    d = json.load(open(os.path.join(RAW, "cad_dump.json")))
    if "error" in d:
        sys.exit("cad_dump.json holds an export error:\n" + d["error"])
    occ = d["occurrences"]
    bodies = []
    for b in d["bodies"]:
        o = occ[b["occ"]]
        T = np.array(o["transform"], float).reshape(4, 4)
        R, t = T[:3, :3], T[:3, 3]
        m = b["mass"]
        c = np.array(b["cg"], float)
        xx, yy, zz, xy, yz, xz = b["inertia_origin"]
        Io = np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])  # kg cm^2 about the component origin
        Icg = Io - m * (np.dot(c, c) * np.eye(3) - np.outer(c, c))
        cg_world = R @ c + t
        I_world = R @ Icg @ R.T
        bodies.append(dict(b, occ_name=o["name"], comp=o["component"], mass=m, cg=P(cg_world),
                           I=M @ I_world @ M.T * 1e-4, R_cad=R, t_cad=t))
    return d, bodies


# ---------------------------------------------------------------------------------------------------------------
# 2. Which link does each part belong to? Mostly the rigid groups of the Fusion model.
# ---------------------------------------------------------------------------------------------------------------
GROUP_TO_LINK = {
    "Rigid: blocker claw R": "claw_R", "Rigid: blocker claw L": "claw_L",
    "Rigid: blocker horn R": "blocker_horn_R", "Rigid: blocker horn L": "blocker_horn_L",
    "Rigid: outer ring + pivot hardware": "outer_ring",
    "Rigid: cartridge assembly (motors, pivots, lugs, blocker frame)": "cartridge",
    "Rigid: airframe + TVC servos + yaw bearings": "airframe",
    "Rigid: TVC horn + stud 1": "tvc_horn_yaw", "Rigid: TVC horn + stud 2": "tvc_horn_pitch",
}
for _i in range(1, 5):
    GROUP_TO_LINK["Rigid: leg %d (knuckle + CF tube + tip cap + foot ball)" % _i] = "leg_%d" % _i


def assign_links(d, bodies):
    by_occ = {}
    for g in d["rigid_groups"]:
        link = GROUP_TO_LINK.get(g["name"])
        if link is None:
            print("note: unknown rigid group", g["name"])
            continue
        for name in g["members"]:
            by_occ[name] = link
    ungrouped = []
    for b in bodies:
        n = b["occ_name"]
        link = by_occ.get(n)
        if n.startswith("Landing F15"):
            link = "landing_motor"
        elif n.startswith("Ascent F15"):
            link = "ascent_motor"
        elif link is None:
            idx = n.rsplit(":", 1)[-1]
            if n.startswith("Blocker link"):
                link = "blocker_link_R" if idx == "1" else "blocker_link_L"
            elif n.startswith("Pushrod"):
                link = "tvc_rod_yaw" if idx == "1" else "tvc_rod_pitch"
            elif n.startswith("Leg keeper ring"):
                link = "keeper_ring"
            elif n.startswith("Servo horn") and idx == "7" or n.startswith("Keeper drive"):
                link = "keeper_horn"
            else:  # shells, decks, forward-body hardware, keeper servo, leg pins / springs / plungers
                link = "airframe"
                ungrouped.append(n)
        b["link"] = link
    return ungrouped


# Links in parent-first order: name, parent, Fusion joint that moves it, kind
LINKS = [
    ("airframe", None, None, "root"),
    ("outer_ring", "airframe", "YAW axle (outer ring - airframe)", "revolute"),
    ("cartridge", "outer_ring", "PITCH axle (cartridge - outer ring)", "revolute"),
    ("landing_motor", "cartridge", None, "fixed"),
    ("ascent_motor", "cartridge", None, "fixed"),
    ("claw_R", "cartridge", "Blocker claw R pivot (M3 bolt)", "revolute"),
    ("claw_L", "cartridge", "Blocker claw L pivot (M3 bolt)", "revolute"),
    ("blocker_horn_R", "cartridge", "Blocker servo R shaft", "revolute"),
    ("blocker_horn_L", "cartridge", "Blocker servo L shaft", "revolute"),
    ("blocker_link_R", "blocker_horn_R", "Blocker link R - horn pin", "passive"),
    ("blocker_link_L", "blocker_horn_L", "Blocker link L - horn pin", "passive"),
    ("tvc_horn_yaw", "airframe", "TVC yaw servo shaft", "revolute"),
    ("tvc_horn_pitch", "airframe", "TVC pitch servo shaft", "revolute"),
    ("tvc_rod_yaw", "tvc_horn_yaw", "TVC yaw U-joint: cross - horn stud (Z)", "passive"),
    ("tvc_rod_pitch", "tvc_horn_pitch", "TVC pitch U-joint: cross - horn stud (Z)", "passive"),
    ("keeper_horn", "airframe", "Keeper servo shaft", "revolute"),
    ("keeper_ring", "airframe", "Keeper ring (rotates -9 deg / clockwise to release)", "revolute"),
] + [("leg_%d" % i, "airframe", "Leg %d (az %d) hinge (revolute, deploys 119 deg)" % (i, 45 + 90 * (i - 1)), "revolute")
     for i in range(1, 5)]
# What the servo (or the leg spring) drives, and which one it is in rocket.cfg
DRIVER = {"tvc_horn_yaw": "tvc_yaw", "tvc_horn_pitch": "tvc_pitch", "blocker_horn_R": "blocker_R",
          "blocker_horn_L": "blocker_L", "keeper_horn": "keeper", "outer_ring": "tvc_chain", "cartridge": "tvc_chain",
          "claw_R": "blocker_chain_R", "claw_L": "blocker_chain_L", "keeper_ring": "keeper_chain"}
for _i in range(1, 5):
    DRIVER["leg_%d" % _i] = "leg_spring"


def closest_point_on_axis_to_z_axis(origin_mm, axis):
    """The Fusion hinge origin is anywhere along the pin: use the pin point nearest the rocket centre line."""
    o = np.asarray(origin_mm, float)
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    s = -(o[0] * a[0] + o[1] * a[1]) / (a[0] ** 2 + a[1] ** 2)
    return o + s * a


def joint_table(d):
    """The Fusion joints we use: origin (body, m), axis (body), limits and the value at neutral (deg)."""
    cad = {j["name"]: j for j in d["joints"]}
    out = {}
    for name, parent, jname, kind in LINKS:
        if jname is None:
            continue
        j = cad[jname]
        origin_mm = np.array(j["origin"]) * 10.0
        axis = np.array(j["axis"]) if j.get("axis") else None
        if name.startswith("leg_"):
            origin_mm = closest_point_on_axis_to_z_axis(origin_mm, axis)
        entry = {"cad_joint": jname, "origin": P(origin_mm / 10.0)}
        if axis is not None:
            entry["axis"] = V(axis / np.linalg.norm(axis))
        if "value" in j:
            v = math.degrees(j["value"])
            v = (v + 180) % 360 - 180  # Fusion hands back 359.5 for -0.5
            entry["value_at_neutral_deg"] = v
            lim = j.get("limits")
            if lim and lim[2] and lim[3]:
                entry["limits_deg"] = [math.degrees(lim[0]), math.degrees(lim[1])]
        out[name] = entry
    return out


def build_links(d, bodies, joints, placeholders):
    """Mass properties of each link, in its own frame (origin at its joint origin, axes = body axes)."""
    links = {}
    for name, parent, jname, kind in LINKS:
        parts = [b for b in bodies if b["link"] == name]
        if name == "airframe":
            ph = [dict(mass=p["mass"], cg=np.array(p["cg_body"]), I=np.array(p["inertia_cg"]), name=p["name"],
                       material="placeholder", count=1, placeholder=True) for p in placeholders if p["mass"] > 0]
        else:
            ph = []
        use = [dict(mass=b["mass"], cg=b["cg"], I=b["I"]) for b in parts] + ph
        m, cg, I = combine(use)
        j = joints.get(name, {})
        origin = j.get("origin")
        if origin is None:  # root / fixed links: same frame as their parent
            origin = links[parent]["origin"] if parent else np.zeros(3)
        listing = {}
        for b in parts:
            key = (b["comp"].split(" (")[0] if False else b["occ_name"].rsplit(":", 1)[0], b["material"])
            e = listing.setdefault(key, {"name": key[0], "material": key[1], "mass_g": 0.0, "count": 0})
            e["mass_g"] += b["mass"] * 1000.0
            e["count"] += 1
        for p in ph:
            listing[("PH", p["name"])] = {"name": p["name"], "material": "placeholder (editable)", "mass_g": p["mass"] * 1000, "count": 1}
        links[name] = dict(name=name, parent=parent, kind=kind, origin=origin, axis=j.get("axis"), joint=j,
                           mass=m, cg_body=cg, cg=cg - origin, I=I, parts=list(listing.values()),
                           bodies=parts, n_bodies=len(parts))
    return links


# ---------------------------------------------------------------------------------------------------------------
# 3. Pose of the linked assembly (used by the self-check; the C++ sim does the same sum every step)
# ---------------------------------------------------------------------------------------------------------------
def poses(links, values_deg):
    """World (body frame) rotation and translation of each link's frame. values_deg: link -> angle relative to neutral."""
    out = {}
    for name, parent, jname, kind in LINKS:
        L = links[name]
        if parent is None:
            out[name] = (np.eye(3), L["origin"].copy())
            continue
        Rp, tp = out[parent]
        a = values_deg.get(name, 0.0)
        if L["axis"] is not None and a != 0.0:
            Rj = rot(L["axis"], math.radians(a))
        else:
            Rj = np.eye(3)
        # child origin in the parent's frame is (child origin - parent origin); rotating about it
        out[name] = (Rp @ Rj, tp + Rp @ (L["origin"] - links[parent]["origin"]))
    return out


def assemble(links, values_deg, include, overrides=None):
    """Total mass, CG (body frame), inertia about that CG, for the links in `include`."""
    ps = poses(links, values_deg)
    parts = []
    for n in include:
        L = links[n]
        R, t = ps[n]
        mass, cg, I = L["mass"], L["cg"], L["I"]
        if overrides and n in overrides:
            mass, cg, I = overrides[n]
        parts.append(dict(mass=mass, cg=R @ cg + t, I=R @ I @ R.T))
    return combine(parts)


# ---------------------------------------------------------------------------------------------------------------
# 4. Editable placeholder masses (things not modelled in CAD). The roll wheel is gone: the bay is empty.
# ---------------------------------------------------------------------------------------------------------------
DEFAULT_PLACEHOLDERS = [
    {"name": "avionics (FC + IMU + GNSS + baro)", "mass_kg": 0.040, "cad_z_mm": 335.0, "box_mm": [60, 50, 20]},
    {"name": "battery (2S ~450-500 mAh LiPo)", "mass_kg": 0.035, "cad_z_mm": 320.0, "box_mm": [55, 30, 15]},
    {"name": "wiring + connectors", "mass_kg": 0.010, "cad_z_mm": 200.0, "box_mm": [60, 60, 40]},
    {"name": "roll reaction wheel (REMOVED: bay is empty)", "mass_kg": 0.0, "cad_z_mm": 204.5, "box_mm": [66, 66, 30]},
]


def load_placeholders():
    path = os.path.join(OUT, "placeholders.json")
    if not os.path.exists(path):
        json.dump({"note": "Point masses for things the CAD does not model. Edit here, then rerun tools/build_vehicle.py. "
                           "box_mm is the X, Y, Z size used for the inertia; cad_z_mm is the Fusion world Z of the centre.",
                   "items": DEFAULT_PLACEHOLDERS}, open(path, "w"), indent=1)
    items = json.load(open(path))["items"]
    out = []
    for it in items:
        m = it["mass_kg"]
        a, b, c = [x / 1000.0 for x in it["box_mm"]]
        I_cad = np.diag([m * (b * b + c * c) / 12, m * (a * a + c * c) / 12, m * (a * a + b * b) / 12])
        out.append({"name": it["name"], "mass": m, "cg_body": P([0, 0, it["cad_z_mm"] / 10.0]).tolist(),
                    "inertia_cg": (M @ I_cad @ M.T).tolist(), "cad_z_mm": it["cad_z_mm"]})
    return out


# ---------------------------------------------------------------------------------------------------------------
# 5. Self-check against the numbers computed from the Fusion model by hand (no reaction wheel)
# ---------------------------------------------------------------------------------------------------------------
EXPECTED = {  # state: (mass kg, CG above the ascent motor's aft end m, Ixx, Iyy, Izz)
    # Re-baselined after Fusion v17 (v16 leg lock A2 sleeves + thread pads; v17 lock arm 22 -> 33 mm: boss/ramp, lobes, +3.7 g, roll inertia +4 %).
    # The original hand-computed handoff table was: A (0.9403, 0.2266, 0.02017, 0.02019, 0.00098), B (0.8463, ...).
    "A liftoff, legs stowed": (0.94668, 0.22586, 0.020279, 0.0203, 0.0010309),
    "B landing motor loaded, stowed": (0.85268, 0.24448, 0.017196, 0.017218, 0.0010209),
    "B loaded, legs deployed": (0.85268, 0.22157, 0.020934, 0.020956, 0.0041546),
    "C landing burnout, deployed": (0.79168, 0.22417, 0.020797, 0.020818, 0.0041516),
}


def motor_models(links, curve_masses=None):
    """Loaded motor from CAD, burnt casing = 33.0 g thin tube with CG 42 % up from the nozzle end (NAR certification)."""
    out = {}
    L_M, R_M = 0.114, 0.0145
    for name in ("landing_motor", "ascent_motor"):
        L = links[name]
        loaded = dict(mass=L["mass"], cg=L["cg"], I=L["I"])  # link frame
        # In the link frame the motor axis is body x and the nozzle end is at the low x end
        cg = L["cg"].copy()
        nozzle_x = cg[0] - L_M / 2
        m_c = 0.033
        c_case = cg.copy()
        c_case[0] = nozzle_x + 0.42 * L_M
        I_case = np.diag([m_c * R_M ** 2, m_c * (R_M ** 2 / 2 + L_M ** 2 / 12), m_c * (R_M ** 2 / 2 + L_M ** 2 / 12)])
        m_p = loaded["mass"] - m_c
        c_prop = (loaded["mass"] * cg - m_c * c_case) / m_p
        I_prop = loaded["I"] - I_case - tensor_shift(m_c, c_case - cg) - tensor_shift(m_p, c_prop - cg)
        out[name] = {"loaded": loaded,
                     "case": dict(mass=m_c, cg=c_case, I=I_case),
                     "propellant": dict(mass=m_p, cg=c_prop, I=I_prop)}
    return out


def self_check(links):
    ps_deployed = {"leg_%d" % i: 119.0 for i in range(1, 5)}
    ps_deployed["keeper_ring"] = -9.0
    all_links = [n for n, *_ in LINKS]
    motors = motor_models(links)
    no_asc = [n for n in all_links if n != "ascent_motor"]
    burnt = {"landing_motor": (motors["landing_motor"]["case"]["mass"], motors["landing_motor"]["case"]["cg"],
                               motors["landing_motor"]["case"]["I"])}
    results = {
        "A liftoff, legs stowed": assemble(links, {}, all_links),
        "B landing motor loaded, stowed": assemble(links, {}, no_asc),
        "B loaded, legs deployed": assemble(links, ps_deployed, no_asc),
        "C landing burnout, deployed": assemble(links, ps_deployed, no_asc, burnt),
    }
    report, ok = {}, True
    print("\nSelf-check against the hand-computed table (no reaction wheel):")
    for k, (m, cg, I) in results.items():
        e = EXPECTED[k]
        got = (m, cg[0], I[1, 1], I[2, 2], I[0, 0])  # body: x nose (roll), y/z pitch/yaw
        # EXPECTED order: mass, cg, Ixx(pitch), Iyy(yaw), Izz(roll)
        cmp = [("mass", m, e[0]), ("cg", cg[0], e[1]), ("I_pitch", I[1, 1], e[2]), ("I_yaw", I[2, 2], e[3]), ("I_roll", I[0, 0], e[4])]
        line = []
        for nm, g, x in cmp:
            err = (g - x) / x * 100
            bad = abs(err) > 0.5
            ok = ok and not bad
            line.append("%s %.5g (table %.5g, %+.2f%%)%s" % (nm, g, x, err, " <-- OUT" if bad else ""))
        print("  %-34s" % k, "; ".join(line))
        report[k] = {nm: {"sim": g, "table": x, "error_percent": (g - x) / x * 100} for nm, g, x in cmp}
        report[k]["cg_offset_from_centreline_mm"] = float(np.hypot(cg[1], cg[2]) * 1000)
    return report, ok


# ---------------------------------------------------------------------------------------------------------------
# 6. Meshes: one GLB per link, in the link frame, with a colour per material
# ---------------------------------------------------------------------------------------------------------------
def material_look(material, occ_name):
    """(rgb, metallic, roughness, is_shell) for the viewer."""
    m = (material or "") + " " + occ_name
    if occ_name.startswith(("Upper Shell", "Lower Shell", "Aft Airframe")):
        return (0.86, 0.87, 0.90), 0.0, 0.55, True
    if occ_name.startswith("Landing F15"):
        return (0.18, 0.36, 0.78), 0.0, 0.6, False
    if occ_name.startswith("Ascent F15"):
        return (0.88, 0.47, 0.14), 0.0, 0.6, False
    if "KST" in m:
        return (0.10, 0.10, 0.12), 0.1, 0.5, False
    if "Macor" in m:
        return (0.93, 0.90, 0.82), 0.0, 0.9, False
    if "Bronze" in m:
        return (0.72, 0.47, 0.20), 0.8, 0.4, False
    if "Brass" in m:
        return (0.85, 0.68, 0.25), 0.8, 0.4, False
    if "Aluminium" in m:
        return (0.78, 0.80, 0.84), 0.9, 0.3, False
    if "Carbon" in m:
        return (0.07, 0.07, 0.08), 0.2, 0.35, False
    if "EPS" in m:
        return (0.97, 0.97, 0.95), 0.0, 0.95, False
    if "Stainless" in m or "Bearing steel" in m or "OVERRIDE" in m:
        return (0.72, 0.74, 0.77), 0.9, 0.35, False
    return (0.27, 0.29, 0.33), 0.0, 0.6, False  # printed PAHT-CF


def read_mesh(body):
    raw = open(os.path.join(RAW, "meshes", body["mesh"]["file"]), "rb").read()
    n, k = body["mesh"]["nodes"], body["mesh"]["indices"]
    v = np.frombuffer(raw[:n * 12], dtype="<f4").reshape(-1, 3).astype(float)  # cm, Fusion world
    f = np.frombuffer(raw[n * 12:n * 12 + k * 4], dtype="<u4").reshape(-1, 3).astype(np.int64)
    return v, f


def body_mesh_world(body):
    """Triangles of one body in the neutral body frame (m)."""
    v, f = read_mesh(body)
    v = v * 10.0  # mm
    vb = np.stack([v[:, 2] - Z_BOTTOM_MM, v[:, 0], v[:, 1]], axis=1) / 1000.0
    return vb, f


def export_meshes(links, budget_faces=4000):
    import trimesh
    os.makedirs(os.path.join(OUT, "meshes"), exist_ok=True)
    sizes = {}
    for name, L in links.items():
        path = os.path.join(OUT, "meshes", name + ".glb")
        if os.path.exists(path) and "--redo-meshes" not in sys.argv:  # resumable: a link already exported is kept
            continue
        t0 = time.time()
        scene = trimesh.Scene()
        groups = {}
        for b in L["bodies"]:
            if "mesh" not in b:
                continue
            rgb, metal, rough, shell = material_look(b["material"], b["occ_name"])
            groups.setdefault((rgb, metal, rough, shell), []).append(b)
        total_faces = 0
        for (rgb, metal, rough, shell), bs in groups.items():
            meshes = []
            for b in bs:
                v, f = body_mesh_world(b)
                v = v - L["origin"]
                mesh = trimesh.Trimesh(v, f, process=False)
                mesh.merge_vertices()
                mesh.update_faces(mesh.nondegenerate_faces())
                n = len(mesh.faces)
                # export_cad.py already asks Fusion for a coarse mesh (~75k triangles in all); only cap the big parts
                target = int(min(n, budget_faces))
                if b["occ_name"].startswith(("Landing leg foot",)):
                    target = 300
                if target < n:
                    mesh = mesh.simplify_quadric_decimation(face_count=target)
                meshes.append(mesh)
            merged = trimesh.util.concatenate(meshes)
            try:
                merged = trimesh.graph.smooth_shade(merged, angle=math.radians(35))
            except Exception:
                pass
            merged.visual = trimesh.visual.TextureVisuals(material=trimesh.visual.material.PBRMaterial(
                baseColorFactor=[int(255 * c) for c in rgb] + [255], metallicFactor=metal, roughnessFactor=rough,
                doubleSided=True))
            label = ("shell:" if shell else "part:") + "%02x%02x%02x" % tuple(int(255 * c) for c in rgb)
            scene.add_geometry(merged, node_name=label, geom_name=label)
            total_faces += len(merged.faces)
        scene.export(path)
        sizes[name] = (os.path.getsize(path), total_faces)
        print("  %-16s done in %.1f s" % (name, time.time() - t0), flush=True)
    for name in links:  # sizes of everything on disk, including links kept from an earlier run
        p = os.path.join(OUT, "meshes", name + ".glb")
        if name not in sizes and os.path.exists(p):
            sizes[name] = (os.path.getsize(p), 0)
    tot = sum(s for s, _ in sizes.values())
    print("meshes: %d files, %.2f MB, %d triangles" % (len(sizes), tot / 1e6, sum(f for _, f in sizes.values())))
    for k, (s, f) in sizes.items():
        print("  %-16s %7.1f kB %6d tri" % (k, s / 1e3, f))
    return sizes


# ---------------------------------------------------------------------------------------------------------------
# 7. vehicle.json
# ---------------------------------------------------------------------------------------------------------------
NOZZLE_EXIT_R_MM = 4.75   # Estes F15 nozzle exit radius, about 9.5 mm diameter: VERIFY (not in the CAD, the motor is a plain cylinder)
PLUME_HALF_ANGLE_DEG = 10.0  # how fast the exhaust spreads behind the nozzle: VERIFY (guess)


def jsonable(o):
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, (np.floating,)):
        return float(o)
    if isinstance(o, (np.integer,)):
        return int(o)
    raise TypeError(type(o))


def write_vehicle_json(d, bodies, links, joints, placeholders, report, ok):
    import vehicle_linkages as vl
    J = {j["name"]: j for j in d["joints"]}
    plume_r = NOZZLE_EXIT_R_MM + (links["claw_R"]["origin"][0] - 0.126) * -1000.0 * math.tan(math.radians(PLUME_HALF_ANGLE_DEG))
    motors = motor_models(links)
    out = {"source": d["source"], "frames": __doc__.split("Frames.")[1].split('"""')[0].strip() if "Frames." in __doc__ else "",
           "units": "m, kg, kg m^2, degrees unless a name says otherwise; inertia is the full 3x3 about the link CG, row major, body axes",
           "self_check": {"passed": ok, "rows": report}, "link_order": [n for n, *_ in LINKS], "links": {}}
    ends = {  # passive links: where their two ends attach (link name, body-frame point at neutral), for the viewer
        "tvc_rod_yaw": [("tvc_horn_yaw", "horn_stud", "yaw"), ("cartridge", "cartridge_stud", "yaw")],
        "tvc_rod_pitch": [("tvc_horn_pitch", "horn_stud", "pitch"), ("cartridge", "cartridge_stud", "pitch")],
    }
    tvc = vl.tvc_section(J)
    blocker = vl.blocker_section(J, links, plume_r)
    keeper = vl.keeper_section(J, links)
    legs = vl.legs_section(d, bodies, links, J)
    for n, L in links.items():
        e = {"parent": L["parent"], "kind": L["kind"], "driver": DRIVER.get(n), "origin": L["origin"], "mass": L["mass"],
             "cg": L["cg"], "inertia": L["I"].reshape(-1), "n_bodies": L["n_bodies"], "parts": L["parts"],
             "mesh": "meshes/%s.glb" % n if any("mesh" in b for b in L["bodies"]) else None}
        if L["joint"]:
            e["joint"] = {k: v for k, v in L["joint"].items()}
        if n in ends:
            e["ends"] = [{"on": a, "point": tvc[k][p]} for a, p, k in ends[n]]
        if n.startswith("blocker_link_"):
            s = n[-1]
            e["ends"] = [{"on": "blocker_horn_" + s, "point": blocker[s]["horn_pin"]}, {"on": "claw_" + s, "point": blocker[s]["claw_pin"]}]
        out["links"][n] = e
    out["placeholders"] = [{"name": p["name"], "mass": p["mass"], "cg": p["cg_body"]} for p in placeholders]
    out["motors"] = {}
    for n, m in motors.items():
        L = links[n]
        bb = np.array([b["bbox_world"] for b in L["bodies"]])[0]
        out["motors"][n] = {"link": n, "length_m": 0.114, "radius_m": 0.0145, "aft_end": [float(P(bb[0])[0]), 0.0, 0.0],
                            "loaded": m["loaded"], "case": m["case"], "propellant": m["propellant"],
                            "note": "CAD loaded mass 94 g (F15-0 dummy); casing 33 g thin tube CG 42 % up from the nozzle; "
                                    "propellant = loaded - casing (the .eng file says 60 g / 103 g total: VERIFY)"}
    out["tvc"] = tvc
    out["blocker"] = blocker
    out["keeper"] = keeper
    out["legs"] = legs
    out["contacts"] = vl.contacts_section(d, bodies, links)
    out["clearances_cad"] = vl.clearance_section(d, bodies)
    out["nozzle"] = {"exit_radius_mm": NOZZLE_EXIT_R_MM, "plume_half_angle_deg": PLUME_HALF_ANGLE_DEG, "plume_radius_at_claws_mm": plume_r,
                     "note": "not in the CAD (the motor is a plain cylinder): VERIFY"}
    path = os.path.join(OUT, "vehicle.json")
    with open(path, "w") as fh:
        json.dump(out, fh, indent=None, default=jsonable, separators=(",", ":"))
    print("wrote vehicle.json (%.1f kB)" % (os.path.getsize(path) / 1e3))
    return out


def main():
    skip_meshes = "--no-meshes" in sys.argv
    d, bodies = load_dump()
    ungrouped = assign_links(d, bodies)
    placeholders = load_placeholders()
    joints = joint_table(d)
    links = build_links(d, bodies, joints, placeholders)
    print("source:", d["source"]["document"], "version", d["source"]["version"])
    print("%d bodies, total %.1f g; links:" % (len(bodies), sum(b["mass"] for b in bodies) * 1000))
    for n, L in links.items():
        print("  %-16s %2d bodies %7.2f g  origin %s" % (n, L["n_bodies"], L["mass"] * 1000 - sum(p["mass"] * 1000 for p in placeholders) * (n == "airframe"), np.round(L["origin"], 4)))
    report, ok = self_check(links)
    print("self-check", "PASS" if ok else "FAIL")
    if not skip_meshes:
        export_meshes(links)
    write_vehicle_json(d, bodies, links, joints, placeholders, report, ok)


if __name__ == "__main__":
    main()
