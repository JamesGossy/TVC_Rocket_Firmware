#!/usr/bin/env python3
"""Second half of the CAD post-processor (imported by tools/build_vehicle.py): the mechanisms.

Takes the links and joints that build_vehicle.py made and adds everything the sim and viewer need beyond mass
properties: TVC pushrod geometry, blocker four-bar and blocked-area table, keeper ring slot, leg data, contact
points, nose profile and pin / bore clearances.

All points are in the BODY frame (m, x out the nose from the aft end of the ascent motor) with the vehicle in its
neutral pose (gimbal 0, claws closed, keeper 0, legs stowed). A link's own frame is the body frame moved to the
link's joint origin, so "point in link frame" = point - links[name]["origin"].
"""
import math
import numpy as np
import build_vehicle as bv

DEG = math.pi / 180.0


def L3(a):
    return [float(x) for x in np.asarray(a, float)]


# ---------------------------------------------------------------------------------------------------------------
# TVC: two servos, two 33.5 mm pushrods, gimbal pivot
# ---------------------------------------------------------------------------------------------------------------
def tvc_section(J):
    out = {}
    for k in ("yaw", "pitch"):
        S = bv.P(J["TVC %s servo shaft" % k]["origin"])
        a = bv.V(np.array(J["TVC %s servo shaft" % k]["axis"]))
        a = a / np.linalg.norm(a)
        H0 = bv.P(J["TVC %s U-joint: cross - horn stud (Z)" % k]["origin"])
        C0 = bv.P(J["TVC %s ball: pushrod - cartridge stud" % k]["origin"])
        out[k] = {"servo_shaft": L3(S), "servo_axis": L3(a), "horn_stud": L3(H0), "cartridge_stud": L3(C0),
                  "rod_length_m": float(np.linalg.norm(H0 - C0)),
                  "horn_radius_m": float(np.linalg.norm(H0 - S - np.dot(H0 - S, a) * a)),
                  "servo_limits_deg": [math.degrees(x) for x in J["TVC %s servo shaft" % k]["limits"][:2]]}
    pivot = bv.P(J["YAW axle (outer ring - airframe)"]["origin"])
    out["pivot"] = L3(pivot)
    out["yaw_axis"] = L3(bv.V(J["YAW axle (outer ring - airframe)"]["axis"]))      # outer ring about the body z axis
    out["pitch_axis"] = L3(bv.V(J["PITCH axle (cartridge - outer ring)"]["axis"]))  # cartridge about the body y axis
    out["cad_joint_limit_deg"] = math.degrees(J["YAW axle (outer ring - airframe)"]["limits"][1])
    out["hard_stop_deg"] = 8.75     # from the handoff (the Fusion joint limit is 8.5 deg, the parameter gimbal_stop 10.5): VERIFY
    out["command_clip_deg"] = 8.0   # tvc_command_map.h circle clip
    out["note"] = ("Cartridge rotation R = Rz(yaw) * Ry(pitch) about the pivot. A positive yaw-servo angle gives positive yaw, "
                   "a positive pitch-servo angle gives NEGATIVE pitch (as in tvc_command_map.h).")
    return out


# ---------------------------------------------------------------------------------------------------------------
# Blocker: servo -> horn -> link -> claw (planar four-bar), claw silhouette against the exhaust plume
# ---------------------------------------------------------------------------------------------------------------
def blocker_chain(J, side):
    s = side
    S = bv.P(J["Blocker servo %s shaft" % s]["origin"])
    aS = bv.V(np.array(J["Blocker servo %s shaft" % s]["axis"]))
    H0 = bv.P(J["Blocker link %s - horn pin" % s]["origin"])
    T0 = bv.P(J["Blocker link %s - claw tab pin (ball)" % s]["origin"])
    Cp = bv.P(J["Blocker claw %s pivot (M3 bolt)" % s]["origin"])
    aC = bv.V(np.array(J["Blocker claw %s pivot (M3 bolt)" % s]["axis"]))
    return dict(S=S, aS=aS / np.linalg.norm(aS), H0=H0, T0=T0, Cp=Cp, aC=aC / np.linalg.norm(aC),
                L=float(np.linalg.norm(H0 - T0)),
                servo_neutral_deg=(math.degrees(J["Blocker servo %s shaft" % s]["value"]) + 180) % 360 - 180,
                servo_limits_deg=[math.degrees(x) for x in J["Blocker servo %s shaft" % s]["limits"][:2]],
                claw_limits_deg=[math.degrees(x) for x in J["Blocker claw %s pivot (M3 bolt)" % s]["limits"][:2]])


def claw_angle(ch, q_deg):
    """Claw angle (deg, Fusion sign) for a servo angle: the link keeps its length. Branch that starts at 0 (closed)."""
    cs = np.arange(-70.0, 5.0, 0.01)
    # the exported horn pin H0 is where it sits at the neutral servo angle (-0.5 deg), so turn it by the difference
    Hp = ch["S"] + bv.rot(ch["aS"], (q_deg - ch["servo_neutral_deg"]) * DEG) @ (ch["H0"] - ch["S"])
    best = None
    for c in cs:
        Tp = ch["Cp"] + bv.rot(ch["aC"], c * DEG) @ (ch["T0"] - ch["Cp"])
        e = abs(np.linalg.norm(Hp - Tp) - ch["L"])
        if best is None or e < best[0] - 1e-12 or (abs(e - best[0]) < 1e-12 and abs(c) < abs(best[1])):
            best = (e, c)
    return float(best[1])


def raster_cover(tris_yz, disk_r_mm, step_mm=0.1):
    """Area (mm^2) and centroid of the union of projected triangles inside a disk centred on the exhaust axis."""
    n = int(2 * disk_r_mm / step_mm) + 1
    xs = (np.arange(n) * step_mm - disk_r_mm)
    gy, gz = np.meshgrid(xs, xs, indexing="ij")
    inside = gy ** 2 + gz ** 2 <= disk_r_mm ** 2
    cover = np.zeros((n, n), bool)
    for t in tris_yz:
        (y0, z0), (y1, z1), (y2, z2) = t
        iy0, iy1 = max(0, int((min(y0, y1, y2) + disk_r_mm) / step_mm)), min(n - 1, int((max(y0, y1, y2) + disk_r_mm) / step_mm) + 1)
        iz0, iz1 = max(0, int((min(z0, z1, z2) + disk_r_mm) / step_mm)), min(n - 1, int((max(z0, z1, z2) + disk_r_mm) / step_mm) + 1)
        if iy0 > iy1 or iz0 > iz1:
            continue
        py, pz = gy[iy0:iy1 + 1, iz0:iz1 + 1], gz[iy0:iy1 + 1, iz0:iz1 + 1]
        den = (z1 - z2) * (y0 - y2) + (y2 - y1) * (z0 - z2)
        if abs(den) < 1e-12:
            continue
        a = ((z1 - z2) * (py - y2) + (y2 - y1) * (pz - z2)) / den
        b = ((z2 - z0) * (py - y2) + (y0 - y2) * (pz - z2)) / den
        m = (a >= 0) & (b >= 0) & (a + b <= 1)
        cover[iy0:iy1 + 1, iz0:iz1 + 1] |= m
    cover &= inside
    area = cover.sum() * step_mm ** 2
    if area == 0:
        return 0.0, 0.0, 0.0
    return float(area), float(gy[cover].mean()), float(gz[cover].mean())


def blocker_section(J, links, plume_radius_mm):
    out = {"plume_radius_mm": plume_radius_mm}
    angs = [-0.5 - i for i in range(0, 83)]
    tab = {}
    for s in ("R", "L"):
        ch = blocker_chain(J, s)
        claw = [claw_angle(ch, q) for q in angs]
        tab[s] = {"servo_deg": angs, "claw_deg": claw}
        out[s] = {"servo_shaft": L3(ch["S"]), "servo_axis": L3(ch["aS"]), "horn_pin": L3(ch["H0"]), "claw_pin": L3(ch["T0"]),
                  "claw_pivot": L3(ch["Cp"]), "claw_axis": L3(ch["aC"]), "link_length_m": ch["L"],
                  "servo_neutral_deg": ch["servo_neutral_deg"], "servo_limits_deg": ch["servo_limits_deg"],
                  "claw_limits_deg": ch["claw_limits_deg"], "servo_deg": angs, "claw_deg": claw}
    # blocked area vs claw angle: project each claw (all its bodies) on the plane normal to the exhaust axis
    disk_area = math.pi * plume_radius_mm ** 2
    out["disk_area_mm2"] = disk_area
    claw_grid = [-2.5 * i for i in range(0, 23)]
    for s in ("R", "L"):
        ch = blocker_chain(J, s)
        verts = []
        for b in links["claw_" + s]["bodies"]:
            if "mesh" in b:
                v, f = bv.body_mesh_world(b)
                verts.append((v, f))
        rows = []
        for c in claw_grid:
            Rm = bv.rot(ch["aC"], c * DEG)
            tris = []
            for v, f in verts:
                w = (ch["Cp"] + (Rm @ (v - ch["Cp"]).T).T) * 1000.0  # mm
                tris.extend(w[f][:, :, 1:3])
            area, cy, cz = raster_cover(tris, plume_radius_mm)
            rows.append([c, area, cy, cz])
        out[s]["blocked_table"] = {"columns": ["claw_deg", "area_mm2", "centroid_y_mm", "centroid_z_mm"], "rows": rows}
    return out


# ---------------------------------------------------------------------------------------------------------------
# Keeper: horn stud in a radial slot of the ring; the ring's pegs leave the leg tab holes after RELEASE deg
# ---------------------------------------------------------------------------------------------------------------
KEEPER_RELEASE_RING_DEG = 8.0     # CAD sweep after the peg shortening (v15): last blocked 7.9 deg, analytic 8.1 deg
KEEPER_RING_CONTACT_DEG = 11.2    # notch end wall meets a stowed leg's tab (CAD sweep)


def keeper_section(J, links):
    S = bv.P(J["Keeper servo shaft"]["origin"])
    aS = bv.V(np.array(J["Keeper servo shaft"]["axis"]))
    ring_o = bv.P(J["Keeper ring (rotates -9 deg / clockwise to release)"]["origin"])
    ring_a = bv.V(np.array(J["Keeper ring (rotates -9 deg / clockwise to release)"]["axis"]))
    lim = J["Keeper ring (rotates -9 deg / clockwise to release)"]["limits"]
    stud_r = 6.0  # horn radius of the M2x8 set screw (BOM: ISO 4026 M2x8 threaded into the horn)
    horn_c = float(np.linalg.norm(S - ring_o))
    return {"servo_shaft": L3(S), "servo_axis": L3(aS), "ring_origin": L3(ring_o), "ring_axis": L3(ring_a),
            "ring_limits_deg": [-KEEPER_RING_CONTACT_DEG, math.degrees(lim[1])],
            "horn_centre_offset_m": horn_c, "stud_radius_m": stud_r / 1000.0,
            "slot_width_m": 0.0023, "slot_radial_range_m": [0.02998, 0.0365],
            "peg_radius_m": 0.036, "peg_diameter_m": 0.0024, "tab_hole_diameter_m": 0.0030,
            "release_ring_deg": KEEPER_RELEASE_RING_DEG,
            "ring_contact_deg": KEEPER_RING_CONTACT_DEG,
            "servo_deg_for_9deg_ring": 56.0,
            "note": ("Ring angle = atan2(c + r cos a, r sin a) - 90 deg with c the horn-to-ring-axis distance and r the stud radius, "
                     "negative (clockwise) for a positive servo angle. The CAD motion link says 9 deg ring : 56 deg horn. "
                     "release_ring_deg: peg clear of the tab hole. CAD sweep (TemporaryBRep, tab turned 0.3 deg about its hinge, v15 pegs "
                     "with tip centre at phi 14.0): last blocked at 7.9 deg, analytic 8.1 -> 8.0 used. There is no physical stop at 9.5 deg "
                     "(that is only the CAD joint limit); the first contact is the notch end wall on a still-stowed leg's tab at "
                     "ring_contact_deg (11.2 deg). The ring limit handed to the sim is that contact angle.")}


# ---------------------------------------------------------------------------------------------------------------
# Legs
# ---------------------------------------------------------------------------------------------------------------
def legs_section(d, bodies, links, J):
    out = {"latch_deg": 119.0, "ramp_start_deg": 91.0, "plunger_arm_m": 0.033, "tube_length_m": 0.260,
           "tube_od_m": 0.014, "tube_id_m": 0.012, "foot_radius_m": 0.0125, "legs": []}
    for i in range(1, 5):
        name = "leg_%d" % i
        L = links[name]
        foot = [b for b in L["bodies"] if b["occ_name"].startswith("Landing leg foot")][0]
        sph = [f for f in foot["faces"] if f["kind"] == "sphere"][0]
        centre = bv.P(sph["origin"])
        tube = [b for b in L["bodies"] if b["occ_name"].startswith("Landing leg tube")][0]
        bb = np.array(tube["bbox_world"])
        ends = [bv.P(bb[0]), bv.P(bb[1])]
        out["legs"].append({"link": name, "azimuth_deg": 45 + 90 * (i - 1), "hinge_origin": L3(L["origin"]),
                            "hinge_axis": L3(L["axis"]), "limits_deg": L["joint"].get("limits_deg"),
                            "foot_centre_stowed": L3(centre), "foot_radius_m": float(sph["radius"] / 100.0)})
    # Deployed foot from the same rotation the sim uses
    chk = []
    for lg in out["legs"]:
        o, a, c = np.array(lg["hinge_origin"]), np.array(lg["hinge_axis"]), np.array(lg["foot_centre_stowed"])
        c2 = o + bv.rot(a, 119 * DEG) @ (c - o)
        chk.append(c2)
    chk = np.array(chk)
    out["deployed_foot_centres"] = [L3(c) for c in chk]
    r = np.hypot(chk[:, 1], chk[:, 2])
    out["deployed_check"] = {"radius_mm": [float(x * 1000) for x in r], "z_cad_mm": [float(x * 1000 - 134.0) for x in chk[:, 0]],
                             "span_m": float(2 * r.mean() + 0), "foot_below_ascent_aft_mm": float(-(chk[:, 0].mean() - 0.0125) * 1000)}
    return out


# ---------------------------------------------------------------------------------------------------------------
# Contact points, nose profile
# ---------------------------------------------------------------------------------------------------------------
def contacts_section(d, bodies, links):
    aft = [b for b in bodies if b["occ_name"].startswith("Aft Airframe")][0]
    v, f = bv.body_mesh_world(aft)
    low = v[v[:, 0] < 0.120 - 0.009]  # more than 9 mm below the aft rim: the hinge-cheek lobes
    lobes = []
    az = np.degrees(np.arctan2(low[:, 2], low[:, 1])) % 360.0
    for a0 in range(0, 360, 15):
        m = (az >= a0) & (az < a0 + 15)
        if m.sum():
            sub = low[m]
            r = np.hypot(sub[:, 1], sub[:, 2])
            lobes.append(L3(sub[np.argmax(r - 20 * sub[:, 0])]))  # outermost, lowest corner of this lobe
    rim = [L3([0.120, 0.03975 * math.cos(t * DEG), 0.03975 * math.sin(t * DEG)]) for t in range(0, 360, 30)]
    asc = links["ascent_motor"]
    land = links["landing_motor"]
    nose_tip = None
    prof = []
    for nm in ("Upper Shell", "Lower Shell", "Aft Airframe"):
        b = [x for x in bodies if x["occ_name"].startswith(nm)][0]
        vv, _ = bv.body_mesh_world(b)
        for x in np.arange(0.0, 0.64, 0.005):
            m = np.abs(vv[:, 0] - x) < 0.0025
            if m.sum():
                prof.append((float(x), float(np.hypot(vv[m, 1], vv[m, 2]).max()), nm))
        if nm == "Upper Shell":
            nose_tip = float(vv[:, 0].max())
    # profile = outer shell radius per station (shell bodies only, not the leg cheeks): upper + lower shells
    shell = {}
    for x, r, nm in prof:
        if nm in ("Upper Shell", "Lower Shell"):
            shell[round(x, 3)] = max(shell.get(round(x, 3), 0.0), r)
    profile = [[x, shell[x]] for x in sorted(shell)] + [[nose_tip, 0.0]]
    return {"cheek_lobes": lobes, "aft_rim": rim, "nose_tip": [nose_tip, 0.0, 0.0],
            "nozzle_exit_ascent": L3([0.0, 0.0, 0.0]), "nozzle_exit_landing": L3([0.126, 0.0, 0.0]),
            "body_radius_m": 0.03975, "body_length_m": nose_tip, "profile": profile,
            "note": "profile = outer radius of the printed shells per station (x from the aft end of the ascent motor)"}


# ---------------------------------------------------------------------------------------------------------------
# Pin / bore clearances read from the CAD faces
# ---------------------------------------------------------------------------------------------------------------
def near_cyl(b, o, ax, tol=0.03):
    out = []
    for f in b.get("faces") or []:
        if f["kind"] != "cyl":
            continue
        a = np.array(f["axis"])
        if ax is not None and abs(np.dot(a, ax)) < 0.99:
            continue
        dv = o - np.array(f["origin"])
        if np.linalg.norm(dv - np.dot(dv, a) * a) < tol:
            out.append({"radius_mm": round(f["radius"] * 10, 3), "hole": not f["convex"]})
    return out


def clearance_section(d, bodies):
    B = {b["occ_name"]: b for b in bodies}
    out = {}
    for j in d["joints"]:
        if "origin" not in j or j["kind"] not in ("joint", "asbuilt"):
            continue
        n = j["name"]
        if not any(k in n for k in ("YAW axle", "PITCH axle", "claw R pivot", "Blocker link R", "TVC yaw", "Leg 1 (az 45) hinge (",
                                    "Keeper ring")):
            continue
        o = np.array(j["origin"])
        ax = np.array(j["axis"]) if j.get("axis") else None
        if ax is not None:
            ax = ax / np.linalg.norm(ax)
        out[n] = {k: near_cyl(B[j[k]], o, ax) for k in ("occurrenceOne", "occurrenceTwo") if j[k] in B}
    return out
