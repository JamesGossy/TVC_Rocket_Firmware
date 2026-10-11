"""Make flight_software/blocker_keeper_map.h from vehicle/vehicle.json.

The flight computer asks for a share of the landing motor's thrust to be blocked. This turns that into the
horn angle of the two blocker servos, using the CAD four-bar linkage and the area each claw covers, and gives
the keeper servo's hold and release angles. Same idea as tvc_command_map.h, and just as portable C.

Run:  python3 tools/make_flight_maps.py
"""
import json
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
OPEN_DEG = -80.0          # horn angle with the claws well clear of the exhaust (the soft stop is -82)
SHARES = 21               # table entries, 0 to 1
RING_MARGIN_DEG = 0.1     # how far past the ring's hard stop the release command asks for


def blocker_share_table(vehicle):
    """Servo horn angle (deg, steps of 1) against the share of the full-close blocking both claws give."""
    sides = [vehicle["blocker"]["R"], vehicle["blocker"]["L"]]
    closed = 0
    curves = []
    for s in sides:
        rows = np.array(s["blocked_table"]["rows"])
        claw, area = rows[::-1, 0], rows[::-1, 1]          # ascending claw angle for interp
        closed += area[-1]
        curves.append((np.array(s["servo_deg"]), np.array(s["claw_deg"]), claw, area))
    servo = curves[0][0]
    share = np.zeros_like(servo)
    for sv, cl, claw, area in curves:
        share += np.interp(cl, claw, area)
    return servo, share / closed


def keeper_release_command(vehicle):
    k = vehicle["keeper"]
    c, r = k["horn_centre_offset_m"], k["stud_radius_m"]
    stop = -k["ring_limits_deg"][0]
    for a in range(30, 80):
        ring = math.degrees(math.atan2(r * math.sin(math.radians(a)), c + r * math.cos(math.radians(a))))
        if ring >= stop + RING_MARGIN_DEG:
            return a, ring
    raise SystemExit("keeper cannot reach its stop")


def main():
    v = json.load(open(os.path.join(ROOT, "vehicle", "vehicle.json")))
    servo, share = blocker_share_table(v)
    # share falls as the horn goes more negative. Invert it on a regular share grid.
    order = np.argsort(share)
    targets = np.linspace(0, 1, SHARES)
    horn = np.interp(targets, share[order], servo[order])
    horn[0] = OPEN_DEG
    release, ring = keeper_release_command(v)
    lines = []
    lines.append("/*")
    lines.append(" * blocker_keeper_map.h  -  made by tools/make_flight_maps.py from vehicle/vehicle.json (the v3 CAD).")
    lines.append(" * Do not edit by hand: change the CAD or the script and run it again.")
    lines.append(" *")
    lines.append(" * Blocker: share of the full-close blocking (0 = claws open, 1 = fully closed) -> servo horn angle (deg).")
    lines.append(" * Full close blocks 1 - throttle_min of the landing motor's thrust (rocket.cfg), so the flight computer")
    lines.append(" * works out the share from the throttle it wants. Both claws get the same command.")
    lines.append(" * Keeper: hold angle, and the release angle that drives the ring onto its hard stop (the pegs let go at")
    lines.append(" * about %.1f deg of ring, the stop is at %.1f deg; a horn angle of %d deg asks for %.2f deg)." % (
        v["keeper"]["release_ring_deg"], -v["keeper"]["ring_limits_deg"][0], release, ring))
    lines.append(" */")
    lines.append("#ifndef BLOCKER_KEEPER_MAP_H")
    lines.append("#define BLOCKER_KEEPER_MAP_H")
    lines.append("")
    lines.append("#define BLOCKER_OPEN_DEG %.1ff" % OPEN_DEG)
    lines.append("#define BLOCKER_N %d" % SHARES)
    lines.append("static const float BLOCKER_SERVO_DEG[BLOCKER_N] = {")
    for i in range(0, SHARES, 7):
        lines.append("    " + ", ".join("%.2ff" % h for h in horn[i:i + 7]) + ",")
    lines.append("};")
    lines.append("")
    lines.append("#define KEEPER_HOLD_DEG 0.0f")
    lines.append("#define KEEPER_RELEASE_DEG %.1ff" % release)
    lines.append("")
    lines.append("/* Horn angle for the share of full-close blocking wanted (0 to 1) */")
    lines.append("static inline float blocker_servo_for_share(float share)")
    lines.append("{")
    lines.append("    if (share <= 0.0f) return BLOCKER_OPEN_DEG;")
    lines.append("    if (share >= 1.0f) return BLOCKER_SERVO_DEG[BLOCKER_N - 1];")
    lines.append("    float f = share * (BLOCKER_N - 1);")
    lines.append("    int i = (int)f;")
    lines.append("    float a = f - i;")
    lines.append("    return BLOCKER_SERVO_DEG[i] * (1 - a) + BLOCKER_SERVO_DEG[i + 1] * a;")
    lines.append("}")
    lines.append("")
    lines.append("#endif")
    path = os.path.join(ROOT, "flight_software", "blocker_keeper_map.h")
    with open(path, "w", newline="\n") as f:
        f.write("\n".join(lines) + "\n")
    print("wrote", path)
    print("share 0..1 -> horn deg:", np.round(horn, 1).tolist())
    print("keeper release command", release, "deg ->", round(ring, 2), "deg of ring")


if __name__ == "__main__":
    main()
