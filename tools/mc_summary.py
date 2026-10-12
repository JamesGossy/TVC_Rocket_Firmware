#!/usr/bin/env python3
"""Summarise montecarlo.csv: landing %, drift, and what went wrong in the failures."""
import csv, statistics, sys

path = sys.argv[1] if len(sys.argv) > 1 else "montecarlo.csv"
max_touchdown = float(sys.argv[2]) if len(sys.argv) > 2 else 4.0  # m/s the legs survive (max_touchdown_speed)
rows = list(csv.DictReader(open(path)))
n = len(rows)

def f(r, k): return float(r[k])

modes = {"good": 0, "hard impact": 0, "tipped over": 0, "never fired landing motor": 0}
for r in rows:
    if int(float(r["good"])): modes["good"] += 1
    elif f(r, "fire_altitude") <= 0 and f(r, "fire_speed") == 0: modes["never fired landing motor"] += 1
    elif f(r, "hardest_impact") >= max_touchdown: modes["hard impact"] += 1
    else: modes["tipped over"] += 1

drift = sorted((f(r, "landing_x") ** 2 + f(r, "landing_y") ** 2) ** 0.5 for r in rows if f(r, "touchdown_speed") != 0)
print(f"runs {n}")
for k, v in modes.items(): print(f"  {k:28s} {v:5d}  {100*v/n:5.1f}%")
if drift:
    print(f"drift median {statistics.median(drift):.1f} m, 95% under {drift[int(0.95*(len(drift)-1))]:.1f} m")
fails = [r for r in rows if not int(float(r["good"]))]
if fails:
    print("failures: mean wind %.1f m/s (all runs %.1f), mean fire speed %.1f, mean burn left %.2f s, mean sideways %.2f m/s" % (
        statistics.mean(f(r, "wind_speed") for r in fails), statistics.mean(f(r, "wind_speed") for r in rows),
        statistics.mean(f(r, "fire_speed") for r in fails), statistics.mean(f(r, "burn_left") for r in fails),
        statistics.mean(f(r, "sideways_speed") for r in fails)))

# Ascent attitude (columns added with the ascent-tumble work; older montecarlo.csv files do not have them)
if rows and "boost_peak_tilt" in rows[0]:
    def pct(k, p):
        v = sorted(f(r, k) for r in rows)
        return v[int(p / 100 * (len(v) - 1))]
    print("ascent attitude (tilt = nose from vertical, deg), median / 90th percentile:")
    for k, label in [("boost_peak_tilt", "worst tilt in boost"), ("boost_mean_tilt", "mean tilt in boost"),
                     ("burnout_tilt", "tilt at burnout"), ("burnout_rate", "turn rate at burnout (deg/s)"),
                     ("coast_peak_tilt", "worst tilt in coast, to apogee"), ("apogee_tilt", "tilt at apogee"),
                     ("fire_tilt", "tilt at landing ignition")]:
        print(f"  {label:34s} {pct(k, 50):7.1f} / {pct(k, 90):7.1f}")
    over = sum(1 for r in rows if f(r, "boost_peak_tilt") > 10)
    print(f"  boost flights that tipped past 10 deg: {over} ({100 * over / n:.0f}%); gimbal asked for 8 deg or more: mean {statistics.mean(f(r, 'boost_sat_s') for r in rows):.2f} s")
