# Ascent tumble: root cause and fix (sim, branch cad-integration, uncommitted)

All numbers are from `tvc_sim` with the CAD v17 vehicle, same Monte Carlo seeds before and after, unless noted. "Tilt" is the angle of the nose from vertical.

## What was wrong

1. **The steering loop was unstable once the rocket was moving fast.** The finless body only has lift at the nose. The aerodynamic centre is at about 0.55 m from the aft end and the CG at 0.23 m (about -3.9 calibres, `tools/aero_strip.py`). At 20 m/s that is an overturning stiffness of 0.89 N m per radian of angle of attack, or 44 rad/s^2 per radian with I = 0.0203 kg m^2 (it grows with speed squared: 11 at 10 m/s, 25 at 15 m/s). The old gains (`kp` 40, `kd` 10) are the same size as that. With about 20 ms of latency and 20 ms of servo lag, the linear loop (`tools/loop_margin.py`) is unstable at 20 m/s unless the loop gain is above 1.3x the design value, and it is unstable at the end of the burn, when thrust tails off to 13 N against the 14.4 N the gains were scaled for.
2. **The remaining boost failures are the gimbal running out of authority, not the loop.** The pivot is 92 mm below the CG, so 8 degrees of gimbal at 14 N gives 0.18 N m. That holds the nose against a crosswind of about 4 m/s at 20 m/s climb (angle of attack limit 11.5 degrees at 20 m/s, 20 degrees at 15 m/s). Above that it cannot be held whatever the gains.
3. **The coast flip is the same geometry with no thrust to fight it.** From burnout the divergence rate is 6.6 per second at 20 m/s, so about 6.7 e-folds (x830) before apogee. By hand estimate, a burnout tilt of about 0.04 degrees would be needed to stay under 30 degrees at apogee. No control change in the boost can do that, so 97-99% of flights flip past 90 degrees in the coast, before and after. This is physical, not a bug.

## What changed (the fix)

- `rocket.cfg`: `kp` 40 -> 100, `kd` 10 -> 20, new `rate_filter_hz = 25` (comments explain the derivation).
- `flight_computer.cpp/.h`: `steer()` low-passes the gyro rate used by the `kd` term (restarts after a gap). Needed because the printed mount's ringing (`gimbal_wobble_hz`, VERIFY) is otherwise fed back by `kd`.
- `config.h/.cpp`: the new key.
- Logging only: `simulation.h/.cpp`, `main.cpp` add burnout/boost/coast/apogee tilt columns to `montecarlo.csv`; `tools/mc_summary.py` prints them; `tools/loop_margin.py` reproduces the linear check.

## Result (1000 runs, same seeds, current rocket.cfg otherwise)

| Item | Before | After |
|---|---|---|
| Landing success | 88.6% | 91.5% |
| Flights with worst boost tilt over 10 deg | 54.5% | 10.0% |
| Worst tilt in boost, median / 90th pct | 10.7 / 140.6 deg | 4.0 / 9.8 deg |
| Mean tilt in boost | 6.2 deg | 3.0 deg |
| Tilt at burnout, median / 90th pct | 9.8 / 97 deg | 2.7 / 9.7 deg |
| Seconds the steering asked for 8 deg or more (mean) | 0.28 | 0.14 |
| Flips past 90 deg in the coast, before apogee | 97% | 99% |
| Tilt at landing ignition, median | 45.6 deg | 52.9 deg |
| Plunger sheared / tipped over | 69 / 92 | 58 / 68 |

By wind (500 runs): below 3 m/s, no flight tips past 10 degrees in the boost any more (before: 19 to 66%). At 4 to 5 m/s, 40% still do, which is the authority limit in point 2.

Caveats: landing success moves by about 1.4 points per 500 runs, so +2.9 over 1000 is about two sigma. Landing does not depend on ignition tilt: fire tilt shows no relation to failure in either run; the failures are hard touchdowns and plunger shear. The ascent fix does not change that, apart from a small gain from the landing burn using the same steering.

The 4 degree median tilt in boost is mostly intended: `boost_max_lean_deg` leans the nose up to 4 degrees toward the pad, on top of wind and launch tilt.

## Things tried that did not help (kept out)

- Thrust-scheduled gain (accelerometer thrust estimate): no gain in the 300-run tests.
- Load relief (shift the target toward the load when the gimbal demand is high): worse in both directions.
- Raising `kp` alone: `kp` 170 / `kd` 10 is worse than the baseline. Damping, not stiffness, did the work.
- Nose ballast 50 g: boost fine, but landing success falls to 77% and apogee to 49 m.
- Moving the 75 g of avionics and battery from 0.46 m to 0.58 m (CG +9 mm, no added mass): boost failures over 10 deg fall from 9% to 4%, but landing success falls to 81% (tipped over 87 against 40).

## CAD suggestion (not made, needs your OK)

A bigger gimbal range is the only change that helped both ascent and landing: emulated in the sim by scaling the thrust angle per servo degree, with the same servo speed.

| Gimbal range | Boost flights over 10 deg | Landing success |
|---|---|---|
| 8 deg (now) | 9% | 90.8% |
| 10 deg (x1.25) | 3% | 90.6% |
| 12 deg (x1.5) | 1% | 92.2% |
| 16 deg (x2) | 0% | not run |

Caveats: the real part is a longer horn and a cartridge hard stop beyond 8.75 degrees; the servo torque, pushrod geometry and ring clearance were not checked. An earlier horn-9 mm / clip-12 degree attempt looked worse only because the gains were still 40 / 10.

## Other notes

- `selftest mass` fails 3 checks on the original repo too (the table value 0.94029 kg is older than CAD v17); not touched.
- `control_delay` has to be shorter than the control period (0.01 s): the sim keeps only one command in flight, so a longer delay stops commands arriving.
