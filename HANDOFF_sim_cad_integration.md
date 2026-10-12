# HANDOFF: drive the TVC landing-rocket sim and viewer from the real v3 CAD

You are picking up a project with no memory of earlier sessions. Everything you need is listed here. Read the whole prompt before starting.

## Goal

Make the 6-DOF simulator and its 3D viewer use the real Fusion CAD of the rocket (design **TVC_Rocket_Gimbal_v3**, Default Project) instead of the hand-typed placeholder rocket. Specifically:

1. **Parameters from CAD.** Mass, CG, inertia, dimensions, joint axes, linkage geometry and contact points all come from an export of the Fusion model. **Assume no reaction wheel**: the roll-wheel bay is empty.
2. **Real actuators.** The sim drives the real servos (KST X08H Plus, 5 of them: TVC yaw, TVC pitch, blocker L, blocker R, leg keeper) through the real linkages, and the assembly actually moves in the sim: outer ring, cartridge, blocker claws, keeper ring, 4 legs.
3. **Backlash on every mechanical joint**, with values taken from the CAD clearances (pin vs hole etc.), configurable, and spread in the Monte Carlo.
4. **The viewer shows this CAD**, meshes exported from Fusion, with every moving part animated from the sim log.

## Ground rules

- **Read only on the CAD.** Do not change any geometry, joints or materials in Fusion. Do not create a Fusion simulation study. You may drive joints temporarily to export poses, but put them back to neutral (legs 0, keeper ring 0, gimbal 0) and **do not save the design**. If you think the CAD is wrong, stop and ask.
- **Repo:** `C:\Users\JamesGoss\Documents\Projects\TVS_rocket_simulation` (git, origin `JamesGossy/TVC_Rocket_Firmware`). The working tree has **uncommitted changes**: a pad-return feature in `flight_computer.*`, `config.*`, `rocket.cfg` and `README.md`, plus CRLF churn. Commit them as-is first ("WIP pad return"). Then work on a new branch `cad-integration` in small commits. Don't push without asking.
- **Keep the house style.** Every number lives in `rocket.cfg` with a plain-English comment. Code is short and commented the same way. `flight_software/` must stay portable: no sim-only includes, and it should still fit on the real flight computer. `make` with g++ (MSYS2 UCRT64) must keep working.
- **Measure before/after.** Run `./tvc_sim montecarlo 2000` on the current `main` first and record landing %, median drift and failure modes. Compare every big step against it.
- **Ask before** changing the flight-software architecture (states, estimator) beyond what's needed to output servo commands and the leg release.

## Sources of truth (all under `C:\Users\JamesGoss\Documents\Projects\TVC_Rocket\`)

| File | What it gives you |
|---|---|
| Fusion design TVC_Rocket_Gimbal_v3 (latest version, comment starts "Legs simplified") | geometry, joints, materials, datasheet mass overrides |
| `CAD/Designs/TVC_Rocket_v3_mass_properties.json/.csv` | mass report (**includes** a 60 g roll-wheel placeholder: remove it) |
| `CAD/Designs/v3_checks/massreport2.py` + `massprops_native.json` | how mass props were computed; `PH` list = editable placeholder masses |
| `CAD/Designs/TVC_Rocket_v3_BOM.md`, `TVC_Rocket_v3_material_spec.md` | parts, materials, pin/hole sizes, leg loads |
| `CAD/Designs/TVC_Rocket_Gimbal_v3_BOM.md` | gimbal/blocker notes (blocker kinematics, servo limits) |
| `firmware_snippets/tvc_command_map.h` | gimbal-angle to servo-angle table from CAD kinematics (9x9 grid, 8 deg circle clip) |
| `CAD/Designs/research_landing_legs.md` | leg design reasoning, foam crush numbers |
| Sim: `rocket.cfg`, `common/vehicle.h`, `sim/hardware.h`, `sim/physics.cpp`, `sim/simulation.cpp`, `flight_software/*`, `tools/view.py`, `docs/index.html` | the current sim and viewer |

## Numbers already computed (no reaction wheel; recompute from your export and they should match within 0.5 %)

| State | Mass | CG above ascent-motor aft end | Ixx / Iyy about CG | Izz (roll) |
|---|---|---|---|---|
| A liftoff, legs stowed | 940.3 g | 0.2266 m | 0.02017 / 0.02019 kg m² | 0.00098 |
| B landing motor loaded, stowed | 846.3 g | 0.2455 m | 0.01706 / 0.01708 | 0.00097 |
| B loaded, legs deployed | 846.3 g | 0.2224 m | 0.02082 / 0.02084 | 0.00410 |
| C landing burnout, deployed | 785.3 g | 0.2251 m | 0.02068 / 0.02070 | 0.00410 |

- **Dry (no motors):** 0.7523 kg, CG 0.2533 m, pitch inertia 0.01655 kg m², roll inertia 0.00096 kg m².
- **Dry, legs deployed:** CG 0.2274 m, pitch inertia 0.02056 kg m², roll inertia 0.00409 kg m². CG offset from the centreline is 1.1 mm.
- **Placeholders kept (editable point masses):** avionics 40 g at CAD z 335 mm, battery 35 g at z 320, wiring 10 g at z 200.
- **Gimbal bodies about the pivot, state B:**
  - cartridge + landing motor: 282.6 g, I ≈ 7.9e-4 / 8.2e-4 / 9.3e-5 kg m²;
  - plus the outer ring: 300.1 g, I ≈ 8.0e-4 / 8.2e-4 / 1.1e-4 kg m².
- **Body:** OD 79.5 mm. Nose tip at CAD z +505, aft end of the body at z −14, ascent-motor aft end at z −134, so the overall length is 0.639 m.
- **Legs:**
  - hinge at radius 49 mm, z −8, legs at azimuth 45/135/225/315 deg;
  - deploy 0 to 119 deg;
  - deployed EPS ball centre at r 298.3, z −151.3 (ball R 12.5, bottom z −163.8), span 0.597 m;
  - feet are 29.8 mm below the ascent-motor aft end.
- **F15-0:** 94.0 g loaded / 33.0 g burnt. Blocker floor is 0.6 of thrust. Gimbal command is clipped to an 8 deg circle; the hard stop is 8.75 deg.

## Frames

- **CAD world:** origin at the gimbal pivot, +Z toward the nose, mm. +X is the yaw-servo side and +Y the pitch-servo side.
- **Joint conventions** (from `tvc_command_map.h`): yaw = outer ring about +Y; pitch = cartridge about +X. Yaw servo + gives yaw +; pitch servo + gives pitch −.
- **Sim body frame:** x out the nose, positions measured up from the bottom of the rocket.
- **Mapping:** use the right-handed cyclic map body (x, y, z) = CAD (Z, X, Y) / 1000, with body x = (Z_cad + 134 mm) / 1000.
- **Verify signs with a test.** Command +yaw and +pitch in the sim and check that the cartridge moves the same way it does when you drive the same Fusion joints. Write the convention into `rocket.cfg` and the code comments.

## Fusion API notes (learned the hard way)

- Use the MCP tool `Autodesk_Fusion__fusion_mcp_execute`, featureType `script`. Scripts define `run(context)`, and API units are **cm**.
- Write results to files under the project folder and read them with the shell. The call often times out at 60 s while the script keeps running, so poll the output file; don't re-run blindly.
- For mass props, use **native** body physical properties (`body.getPhysicalProperties(High)`) plus occurrence `transform2`. Proxy physical properties go stale after driving joints in the same script.
  - `getXYZMomentsOfInertia()` returns (xx, yy, zz, xy, yz, xz) in kg cm² about the component origin.
  - The off-diagonals are already signed as tensor terms (see `massreport2.py`).
- Joints are mostly as-built joints (`root.asBuiltJoints`); the gimbal axles are `root.joints` ("YAW axle (outer ring - airframe)", "PITCH axle (cartridge - outer ring)").
  - Other names: "TVC yaw/pitch servo shaft", "TVC … U-joint …", "TVC … ball: pushrod - cartridge stud", "Blocker claw R/L pivot", "Blocker servo R/L shaft", "Blocker link R/L - horn pin / claw tab pin", "Leg N (az …) hinge (revolute, deploys 119 deg)", "Keeper ring (rotates -9 deg / clockwise to release)", "Keeper servo shaft".
  - Motion link: "Keeper servo drives ring (9 deg ring : 56 deg horn)".
- Rigid groups define which occurrences move together. Use them to build the links.
- Purchased parts use `OVERRIDE … g` materials (density chosen to match the datasheet mass). Keep those masses.
- The Ascent F15 dummy interferes with the closed blocker claws in CAD on purpose: the claws only close after the spent ascent case has been ejected.

## Work plan

### 1. CAD export → `vehicle/` folder in the sim repo

Write `tools/export_cad.py` (the Fusion script, kept in the repo) plus a small Python post-processor. It produces:

- **`vehicle/vehicle.json`**
  - **links:** rigid bodies. At least: airframe+forward body (fixed), outer ring, cartridge (with landing motor, and ascent motor until ejected), blocker claw L/R, blocker links L/R, servo horns (5), TVC pushrods (2), keeper ring, legs 1-4 (knuckle+tube+cap+ball), and ascent motor as its own link for ejection.
  - **Per link:** mass, CG, full inertia tensor in the link frame, member parts with materials.
  - **joints:** type, parent, child, origin, axis, limits, the servo that drives it, and the neutral angle.
  - **linkage points:** horn ball-stud centres, cartridge drive-stud centres, pushrod c-c length (33.5), U-joint/ball centres, blocker horn pin, claw tab pin, link c-c (9.87), keeper horn set-screw radius and ring drive-slot geometry.
  - **contact geometry:** foot ball centres and radius per leg, nozzle exit, aft rim, nose tip, cheek lobes.
  - **motor positions and lengths**, nozzle exit point and axis in the cartridge frame.
  - **blocker table:** blocked share of the landing-motor exit vs claw angle. Project each claw silhouette onto the nozzle exit plane; give it per claw so L/R asymmetry gives side force.
  - **nose profile points** for the aero check.
  - **placeholders.**
  - **`source`:** Fusion document name and version number.
- **`vehicle/meshes/*.glb`:** one per link, in the link frame. Export STL/OBJ from Fusion (fine refinement), then decimate and convert to GLB with trimesh. Carry an approximate colour per material. Budget about 5 MB total, since it ships on GitHub Pages; use Draco/meshopt if needed.
- **Self-check:** sum of link masses equals the Fusion total (minus the reaction wheel); neutral-pose CG and inertia match the table above.

### 2. Sim: multi-body mass properties

- Add a `vehicle_file` setting in `rocket.cfg`. Replace the scalar `dry_mass/dry_cg/dry_inertia_*` path with mass properties assembled each physics step from the links at their current joint angles: gimbal tilt, blocker, legs, motor propellant burn and ascent-case ejection.
- Use the full 3x3 inertia in Euler's equations.
- Keep the old scalar model selectable (`vehicle_model = 0/1`) so you can compare.
- The gimbal reaction torque comes from the CAD gimbal-body inertia about the pivot, replacing the point-motor "swinging" estimate. The thrust line runs along the actual cartridge/nozzle axis.

### 3. Sim: actuators and linkages

- **Servo model (shared by all 5).**
  - KST X08H Plus datasheet: speed and torque at your supply voltage, refresh rate (50 Hz analog-style or faster if digital: check and cite), deadband, internal lag, gear backlash.
  - Speed sags with load torque, computed from the linkage reaction. Stall when overloaded.
  - Quote every datasheet value with its source in `rocket.cfg`; mark unknowns `VERIFY`.
- **TVC chain.**
  - Firmware outputs servo horn angles through a port of `tvc_command_map.h`: gimbal angle in, servo angle out, 8 deg circle clip. That is what the real flight computer will send.
  - The sim solves the closed-loop forward kinematics every step: horn angles → ball studs → fixed-length pushrods → outer-ring and cartridge angles. Newton on the 2 constraint equations, from the CAD points.
  - Validate against the `tvc_command_map.h` table: within 0.1 deg across the grid.
  - Model the 8.75 deg hard stop as stiff, damped contact.
  - Keep `gimbal_wobble_*` as link compliance.
- **Blocker chain.**
  - Throttle request → claw servo angles via a map built from the blocker table. From the BOM: closed at linkage dead-centre (servo −0.5 deg limit); open is −55 deg claw at −79 deg servo.
  - Servo → link → claw angle with real kinematics.
  - Claw angle → blocked share and side force from the CAD table.
  - Claws stay open (locked out) while the ascent motor is present.
  - Keep erosion/effectiveness as config factors.
- **Keeper chain.**
  - New firmware command `release_legs`. Decide when to send it (e.g. after apogee in coast, or a set time before the landing burn) and justify it with Monte Carlo results.
  - Servo → horn → M2 set screw in the ring drive slot → ring rotation (9 deg ring : 56 deg horn).
  - Each leg's printed peg leaves its tab hole once the ring passes its release angle.

### 4. Backlash on every joint

- **Model:** a dead-zone in the joint's relative coordinate (angular, or linear along the link) with contact stiffness and damping. It then shows as lost motion, rattle under motor vibration, and limit cycles in the control loop.
- **Defaults from CAD clearances.**
  - Pins: 3.0 pin in 3.2 knuckle bore; 3 mm dowels in printed/Macor holes; shoulder screws in bronze bushes.
  - MF84ZZ bearing radial play; heli metal ball links; U-joint cross.
  - Servo gear backlash; clamped horns.
  - Blocker clevis/axle pins.
  - Leg hinge: 0.1 mm radial over the eye width.
  - Plunger lock: 3.0 pin in 3.2 hole at r 22 gives about ±0.26 deg of leg rattle when latched.
  - Keeper ring: M2 in its slot; 2.4 peg in a 3.0 hole.
- Put one `backlash_*` line per joint in `rocket.cfg`, each saying where its number came from. Add Monte Carlo spreads.
- Replace the single `gimbal_backlash_deg` with the per-joint values. Report the effective lumped gimbal backlash in degrees for comparison with the old 0.5.

### 5. Legs and ground

- **Remove** the instant "legs out when falling" and the crush-damper leg model.
- **Each leg is a 1-DOF hinge link coupled to the body.** Torques on it:
  - torsion spring, ~0.06 N m at 119 deg (derive k and preload, mark VERIFY);
  - leg weight under body acceleration;
  - aero drag on the 14 mm × 260 mm tube in the descent airflow;
  - hinge friction.
- **Stop and lock.** Stop-bar contact at 119 deg. The spring plunger rides the ramp from about 91 deg and latches only at 119 deg. An unlatched leg can be folded back by ground load.
- **Plunger failure.** Flag leg failure if the plunger shear exceeds a configurable limit. The printed hole bearing is the weak point: ~620 N at the 54 N design foot load.
- **Foot.** The EPS ball (R 12.5) crushes progressively: area π(2Rs − s²) × crush stress (config, ~0.085 MPa), with no spring-back. Add tube bending stiffness as a series spring (CF E is uncertain, 35-138 GPa; config + VERIFY). Keep friction, slope, bumps and soft patches from the current ground model.
- Other contact points (nozzle exit, aft rim, nose tip) come from the CAD.

### 6. Aero, config, firmware, logging

- **Aero.** Body length/diameter from CAD. Re-run `validation/compare_rocketpy.py` with the real nose profile and update the nose-first CP. Add deployed-leg drag. Leave the tail-first values flagged as guesses.
- **Config.** Clean up `rocket.cfg`: remove or rename the paddle settings to blocker settings, update the README table (legs 0.60 m, no damper stroke, gimbal 8 deg, mass ~0.94 kg).
- **Firmware.**
  - Nominal mass properties come from the same vehicle data (the flight computer's "nominal" config).
  - Re-check `kp/kd` (pitch inertia is now ~0.017 vs the old 0.05 placeholder) and the boost lean and landing logic. Report anything you retune.
- **Logging.** Keep the existing CSV columns and add:
  - all 5 servo command and actual horn angles;
  - outer-ring and cartridge angles;
  - claw angles and blocked share;
  - keeper ring angle;
  - the 4 leg hinge angles and latch states;
  - plunger/foot loads;
  - each joint's backlash state (optional, behind a flag).

### 7. Viewer (`docs/index.html`, `tools/view.py`)

- **Mechanism.**
  - Replace the primitive rocket with the GLB links, assembled as a hierarchy from `vehicle.json`.
  - Animate every joint from the log: servo horns, pushrods and blocker links (placed between their two end points each frame), outer ring, cartridge, claws, keeper ring, leg hinges.
  - Ascent motor ejection uses its own mesh; the flame sits at the real nozzle exit and follows the cartridge.
- **Keep** the trail, smoke, chart, follow/free cameras and random flights.
- **Add:**
  - an "aft close-up" camera;
  - a slow-motion control;
  - a toggle to make the shells transparent so the mechanism is visible.
- **Serving.** `view.py` serves `vehicle/` locally. `--web` copies `vehicle.json` and the meshes into `docs/` and stays within the size budget.

## Acceptance checks (report each with numbers)

1. **Mass and geometry.** Neutral-pose mass/CG/inertia match the table above (≤0.5 %). Deployed foot positions and span match the CAD (≤1 mm).
2. **TVC kinematics.** Forward kinematics vs the `tvc_command_map.h` grid ≤0.1 deg. The hard stop holds at 8.75 deg.
3. **Blocker.** Thrust floor 0.6 reached at full close. L/R mismatch produces side force with the right sign.
4. **Legs.**
   - Deploy time in still air and in 10 / 15 / 20 m/s descent airflow; state whether the spring is enough.
   - Latch engages at 119 deg.
   - An unlatched leg folds under load.
5. **Backlash.** Monte Carlo 2000 with backlash off vs on (and ×2). Report landing %, median drift, failure modes, and any gimbal limit cycling seen in the logs.
6. **Overall.** New model vs the old `main` baseline in Monte Carlo 2000, with the reasons for any change. The RocketPy comparison still passes. Monte Carlo runtime stays within about 3× the old one; pick `physics_dt` or sub-stepping so stiff contacts stay stable.
7. **Viewer.** Screenshots of: pad, boost with the gimbal moving, ejection, claws closing, keeper release + legs swinging out, touchdown.

## Deliverables

- Branch `cad-integration` with small commits.
- `tools/export_cad.py`, the `vehicle/` folder, and the updated sim, firmware, config, viewer and README.
- A short `docs/CAD_INTEGRATION.md`: what is modelled, where every number came from, what is still a guess (marked VERIFY), and the acceptance results.

## Open questions to raise with me rather than guess

- KST X08H Plus supply voltage and refresh rate on the real flight computer.
- When to release the legs.
- Whether retuned control gains are acceptable.
- Any CAD defect you find.
