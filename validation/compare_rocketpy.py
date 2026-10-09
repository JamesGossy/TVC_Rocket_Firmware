"""Fly the same rocket in tvc_sim and RocketPy and compare the results.

RocketPy can't do TVC, so both fly with the gimbal locked and only the ascent motor.

    Test A: straight up, no wind. Checks thrust, mass burn-off, drag, gravity.
    Test B: launched with a small tilt, no fins, no TVC. The rocket is unstable and
            starts tipping over. Checks the rotation and aero moment physics.

Run from the project folder:
    .venv\\Scripts\\python validation\\compare_rocketpy.py
"""
import csv
import os
import subprocess
import sys

import numpy as np
from rocketpy import Environment, Flight, GenericMotor, Rocket

PROJECT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUTPUT = os.path.join(PROJECT, "validation", "output")
SIM = os.path.join(PROJECT, "tvc_sim.exe" if os.name == "nt" else "tvc_sim")

NOSE_LENGTH = 0.15  # not in rocket.cfg yet, only RocketPy needs it


def read_config(path):
    values = {}
    for line in open(path):
        line = line.split("#")[0]
        if "=" in line:
            name, value = line.split("=", 1)
            values[name.strip()] = value.strip()
    return values


def read_eng_header(path):
    # Second token group of the first non-comment line: name dia len delays prop_mass total_mass maker
    for line in open(path):
        if line.strip() and not line.strip().startswith(";"):
            parts = line.split()
            return float(parts[4]), float(parts[5])


def build_rocketpy_rocket(cfg):
    motor_file = os.path.join(PROJECT, cfg["motor_file"])
    propellant_mass, total_mass = read_eng_header(motor_file)
    motor_length = 0.114
    motor_center = float(cfg["motor_position"])

    # Positions inside the motor are measured up from the nozzle
    motor = GenericMotor(
        thrust_source=motor_file,
        burn_time=3.45,
        chamber_radius=0.0145,
        chamber_height=motor_length,
        chamber_position=motor_length / 2,
        propellant_initial_mass=propellant_mass,
        nozzle_radius=0.005,
        dry_mass=total_mass - propellant_mass,
        center_of_dry_mass_position=motor_length / 2,
    )

    # Same as tvc_sim: positions measured up from the bottom of the rocket
    rocket = Rocket(
        radius=float(cfg["body_diameter"]) / 2,
        mass=float(cfg["dry_mass"]),
        inertia=(float(cfg["dry_inertia_pitch"]), float(cfg["dry_inertia_pitch"]), float(cfg["dry_inertia_roll"])),
        power_off_drag=float(cfg["cd_nose_first"]),
        power_on_drag=float(cfg["cd_nose_first"]),
        center_of_mass_without_motor=float(cfg["dry_cg"]),
        coordinate_system_orientation="tail_to_nose",
    )
    rocket.add_motor(motor, position=motor_center - motor_length / 2)
    rocket.add_nose(length=NOSE_LENGTH, kind="ogive", position=float(cfg["body_length"]))
    return rocket


def run_rocketpy(cfg, tilt_deg, max_time, stop_at_apogee=False):
    # A finless rocket tumbles once it stops thrusting, and RocketPy's solver crawls
    # through that, so only fly the part we want to compare
    env = Environment(gravity=9.81, latitude=0, longitude=0, elevation=0)
    rocket = build_rocketpy_rocket(cfg)
    flight = Flight(rocket=rocket, environment=env, rail_length=0.05,
                    inclination=90 - tilt_deg, heading=0, max_time=max_time,
                    terminate_on_apogee=stop_at_apogee, max_time_step=0.01)

    t = np.arange(0, flight.t_final, 0.01)
    altitude = np.array([flight.z(s) for s in t]) - flight.z(0)
    vz = np.array([flight.vz(s) for s in t])
    tilt = 90 - np.array([flight.attitude_angle(s) for s in t])
    return {"t": t, "altitude": altitude, "vz": vz, "tilt": tilt}, rocket


def run_tvc_sim(cfg_path, overrides, name):
    # Write a config that is rocket.cfg plus overrides, then fly it in its own folder
    run_dir = os.path.join(OUTPUT, name)
    os.makedirs(run_dir, exist_ok=True)
    test_cfg = os.path.join(run_dir, "test.cfg")
    with open(test_cfg, "w") as f:
        f.write(open(cfg_path).read() + "\n")
        for key, value in overrides.items():
            f.write(f"{key} = {value}\n")

    subprocess.run([SIM, "test.cfg"], cwd=run_dir, check=True, capture_output=True)

    rows = list(csv.DictReader(open(os.path.join(run_dir, "flight.csv"))))
    launch = float(overrides["launch_time"])
    rows = [r for r in rows if float(r["time"]) >= launch]
    return {
        "t": np.array([float(r["time"]) - launch for r in rows]),
        "altitude": np.array([float(r["altitude"]) for r in rows]),
        "vz": np.array([float(r["vz"]) for r in rows]),
        "tilt": np.array([float(r["tilt_deg"]) for r in rows]),
    }


def compare(name, ours, theirs, stop_time=None):
    # Line both up on the same time steps
    end = min(ours["t"][-1], theirs["t"][-1])
    if stop_time:
        end = min(end, stop_time)
    t = np.arange(0, end, 0.01)
    a = {k: np.interp(t, ours["t"], ours[k]) for k in ("altitude", "vz", "tilt")}
    b = {k: np.interp(t, theirs["t"], theirs[k]) for k in ("altitude", "vz", "tilt")}

    print(f"\n=== {name} ===")
    print(f"{'':28s}{'tvc_sim':>10s}{'RocketPy':>10s}{'diff':>10s}")

    def row(label, x, y, unit):
        print(f"{label:28s}{x:10.2f}{y:10.2f}{x - y:+10.2f} {unit}")

    # Everything below only looks at the time both sims flew
    row("highest point", a["altitude"].max(), b["altitude"].max(), "m")
    row("max upward speed", a["vz"].max(), b["vz"].max(), "m/s")
    print(f"{'worst altitude difference':28s}{np.abs(a['altitude'] - b['altitude']).max():30.2f} m")
    print(f"{'worst vertical speed diff':28s}{np.abs(a['vz'] - b['vz']).max():30.2f} m/s")

    # For a rocket that tips over, when it gets there matters more than the angle at a set time
    for angle in (10, 45, 90):
        if a["tilt"].max() > angle and b["tilt"].max() > angle:
            row(f"time to tip past {angle} deg", t[np.argmax(a["tilt"] > angle)], t[np.argmax(b["tilt"] > angle)], "s")
    return t, a, b


def plot(results):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("\n(install matplotlib to see plots)")
        return
    fig, ax = plt.subplots(len(results), 3, figsize=(14, 4 * len(results)), squeeze=False)
    for row, (name, t, a, b) in enumerate(results):
        for col, (key, label) in enumerate([("altitude", "altitude (m)"), ("vz", "vertical speed (m/s)"),
                                            ("tilt", "tilt (deg)")]):
            ax[row, col].plot(t, a[key], label="tvc_sim")
            ax[row, col].plot(t, b[key], "--", label="RocketPy")
            ax[row, col].set_ylabel(label)
            ax[row, col].set_xlabel("time since ignition (s)")
            ax[row, col].grid(True, alpha=0.3)
            ax[row, col].legend(fontsize=8)
        ax[row, 0].set_title(name)
    plt.tight_layout()
    path = os.path.join(OUTPUT, "comparison.png")
    plt.savefig(path, dpi=110)
    print(f"\nPlot saved to {path}")


def main():
    cfg_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(PROJECT, "rocket.cfg")
    cfg = read_config(cfg_path)
    os.makedirs(OUTPUT, exist_ok=True)

    # Make tvc_sim fly the way RocketPy does: no TVC, one motor, no wind, no igniter delay,
    # same drag going up and down, and none of the hardware messiness RocketPy doesn't model
    same_as_rocketpy = {
        "tvc_enabled": 0, "landing_motor": 0, "wind_speed": 0, "gust_strength": 0,
        "ascent_ignition_delay": 0, "launch_time": 2, "cd_tail_first": cfg["cd_nose_first"],
        "cd_crossflow": 0, "thrust_roughness": 0, "roll_torque": 0,
        "gimbal_misalignment_y_deg": 0, "gimbal_misalignment_z_deg": 0, "cg_offset": 0,
        "motor_file": os.path.join(PROJECT, cfg["motor_file"]),
    }

    results = []

    # Test A: straight up, compared up to apogee
    theirs, rocket = run_rocketpy(cfg, tilt_deg=0, max_time=60, stop_at_apogee=True)
    ours = run_tvc_sim(cfg_path, {**same_as_rocketpy, "launch_tilt_deg": 0}, "straight_up")
    results.append(("A: straight up", *compare("Test A: straight up, no wind", ours, theirs,
                                               stop_time=theirs["t"][-1])))

    # Test B: tilted launch. Use RocketPy's own lift numbers so both sims have the same aero.
    rocketpy_cna = rocket.total_lift_coeff_der(0)
    rocketpy_cp = rocket.cp_position(0)
    print(f"\nRocketPy aero for this rocket: CN_alpha {rocketpy_cna:.2f} /rad, CP {rocketpy_cp:.3f} m from the bottom")
    print(f"rocket.cfg has:                CN_alpha {float(cfg['cn_alpha']):.2f} /rad, CP {float(cfg['cp_nose_first']):.3f} m")

    tilt = 3
    theirs, _ = run_rocketpy(cfg, tilt_deg=tilt, max_time=4.0)
    matched = {**same_as_rocketpy, "launch_tilt_deg": tilt, "cn_alpha": rocketpy_cna,
               "cp_nose_first": rocketpy_cp, "cp_tail_first": rocketpy_cp}
    ours = run_tvc_sim(cfg_path, matched, "tilted")
    results.append(("B: 3 deg tilt, no TVC", *compare(f"Test B: {tilt} deg tilt, no fins, no TVC", ours, theirs,
                                                     stop_time=4.0)))

    plot(results)


if __name__ == "__main__":
    main()
