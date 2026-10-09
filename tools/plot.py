"""Plot the output of tvc_sim.

    python tools/plot.py                   plots flight.csv
    python tools/plot.py montecarlo.csv    plots a Monte Carlo run

Needs matplotlib.
"""
import csv
import sys

import matplotlib.pyplot as plt


def read_csv(path):
    # Returns {column name: list of values}. Numbers become floats, text stays text.
    with open(path) as f:
        rows = list(csv.DictReader(f))
    columns = {}
    for name in rows[0]:
        values = [row[name] for row in rows]
        try:
            columns[name] = [float(v) for v in values]
        except ValueError:
            columns[name] = values
    return columns


def plot_flight(path):
    d = read_csv(path)
    t = d["time"]

    # Find when the landing motor was fired so we can mark it on every plot
    fire_time = next((t[i] for i, s in enumerate(d["state"]) if s == "LandingBurn"), None)

    fig, ax = plt.subplots(3, 2, figsize=(13, 10))
    fig.suptitle(path)

    ax[0, 0].plot(t, d["altitude"], label="true")
    ax[0, 0].plot(t, d["est_altitude"], "--", label="flight computer")
    ax[0, 0].set_ylabel("altitude (m)")

    ax[0, 1].plot(t, d["vz"], label="true")
    ax[0, 1].plot(t, d["est_vertical_speed"], "--", label="flight computer")
    ax[0, 1].set_ylabel("vertical speed (m/s)")

    ax[1, 0].plot(t, d["tilt_deg"], label="true")
    ax[1, 0].plot(t, d["est_tilt_deg"], "--", label="flight computer")
    ax[1, 0].set_ylabel("tilt from vertical (deg)")

    ax[1, 1].plot(t, d["gimbal_y_deg"], label="gimbal y")
    ax[1, 1].plot(t, d["gimbal_z_deg"], label="gimbal z")
    ax[1, 1].set_ylabel("gimbal angle (deg)")

    ax[2, 0].plot(t, d["thrust"])
    ax[2, 0].set_ylabel("thrust (N)")
    ax[2, 0].set_xlabel("time (s)")

    # Ground track, seen from above
    ax[2, 1].plot(d["x"], d["y"])
    ax[2, 1].plot(0, 0, "k^", label="pad")
    ax[2, 1].plot(d["x"][-1], d["y"][-1], "rx", label="touchdown")
    ax[2, 1].set_xlabel("x (m)")
    ax[2, 1].set_ylabel("y (m)")
    ax[2, 1].axis("equal")

    for a in ax.flat:
        a.grid(True, alpha=0.3)
        if fire_time is not None and a is not ax[2, 1]:
            a.axvline(fire_time, color="gray", linestyle=":", label="landing fire")
        if a.get_legend_handles_labels()[0]:
            a.legend(fontsize=8)

    plt.tight_layout()
    plt.show()


def plot_monte_carlo(path):
    d = read_csv(path)
    good = [g == 1 for g in d["good"]]
    colors = ["tab:green" if g else "tab:red" for g in good]

    fig, ax = plt.subplots(2, 2, figsize=(12, 9))
    fig.suptitle(f"{path}: {sum(good)} of {len(good)} good landings")

    ax[0, 0].hist(d["touchdown_speed"], bins=40)
    ax[0, 0].set_xlabel("touchdown speed (m/s)")
    ax[0, 0].set_ylabel("runs")

    ax[0, 1].scatter(d["landing_x"], d["landing_y"], c=colors, s=10)
    ax[0, 1].plot(0, 0, "k^")
    ax[0, 1].set_xlabel("x (m)")
    ax[0, 1].set_ylabel("y (m)")
    ax[0, 1].set_title("where it landed")
    ax[0, 1].axis("equal")

    ax[1, 0].scatter(d["landing_ignition_delay"], d["touchdown_speed"], c=colors, s=10)
    ax[1, 0].set_xlabel("landing igniter delay (s)")
    ax[1, 0].set_ylabel("touchdown speed (m/s)")

    ax[1, 1].scatter(d["landing_thrust_scale"], d["touchdown_speed"], c=colors, s=10)
    ax[1, 1].set_xlabel("landing motor thrust scale")
    ax[1, 1].set_ylabel("touchdown speed (m/s)")

    for a in ax.flat:
        a.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else "flight.csv"
    if "montecarlo" in path:
        plot_monte_carlo(path)
    else:
        plot_flight(path)
