#!/usr/bin/env python3
"""Linear check of the boost steering loop (one axis) against the aerodynamic instability.

Plant:      theta'' = (T L / I) * delta + Ka * (theta - flow angle)     Ka = aero overturning stiffness / I  (rad/s^2 per rad)
Controller: delta = I / (T L) * (kp * e + kd * e')                      (flight_computer.cpp, steer())
Actuator:   pure delay tau_d (100 Hz hold + control_delay + half the 50 Hz servo refresh, about 20 ms) and a first-order servo lag tau_l (20 ms)
Prints the slowest closed-loop decay rate (negative = the loop is unstable) for the gains you give it.
Ka comes from tools/aero_strip.py: dM/dalpha = 0.00364 m^3/rad at CG 0.226, times q = 0.5 rho v^2, divided by I = 0.0203 kg m^2:
   10 m/s: 11,  15 m/s: 25,  20 m/s: 44 rad/s^2 per rad.
Usage: python3 tools/loop_margin.py [kp kd]
"""
import sys
from math import factorial
import numpy as np


def decay_rate(kp, kd, Ka, gain=1.0, tau_d=0.02, tau_l=0.02, n=3):
    """Slowest closed-loop pole, as -Re(pole) (so > 0 is stable). gain = actual loop gain / design loop gain (thrust, lever arm, inertia)."""
    c = [factorial(2 * n - k) * factorial(n) / (factorial(2 * n) * factorial(k) * factorial(n - k)) for k in range(n + 1)]
    num = np.array([c[k] * (-tau_d) ** k for k in range(n + 1)])[::-1]   # Pade approximation of exp(-s tau_d)
    den = np.array([c[k] * tau_d ** k for k in range(n + 1)])[::-1]
    numerator = np.polymul([gain * kd, gain * kp], num)
    denominator = np.polymul(np.polymul(den, [tau_l, 1.0]), [1, 0, -Ka])
    return -max(np.roots(np.polyadd(denominator, numerator)).real)


if __name__ == "__main__":
    kp = float(sys.argv[1]) if len(sys.argv) > 1 else 100.0
    kd = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0
    print(f"kp {kp:g}, kd {kd:g}: decay rate (1/s), > 0 is stable")
    print("  Ka (rad/s^2 per rad)  loop gain 0.6x   1.0x   1.6x    (20 ms delay | 30 ms delay)")
    for Ka in (0, 11, 25, 44):
        cells = []
        for g in (0.6, 1.0, 1.6):
            cells.append(f"{decay_rate(kp, kd, Ka, g, tau_d=0.02):6.2f}|{decay_rate(kp, kd, Ka, g, tau_d=0.03):6.2f}")
        print(f"  {Ka:6d}              " + "  ".join(cells))
