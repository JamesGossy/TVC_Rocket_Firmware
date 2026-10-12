"""Strip / crossflow aerodynamics of the finless TVC rocket (slender-body potential flow + Allen-Perkins crossflow).
Stand-alone check of the sim's aero_model = 1 (CP vs angle of attack, stability margin vs CG).
Frame: x along the axis from the aft end of the ascent motor (m), y lateral in the plane of the flow, z out of plane.
Everything is per unit (1/2 rho V^2) so forces are areas (m^2) and moments are volumes (m^3).
"""
import numpy as np

D = 0.0795; R = D / 2
X_NOSE_BASE = 0.520; X_TIP = 0.639; L_NOSE = X_TIP - X_NOSE_BASE
X_BODY_START = 0.120        # OD 79.5 mm starts here; below it only the 29 mm motor + TVC hardware
R_LOW = 0.0225              # effective radius of the motor / TVC / blocker hardware below the body
CDC = 1.2                   # crossflow drag of a cylinder
ETA = 0.7                   # finite-length correction, L/D ~ 8 (Hoerner)
CD_LEG = 1.1
LEG_D = 0.014
LEG_R = 0.049               # hinge / stowed tube axis radius from the rocket axis
LEG_X0, LEG_X1 = 0.136, 0.396   # stowed tube extent
HINGE_X = 0.126
HINGE_DEPLOY = np.radians(119)
S_B = np.pi * R ** 2


def ogive_r(xt):
    rho = (R ** 2 + L_NOSE ** 2) / (2 * R)
    return np.sqrt(np.maximum(rho ** 2 - (L_NOSE - xt) ** 2, 0)) + R - rho


def body_profile(n=640):
    x = (np.arange(n) + 0.5) * X_TIP / n
    r = np.full(n, R)
    r[x < X_BODY_START] = R_LOW
    m = x > X_NOSE_BASE
    r[m] = ogive_r(X_TIP - x[m])
    return x, X_TIP / n, r


def forces(theta, cg, V=1.0, omega=0.0, legs="stowed", azimuth_deg=0.0, kT=0.0, kS=0.0, leg_eta=0.8):
    """Force (3-vector, per 1/2 rho) and moment about the CG for velocity V at angle theta from +x.
    theta=0: nose-first aligned, theta=180: tail-first aligned. kT: share of the (adverse) potential force on the
    nose kept tail-first; kS: share of the stabilising force kept at the aft step."""
    az = np.radians(azimuth_deg)
    vx = V * np.cos(theta)
    vlat = V * np.sin(theta)
    vy, vz = vlat * np.cos(az), vlat * np.sin(az)
    x, dx, r = body_profile()
    n = len(x)
    el = np.array([0, np.cos(az), np.sin(az)])
    ex = np.array([1.0, 0, 0])
    ew = np.cross(ex, el) * omega
    F = np.zeros(3); M = np.zeros(3)
    pos = np.stack([x - cg, np.zeros(n), np.zeros(n)], 1)
    v0 = np.array([vx, vy, vz])
    vloc = v0 + np.cross(ew, pos)
    vn = vloc.copy(); vn[:, 0] = 0
    vnm = np.linalg.norm(vn, axis=1)
    width = 2 * r
    f_visc = -(ETA * CDC) * (width * dx)[:, None] * vnm[:, None] * vn
    S = np.pi * r ** 2
    dS = np.gradient(S, x) * dx
    ux = vloc[:, 0]
    f_pot = 2 * ux[:, None] * vn * dS[:, None]
    if vx >= 0:
        mask = (x > X_NOSE_BASE - 0.001)   # nose-first: only the smooth nose keeps the potential force
        f_pot = f_pot * mask[:, None]
    else:
        nose = (x > X_NOSE_BASE - 0.001)
        step = (np.abs(x - X_BODY_START) < 0.01)
        f_pot = f_pot * (kT * nose + kS * step)[:, None]
    f_body = f_visc + f_pot
    F += f_body.sum(0); M += np.cross(pos, f_body).sum(0)
    pieces = 8
    if legs == "stowed":
        for k in range(4):
            phi = np.radians(45 + 90 * k)
            ypos, zpos = LEG_R * np.cos(phi), LEG_R * np.sin(phi)
            xs = LEG_X0 + (LEG_X1 - LEG_X0) * (np.arange(pieces) + 0.5) / pieces
            ds = (LEG_X1 - LEG_X0) / pieces
            for xp in xs:
                p = np.array([xp - cg, ypos, zpos])
                vl = v0 + np.cross(ew, p)
                vnl = vl.copy(); vnl[0] = 0
                f = -(leg_eta * ETA * CD_LEG) * LEG_D * ds * np.linalg.norm(vnl) * vnl
                F += f; M += np.cross(p, f)
    elif legs == "deployed":
        for k in range(4):
            phi = np.radians(45 + 90 * k)
            e_rad = np.array([0, np.cos(phi), np.sin(phi)])
            e = np.cos(HINGE_DEPLOY) * ex + np.sin(HINGE_DEPLOY) * e_rad
            hinge = np.array([HINGE_X, 0, 0]) + LEG_R * e_rad
            s0, s1 = 0.010, 0.272
            ds = (s1 - s0) / pieces
            for j in range(pieces):
                s = s0 + ds * (j + 0.5)
                p = hinge + e * s - np.array([cg, 0, 0])
                vl = v0 + np.cross(ew, p)
                vnl = vl - e * np.dot(vl, e)
                f = -(CD_LEG * leg_eta) * LEG_D * ds * np.linalg.norm(vnl) * vnl
                F += f; M += np.cross(p, f)
            p = hinge + e * 0.2875 - np.array([cg, 0, 0])
            vl = v0 + np.cross(ew, p)
            f = -0.5 * np.pi * 0.0125 ** 2 * np.linalg.norm(vl) * vl
            F += f; M += np.cross(p, f)
    return F, M


def cp_and_margin(cg, legs="stowed", azimuth_deg=0.0, direction="nose", theta_deg=2.0, **kw):
    """Centre of pressure x (m from the aft end), lateral force and moment at small angle of attack."""
    th = np.radians(theta_deg) if direction == "nose" else np.radians(180 - theta_deg)
    F, M = forces(th, cg, legs=legs, azimuth_deg=azimuth_deg, **kw)
    az = np.radians(azimuth_deg)
    el = np.array([0, np.cos(az), np.sin(az)])
    Fl = F @ el
    ew = np.cross(np.array([1.0, 0, 0]), el)
    Mw = M @ ew
    xcp = cg + Mw / Fl if abs(Fl) > 1e-12 else np.nan
    return xcp, Fl, Mw


def moment_curve(cg, legs="stowed", azimuth_deg=0.0, kT=0.0, kS=0.0, ths=None):
    """(lateral force, moment) vs theta (deg from +x). Positive moment slope = unstable equilibrium at that end."""
    if ths is None:
        ths = np.arange(0.5, 180, 1.0)
    az = np.radians(azimuth_deg)
    el = np.array([0, np.cos(az), np.sin(az)])
    ew = np.cross(np.array([1.0, 0, 0]), el)
    out = []
    for t in ths:
        F, M = forces(np.radians(t), cg, legs=legs, azimuth_deg=azimuth_deg, kT=kT, kS=kS)
        out.append((F @ el, M @ ew))
    return ths, np.array(out)


def margin_cal(cg, legs="stowed", direction="nose", theta_deg=3.0, **kw):
    """Static margin in calibres at a small angle of attack. Positive = stable, negative = unstable.
    Nose-first the centre of pressure must be aft of the CG; tail-first it must be forward of the CG (it trails the motion)."""
    x, _, _ = cp_and_margin(cg, legs=legs, direction=direction, theta_deg=theta_deg, **kw)
    return (cg - x) / D if direction == "nose" else (x - cg) / D


if __name__ == "__main__":
    for legs in ("stowed", "deployed"):
        for d in ("nose", "tail"):
            print(f"{legs:9s} {d}-first  margin at CG 0.222 m: {margin_cal(0.222, legs, d):+.2f} cal")
