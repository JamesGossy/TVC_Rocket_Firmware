#include "selftest.h"
#include <cmath>
#include <cstdio>
#include <random>
#include "mechanism.h"
#include "tvc_command_map.h"

static int failures = 0;
static void check(bool ok, const char* what) {
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) failures++;
}

// Run the mechanism for a while with fixed commands
static double run(Mechanism& m, double t0, double seconds, const ServoTargets& tg, const MechanismMotion& mo) {
    const double dt = 0.001;
    double t = t0;
    for (int i = 0; i < (int)std::round(seconds / dt); i++, t += dt) m.update(t, dt, tg, mo);
    return t;
}

// ---------------------------------------------------------------------------------------------------------------
// Check 1: mass properties and foot positions against the CAD numbers
// ---------------------------------------------------------------------------------------------------------------
static void testMass(const Config& cfg, const VehicleData& v) {
    std::printf("Mass properties and geometry (CAD, neutral pose, both motors loaded, legs stowed)\n");
    std::mt19937 rng(1);
    Mechanism m(v, cfg, cfg, rng);
    m.setMotors(true, 0, true, 0);
    m.refreshProps();
    const BodyProps& p = m.props();
    double I_pitch = (p.inertia.m[1][1] + p.inertia.m[2][2]) / 2;
    std::printf("  mass %.5f kg, CG x %.5f m (y %.1f z %.1f mm), I_roll %.6f, I_pitch %.6f kg m^2\n", p.mass, p.cg.x,
                p.cg.y * 1e3, p.cg.z * 1e3, p.inertia.m[0][0], I_pitch);
    auto within = [](double a, double b, double tol) { return std::fabs(a - b) <= tol * std::fabs(b); };
    check(within(p.mass, 0.94029, 0.005), "mass within 0.5% of the CAD table (0.94029 kg)");
    check(within(p.cg.x, 0.22662, 0.005), "CG within 0.5% of the CAD table (0.22662 m)");
    check(within(p.inertia.m[0][0], 0.00097825, 0.005), "roll inertia within 0.5% of the CAD table (0.00097825)");
    check(within(I_pitch, 0.020178, 0.005), "pitch inertia within 0.5% of the CAD table (0.020178)");

    // Feet: stowed and deployed
    double worst_stowed = 0;
    for (int i = 0; i < 4; i++) {
        Vec3 f = m.footCentre(i);
        worst_stowed = std::max(worst_stowed, std::fabs(std::hypot(f.y, f.z) - 0.0378 * std::sqrt(2.0)));
        worst_stowed = std::max(worst_stowed, std::fabs(f.x - 0.4135));
    }
    std::printf("  stowed feet: worst error against (0.4135, +-0.0378, +-0.0378) = %.2f mm\n", worst_stowed * 1e3);
    check(worst_stowed < 1e-3, "stowed foot positions within 1 mm");
    for (int i = 0; i < 4; i++) m.leg[i].angle = v.legs.latch_deg * DEG;
    double span_a = length(m.footCentre(0) - m.footCentre(2)), span_b = length(m.footCentre(1) - m.footCentre(3));
    Vec3 f0 = m.footCentre(0);
    std::printf("  deployed feet: radius %.2f mm from the axis, x %.2f mm (below the aft end of the ascent motor), span %.1f / %.1f mm\n",
                std::hypot(f0.y, f0.z) * 1e3, f0.x * 1e3, span_a * 1e3, span_b * 1e3);
    check(std::fabs(span_a - 0.5965) < 1e-3 && std::fabs(span_b - 0.5965) < 1e-3, "deployed foot span within 1 mm of 0.5965 m");
}

// ---------------------------------------------------------------------------------------------------------------
// Check 2: the gimbal linkage
// ---------------------------------------------------------------------------------------------------------------
// Horn angle that puts a rod's stud exactly one rod length from where the cartridge stud is
static double hornFor(const TvcAxisData& ax, Vec3 target) {
    Vec3 arm = ax.horn_stud - ax.shaft;
    double best = 0, best_error = 1e9;
    for (double q = -90; q <= 90; q += 0.05) {
        Vec3 h = ax.shaft + rotationMatrix(ax.shaft_axis, q * DEG) * arm;
        double e = std::fabs(length(h - target) - ax.rod_length);
        if (e < best_error) { best_error = e; best = q; }
    }
    return best;
}

static void testTvc(const Config& cfg, const VehicleData& v) {
    std::printf("TVC linkage against flight_software/tvc_command_map.h\n");
    std::mt19937 rng(1);
    Mechanism m(v, cfg, cfg, rng);
    double worst = 0, worst_yaw = 0, worst_pitch = 0;
    int n = 0;
    for (double yaw = -8; yaw <= 8.001; yaw += 1)
        for (double pitch = -8; pitch <= 8.001; pitch += 1) {
            if (std::hypot(yaw, pitch) > 8.0) continue;
            float ys, ps;
            tvc_command((float)yaw, (float)pitch, &ys, &ps);
            double got_yaw, got_pitch;
            m.tvcForward(ys * DEG, ps * DEG, got_yaw, got_pitch);
            m.tvcForward(ys * DEG, ps * DEG, got_yaw, got_pitch);
            double e = std::hypot(got_yaw / DEG - yaw, got_pitch / DEG - pitch);
            if (e > worst) { worst = e; worst_yaw = yaw; worst_pitch = pitch; }
            n++;
        }
    std::printf("  %d points inside the 8 deg circle: worst forward-kinematics error %.4f deg (at yaw %.0f pitch %.0f)\n", n, worst,
                worst_yaw, worst_pitch);
    check(worst < 0.1, "forward kinematics within 0.1 deg of the command map");

    // Push the horns past the 8.75 deg stop (a command the flight computer never sends) and see where the cartridge ends up
    for (double over : {9.5, 12.0}) {
        Mechanism t(v, cfg, cfg, rng);
        Mat3 R = rotationMatrix({0, 0, 1}, over * DEG);
        double q[2];
        const TvcAxisData* ax[2] = {&v.tvc.yaw, &v.tvc.pitch};
        for (int k = 0; k < 2; k++) q[k] = hornFor(*ax[k], v.tvc.pivot + R * (ax[k]->cart_stud - v.tvc.pivot));
        t.servo_tvc[0].start(q[0] * DEG, -80 * DEG, 80 * DEG);
        t.servo_tvc[1].start(q[1] * DEG, -80 * DEG, 80 * DEG);
        ServoTargets tg;
        tg.tvc_yaw = q[0];
        tg.tvc_pitch = q[1];
        MechanismMotion mo;
        run(t, 0, 1.0, tg, mo);
        double radius = std::hypot(t.gimbal[0], t.gimbal[1]) / DEG;
        std::printf("  linkage driven to %.1f deg of yaw: cartridge settles at %.3f deg (stop %.2f deg), on the stop: %s\n", over,
                    radius, v.tvc.hard_stop_deg, t.tvc_at_stop ? "yes" : "no");
        check(radius < v.tvc.hard_stop_deg + 0.15 && t.tvc_at_stop, "hard stop holds at the stop angle");
    }
}

// ---------------------------------------------------------------------------------------------------------------
// Check 3: the blocker claws
// ---------------------------------------------------------------------------------------------------------------
static void testBlocker(const Config& cfg, const VehicleData& v) {
    std::printf("Blocker claws\n");
    std::mt19937 rng(1);
    Config c = cfg;
    c.backlash_scale = 0;
    c.blocker_servo_offset_r_deg = c.blocker_servo_offset_l_deg = 0;
    c.blocker_erosion = 0;
    c.blocker_effectiveness = 1;
    Mechanism m(v, c, c, rng);
    m.setMotors(false, 0, true, 0);
    ServoTargets tg;
    MechanismMotion mo;
    double t = 0;
    std::printf("  horn deg   claw R deg   claw L deg   blocked share   thrust left\n");
    for (double horn : {-80.0, -50.0, -40.0, -30.0, -20.0, -10.0, -0.5}) {
        tg.blocker_r = tg.blocker_l = horn;
        t = run(m, t, 0.6, tg, mo);
        double left = 1 - (1 - c.throttle_min) * m.blockedShare();
        std::printf("  %8.1f   %10.2f   %10.2f   %13.3f   %11.3f\n", horn, m.claw[0] / DEG, m.claw[1] / DEG, m.blockedShare(), left);
    }
    double floor = 1 - (1 - c.throttle_min) * m.blockedShare();
    check(std::fabs(floor - c.throttle_min) < 0.005, "full close leaves exactly throttle_min of the thrust");

    // One claw closed, the other half way: a side force and a twist, in the right direction
    Mechanism u(v, c, c, rng);
    u.setMotors(false, 0, true, 0);
    tg.blocker_r = -0.5;
    tg.blocker_l = -25;
    run(u, 0, 0.8, tg, mo);
    Vec3 push;
    Vec3 twist = u.blockerForceTorque(10.0, push);  // 10 N of thrust lost at full close
    std::printf("  R closed, L at %.0f deg of horn: claws %.1f / %.1f deg, side force y %.3f z %.3f N, twist about z %.4f N m\n", tg.blocker_l,
                u.claw[0] / DEG, u.claw[1] / DEG, push.y, push.z, twist.z);
    // The R claw is at +y. It covers more of the exhaust, so its missing thrust (at +y) twists the nose toward +y... and
    // the blocked exhaust pushes the rocket away from the claw (toward -y)
    check(push.y < 0 && twist.z > 0, "R more closed than L: side force toward -y, twist about +z");
    Mechanism w(v, c, c, rng);
    w.setMotors(false, 0, true, 0);
    tg.blocker_r = -25;
    tg.blocker_l = -0.5;
    run(w, 0, 0.8, tg, mo);
    twist = w.blockerForceTorque(10.0, push);
    check(push.y > 0 && twist.z < 0, "L more closed than R: the opposite way");
}

// ---------------------------------------------------------------------------------------------------------------
// Check 4: the legs
// ---------------------------------------------------------------------------------------------------------------
struct Deploy {
    double release_at = -1, last_latched = -1, worst_angle = 0;
    int latched = 0;
};

static Deploy deployLegs(const Config& cfg, const VehicleData& v, double airspeed, bool weight, bool nose_first, unsigned seed) {
    std::mt19937 rng(seed);
    Mechanism m(v, cfg, cfg, rng);
    m.setMotors(false, 0, false, 0);
    ServoTargets tg;
    tg.keeper = 62;  // the flight computer's release command
    MechanismMotion mo;
    mo.air = Vec3{nose_first ? airspeed : -airspeed, 0, 0};
    mo.air_density = 1.2;
    if (weight) mo.g_minus_a = Vec3{-GRAVITY, 0, 0};
    const double dt = 0.001;
    Deploy d;
    double t = 0;
    for (int i = 0; i < 3000; i++, t += dt) {
        m.update(t, dt, tg, mo);
        if (d.release_at < 0)
            for (int k = 0; k < 4; k++)
                if (m.leg[k].released) d.release_at = t;
    }
    double worst = 0;
    for (int k = 0; k < 4; k++) {
        if (m.leg[k].latched) d.latched++;
        d.last_latched = std::max(d.last_latched, m.leg[k].deploy_time);
        worst = std::max(worst, m.leg[k].angle / DEG);
    }
    for (int k = 0; k < 4; k++) d.worst_angle = (k == 0) ? m.leg[k].angle / DEG : std::min(d.worst_angle, m.leg[k].angle / DEG);
    (void)worst;
    return d;
}

static void testLegs(const Config& cfg, const VehicleData& v) {
    std::printf("Landing legs (command sent at t = 0, rocket in free fall unless noted)\n");
    std::printf("  airflow along the rocket, m/s | legs latched | first released | last latched | least-open leg\n");
    int still_ok = 0;
    for (bool nose_first : {false, true})
        for (double speed : {0.0, 10.0, 15.0, 20.0}) {
            if (speed == 0 && nose_first) continue;
            Deploy d = deployLegs(cfg, v, speed, false, nose_first, 7);
            std::printf("  %5.1f %-10s | %d of 4       | %5.3f s       | %5.3f s      | %6.1f deg\n", speed,
                        speed == 0 ? "" : (nose_first ? "nose first" : "tail first"), d.latched, d.release_at, d.last_latched, d.worst_angle);
            if (speed == 0 && d.latched == 4) still_ok = 1;
        }
    Deploy g = deployLegs(cfg, v, 0, true, false, 7);
    std::printf("  still air, with the rocket's weight on them (on the pad): %d of 4 latched, last at %.3f s\n", g.latched, g.last_latched);
    check(still_ok == 1, "all four legs swing out and latch in still air");
    check(g.latched == 4, "all four also latch with 1 g acting on them");

    // The latched leg under the design load, then the same load on a leg whose plunger has sheared
    std::printf("  latched leg under 54 N at each foot (design load):\n");
    {
        std::mt19937 rng(3);
        Mechanism m(v, cfg, cfg, rng);
        m.setMotors(false, 0, false, 0);
        for (int k = 0; k < 4; k++) {
            m.leg[k].released = true;
            m.leg[k].angle = v.legs.latch_deg * DEG;
            m.leg[k].latched = true;
        }
        MechanismMotion mo;
        ServoTargets tg;
        for (int k = 0; k < 4; k++) {
            mo.foot_force[k] = Vec3{54, 0, 0};
            mo.foot_arm[k] = m.footCentre(k);
        }
        run(m, 0, 0.3, tg, mo);
        std::printf("    leg angle %.2f deg, plunger load %.0f N (shear limit %.0f N), failed: %s\n", m.leg[0].angle / DEG,
                    m.leg[0].plunger_load, cfg.plunger_shear_limit, m.leg[0].failed ? "yes" : "no");
        check(!m.leg[0].failed && m.leg[0].angle > (v.legs.latch_deg - 2) * DEG, "latched leg holds the design load");
    }
    {
        std::mt19937 rng(3);
        Mechanism m(v, cfg, cfg, rng);
        m.setMotors(false, 0, false, 0);
        for (int k = 0; k < 4; k++) {
            m.leg[k].released = true;
            m.leg[k].angle = v.legs.latch_deg * DEG;
            m.leg[k].failed = true;
        }
        MechanismMotion mo;
        ServoTargets tg;
        for (int k = 0; k < 4; k++) {
            mo.foot_force[k] = Vec3{54, 0, 0};
            mo.foot_arm[k] = m.footCentre(k);
        }
        run(m, 0, 0.3, tg, mo);
        std::printf("  unlatched leg under the same load: folds to %.1f deg in 0.3 s\n", m.leg[0].angle / DEG);
        check(m.leg[0].angle < 90 * DEG, "an unlatched leg folds under load");
    }
}

int runSelfTest(const std::string& name, const Config& cfg, const VehicleData& vehicle, const ThrustCurve& curve) {
    (void)curve;
    failures = 0;
    bool all = name == "hardware" || name.empty();
    if (all || name == "mass") testMass(cfg, vehicle);
    if (all || name == "tvc") testTvc(cfg, vehicle);
    if (all || name == "blocker") testBlocker(cfg, vehicle);
    if (all || name == "legs") testLegs(cfg, vehicle);
    std::printf(failures ? "\n%d check(s) FAILED\n" : "\nall checks passed\n", failures);
    return failures ? 1 : 0;
}
