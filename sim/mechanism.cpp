#include "mechanism.h"
#include <algorithm>
#include <cmath>

// ---------------------------------------------------------------------------------------------------------------
// Servo
// ---------------------------------------------------------------------------------------------------------------
void Servo::update(double target, double t, double dt, const Config& c, double backlash) {
    // Only takes a new position every 1/servo_update_rate seconds, and ignores changes smaller than its dead band
    if (t >= next_update) {
        next_update = t + 1.0 / c.servo_update_rate;
        target = std::clamp(target + offset, lo, hi);
        if (std::fabs(target - goal) > c.servo_deadband_deg * DEG) goal = target;
    }
    // Top speed falls off as the load torque rises, and it stalls at the stall torque
    double speed = c.servo_speed_deg * DEG * std::max(0.0, 1 - std::fabs(load) / c.servo_stall_torque);
    stalled = speed <= 0;
    arm += std::clamp((goal - arm) / std::max(c.servo_lag, dt), -speed, speed) * dt;
    angle = gear.apply(arm, backlash);
}

// ---------------------------------------------------------------------------------------------------------------
// Set-up
// ---------------------------------------------------------------------------------------------------------------
Mechanism::Mechanism(const VehicleData& v, const Config& truth_cfg, const Config& nominal, std::mt19937& rng)
    : vehicle(v), truth(truth_cfg) {
    pose_.assign(v.links.size(), Pose{});
    q_.assign(v.links.size(), 0.0);

    // Lost motion in every joint, from the CAD clearances (rocket.cfg says where each number came from)
    double s = truth.backlash_scale;
    bl_servo = truth.backlash_servo_deg * DEG * s;
    bl_rod = (truth.backlash_tvc_horn_end_mm + truth.backlash_tvc_ujoint_mm + truth.backlash_tvc_drive_end_mm) * 1e-3 * s;
    bl_axle[0] = truth.backlash_tvc_yaw_axle_deg * DEG * s;
    bl_axle[1] = truth.backlash_tvc_pitch_axle_deg * DEG * s;
    bl_blocker = (truth.backlash_blocker_horn_pin_mm + truth.backlash_blocker_claw_pin_mm + truth.backlash_claw_pivot_mm) * 1e-3 * s;
    bl_keeper = truth.backlash_keeper_slot_mm * 1e-3 * s;
    bl_latch = truth.backlash_leg_latch_mm * 1e-3 * s;

    // Servos: the TVC horns start at 0, the blocker claws open (the claws are locked out while the ascent motor is
    // in the way), the keeper at 0
    for (int k = 0; k < 2; k++) {
        const TvcAxisData& a = k == 0 ? v.tvc.yaw : v.tvc.pitch;
        (void)a;
        servo_tvc[k].start(0, -60 * DEG, 60 * DEG);
    }
    for (int side = 0; side < 2; side++) {
        const BlockerSide& b = v.blocker.side[side];
        servo_blocker[side].start(-80 * DEG, -82 * DEG, b.servo_neutral_deg * DEG);
        servo_blocker[side].offset = (side == 0 ? truth.blocker_servo_offset_r_deg : truth.blocker_servo_offset_l_deg) * DEG;
    }
    servo_keeper.start(0, -5 * DEG, 75 * DEG);

    // Random rocket: mass and CG differ from what the flight computer thinks
    mass_scale = nominal.dry_mass > 0 ? truth.dry_mass / nominal.dry_mass : 1.0;
    cg_shift = truth.dry_cg - nominal.dry_cg;

    // Leg keeper: each peg lets go at a slightly different ring angle
    std::normal_distribution<double> gauss(0.0, 1.0);
    spring_free_angle = v.legs.latch_deg * DEG + truth.leg_spring_torque_deployed / truth.leg_spring_k;
    for (int i = 0; i < 4; i++) {
        leg[i].release_at = (v.keeper.release_ring_deg + truth.keeper_release_spread_deg * gauss(rng)) * DEG;
        const LinkData& L = v.links[v.leg[i]];
        leg_cg_rel[i] = L.item.cg;
        foot_rel[i] = v.legs.leg[i].foot_stowed - v.legs.leg[i].hinge_origin;
        Vec3 a = v.legs.leg[i].hinge_axis;
        Vec3 rho = L.item.cg;
        leg_axis_inertia[i] = dot(a, L.item.inertia * a) + L.item.mass * (dot(rho, rho) - dot(rho, a) * dot(rho, a));
        leg[i].inertia = leg_axis_inertia[i];
    }

    // Blocker: how much each claw covers when closed (everything else is a share of that)
    for (int side = 0; side < 2; side++) claw_area_closed[side] = std::max(1e-9, v.blocker.side[side].table_area[0]);

    // Keeper horn geometry in the plane of the ring
    double axial = dot(v.keeper.servo_shaft - v.keeper.ring_origin, v.keeper.ring_axis);
    keeper_c = std::sqrt(std::max(0.0, v.keeper.horn_centre * v.keeper.horn_centre - axial * axial));

    yaw_misalign = truth.gimbal_misalignment_z_deg * DEG;
    pitch_misalign = truth.gimbal_misalignment_y_deg * DEG;
    refreshProps();
}

// ---------------------------------------------------------------------------------------------------------------
// Linkage kinematics
// ---------------------------------------------------------------------------------------------------------------
double Mechanism::interp(const std::vector<double>& xs, const std::vector<double>& ys, double x) const {
    size_t n = xs.size();
    bool rising = xs.back() > xs.front();
    if (rising ? x <= xs.front() : x >= xs.front()) return ys.front();
    if (rising ? x >= xs.back() : x <= xs.back()) return ys.back();
    size_t lo = 0, hi = n - 1;
    while (hi - lo > 1) {
        size_t mid = (lo + hi) / 2;
        if (rising ? xs[mid] <= x : xs[mid] >= x) lo = mid;
        else hi = mid;
    }
    double f = (x - xs[lo]) / (xs[hi] - xs[lo]);
    return ys[lo] + f * (ys[hi] - ys[lo]);
}

// TVC: horn angles -> where the 33.5 mm pushrods let the gimbal sit. The rods keep their length, the cartridge
// turns about the pivot as Rz(yaw) Ry(pitch). Two length constraints, two unknowns: Newton, starting from where
// we were last time.
bool Mechanism::tvcForward(double q_yaw, double q_pitch, double& yaw, double& pitch, double rod_extra) const {
    const TvcAxisData* ax[2] = {&vehicle.tvc.yaw, &vehicle.tvc.pitch};
    double q[2] = {q_yaw, q_pitch};
    Vec3 H[2], dH[2];
    for (int k = 0; k < 2; k++) {
        Vec3 arm = ax[k]->horn_stud - ax[k]->shaft;
        H[k] = ax[k]->shaft + rotationMatrix(ax[k]->shaft_axis, q[k]) * arm;
        dH[k] = cross(ax[k]->shaft_axis, H[k] - ax[k]->shaft);  // how the stud moves per radian of horn
    }
    const Vec3 z{0, 0, 1}, y{0, 1, 0};
    double x[2] = {tvc_warm[0], tvc_warm[1]};
    double J[2][2] = {{1, 0}, {0, 1}};
    bool ok = false;
    for (int it = 0; it < 12; it++) {
        Mat3 Rz = rotationMatrix(z, x[0]), Ry = rotationMatrix(y, x[1]);
        Mat3 R = Rz * Ry;
        double res[2];
        for (int k = 0; k < 2; k++) {
            Vec3 d = ax[k]->cart_stud - vehicle.tvc.pivot;
            Vec3 Rd = R * d;
            Vec3 w = vehicle.tvc.pivot + Rd - H[k];
            double len = length(w);
            Vec3 u = w / len;
            res[k] = len - (ax[k]->rod_length + rod_extra);
            J[k][0] = dot(u, cross(z, Rd));
            J[k][1] = dot(u, Rz * cross(y, Ry * d));
        }
        if (std::fabs(res[0]) < 1e-13 && std::fabs(res[1]) < 1e-13) { ok = true; break; }
        double det = J[0][0] * J[1][1] - J[0][1] * J[1][0];
        x[0] -= (J[1][1] * res[0] - J[0][1] * res[1]) / det;
        x[1] -= (-J[1][0] * res[0] + J[0][0] * res[1]) / det;
    }
    tvc_warm[0] = x[0];
    tvc_warm[1] = x[1];
    yaw = x[0];
    pitch = x[1];

    // Lever ratios for the servo load and the backlash width: d(gimbal)/d(horn) and d(rod length)/d(gimbal)
    double det = J[0][0] * J[1][1] - J[0][1] * J[1][0];
    if (std::fabs(det) > 1e-18) {
        for (int k = 0; k < 2; k++) {
            Vec3 d = ax[k]->cart_stud - vehicle.tvc.pivot;
            Mat3 R = rotationMatrix(z, x[0]) * rotationMatrix(y, x[1]);
            Vec3 w = vehicle.tvc.pivot + R * d - H[k];
            double dr_dq = -dot(w / length(w), dH[k]);
            // dx = -J^-1 e_k dr_dq, we want component k
            double inv_kk = (k == 0 ? J[1][1] : J[0][0]) / det;
            tvc_gain[k] = std::fabs(inv_kk * dr_dq);
            tvc_lever[k] = std::fabs(J[k][k]);
        }
    }
    return ok;
}

// Blocker: servo horn -> link -> claw, a planar four bar. Solved exactly (two circles meet).
double Mechanism::clawFromServo(int side, double q, double link_extra) const {
    const BlockerSide& b = vehicle.blocker.side[side];
    Vec3 u = normalize(b.claw_axis);
    Vec3 e1{1, 0, 0};
    Vec3 e2 = cross(u, e1);
    double rel = q - b.servo_neutral_deg * DEG;
    Vec3 horn = b.shaft + rotationMatrix(b.shaft_axis, rel) * (b.horn_pin - b.shaft);
    Vec3 dh = horn - b.claw_pivot, dt = b.claw_pin - b.claw_pivot;
    double hx = dot(dh, e1), hy = dot(dh, e2), tx = dot(dt, e1), ty = dot(dt, e2);
    double axial = dot(b.horn_pin - b.claw_pin, u);
    double link = std::sqrt(std::max(0.0, b.link_length * b.link_length - axial * axial)) + link_extra;
    double dd = std::hypot(hx, hy), r = std::hypot(tx, ty);
    double K = (dd * dd + r * r - link * link) / 2;
    double A = std::acos(std::clamp(K / (dd * r), -1.0, 1.0));
    double base = std::atan2(hy, hx) - std::atan2(ty, tx);
    auto wrap = [](double a) { return std::remainder(a, 2 * PI); };
    double c1 = wrap(base + A), c2 = wrap(base - A);
    return std::fabs(c1) < std::fabs(c2) ? c1 : c2;
}

// Keeper: the horn's set screw rides in a radial slot of the ring, so the ring turns by atan(r sin a / (c + r cos a))
double Mechanism::ringFromHorn(double alpha) const {
    double r = vehicle.keeper.stud_radius;
    return -std::atan2(r * std::sin(alpha), keeper_c + r * std::cos(alpha));
}

// ---------------------------------------------------------------------------------------------------------------
// Poses and mass properties
// ---------------------------------------------------------------------------------------------------------------
void Mechanism::computePoses() {
    const VehicleData& v = vehicle;
    q_[v.outer_ring] = gimbal[0];
    q_[v.cartridge] = gimbal[1];
    for (int s = 0; s < 2; s++) {
        q_[v.claw[s]] = claw[s];
        q_[v.blocker_horn[s]] = servo_blocker[s].angle - v.blocker.side[s].servo_neutral_deg * DEG;
    }
    for (int k = 0; k < 2; k++) q_[v.tvc_horn[k]] = servo_tvc[k].angle;
    q_[v.keeper_horn] = servo_keeper.angle;
    q_[v.keeper_ring] = ring_angle;
    for (int i = 0; i < 4; i++) q_[v.leg[i]] = leg[i].angle;

    for (size_t i = 0; i < v.links.size(); i++) {
        const LinkData& L = v.links[i];
        if (L.parent < 0) {
            pose_[i] = {Mat3{}, L.origin};
            continue;
        }
        const Pose& P = pose_[L.parent];
        Mat3 Rj = (L.has_axis && q_[i] != 0.0) ? rotationMatrix(L.axis, q_[i]) : Mat3{};
        pose_[i] = {P.R * Rj, P.t + P.R * (L.origin - v.links[L.parent].origin)};
    }

    // The pushrods and blocker links are free: put each between its two ends
    auto between = [&](int link, int on_a, Vec3 a0, int on_b, Vec3 b0) {
        const Pose& pa = pose_[on_a];
        const Pose& pb = pose_[on_b];
        Vec3 a = pa.t + pa.R * (a0 - v.links[on_a].origin);
        Vec3 b = pb.t + pb.R * (b0 - v.links[on_b].origin);
        pose_[link] = {rotationBetweenMatrix(b0 - a0, b - a), a};
    };
    between(v.tvc_rod[0], v.tvc_horn[0], v.tvc.yaw.horn_stud, v.cartridge, v.tvc.yaw.cart_stud);
    between(v.tvc_rod[1], v.tvc_horn[1], v.tvc.pitch.horn_stud, v.cartridge, v.tvc.pitch.cart_stud);
    for (int s = 0; s < 2; s++)
        between(v.blocker_link[s], v.blocker_horn[s], v.blocker.side[s].horn_pin, v.claw[s], v.blocker.side[s].claw_pin);
}

MassItem Mechanism::motorItem(const MotorData& m, double burned) const {
    // Casing plus the propellant that is left (its inertia and mass shrink together)
    double keep = 1.0 - std::clamp(burned, 0.0, 1.0);
    MassItem prop = m.propellant;
    prop.mass *= keep;
    prop.inertia = prop.inertia * keep;
    double mt = m.casing.mass + prop.mass;
    MassItem out;
    out.mass = mt;
    out.cg = (m.casing.cg * m.casing.mass + prop.cg * prop.mass) / mt;
    out.inertia = m.casing.inertia + prop.inertia + parallelAxis(m.casing.mass, m.casing.cg - out.cg) +
                  parallelAxis(prop.mass, prop.cg - out.cg);
    return out;
}

void Mechanism::setMotors(bool ascent, double ascent_b, bool landing, double landing_b) {
    if (ascent != ascent_present || landing != landing_present) props_dirty = true;  // a motor came out
    ascent_present = ascent;
    landing_present = landing;
    ascent_burned = ascent_b;
    landing_burned_ = landing_b;
}

void Mechanism::refreshProps() {
    computePoses();
    const VehicleData& v = vehicle;
    double m_sum = 0;
    Vec3 first_moment;
    struct Item { double m; Vec3 c; Mat3 I; bool gimbal; };
    std::vector<Item> items;
    items.reserve(v.links.size());

    // The gimbal's moving parts: everything below the outer ring except the pushrod and horn halves
    auto inGimbal = [&](int i) {
        for (int p = i; p >= 0; p = v.links[p].parent)
            if (p == v.outer_ring) return true;
        return false;
    };
    double dry_total = 0;
    for (size_t i = 0; i < v.links.size(); i++)
        if ((int)i != v.landing_motor && (int)i != v.ascent_motor) dry_total += v.links[i].item.mass;
    double af_shift = cg_shift * dry_total / v.links[v.airframe].item.mass;

    for (size_t i = 0; i < v.links.size(); i++) {
        MassItem it = v.links[i].item;
        bool motor = (int)i == v.landing_motor || (int)i == v.ascent_motor;
        if ((int)i == v.ascent_motor) {
            if (!ascent_present) continue;
            it = motorItem(v.ascent, ascent_burned);
        } else if ((int)i == v.landing_motor) {
            if (!landing_present) continue;
            it = motorItem(v.landing, landing_burned_);
        } else {
            it.mass *= mass_scale;
            it.inertia = it.inertia * mass_scale;
            if ((int)i == v.airframe) it.cg.x += af_shift;
        }
        (void)motor;
        const Pose& P = pose_[i];
        Item e;
        e.m = it.mass;
        e.c = P.t + P.R * it.cg;
        e.I = P.R * it.inertia * transpose(P.R);
        e.gimbal = inGimbal((int)i);
        items.push_back(e);
        m_sum += e.m;
        first_moment += e.c * e.m;
    }
    Vec3 cg = first_moment / m_sum;
    Mat3 I = zeroMatrix();
    Mat3 Ig = zeroMatrix();
    Vec3 gimbal_first;
    double gimbal_mass = 0;
    for (const Item& e : items) {
        I += e.I + parallelAxis(e.m, e.c - cg);
        if (e.gimbal) {
            Ig += e.I + parallelAxis(e.m, e.c - v.tvc.pivot);
            gimbal_first += e.c * e.m;
            gimbal_mass += e.m;
        }
    }
    props_.mass = m_sum;
    props_.cg = cg;
    props_.inertia = I;
    props_.inverse_inertia = inverse(I);
    gimbal_inertia = Ig;
    gimbal_cg_rel = gimbal_mass > 0 ? gimbal_first / gimbal_mass - v.tvc.pivot : Vec3{};
    gimbal_mass_ = gimbal_mass;

    // Gimbal flex: a spring and damper whose stiffness gives the ringing frequency in rocket.cfg
    double w = 2 * PI * truth.gimbal_wobble_hz;
    wobble_k[0] = Ig.m[2][2] * w * w;   // yaw turns about body z
    wobble_k[1] = Ig.m[1][1] * w * w;   // pitch turns about body y
    wobble_c[0] = 2 * truth.gimbal_wobble_damping * w * Ig.m[2][2];
    wobble_c[1] = 2 * truth.gimbal_wobble_damping * w * Ig.m[1][1];
    props_dirty = false;
    steps_since_props = 0;
}

Vec3 Mechanism::thrustDirection() const {
    // The motor sits in the cartridge, a little crooked (mount misalignment), and the cartridge turns with the gimbal
    Mat3 R = rotationMatrix({0, 0, 1}, gimbal[0]) * rotationMatrix({0, 1, 0}, gimbal[1]);
    Mat3 M = rotationMatrix({0, 0, 1}, yaw_misalign) * rotationMatrix({0, 1, 0}, pitch_misalign);
    return R * (M * Vec3{1, 0, 0});
}

// ---------------------------------------------------------------------------------------------------------------
// Blocker effects
// ---------------------------------------------------------------------------------------------------------------
// lost_per_share = full-close thrust loss (N) after effectiveness and erosion. Each claw takes its own share of
// that, the missing thrust acts where the claw covers the exhaust (so an uneven pair twists the rocket), and a
// share of it is pushed sideways away from the claw.
Vec3 Mechanism::blockerForceTorque(double lost_per_share, Vec3& force) const {
    Vec3 torque;
    force = Vec3{};
    double total_closed = claw_area_closed[0] + claw_area_closed[1];
    for (int s = 0; s < 2; s++) {
        const BlockerSide& b = vehicle.blocker.side[s];
        double area = interp(b.table_claw, b.table_area, claw[s] / DEG);
        double cy = interp(b.table_claw, b.table_cy, claw[s] / DEG) * 1e-3;
        double cz = interp(b.table_claw, b.table_cz, claw[s] / DEG) * 1e-3;
        double lost = lost_per_share * area / total_closed;  // N of thrust this claw takes away
        torque += Vec3{0, -cz, cy} * lost;
        double r = std::hypot(cy, cz);
        if (r > 1e-9) force += Vec3{0, -cy, -cz} / r * (lost * truth.blocker_side_force);
    }
    return torque;
}

// ---------------------------------------------------------------------------------------------------------------
// Legs
// ---------------------------------------------------------------------------------------------------------------
Vec3 Mechanism::footCentre(int i) const {
    const LegGeom& g = vehicle.legs.leg[i];
    return g.hinge_origin + rotationMatrix(g.hinge_axis, leg[i].angle) * foot_rel[i];
}

void Mechanism::tubeSegment(int i, Vec3& a, Vec3& b) const {
    const LegGeom& g = vehicle.legs.leg[i];
    Mat3 R = rotationMatrix(g.hinge_axis, leg[i].angle);
    a = g.hinge_origin + R * Vec3{0.010, 0, 0};
    b = g.hinge_origin + R * Vec3{0.272, 0, 0};
}

// Crossflow drag on the tube (in four pieces, each seeing its own airspeed) and drag on the foot ball.
// torque_cg = torque about the rocket's CG. hinge_torque = torque about the hinge axis (for the leg's own motion).
Vec3 Mechanism::legAeroForce(int i, const Vec3& cg, const Vec3& velocity_rel_air, const Vec3& omega, double rho,
                             Vec3& torque_cg) const {
    const LegGeom& g = vehicle.legs.leg[i];
    Mat3 R = rotationMatrix(g.hinge_axis, leg[i].angle);
    Vec3 e = R * Vec3{1, 0, 0};
    Vec3 spin = g.hinge_axis * leg[i].rate;  // the leg's own swing
    Vec3 total, tq;
    const int pieces = 4;
    double s0 = 0.010, s1 = 0.272, len = (s1 - s0) / pieces;
    for (int k = 0; k < pieces; k++) {
        Vec3 p = g.hinge_origin + e * (s0 + len * (k + 0.5));
        Vec3 v = velocity_rel_air + cross(omega, p - cg) + cross(spin, p - g.hinge_origin);
        Vec3 vn = v - e * dot(v, e);
        double speed = length(vn);
        Vec3 f = vn * (-0.5 * rho * truth.cd_leg * vehicle.legs.tube_od * len * speed);
        total += f;
        tq += cross(p - cg, f);
    }
    Vec3 pf = footCentre(i);
    Vec3 v = velocity_rel_air + cross(omega, pf - cg) + cross(spin, pf - g.hinge_origin);
    double r = vehicle.legs.foot_radius;
    Vec3 f = v * (-0.5 * rho * truth.cd_foot * PI * r * r * length(v));
    total += f;
    tq += cross(pf - cg, f);
    torque_cg = tq;
    return total;
}

void Mechanism::updateLegs(double t, double dt, const MechanismMotion& m) {
    const VehicleData& v = vehicle;
    for (int i = 0; i < 4; i++) {
        Leg& L = leg[i];
        const LegGeom& g = v.legs.leg[i];
        double lo = g.lo_deg * DEG, hi = g.hi_deg * DEG;

        // The keeper's peg holds the leg stowed until the ring has turned far enough
        if (!L.released) {
            if (-ring_angle >= L.release_at) L.released = true;
            else { L.angle = lo; L.rate = 0; continue; }
        }

        Vec3 a = g.hinge_axis;
        Mat3 R = rotationMatrix(a, L.angle);
        Vec3 cg_rel = R * leg_cg_rel[i];  // from the hinge
        double ml = v.links[v.leg[i]].item.mass * mass_scale;

        // 1. the deploy spring
        double tau = truth.leg_spring_k * std::max(0.0, spring_free_angle - L.angle);

        // 2. weight and the rocket's own acceleration: gravity minus the CG's acceleration, plus centrifugal and
        //    Euler forces from the rocket turning
        Vec3 rho = g.hinge_origin + cg_rel - props_.cg;
        Vec3 apparent = m.g_minus_a - cross(m.alpha, rho) - cross(m.omega, cross(m.omega, rho));
        tau += dot(cross(cg_rel, apparent * ml), a);

        // 3. drag in the descent airflow
        // (the same pieces as legAeroForce, with the hinge as the reference point)
        {
            Vec3 e = R * Vec3{1, 0, 0};
            Vec3 spin = a * L.rate;
            const int pieces = 4;
            double s0 = 0.010, s1 = 0.272, len = (s1 - s0) / pieces;
            for (int k = 0; k < pieces; k++) {
                Vec3 p = e * (s0 + len * (k + 0.5));  // from the hinge
                Vec3 vv = m.air + cross(m.omega, g.hinge_origin + p - props_.cg) + cross(spin, p);
                Vec3 vn = vv - e * dot(vv, e);
                Vec3 f = vn * (-0.5 * m.air_density * truth.cd_leg * v.legs.tube_od * len * length(vn));
                tau += dot(cross(p, f), a);
            }
            Vec3 pf = R * foot_rel[i];
            Vec3 vv = m.air + cross(m.omega, g.hinge_origin + pf - props_.cg) + cross(spin, pf);
            double rr = v.legs.foot_radius;
            Vec3 f = vv * (-0.5 * m.air_density * truth.cd_foot * PI * rr * rr * length(vv));
            tau += dot(cross(pf, f), a);
        }

        // 4. the ground pushing on the foot
        tau += dot(cross(m.foot_arm[i] - g.hinge_origin, m.foot_force[i]), a);

        // 5. the plunger riding up its ramp (from the start of the ramp to the latch angle)
        double ramp_start = v.legs.ramp_start_deg * DEG, latch = v.legs.latch_deg * DEG;
        double arm = v.legs.plunger_arm;
        if (!L.latched && !L.failed && L.angle > ramp_start) {
            double f = std::clamp((L.angle - ramp_start) / (latch - ramp_start), 0.0, 1.0);
            double pin = truth.plunger_force_start + (truth.plunger_force_end - truth.plunger_force_start) * f;
            double slope = truth.plunger_ramp_rise / (arm * (latch - ramp_start));
            double sign = L.rate >= 0 ? 1.0 : -1.0;
            tau -= pin * (slope + sign * truth.plunger_ramp_friction) * arm;
        }

        // 6. hinge friction
        double net = tau;
        double fr = truth.leg_hinge_friction;
        if (std::fabs(L.rate) < 1e-3 && std::fabs(net) <= fr) {
            net = 0;
            L.rate = 0;
        } else {
            net -= fr * (L.rate > 1e-3 ? 1.0 : (L.rate < -1e-3 ? -1.0 : (net > 0 ? 1.0 : -1.0)));
        }
        double external = tau;  // everything except constraints, for the plunger load

        L.rate += net / L.inertia * dt;
        L.angle += L.rate * dt;

        // 7. stops: the stow stop, the stop bar, and the plunger's hole
        double window = bl_latch / arm;  // the pin rattles in its hole by this much
        double low = L.latched ? hi - window : lo;
        L.plunger_load = 0;
        if (L.angle < low) {
            L.angle = low;
            if (L.rate < 0) L.rate = -truth.leg_stop_restitution * L.rate;
            if (L.latched) {
                // The pin takes whatever pushes the leg back toward stowed, quasi-statically
                L.plunger_load = std::max(0.0, -external) / arm;
                L.peak_plunger_load = std::max(L.peak_plunger_load, L.plunger_load);
                if (L.plunger_load > truth.plunger_shear_limit) {
                    L.failed = true;
                    L.latched = false;
                }
            }
        }
        if (L.angle > hi) {
            L.angle = hi;
            if (L.rate > 0) L.rate = -truth.leg_stop_restitution * L.rate;
            if (!L.failed && !L.latched) {
                L.latched = true;
                if (L.deploy_time < 0) L.deploy_time = t;
            }
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------
// TVC
// ---------------------------------------------------------------------------------------------------------------
void Mechanism::updateTvc(double t, double dt, const MechanismMotion& m) {
    (void)t;
    // The linkage only needs solving when a horn has moved
    if (servo_tvc[0].angle != tvc_q_last[0] || servo_tvc[1].angle != tvc_q_last[1]) {
        tvcForward(servo_tvc[0].angle, servo_tvc[1].angle, gimbal_target[0], gimbal_target[1]);
        tvc_q_last[0] = servo_tvc[0].angle;
        tvc_q_last[1] = servo_tvc[1].angle;
    }

    // Lost motion: the pushrod ends, the cross of the universal joint and the axle bearings. Each is a dead zone
    // in the gimbal angle; the rod's slop (a length) becomes an angle through the linkage's lever.
    double link[2];
    for (int k = 0; k < 2; k++) {
        double width = bl_rod / std::max(tvc_lever[k], 1e-6) + bl_axle[k];
        link[k] = link_dead[k].apply(gimbal_target[k], width);
        gimbal_link[k] = link[k];
    }

    // The gimbal flexes: a spring and damper between the linkage and the cartridge. Inertia from the apparent
    // acceleration of the cartridge's CG and from the rocket turning acts on it too.
    double accel[2];
    Vec3 inertial = cross(gimbal_cg_rel, m.g_minus_a * gimbal_mass_);  // torque about the pivot, body frame
    double axis_alpha[2] = {m.alpha.z, m.alpha.y};
    double axis_inertia[2] = {gimbal_inertia.m[2][2], gimbal_inertia.m[1][1]};
    double axis_inertial[2] = {inertial.z, inertial.y};
    double torque[2];
    for (int k = 0; k < 2; k++) {
        if (truth.gimbal_wobble_hz <= 0) torque[k] = 0;
        else torque[k] = wobble_k[k] * (link[k] - gimbal[k]) - wobble_c[k] * gimbal_rate[k];
    }

    // The hard stop: a stiff, damped ring at the CAD stop angle (the commands stay inside an 8 deg circle)
    double stop = vehicle.tvc.hard_stop_deg * DEG;
    double radius = std::hypot(gimbal[0], gimbal[1]);
    tvc_at_stop = false;
    double stop_torque[2] = {0, 0};
    if (radius > stop) {
        tvc_at_stop = true;
        double u0 = gimbal[0] / radius, u1 = gimbal[1] / radius;
        double radial_rate = gimbal_rate[0] * u0 + gimbal_rate[1] * u1;
        double w = 2 * PI * 100.0;  // 100 Hz stop, stiff but stable at the physics step
        for (int k = 0; k < 2; k++) {
            double u = k == 0 ? u0 : u1;
            stop_torque[k] = -(axis_inertia[k] * w * w * (radius - stop) + 2 * 0.5 * w * axis_inertia[k] * std::max(radial_rate, 0.0)) * u;
        }
    }

    for (int k = 0; k < 2; k++) {
        if (truth.gimbal_wobble_hz <= 0) {  // perfectly stiff: follows the linkage exactly (still stops at the ring)
            double new_rate = (link[k] - gimbal[k]) / dt;
            accel[k] = (new_rate - gimbal_rate[k]) / dt;
            gimbal_rate[k] = new_rate;
            gimbal[k] = link[k];
        } else {
            accel[k] = (torque[k] + stop_torque[k] + axis_inertial[k]) / axis_inertia[k] - axis_alpha[k];
            gimbal_rate[k] += accel[k] * dt;
            gimbal[k] += gimbal_rate[k] * dt;
        }
        gimbal_accel[k] = accel[k];
        // What the servo feels: the spring force through the lever ratio
        servo_tvc[k].load = (truth.gimbal_wobble_hz <= 0 ? 0.0 : wobble_k[k] * (link[k] - gimbal[k])) * tvc_gain[k];
    }
}

// ---------------------------------------------------------------------------------------------------------------
// One step
// ---------------------------------------------------------------------------------------------------------------
void Mechanism::update(double t, double dt, const ServoTargets& targets, const MechanismMotion& m) {
    int n = std::max(1, (int)std::ceil(dt / truth.mechanism_dt - 1e-9));
    double ds = dt / n;
    for (int s = 0; s < n; s++) {
        double ts = t + s * ds;

        // Servos. The claws are locked open while the ascent motor is in the way.
        double r_cmd = ascent_present ? -80.0 : targets.blocker_r;
        double l_cmd = ascent_present ? -80.0 : targets.blocker_l;
        servo_tvc[0].update(targets.tvc_yaw * DEG, ts, ds, truth, bl_servo);
        servo_tvc[1].update(targets.tvc_pitch * DEG, ts, ds, truth, bl_servo);
        servo_blocker[0].update(r_cmd * DEG, ts, ds, truth, bl_servo);
        servo_blocker[1].update(l_cmd * DEG, ts, ds, truth, bl_servo);
        servo_keeper.update(targets.keeper * DEG, ts, ds, truth, bl_servo);

        // Keeper: horn -> ring, with the set screw's play in the slot, stopped at the ring's hard stop
        double ring_target = ringFromHorn(servo_keeper.angle);
        double slot_width = bl_keeper / std::max(keeper_c, 1e-3);
        double ring_free = ring_dead.apply(ring_target, slot_width);
        double ring_lo = vehicle.keeper.ring_lo_deg * DEG, ring_hi = vehicle.keeper.ring_hi_deg * DEG;
        keeper_stalled = ring_free < ring_lo;
        ring_angle = std::clamp(ring_free, ring_lo, ring_hi);
        if (keeper_stalled) ring_dead.state = ring_lo + 0.5 * slot_width;  // pressed on the stop, no slack
        servo_keeper.load = keeper_stalled ? 0.05 : 0.0;

        updateTvc(ts, ds, m);

        // Blocker claws
        for (int side = 0; side < 2; side++) {
            const BlockerSide& b = vehicle.blocker.side[side];
            double q = servo_blocker[side].angle;
            if (q != claw_q_last[side]) {  // the four-bar only needs solving when the horn has moved
                claw_q_last[side] = q;
                claw_solution[side][0] = clawFromServo(side, q);
                claw_solution[side][1] = clawFromServo(side, q, bl_blocker / 2);
                claw_solution[side][2] = clawFromServo(side, q, -bl_blocker / 2);
            }
            double nominal = claw_solution[side][0], more = claw_solution[side][1], less = claw_solution[side][2];
            double c;
            if (m.landing_burning) {
                c = std::min(more, less);  // the exhaust pushes the claws open, taking up the slop that way
                claw_dead[side].state = c;
            } else {
                c = claw_dead[side].apply(nominal, std::fabs(more - less));
            }
            (void)b;
            claw[side] = std::clamp(c, -55.0 * DEG, 0.0);
        }

        updateLegs(ts, ds, m);
    }

    // The blocked share of the exhaust, as a share of both claws fully closed
    double closed = claw_area_closed[0] + claw_area_closed[1];
    for (int side = 0; side < 2; side++) {
        const BlockerSide& b = vehicle.blocker.side[side];
        blocked_share_side[side] = interp(b.table_claw, b.table_area, claw[side] / DEG) / closed;
    }
    blocked_share = blocked_share_side[0] + blocked_share_side[1];

    // Reaction of the gimbal's acceleration on the airframe (it twists the rocket the other way)
    reaction_torque = Vec3{0, -gimbal_inertia.m[1][1] * gimbal_accel[1], -gimbal_inertia.m[2][2] * gimbal_accel[0]};

    if (props_dirty || ++steps_since_props >= 5) refreshProps();
}

// ---------------------------------------------------------------------------------------------------------------
BodyProps neutralProps(const VehicleData& vehicle, bool legs_out, const Config& cfg) {
    std::mt19937 rng(1);
    Mechanism m(vehicle, cfg, cfg, rng);
    m.setMotors(false, 0, false, 0);
    if (legs_out)
        for (int i = 0; i < 4; i++) m.leg[i].angle = vehicle.legs.latch_deg * DEG;
    m.refreshProps();
    return m.props();
}
