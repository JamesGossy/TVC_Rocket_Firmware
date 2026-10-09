#include "physics.h"
#include <cmath>

// How fast each part of the state is changing
struct Derivative {
    Vec3 velocity;
    Vec3 acceleration;
    Quat attitude_rate;
    Vec3 angular_acceleration;
};

static double airDensity(double altitude) {
    // Simple exponential atmosphere, fine for a few hundred meters
    return 1.225 * std::exp(-altitude / 8500.0);
}

// Which way the motor pushes (body frame) for a given gimbal angle
static Vec3 thrustDirection(double gimbal_y, double gimbal_z) {
    return {std::cos(gimbal_y) * std::cos(gimbal_z),
            std::sin(gimbal_z),
            -std::sin(gimbal_y) * std::cos(gimbal_z)};
}

Ground::Ground(const Config& cfg, std::mt19937& rng) : cfg(cfg) {
    double uphill = cfg.ground_slope_direction_deg * DEG;
    double rise = std::tan(cfg.ground_slope_deg * DEG);
    slope_x = rise * std::cos(uphill);
    slope_y = rise * std::sin(uphill);

    std::uniform_real_distribution<double> uniform(0.0, 1.0);
    auto makeWaves = [&](double spacing, double size) {
        std::vector<Wave> list;
        int count = 6;
        for (int i = 0; i < count; i++) {
            double direction = 2 * PI * uniform(rng);
            double k = 2 * PI / (spacing * (0.5 + uniform(rng)));
            list.push_back({k * std::cos(direction), k * std::sin(direction), 2 * PI * uniform(rng),
                            size / std::sqrt(count / 2.0)});
        }
        return list;
    };
    bumps = makeWaves(cfg.ground_bump_size, cfg.ground_bumps);
    soft = makeWaves(3.0, cfg.ground_softness);
    bump_at_launch = waves(bumps, 0, 0);
}

double Ground::waves(const std::vector<Wave>& list, double x, double y) const {
    double sum = 0;
    for (const Wave& w : list) sum += w.size * std::sin(w.kx * x + w.ky * y + w.phase);
    return sum;
}

double Ground::height(double x, double y) const {
    return slope_x * x + slope_y * y + waves(bumps, x, y) - bump_at_launch;
}

Vec3 Ground::normal(double x, double y) const {
    double rise_x = slope_x, rise_y = slope_y;
    for (const Wave& w : bumps) {
        double c = w.size * std::cos(w.kx * x + w.ky * y + w.phase);
        rise_x += c * w.kx;
        rise_y += c * w.ky;
    }
    return normalize(Vec3{-rise_x, -rise_y, 1});
}

double Ground::stiffness(double x, double y) const {
    return cfg.leg_stiffness * std::max(0.3, 1 + waves(soft, x, y));
}

// Points that can touch the ground, measured from the bottom of the rocket.
// The four feet (sitting higher once their leg has stroked), plus the nozzle and the nose tip
// for when it falls over.
static std::array<Vec3, 6> contactPoints(const Config& cfg, const LegTravel& travel) {
    double d = cfg.leg_span / 2 * std::sqrt(0.5);  // feet sit at 45 deg between the y and z axes
    double foot = -cfg.leg_drop;
    return {{{foot + travel[0], d, d}, {foot + travel[1], -d, d}, {foot + travel[2], -d, -d},
             {foot + travel[3], d, -d}, {0, 0, 0}, {cfg.body_length, 0, 0}}};
}

// How high the lowest part of the rocket is above the ground under it
double lowestPoint(const Config& cfg, const Ground& ground, const MassProps& mp, const RigidBodyState& s,
                   const LegTravel& travel) {
    double lowest = 1e9;
    for (Vec3 p : contactPoints(cfg, travel)) {
        Vec3 point = s.position + rotate(s.attitude, {p.x - mp.cg, p.y, p.z});
        lowest = std::min(lowest, point.z - ground.height(point.x, point.y));
    }
    return lowest;
}

// After each step: any leg pressed in harder than its crush force gives way and gets shorter
void strokeLegs(const Config& cfg, const Ground& ground, const MassProps& mp, const RigidBodyState& s,
                LegTravel& travel) {
    auto points = contactPoints(cfg, travel);
    for (int i = 0; i < 4; i++) {
        Vec3 point = s.position + rotate(s.attitude, {points[i].x - mp.cg, points[i].y, points[i].z});
        double depth = (ground.height(point.x, point.y) - point.z) * ground.normal(point.x, point.y).z;
        double too_far = depth - cfg.leg_crush_force / ground.stiffness(point.x, point.y);
        if (too_far > 0) travel[i] = std::min(cfg.leg_stroke, travel[i] + too_far);
    }
}

// Any point poking into the ground gets pushed back out by the grass and leg struts (a stiff,
// lossy spring), straight out of the ground's surface, which tilts on slopes and bumps. A leg
// can't push harder than its damper's crush force, since past that the damper strokes instead.
// Friction fights sliding. That gives landings, skids and tipping over.
static void groundForces(const Config& cfg, const StepInputs& in, const RigidBodyState& s,
                         Vec3& force_world, Vec3& torque_body) {
    const MassProps& mp = in.mass_props;
    const Ground& ground = *in.ground;

    // How easily a point on the rocket moves when pushed in a (world) direction, counting the
    // rocket turning about its CG. Used to keep the ground forces from overshooting in one time step.
    auto give = [&](const Vec3& arm, Vec3 direction) {
        Vec3 turn = cross(arm, rotateBack(s.attitude, direction));
        return 1 / mp.mass + turn.x * turn.x / mp.inertia_roll + (turn.y * turn.y + turn.z * turn.z) / mp.inertia_pitch;
    };

    auto points = contactPoints(cfg, in.leg_travel);
    for (int i = 0; i < 6; i++) {
        Vec3 arm = {points[i].x - mp.cg, points[i].y, points[i].z};
        Vec3 point = s.position + rotate(s.attitude, arm);
        double below = ground.height(point.x, point.y) - point.z;
        if (below <= 0) continue;

        Vec3 out = ground.normal(point.x, point.y);
        double depth = below * out.z;
        Vec3 v = s.velocity + rotate(s.attitude, cross(s.angular_velocity, arm));
        double sinking = dot(v, out);  // negative while going in

        // Pick the damping that gives the bounciness asked for, even if a single point takes the
        // whole rocket. With several points touching it's deader still, which suits grass.
        double stiffness = ground.stiffness(point.x, point.y);
        double loss = -std::log(std::clamp(cfg.bounciness, 0.001, 0.999));
        double damping_ratio = loss / std::sqrt(PI * PI + loss * loss);
        double damping = 2 * damping_ratio * std::sqrt(stiffness * mp.mass);

        // The damper stops this point sinking in, but can't fling it back out, so it's capped at the
        // force that would just stop the point within one time step (keeps hard crashes stable)
        double most = 0.2 * std::fabs(sinking) / (give(arm, out) * cfg.physics_dt);
        double damper = std::clamp(damping * sinking, -most, most);
        double push = std::max(stiffness * depth - damper, 0.0);

        bool is_foot = i < 4;
        if (is_foot && in.leg_travel[i] < cfg.leg_stroke) push = std::min(push, cfg.leg_crush_force);

        // Friction fights sliding. It can stop a sliding foot but never push it backwards, so it's
        // capped at the force that would just stop this point within one time step.
        Vec3 f = out * push;
        Vec3 sliding = v - out * sinking;
        double slide = length(sliding);
        if (slide > 1e-6) {
            Vec3 direction = sliding / slide;
            double stopping_force = 0.2 * slide / (give(arm, direction) * cfg.physics_dt);
            double friction = std::min(cfg.ground_friction * push, stopping_force);
            f = f - direction * friction;
        }

        force_world += f;
        torque_body += cross(arm, rotateBack(s.attitude, f));
    }
}

static Derivative derivative(const Config& cfg, const StepInputs& in, const RigidBodyState& s) {
    const MassProps& mp = in.mass_props;
    Vec3 force;   // body frame
    Vec3 torque;  // body frame, about the CG

    // Thrust pushes along the gimbal direction, through the gimbal pivot
    Vec3 push = thrustDirection(in.gimbal_y, in.gimbal_z) * in.thrust + in.side_force;
    Vec3 pivot_arm = Vec3{cfg.gimbal_position - mp.cg, 0, 0} - in.cg_shift;
    force += push;
    torque += cross(pivot_arm, push);
    torque.x += in.roll_torque;
    torque += in.extra_torque;

    // Jet damping: exhaust leaving the nozzle (at the bottom of the rocket) carries some of the
    // rocket's wobble away with it
    double nozzle_arm = mp.cg;
    torque.y -= in.mass_flow * nozzle_arm * nozzle_arm * s.angular_velocity.y;
    torque.z -= in.mass_flow * nozzle_arm * nozzle_arm * s.angular_velocity.z;

    // Aerodynamics. First, how is the rocket moving through the air, seen from the rocket?
    Vec3 air_velocity = rotateBack(s.attitude, s.velocity - in.wind);
    bool tail_first = air_velocity.x < 0;
    double cd = tail_first ? cfg.cd_tail_first : cfg.cd_nose_first;
    double cp = tail_first ? cfg.cp_tail_first : cfg.cp_nose_first;
    Vec3 cp_arm = {cp - mp.cg, 0, 0};

    // Spinning also swings the CP through the air. This is what damps out wobbles.
    Vec3 v = air_velocity + cross(s.angular_velocity, cp_arm);
    double speed = length(v);
    double area = PI * cfg.body_diameter * cfg.body_diameter / 4;
    double k = 0.5 * airDensity(s.position.z) * area;

    Vec3 aero;
    aero.x = -k * cd * std::fabs(v.x) * v.x;   // drag along the body
    aero.y = -k * cfg.cn_alpha * speed * v.y;  // normal force from flying at an angle
    aero.z = -k * cfg.cn_alpha * speed * v.z;
    force += aero;
    torque += cross(cp_arm, aero);

    // Crossflow drag: when the rocket goes sideways through the air, the whole side of the tube
    // acts like a flat plate. Tiny at small angles, but it's what calms down a tumbling rocket.
    Vec3 middle_arm = {cfg.body_length / 2 - mp.cg, 0, 0};
    Vec3 side = air_velocity + cross(s.angular_velocity, middle_arm);
    double side_speed = std::hypot(side.y, side.z);
    double side_k = 0.5 * airDensity(s.position.z) * cfg.cd_crossflow * cfg.body_length * cfg.body_diameter;
    Vec3 crossflow = {0, -side_k * side_speed * side.y, -side_k * side_speed * side.z};
    force += crossflow;
    torque += cross(middle_arm, crossflow);

    Vec3 ground;  // world frame
    if (in.legs_out) groundForces(cfg, in, s, ground, torque);

    Derivative d;
    d.velocity = s.velocity;
    d.acceleration = (rotate(s.attitude, force) + ground) / mp.mass + Vec3{0, 0, -GRAVITY};

    // Euler's rotation equations
    Vec3 w = s.angular_velocity;
    Vec3 inertia = {mp.inertia_roll, mp.inertia_pitch, mp.inertia_pitch};
    Vec3 momentum = {inertia.x * w.x, inertia.y * w.y, inertia.z * w.z};
    Vec3 net = torque - cross(w, momentum);
    d.angular_acceleration = {net.x / inertia.x, net.y / inertia.y, net.z / inertia.z};

    Quat q = s.attitude * Quat{0, w.x, w.y, w.z};
    d.attitude_rate = {0.5 * q.w, 0.5 * q.x, 0.5 * q.y, 0.5 * q.z};
    return d;
}

static RigidBodyState advance(RigidBodyState s, const Derivative& d, double dt) {
    s.position += d.velocity * dt;
    s.velocity += d.acceleration * dt;
    s.attitude.w += d.attitude_rate.w * dt;
    s.attitude.x += d.attitude_rate.x * dt;
    s.attitude.y += d.attitude_rate.y * dt;
    s.attitude.z += d.attitude_rate.z * dt;
    s.angular_velocity += d.angular_acceleration * dt;
    return s;
}

// Classic 4th order Runge-Kutta
RigidBodyState rk4Step(const Config& cfg, const StepInputs& in, const RigidBodyState& s, double dt) {
    Derivative k1 = derivative(cfg, in, s);
    Derivative k2 = derivative(cfg, in, advance(s, k1, dt / 2));
    Derivative k3 = derivative(cfg, in, advance(s, k2, dt / 2));
    Derivative k4 = derivative(cfg, in, advance(s, k3, dt));

    // Same as s + dt * (k1 + 2*k2 + 2*k3 + k4) / 6
    RigidBodyState next = s;
    next = advance(next, k1, dt / 6);
    next = advance(next, k2, dt / 3);
    next = advance(next, k3, dt / 3);
    next = advance(next, k4, dt / 6);
    next.attitude = normalize(next.attitude);
    return next;
}
