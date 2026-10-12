#include "physics.h"
#include <cmath>

// How fast each part of the state is changing
struct Derivative {
    Vec3 velocity;
    Vec3 acceleration;
    Quat attitude_rate;
    Vec3 angular_acceleration;
};

double airDensity(double altitude) {
    // Simple exponential atmosphere, fine for a few hundred meters
    return 1.225 * std::exp(-altitude / 8500.0);
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
    return cfg.ground_stiffness * std::max(0.3, 1 + waves(soft, x, y));
}

// ---------------------------------------------------------------------------------------------------------------
// The feet. Each is an EPS ball on the end of a carbon tube. Pressing it into the ground has two parts:
//   - springy: the grass, in series with the tube bending sideways under the part of the force that is across it
//   - permanent: the ball crushes. Pushing the ball in by s flattens a patch of area pi (2 R s - s^2), and the foam
//     holds up a crush stress sigma over that patch, so the crush force is sigma pi (2 R s - s^2).
// A hard hit: the force climbs the crush curve. Letting go: it springs back along the stiff part only, and a
// second hit does nothing new until it pushes past the deepest crush so far.
// Past half way (s = R) the patch can't get any wider than the ball, so the crush force levels off at
// sigma pi R^2 until the ball is squashed flat (s = 2 R), and after that the foot is rigid.
// ---------------------------------------------------------------------------------------------------------------
struct FootContact {
    double force = 0;     // N along the ground normal, before damping
    double stiffness = 0; // N/m of the springy part
    double compression = 0;
    Vec3 out;             // ground normal, world
    Vec3 centre, point;   // world
    Vec3 arm;             // from the CG to the contact point, body frame
    bool touching = false;
    bool crushing = false;  // the foam is giving way right now (no extra damping then, the crush already soaks up the energy)
};

// What the foam holds up when it has been crushed s deep
static double crushForce(double s, double R, double sigma) {
    s = std::min(s, R);
    return sigma * PI * (2 * R * s - s * s);
}

// Crush depth that goes with a given total compression p (the plastic branch), for crush stress sigma.
// p = s + F(s) / k_e: the foam crushes s and the grass and tube give the rest.
static double plasticCrush(double p, double R, double sigma, double k_e) {
    double c = sigma * PI / k_e;
    double b = 1 + 2 * R * c;
    double disc = std::max(0.0, b * b - 4 * c * p);
    double s = (b - std::sqrt(disc)) / (2 * c);
    if (s <= R) return s;
    double plateau = crushForce(R, R, sigma);
    return std::min(2 * R, std::max(R, p - plateau / k_e));
}

// The foot's force for a total compression p, with s_old crushed so far (the deepest crush before this)
static double footForce(double p, double s_old, double R, double sigma, double k_e, double& s_now) {
    double springy = k_e * std::max(0.0, p - s_old);
    s_now = s_old;
    if (springy <= crushForce(s_old, R, sigma) && s_old < 2 * R) return springy;
    s_now = std::max(s_old, plasticCrush(p, R, sigma, k_e));
    double crush = crushForce(s_now, R, sigma);
    if (s_now >= 2 * R) return std::max(crush, k_e * (p - 2 * R));  // squashed flat: rigid
    return crush;
}

static FootContact footContact(const Config& cfg, const StepInputs& in, const RigidBodyState& s, int i) {
    FootContact f;
    const Ground& ground = *in.ground;
    double R = in.foot_radius;
    Vec3 arm = in.foot_centre[i] - in.body.cg;
    f.centre = s.position + rotate(s.attitude, arm);
    double clear = f.centre.z - ground.height(f.centre.x, f.centre.y);
    if (clear > 1.5 * R) return f;  // well off the ground (the ground never tilts that far)
    f.out = ground.normal(f.centre.x, f.centre.y);
    double above = clear * f.out.z;
    f.compression = R - above;
    if (f.compression <= 0) return f;
    f.touching = true;

    Vec3 out_body = rotateBack(s.attitude, f.out);
    f.arm = arm - out_body * R;
    f.point = f.centre - f.out * R;

    // How springy: the grass, with the tube bending under the part of the push that is across it
    double across = 1 - std::pow(dot(in.leg_dir[i], out_body), 2);
    f.stiffness = 1 / (1 / ground.stiffness(f.centre.x, f.centre.y) + across / in.tube_stiffness);

    double s_now;
    f.force = footForce(f.compression, in.foot_crush[i], R, cfg.foot_crush_stress, f.stiffness, s_now);
    f.crushing = s_now > in.foot_crush[i];
    return f;
}

// Anything but the feet touching the ground is a stiff, lossy spring. Only worth checking when we're low.
static bool nearGround(const StepInputs& in, const RigidBodyState& s) {
    return s.position.z - in.ground->height(s.position.x, s.position.y) < 1.0;
}

// How high the lowest part of the rocket is above the ground under it
double lowestPoint(const Config& cfg, const Ground& ground, const StepInputs& in, const RigidBodyState& s) {
    (void)cfg;
    if (s.position.z - ground.height(s.position.x, s.position.y) > 2.0) return s.position.z - ground.height(s.position.x, s.position.y) - 0.65;
    // (nothing reaches the ground from up here, and the CG is at most 0.65 m from the lowest point)
    double lowest = 1e9;
    for (int i = 0; i < 4; i++) {
        Vec3 c = s.position + rotate(s.attitude, in.foot_centre[i] - in.body.cg);
        lowest = std::min(lowest, (c.z - ground.height(c.x, c.y)) * ground.normal(c.x, c.y).z - in.foot_radius);
    }
    if (in.body_points)
        for (const Vec3& p : *in.body_points) {
            Vec3 w = s.position + rotate(s.attitude, p - in.body.cg);
            lowest = std::min(lowest, w.z - ground.height(w.x, w.y));
        }
    return lowest;
}

void crushFeet(const Config& cfg, const Ground& ground, const StepInputs& in, const RigidBodyState& s, FootCrush& crush) {
    (void)ground;
    StepInputs now = in;
    now.foot_crush = crush;
    for (int i = 0; i < 4; i++) {
        FootContact f = footContact(cfg, now, s, i);
        if (!f.touching) continue;
        double s_now;
        footForce(f.compression, crush[i], in.foot_radius, cfg.foot_crush_stress, f.stiffness, s_now);
        crush[i] = s_now;
    }
}

// Pushes from the ground on everything that touches it. Each point gets pushed back out along the ground's
// surface normal (which tilts on slopes and bumps), with a damper for the bounciness asked for. Friction fights
// sliding. That gives landings, skids and tipping over. When foot_out is given, the foot forces are saved there.
static void groundForces(const Config& cfg, const StepInputs& in, const RigidBodyState& s, Vec3& force_world,
                         Vec3& torque_body, FootLoads* foot_out) {
    const BodyProps& b = in.body;
    const Ground& ground = *in.ground;

    // How easily a point on the rocket moves when pushed in a (world) direction, counting the
    // rocket turning about its CG. Used to keep the ground forces from overshooting in one time step.
    auto give = [&](const Vec3& arm, Vec3 direction) {
        Vec3 turn = cross(arm, rotateBack(s.attitude, direction));
        return 1 / b.mass + dot(turn, b.inverse_inertia * turn);
    };

    // One contact: push is the springy force along the normal, before damping
    auto apply = [&](const Vec3& arm, const Vec3& out, double push_in, double stiffness, double depth, int foot, bool crushing) {
        Vec3 v = s.velocity + rotate(s.attitude, cross(s.angular_velocity, arm));
        double sinking = dot(v, out);  // negative while going in

        // Pick the damping that gives the bounciness asked for, even if a single point takes the
        // whole rocket. With several points touching it's deader still, which suits grass.
        double loss = -std::log(std::clamp(cfg.bounciness, 0.001, 0.999));
        double damping_ratio = loss / std::sqrt(PI * PI + loss * loss);
        double damping = 2 * damping_ratio * std::sqrt(stiffness * b.mass);

        // The damper stops this point sinking in, but can't fling it back out, so it's capped at the
        // force that would just stop the point within one time step (keeps hard crashes stable)
        double most = 0.2 * std::fabs(sinking) / (give(arm, out) * cfg.physics_dt);
        double damper = crushing ? 0.0 : std::clamp(damping * sinking, -most, most);
        double push = std::max(push_in - damper, 0.0);
        (void)depth;

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
        if (foot_out && foot >= 0) foot_out->force[foot] += rotateBack(s.attitude, f);
    };

    for (int i = 0; i < 4; i++) {
        FootContact f = footContact(cfg, in, s, i);
        if (!f.touching) continue;
        if (foot_out) {
            foot_out->point[i] = in.foot_centre[i];
            foot_out->compression[i] = f.compression;
        }
        apply(f.arm, f.out, f.force, f.stiffness, f.compression, i, f.crushing);
        if (foot_out) foot_out->point[i] = f.arm + b.cg;
    }

    if (!in.body_points || !nearGround(in, s)) return;
    for (const Vec3& p : *in.body_points) {
        Vec3 arm = p - b.cg;
        Vec3 point = s.position + rotate(s.attitude, arm);
        double below = ground.height(point.x, point.y) - point.z;
        if (below <= 0) continue;
        Vec3 out = ground.normal(point.x, point.y);
        double k = ground.stiffness(point.x, point.y);
        apply(arm, out, k * below * out.z, k, below * out.z, -1, false);
    }
}

void footLoads(const Config& cfg, const StepInputs& in, const RigidBodyState& s, FootLoads& out) {
    out = FootLoads{};
    Vec3 force, torque;
    // Only the feet matter here
    StepInputs feet_only = in;
    feet_only.body_points = nullptr;
    if (in.ground_active) groundForces(cfg, feet_only, s, force, torque, &out);
}

static Derivative derivative(const Config& cfg, const StepInputs& in, const RigidBodyState& s) {
    const BodyProps& b = in.body;
    Vec3 force;   // body frame
    Vec3 torque;  // body frame, about the CG

    // Thrust pushes along the gimbal direction, through the gimbal pivot
    Vec3 push = in.thrust_dir * in.thrust + in.side_force;
    force += push;
    torque += cross(in.thrust_point - b.cg, push);
    torque.x += in.roll_torque;
    torque += in.extra_torque;

    // Jet damping: exhaust leaving the nozzle carries some of the rocket's wobble away with it
    double nozzle_arm = b.cg.x - in.nozzle_x;
    torque.y -= in.mass_flow * nozzle_arm * nozzle_arm * s.angular_velocity.y;
    torque.z -= in.mass_flow * nozzle_arm * nozzle_arm * s.angular_velocity.z;

    // Aerodynamics. First, how is the rocket moving through the air, seen from the rocket?
    Vec3 air_velocity = rotateBack(s.attitude, s.velocity - in.wind);
    bool tail_first = air_velocity.x < 0;
    double cd = tail_first ? cfg.cd_tail_first : cfg.cd_nose_first;
    if (cfg.aero_model >= 0.5) {
        // Strip model. Axial drag as before; the sideways force comes from adding up strips along the body:
        //  - slender-body (Munk) force on the nose, rho * u * v_n * dS, which is what makes a nose lift going forward
        //    and (kept at aero_tail_potential) pushes the wrong way going tail-first
        //  - Allen-Perkins crossflow drag, eta * cd_crossflow * width * |v_n| * v_n, on every strip
        // Every strip sees its own sideways speed (the rocket's turning is included), so damping comes out by itself.
        double rho_air = airDensity(s.position.z);
        double area0 = PI * cfg.body_diameter * cfg.body_diameter / 4;
        Vec3 w = s.angular_velocity;
        Vec3 drag = {-0.5 * rho_air * area0 * cd * std::fabs(air_velocity.x) * air_velocity.x, 0, 0};
        force += drag;
        int n = (int)cfg.aero_strips;
        double dx = cfg.body_length / n;
        double Rb = cfg.body_diameter / 2, Ln = cfg.body_length - cfg.aero_nose_base;
        double rho_og = (Rb * Rb + Ln * Ln) / (2 * Rb);
        auto radius = [&](double x) {
            if (x > cfg.aero_nose_base) {
                double xt = cfg.body_length - x;
                return std::max(0.0, std::sqrt(std::max(rho_og * rho_og - (Ln - xt) * (Ln - xt), 0.0)) + Rb - rho_og);
            }
            return x < cfg.aero_body_start ? cfg.aero_low_radius : Rb;
        };
        double kvis = 0.5 * rho_air * cfg.aero_eta * cfg.cd_crossflow;
        Vec3 F_side, M_side;
        for (int i = 0; i < n; i++) {
            double x = (i + 0.5) * dx;
            double r0 = radius(x - dx / 2), r1 = radius(x + dx / 2), r = radius(x);
            double px = x - b.cg.x;
            double vny = air_velocity.y + w.z * px, vnz = air_velocity.z - w.y * px;
            double vnm = std::hypot(vny, vnz);
            double fy = -kvis * 2 * r * dx * vnm * vny, fz = -kvis * 2 * r * dx * vnm * vnz;
            if (x > cfg.aero_nose_base) {
                double dS = PI * (r1 * r1 - r0 * r0);  // negative toward the tip
                double kp = tail_first ? cfg.aero_tail_potential : 1.0;
                fy += rho_air * air_velocity.x * vny * dS * kp;
                fz += rho_air * air_velocity.x * vnz * dS * kp;
            }
            F_side += Vec3{0, fy, fz};
            M_side += Vec3{0, -fz * px, fy * px};   // (px,0,0) x (0,fy,fz)
        }
        // Optional fin / flare sets (A aft, B forward). A fin set gives a lift-type sideways force 0.5 rho A_n |u| v_n
        // (A_n = CN_alpha * S_ref, stalls at about 14 deg) plus a crossflow term on its planform; it opposes the sideways
        // speed whichever way the air flows, so an aft set steadies nose-first flight and upsets tail-first flight.
        const double fin_set[2][3] = {{cfg.aero_finA_area, cfg.aero_finA_x, cfg.aero_finA_cross},
                                      {cfg.aero_finB_area, cfg.aero_finB_x, cfg.aero_finB_cross}};
        for (int k = 0; k < 2; k++) {
            if (fin_set[k][0] <= 0 && fin_set[k][2] <= 0) continue;
            double px = fin_set[k][1] - b.cg.x;
            double vny = air_velocity.y + w.z * px, vnz = air_velocity.z - w.y * px;
            double vnm = std::hypot(vny, vnz);
            double ueff = std::min(std::fabs(air_velocity.x), 4.0 * vnm);
            double kf = 0.5 * rho_air * fin_set[k][0] * ueff;
            double kc = 0.5 * rho_air * 1.2 * fin_set[k][2] * vnm;
            double fy = -(kf + kc) * vny, fz = -(kf + kc) * vnz;
            F_side += Vec3{0, fy, fz};
            M_side += Vec3{0, -fz * px, fy * px};
        }
        force += F_side;
        torque += M_side;
    } else {
    double cp = tail_first ? cfg.cp_tail_first : cfg.cp_nose_first;
    Vec3 cp_arm = Vec3{cp, 0, 0} - b.cg;

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
    Vec3 middle_arm = Vec3{cfg.body_length / 2, 0, 0} - b.cg;
    Vec3 side = air_velocity + cross(s.angular_velocity, middle_arm);
    double side_speed = std::hypot(side.y, side.z);
    double side_k = 0.5 * airDensity(s.position.z) * cfg.cd_crossflow * cfg.body_length * cfg.body_diameter;
    Vec3 crossflow = {0, -side_k * side_speed * side.y, -side_k * side_speed * side.z};
    force += crossflow;
    torque += cross(middle_arm, crossflow);
    }

    force += in.leg_force;
    torque += in.leg_torque;

    Vec3 ground;  // world frame
    if (in.ground_active) groundForces(cfg, in, s, ground, torque, nullptr);

    Derivative d;
    d.velocity = s.velocity;
    d.acceleration = (rotate(s.attitude, force) + ground) / b.mass + Vec3{0, 0, -GRAVITY};

    // Euler's rotation equations, with the full inertia tensor (the CAD rocket isn't perfectly symmetric)
    Vec3 w = s.angular_velocity;
    Vec3 net = torque - cross(w, b.inertia * w);
    d.angular_acceleration = b.inverse_inertia * net;

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
