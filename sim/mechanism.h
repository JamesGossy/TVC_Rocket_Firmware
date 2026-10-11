#pragma once
#include <array>
#include <random>
#include <vector>
#include "config.h"
#include "math3d.h"
#include "vehicle_data.h"

// The moving parts of the rocket, driven by the real CAD geometry: five servos, the TVC pushrods and gimbal
// rings, the blocker claws, the leg keeper ring and the four landing legs. Every joint has the lost motion
// (backlash) the CAD clearances give it.
//
// Frames: the body frame of vehicle.json (x out the nose, measured up from the aft end of the ascent motor).

// Mass, CG (body frame) and inertia about that CG
struct BodyProps {
    double mass = 0;
    Vec3 cg;
    Mat3 inertia, inverse_inertia;
};

// A joint with lost motion: the output follows the input but lags by half the width on whichever side
// it last moved
struct DeadZone {
    double state = 0;
    double apply(double input, double width) {
        double half = width / 2;
        if (input - state > half) state = input - half;
        else if (state - input > half) state = input + half;
        return state;
    }
};

// One KST X08H Plus. It takes a new position every 20 ms (analog-style refresh), ignores tiny changes, answers
// with a short lag at a top speed that drops as the load torque rises, and its output gear train has slop.
struct Servo {
    double lo = -1e9, hi = 1e9;  // travel limits, radians
    double goal = 0;             // the last position it accepted
    double arm = 0;              // where the motor has got to
    double angle = 0;            // the horn, after the gear slop. This is what the linkage sees
    double next_update = 0;
    double offset = 0;           // horn spline clocking error added to every command (random per rocket)
    double load = 0;             // torque the linkage asks of it right now, N m
    DeadZone gear;
    bool stalled = false;

    void start(double position, double low, double high) {
        goal = arm = angle = gear.state = position;
        lo = low;
        hi = high;
    }
    void update(double target, double t, double dt, const Config& c, double backlash);
};

// Where each link is and how it is turned
struct Pose {
    Mat3 R;
    Vec3 t;  // link origin in the body frame
};

struct Leg {
    double angle = 0, rate = 0;   // radians from stowed, positive = swinging out. Rate in rad/s
    double release_at = 0;        // keeper ring angle that frees this leg's peg
    bool released = false;
    bool latched = false;         // the spring plunger has dropped into its hole
    bool failed = false;          // the plunger hole or pin gave way
    double plunger_load = 0;      // N on the plunger pin now
    double peak_plunger_load = 0;
    double deploy_time = -1;      // when it first reached the stop
    double inertia = 0;           // about the hinge axis
};

// What the rest of the rocket is doing, for the loads on the joints
struct MechanismMotion {
    Vec3 g_minus_a;       // gravity minus the CG's acceleration, body frame. Zero in free fall
    Vec3 omega, alpha;    // the rocket's angular velocity and acceleration, body frame
    Vec3 air;             // the CG's velocity through the air, body frame
    double air_density = 1.2;
    double thrust_scale = 1;           // 1 normally
    bool landing_burning = false;
    double landing_burned = 0;         // share of the landing propellant gone (the blocker erodes)
    std::array<Vec3, 4> foot_force{};  // what the ground pushes each foot with, body frame, N
    std::array<Vec3, 4> foot_arm{};    // where, body frame
};

// The commands a flight computer can send: horn angles in degrees
struct ServoTargets {
    double tvc_yaw = 0, tvc_pitch = 0, blocker_r = -80, blocker_l = -80, keeper = 0;
};

class Mechanism {
public:
    // truth = this particular rocket, nominal = what the flight computer assumes (the mass scale and CG shift
    // of a random rocket are the difference between them)
    Mechanism(const VehicleData& vehicle, const Config& truth, const Config& nominal, std::mt19937& rng);

    void update(double t, double dt, const ServoTargets& targets, const MechanismMotion& motion);

    // The motors (the composite mass depends on how much is left in them)
    void setMotors(bool ascent_present, double ascent_burned, bool landing_present, double landing_burned);
    // Mass properties right now (cached, refreshed every few steps or when something big changes)
    const BodyProps& props() const { return props_; }
    void refreshProps();

    // Which way the thrust points and where it acts
    Vec3 thrustDirection() const;
    Vec3 thrustPoint() const { return vehicle.tvc.pivot; }
    Vec3 gimbalReactionTorque() const { return reaction_torque; }

    // Blocker: share of the full-close blocking, and the sideways push and twist from an uneven pair
    double blockedShare() const { return blocked_share; }
    Vec3 blockerForceTorque(double lost_thrust, Vec3& force) const;

    // Legs
    Vec3 footCentre(int i) const;                       // body frame
    void tubeSegment(int i, Vec3& a, Vec3& b) const;    // hinge end and tip end
    Vec3 legAeroForce(int i, const Vec3& cg, const Vec3& velocity_rel_air, const Vec3& omega, double rho,
                      Vec3& torque_about_cg) const;     // drag on one leg for the body equations

    // Open state, for logging and for checks
    const VehicleData& vehicle;
    Servo servo_tvc[2], servo_blocker[2], servo_keeper;
    double gimbal[2] = {0, 0};        // yaw (about body z), pitch (about body y), radians
    double gimbal_rate[2] = {0, 0};
    double gimbal_target[2] = {0, 0}; // from the kinematics, before slop and flex
    double gimbal_link[2] = {0, 0};   // after the lost motion in the rods and axles, before the flex
    double ring_angle = 0;            // leg keeper ring, radians (negative = released)
    double claw[2] = {0, 0};          // R, L, radians (negative = open)
    double blocked_share_side[2] = {0, 0};
    Leg leg[4];
    bool tvc_at_stop = false;
    bool keeper_stalled = false;

    // Backlash widths actually in use (after the global scale)
    double bl_servo = 0, bl_rod = 0, bl_axle[2] = {0, 0}, bl_blocker = 0, bl_keeper = 0, bl_latch = 0;

    // Forward kinematics of the TVC linkage: horn angles (radians) -> gimbal yaw and pitch (radians).
    // Public so the tests can compare it with the CAD command map.
    bool tvcForward(double q_yaw, double q_pitch, double& yaw, double& pitch, double rod_extra = 0) const;
    // Blocker linkage: horn angle (absolute, radians) -> claw angle (radians). link_extra changes the link length.
    double clawFromServo(int side, double q, double link_extra = 0) const;
    // Keeper: horn angle -> ring angle
    double ringFromHorn(double alpha) const;

    const Pose& pose(int link) const { return pose_[link]; }

private:
    const Config& truth;
    void computePoses();
    void updateLegs(double t, double dt, const MechanismMotion& m);
    void updateTvc(double t, double dt, const MechanismMotion& m);
    MassItem motorItem(const MotorData& m, double burned) const;

    std::vector<Pose> pose_;
    std::vector<double> q_;             // joint angle of each link, relative to the neutral pose
    BodyProps props_;
    Mat3 gimbal_inertia = zeroMatrix(); // gimbal subtree about the pivot
    Vec3 reaction_torque;
    double blocked_share = 0;
    double mass_scale = 1, cg_shift = 0;
    bool ascent_present = true, landing_present = true;
    double ascent_burned = 0, landing_burned_ = 0;
    int steps_since_props = 0;
    bool props_dirty = true;

    // TVC
    DeadZone link_dead[2];
    mutable double tvc_warm[2] = {0, 0};     // last solution, the next solve starts here
    mutable double tvc_gain[2] = {0.2, 0.2}; // gimbal angle per horn angle (from the kinematics), for the servo load
    mutable double tvc_lever[2] = {1, 1};    // rod length change per gimbal angle, for the backlash width
    double wobble_k[2] = {0, 0}, wobble_c[2] = {0, 0};
    double gimbal_accel[2] = {0, 0};
    Vec3 gimbal_cg_rel;                  // gimbal subtree CG from the pivot
    double gimbal_mass_ = 0;
    double yaw_misalign = 0, pitch_misalign = 0;
    double tvc_q_last[2] = {1e9, 1e9};                 // horn angles the linkage was last solved for
    double claw_q_last[2] = {1e9, 1e9};
    double claw_solution[2][3] = {{0, 0, 0}, {0, 0, 0}};  // claw angle at the nominal, longest and shortest link

    // Blocker
    DeadZone claw_dead[2];
    double claw_area_closed[2] = {1, 1};
    double bl_claw_link = 0;
    // Keeper
    DeadZone ring_dead;
    double keeper_c = 0.028;
    // Legs
    double spring_free_angle = 0;
    Vec3 leg_cg_rel[4];                  // CG from the hinge, link frame (stowed)
    Vec3 foot_rel[4];
    double leg_axis_inertia[4] = {0, 0, 0, 0};

    double interp(const std::vector<double>& xs, const std::vector<double>& ys, double x) const;
};

// Mass properties of the CAD rocket in its neutral pose (no motors), with the legs stowed or fully out.
// This is what the flight computer is told the rocket weighs.
BodyProps neutralProps(const VehicleData& vehicle, bool legs_out, const Config& cfg);
