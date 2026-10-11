#pragma once
#include <array>
#include <random>
#include <vector>
#include "config.h"
#include "math3d.h"
#include "mechanism.h"

// World frame: x east, y north, z up. Body frame: x points out the nose.
struct RigidBodyState {
    Vec3 position;  // of the CG
    Vec3 velocity;
    Quat attitude;          // body -> world
    Vec3 angular_velocity;  // body frame
};

// The field we land on: a gentle slope, lumps in the grass, and softer and harder patches
class Ground {
public:
    Ground(const Config& cfg, std::mt19937& rng);
    double height(double x, double y) const;  // the launch spot is at 0
    Vec3 normal(double x, double y) const;    // which way is straight out of the ground here
    double stiffness(double x, double y) const;

    double slope_x = 0, slope_y = 0;  // rise per meter going east and north

private:
    // Bumps and soft patches are a few smooth waves running in random directions, added up
    struct Wave {
        double kx, ky, phase, size;
    };
    double waves(const std::vector<Wave>& list, double x, double y) const;

    const Config& cfg;
    std::vector<Wave> bumps, soft;
    double bump_at_launch = 0;
};

// How far each foot's EPS ball has been crushed flat. It never springs back, so this only ever grows.
using FootCrush = std::array<double, 4>;

// Everything that stays the same during one physics step
struct StepInputs {
    BodyProps body;          // mass, CG and inertia now (body frame). The CG already includes the random offset
    double thrust = 0;       // the motor's push, along thrust_dir through thrust_point
    Vec3 thrust_dir{1, 0, 0};
    Vec3 thrust_point;       // the gimbal pivot, body frame
    double nozzle_x = 0;     // where the exhaust leaves (jet damping), body x
    double mass_flow = 0;    // kg/s of exhaust leaving the nozzle
    double roll_torque = 0;  // spin from the motor and small asymmetries
    Vec3 extra_torque;       // body frame, about the CG: the swinging gimbal, an uneven blocker
    Vec3 side_force;         // body frame: the blocker claws pushing the exhaust sideways
    Vec3 leg_force, leg_torque;  // drag on the deployed legs (body frame, torque about the CG)
    Vec3 wind;
    bool ground_active = false;  // on the way down, so things can touch the ground
    std::array<Vec3, 4> foot_centre{};  // ball centres, body frame
    std::array<Vec3, 4> leg_dir{};      // unit vector along each leg tube, hinge to foot
    FootCrush foot_crush{};
    double foot_radius = 0.0125;   // EPS ball
    double tube_stiffness = 1e4;   // N/m: how much the carbon tube bends at its tip (3 E I / L^3)
    const std::vector<Vec3>* body_points = nullptr;  // other things that can touch down: nozzle, rim, lobes, nose
    const Ground* ground = nullptr;
};

// What the ground does to each foot, body frame (for the leg hinge loads)
struct FootLoads {
    std::array<Vec3, 4> force{};
    std::array<Vec3, 4> point{};  // where the force acts: the ball's contact point, body frame
    std::array<double, 4> compression{};
};

double airDensity(double altitude);  // kg/m^3 at a height above the pad
double lowestPoint(const Config& cfg, const Ground& ground, const StepInputs& in, const RigidBodyState& s);
// After each step: any foot ball pressed in further than before gives way and stays flat
void crushFeet(const Config& cfg, const Ground& ground, const StepInputs& in, const RigidBodyState& s, FootCrush& crush);
// The ground forces on the feet right now
void footLoads(const Config& cfg, const StepInputs& in, const RigidBodyState& s, FootLoads& out);
RigidBodyState rk4Step(const Config& cfg, const StepInputs& in, const RigidBodyState& s, double dt);
