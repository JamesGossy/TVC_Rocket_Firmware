#pragma once
#include <array>
#include <random>
#include <vector>
#include "config.h"
#include "math3d.h"
#include "vehicle.h"

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

// How far each landing leg's damper has stroked. They ratchet, so this only ever grows.
using LegTravel = std::array<double, 4>;

// Everything that stays the same during one physics step
struct StepInputs {
    MassProps mass_props;
    double thrust = 0;       // both motors, pushing through the gimbal pivot
    double mass_flow = 0;    // kg/s of exhaust leaving the nozzle
    double roll_torque = 0;  // spin from the motor and small asymmetries
    Vec3 extra_torque;       // body frame, e.g. the swinging gimbal twisting the rocket back
    Vec3 cg_shift;           // body frame, CG sitting off the centerline
    Vec3 side_force;         // body frame, pushed sideways at the nozzle (uneven paddles)
    double gimbal_y = 0;     // radians
    double gimbal_z = 0;
    Vec3 wind;
    bool legs_out = false;   // landing legs deployed, so they can touch the ground
    LegTravel leg_travel{};
    const Ground* ground = nullptr;
};

double lowestPoint(const Config& cfg, const Ground& ground, const MassProps& mp, const RigidBodyState& s,
                   const LegTravel& travel);
void strokeLegs(const Config& cfg, const Ground& ground, const MassProps& mp, const RigidBodyState& s,
                LegTravel& travel);
RigidBodyState rk4Step(const Config& cfg, const StepInputs& in, const RigidBodyState& s, double dt);
