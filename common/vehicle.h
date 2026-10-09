#pragma once
#include "config.h"

struct MassProps {
    double mass = 0;
    double cg = 0;             // up from the bottom of the rocket
    double inertia_pitch = 0;  // about the CG (yaw is the same)
    double inertia_roll = 0;
};

// The landing motor sits right on top of the ascent motor
inline double landingMotorPosition(const Config& cfg) { return cfg.motor_position + cfg.motor_length; }

// Combine the dry rocket and both motors into one mass, CG and inertia.
// The motors are treated as point masses on the centerline, close enough for small motors.
inline MassProps massProperties(const Config& cfg, double ascent_motor_mass, double landing_motor_mass) {
    MassProps p;
    double ascent_at = cfg.motor_position;
    double landing_at = landingMotorPosition(cfg);
    p.mass = cfg.dry_mass + ascent_motor_mass + landing_motor_mass;
    p.cg = (cfg.dry_mass * cfg.dry_cg + ascent_motor_mass * ascent_at + landing_motor_mass * landing_at) / p.mass;

    // Parallel axis theorem: shift each part's inertia over to the combined CG
    double dry_arm = cfg.dry_cg - p.cg;
    double ascent_arm = ascent_at - p.cg;
    double landing_arm = landing_at - p.cg;
    p.inertia_pitch = cfg.dry_inertia_pitch + cfg.dry_mass * dry_arm * dry_arm +
                      ascent_motor_mass * ascent_arm * ascent_arm + landing_motor_mass * landing_arm * landing_arm;
    p.inertia_roll = cfg.dry_inertia_roll;
    return p;
}
