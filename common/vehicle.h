#pragma once
#include "config.h"

// What the flight computer believes the rocket weighs. The sim itself assembles the real mass, CG and
// inertia from the CAD links every few steps (sim/mechanism.cpp). This is the quick version for the
// steering and braking math: the dry rocket plus the motors as point masses on the centerline.
struct MassProps {
    double mass = 0;
    double cg = 0;             // up from the aft end of the ascent motor
    double inertia_pitch = 0;  // about the CG (yaw is the same)
    double inertia_roll = 0;
};

// The landing motor sits above the ascent motor with a small gap between them
inline double landingMotorPosition(const Config& cfg) { return cfg.motor_position + cfg.motor_length + cfg.motor_gap; }

// Combine the dry rocket and both motors into one mass, CG and inertia.
// The motors are treated as point masses on the centerline, close enough for small motors.
// With legs_out the dry numbers are the ones with the four legs swung out.
inline MassProps massProperties(const Config& cfg, double ascent_motor_mass, double landing_motor_mass,
                                bool legs_out = false) {
    MassProps p;
    double dry_cg = legs_out ? cfg.dry_cg_legs_out : cfg.dry_cg;
    double dry_pitch = legs_out ? cfg.dry_inertia_pitch_legs_out : cfg.dry_inertia_pitch;
    double dry_roll = legs_out ? cfg.dry_inertia_roll_legs_out : cfg.dry_inertia_roll;
    double ascent_at = cfg.motor_position;
    double landing_at = landingMotorPosition(cfg);
    p.mass = cfg.dry_mass + ascent_motor_mass + landing_motor_mass;
    p.cg = (cfg.dry_mass * dry_cg + ascent_motor_mass * ascent_at + landing_motor_mass * landing_at) / p.mass;

    // Parallel axis theorem: shift each part's inertia over to the combined CG
    double dry_arm = dry_cg - p.cg;
    double ascent_arm = ascent_at - p.cg;
    double landing_arm = landing_at - p.cg;
    p.inertia_pitch = dry_pitch + cfg.dry_mass * dry_arm * dry_arm +
                      ascent_motor_mass * ascent_arm * ascent_arm + landing_motor_mass * landing_arm * landing_arm;
    p.inertia_roll = dry_roll;
    return p;
}
