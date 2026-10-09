#pragma once
#include <string>

// Every setting for the rocket and the flight. They all come from rocket.cfg, which explains
// each one. Units are SI (m, kg, s, N) unless the name ends in _deg. Positions along the rocket
// are measured up from the bottom.
struct Config {
    // Rocket body
    double body_diameter, body_length, dry_mass, dry_cg, dry_inertia_pitch, dry_inertia_roll;

    // Aerodynamics
    double cd_nose_first, cd_tail_first, cn_alpha, cd_crossflow, cp_nose_first, cp_tail_first;

    // Motors: the landing motor sits on top of the ascent motor and kicks it out when it lights
    std::string motor_file;
    double motor_position, motor_length, eject_speed;
    double ascent_ignition_delay, landing_ignition_delay, ascent_thrust_scale, landing_thrust_scale;
    bool landing_motor;
    double thrust_roughness, roll_torque;

    // Thrust blocking paddles on the landing motor
    double throttle_min, throttle_rate;

    // TVC mount
    double gimbal_position, gimbal_max_deg, servo_rate_deg, servo_update_rate, servo_deadband_deg;
    double gimbal_backlash_deg, gimbal_misalignment_y_deg, gimbal_misalignment_z_deg;

    // Hardware imperfections
    double servo_lag, gimbal_wobble_hz, gimbal_wobble_damping, control_delay, cg_offset, eject_kick_deg;
    double paddle_effectiveness, paddle_erosion, paddle_side_force;

    // Sensors
    double gyro_noise_deg, gyro_bias_deg, gyro_scale_error, vibration_gyro_deg;
    double accel_noise, accel_bias, vibration_accel, imu_misalignment_deg;
    double baro_noise, baro_lag, baro_airspeed_error;
    double gps_rate, gps_latency, gps_latency_jitter, gps_velocity_noise;
    double gps_dropout, gps_outage_rate, gps_multipath_height, gps_glitch;

    // Flight computer
    bool tvc_enabled;
    double control_rate, launch_time, kp, kd, estimator_gain, gps_velocity_gain;
    double landing_brake, touchdown_speed, max_fire_tilt_deg, drift_kill_time, max_lean_deg;

    // Launch day
    double launch_tilt_deg, wind_speed, wind_direction_deg, gust_strength, wind_height, wind_shear;

    // Landing legs and the ground
    double leg_span, leg_drop, leg_crush_force, leg_stroke, leg_stiffness, max_touchdown_speed;
    double bounciness, ground_friction, ground_slope_deg, ground_slope_direction_deg;
    double ground_bumps, ground_bump_size, ground_softness;

    // Simulation
    double physics_dt, max_time;

    // Monte Carlo spreads
    double mc_dry_mass_percent, mc_dry_cg, mc_thrust_percent, mc_ignition_delay, mc_misalignment_deg;
    double mc_launch_tilt_deg, mc_max_wind, mc_paddle_effect, mc_baro_airspeed, mc_max_slope_deg;
};

Config loadConfig(const std::string& path);
