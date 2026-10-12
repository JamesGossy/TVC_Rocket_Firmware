#pragma once
#include <string>

// Every setting for the rocket and the flight. They all come from rocket.cfg, which explains
// each one. Units are SI (m, kg, s, N) unless the name ends in _deg. Positions along the rocket
// are measured up from the bottom (the aft end of the ascent motor).
struct Config {
    // The rocket from the CAD
    std::string vehicle_file;
    bool vehicle_model;      // 1 = mass properties assembled from the CAD links every step, 0 = the fixed dry_* numbers
    double mechanism_dt;     // sub-step for the gimbal and leg joints (their stops are stiff)

    // Rocket body (the dry_* numbers are what the flight computer assumes; with vehicle_model = 1 they are
    // filled in from the CAD at start-up)
    double body_diameter, body_length, dry_mass, dry_cg, dry_inertia_pitch, dry_inertia_roll;
    double dry_cg_legs_out, dry_inertia_pitch_legs_out, dry_inertia_roll_legs_out;

    // Aerodynamics
    double cd_nose_first, cd_tail_first, cn_alpha, cd_crossflow, cp_nose_first, cp_tail_first;
    double cd_leg, cd_foot;
    // Strip aerodynamics (aero_model = 1): slender-body nose lift + Allen-Perkins crossflow on the body, see tools/aero_strip.py
    double aero_model, aero_eta, aero_body_start, aero_low_radius, aero_nose_base, aero_tail_potential, aero_strips;
    double aero_finA_area, aero_finA_x, aero_finA_cross, aero_finB_area, aero_finB_x, aero_finB_cross;

    // Motors: the landing motor sits above the ascent motor and kicks it out when it lights
    std::string motor_file;
    double motor_position, motor_length, motor_gap, eject_speed;
    double ascent_ignition_delay, landing_ignition_delay, ascent_thrust_scale, landing_thrust_scale;
    bool landing_motor;
    double thrust_roughness, roll_torque;

    // Thrust blocker claws on the landing motor
    double throttle_min;

    // TVC mount
    double gimbal_position, gimbal_misalignment_y_deg, gimbal_misalignment_z_deg;

    // Servos (KST X08H Plus, all five)
    double servo_supply_volts, servo_stall_torque, servo_speed_deg, servo_update_rate, servo_deadband_deg, servo_lag;

    // Hardware imperfections
    double gimbal_wobble_hz, gimbal_wobble_damping, control_delay, cg_offset, eject_kick_deg;
    double blocker_effectiveness, blocker_erosion, blocker_side_force, blocker_servo_offset_r_deg, blocker_servo_offset_l_deg;

    // Backlash in every joint (lost motion from the CAD clearances). backlash_scale multiplies all of them.
    double backlash_scale;
    double backlash_servo_deg, backlash_tvc_horn_end_mm, backlash_tvc_ujoint_mm, backlash_tvc_drive_end_mm;
    double backlash_tvc_yaw_axle_deg, backlash_tvc_pitch_axle_deg;
    double backlash_blocker_horn_pin_mm, backlash_blocker_claw_pin_mm, backlash_claw_pivot_mm;
    double backlash_keeper_slot_mm, backlash_leg_latch_mm;

    // Leg keeper, deploy spring, latch and feet
    double keeper_release_spread_deg;
    double leg_spring_k, leg_spring_torque_deployed, leg_hinge_friction, leg_stop_restitution;
    double plunger_force_start, plunger_force_end, plunger_ramp_friction, plunger_ramp_rise, plunger_shear_limit;
    double foot_crush_stress, tube_modulus;

    // Sensors
    double gyro_noise_deg, gyro_bias_deg, gyro_scale_error, vibration_gyro_deg;
    double accel_noise, accel_bias, vibration_accel, imu_misalignment_deg;
    double baro_noise, baro_lag, baro_airspeed_error;
    double gps_rate, gps_latency, gps_latency_jitter, gps_velocity_noise;
    double gps_dropout, gps_outage_rate, gps_multipath_height, gps_glitch;

    // Flight computer
    bool tvc_enabled;
    double control_rate, launch_time, kp, kd, rate_filter_hz, estimator_gain, gps_velocity_gain;
    double predict_ignition, predict_share, predict_speed;
    double landing_brake, touchdown_speed, max_fire_tilt_deg, drift_kill_time, max_lean_deg;
    double pad_return, pad_return_height, boost_max_lean_deg;
    double leg_release_speed;

    // Launch day
    double launch_tilt_deg, wind_speed, wind_direction_deg, gust_strength, wind_height, wind_shear;

    // The ground
    double ground_stiffness, max_touchdown_speed;
    double bounciness, ground_friction, ground_slope_deg, ground_slope_direction_deg;
    double ground_bumps, ground_bump_size, ground_softness;

    // Simulation
    double physics_dt, max_time;

    // Monte Carlo spreads
    double mc_dry_mass_percent, mc_dry_cg, mc_thrust_percent, mc_ignition_delay, mc_misalignment_deg;
    double mc_launch_tilt_deg, mc_max_wind, mc_blocker_effect, mc_baro_airspeed, mc_max_slope_deg;
    double mc_backlash_spread, mc_servo_offset_deg, mc_leg_spring_percent, mc_foot_crush_percent, mc_plunger_limit_percent;
};

Config loadConfig(const std::string& path);
