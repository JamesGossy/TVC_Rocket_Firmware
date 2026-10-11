#include "config.h"
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>

Config loadConfig(const std::string& path) {
    std::ifstream file(path);
    if (!file) {
        std::cerr << "Could not open " << path << "\n";
        std::exit(1);
    }

    // Read "name = value" lines. Anything after a # is a comment. If a name shows up twice,
    // the last one wins, so a test can add overrides to the end of the file.
    std::map<std::string, std::string> values;
    std::string line;
    while (std::getline(file, line)) {
        line = line.substr(0, line.find('#'));
        size_t equals = line.find('=');
        if (equals == std::string::npos) continue;

        std::string name, value;
        std::stringstream(line.substr(0, equals)) >> name;
        std::stringstream(line.substr(equals + 1)) >> value;
        values[name] = value;
    }

    // Take each setting out of the file. Every one has to be there.
    bool missing = false;
    auto take = [&](const std::string& name) {
        if (!values.count(name)) {
            std::cerr << "Missing setting '" << name << "' in " << path << "\n";
            missing = true;
            return std::string("0");
        }
        std::string value = values[name];
        values.erase(name);
        return value;
    };

    Config c;
    #define READ(name) c.name = std::stod(take(#name))
    c.vehicle_file = take("vehicle_file");
    READ(vehicle_model); READ(mechanism_dt);

    READ(body_diameter); READ(body_length); READ(dry_mass); READ(dry_cg); READ(dry_inertia_pitch); READ(dry_inertia_roll);
    READ(dry_cg_legs_out); READ(dry_inertia_pitch_legs_out); READ(dry_inertia_roll_legs_out);
    READ(cd_nose_first); READ(cd_tail_first); READ(cn_alpha); READ(cd_crossflow); READ(cp_nose_first); READ(cp_tail_first);
    READ(cd_leg); READ(cd_foot);

    c.motor_file = take("motor_file");
    READ(motor_position); READ(motor_length); READ(motor_gap); READ(eject_speed);
    READ(ascent_ignition_delay); READ(landing_ignition_delay); READ(ascent_thrust_scale); READ(landing_thrust_scale);
    READ(landing_motor); READ(thrust_roughness); READ(roll_torque);
    READ(throttle_min);

    READ(gimbal_position); READ(gimbal_misalignment_y_deg); READ(gimbal_misalignment_z_deg);

    READ(servo_supply_volts); READ(servo_stall_torque); READ(servo_speed_deg); READ(servo_update_rate);
    READ(servo_deadband_deg); READ(servo_lag);

    READ(gimbal_wobble_hz); READ(gimbal_wobble_damping); READ(control_delay); READ(cg_offset); READ(eject_kick_deg);
    READ(blocker_effectiveness); READ(blocker_erosion); READ(blocker_side_force);
    READ(blocker_servo_offset_r_deg); READ(blocker_servo_offset_l_deg);

    READ(backlash_scale);
    READ(backlash_servo_deg); READ(backlash_tvc_horn_end_mm); READ(backlash_tvc_ujoint_mm); READ(backlash_tvc_drive_end_mm);
    READ(backlash_tvc_yaw_axle_deg); READ(backlash_tvc_pitch_axle_deg);
    READ(backlash_blocker_horn_pin_mm); READ(backlash_blocker_claw_pin_mm); READ(backlash_claw_pivot_mm);
    READ(backlash_keeper_slot_mm); READ(backlash_leg_latch_mm);

    READ(keeper_release_spread_deg);
    READ(leg_spring_k); READ(leg_spring_torque_deployed); READ(leg_hinge_friction); READ(leg_stop_restitution);
    READ(plunger_force_start); READ(plunger_force_end); READ(plunger_ramp_friction); READ(plunger_ramp_rise);
    READ(plunger_shear_limit);
    READ(foot_crush_stress); READ(tube_modulus);

    READ(gyro_noise_deg); READ(gyro_bias_deg); READ(gyro_scale_error); READ(vibration_gyro_deg);
    READ(accel_noise); READ(accel_bias); READ(vibration_accel); READ(imu_misalignment_deg);
    READ(baro_noise); READ(baro_lag); READ(baro_airspeed_error);
    READ(gps_rate); READ(gps_latency); READ(gps_latency_jitter); READ(gps_velocity_noise);
    READ(gps_dropout); READ(gps_outage_rate); READ(gps_multipath_height); READ(gps_glitch);

    READ(tvc_enabled); READ(control_rate); READ(launch_time); READ(kp); READ(kd); READ(estimator_gain);
    READ(gps_velocity_gain); READ(landing_brake); READ(touchdown_speed); READ(max_fire_tilt_deg);
    READ(drift_kill_time); READ(max_lean_deg); READ(pad_return);
    READ(pad_return_height); READ(boost_max_lean_deg); READ(leg_release_speed);

    READ(launch_tilt_deg); READ(wind_speed); READ(wind_direction_deg); READ(gust_strength); READ(wind_height); READ(wind_shear);

    READ(ground_stiffness); READ(max_touchdown_speed);
    READ(bounciness); READ(ground_friction); READ(ground_slope_deg); READ(ground_slope_direction_deg);
    READ(ground_bumps); READ(ground_bump_size); READ(ground_softness);

    READ(physics_dt); READ(max_time);

    READ(mc_dry_mass_percent); READ(mc_dry_cg); READ(mc_thrust_percent); READ(mc_ignition_delay); READ(mc_misalignment_deg);
    READ(mc_launch_tilt_deg); READ(mc_max_wind); READ(mc_blocker_effect); READ(mc_baro_airspeed); READ(mc_max_slope_deg);
    READ(mc_backlash_spread); READ(mc_servo_offset_deg); READ(mc_leg_spring_percent); READ(mc_foot_crush_percent);
    #undef READ

    // Anything left over is probably a typo
    for (auto& [name, value] : values)
        std::cerr << "Warning: unknown setting '" << name << "' in " << path << "\n";
    if (missing) std::exit(1);
    return c;
}
