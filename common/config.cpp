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
    READ(body_diameter); READ(body_length); READ(dry_mass); READ(dry_cg); READ(dry_inertia_pitch); READ(dry_inertia_roll);
    READ(cd_nose_first); READ(cd_tail_first); READ(cn_alpha); READ(cd_crossflow); READ(cp_nose_first); READ(cp_tail_first);

    c.motor_file = take("motor_file");
    READ(motor_position); READ(motor_length); READ(eject_speed);
    READ(ascent_ignition_delay); READ(landing_ignition_delay); READ(ascent_thrust_scale); READ(landing_thrust_scale);
    READ(landing_motor); READ(thrust_roughness); READ(roll_torque);
    READ(throttle_min); READ(throttle_rate);

    READ(gimbal_position); READ(gimbal_max_deg); READ(servo_rate_deg); READ(servo_update_rate); READ(servo_deadband_deg);
    READ(gimbal_backlash_deg); READ(gimbal_misalignment_y_deg); READ(gimbal_misalignment_z_deg);

    READ(servo_lag); READ(gimbal_wobble_hz); READ(gimbal_wobble_damping); READ(control_delay); READ(cg_offset);
    READ(eject_kick_deg); READ(paddle_effectiveness); READ(paddle_erosion); READ(paddle_side_force);

    READ(gyro_noise_deg); READ(gyro_bias_deg); READ(gyro_scale_error); READ(vibration_gyro_deg);
    READ(accel_noise); READ(accel_bias); READ(vibration_accel); READ(imu_misalignment_deg);
    READ(baro_noise); READ(baro_lag); READ(baro_airspeed_error);
    READ(gps_rate); READ(gps_latency); READ(gps_latency_jitter); READ(gps_velocity_noise);
    READ(gps_dropout); READ(gps_outage_rate); READ(gps_multipath_height); READ(gps_glitch);

    READ(tvc_enabled); READ(control_rate); READ(launch_time); READ(kp); READ(kd); READ(estimator_gain);
    READ(gps_velocity_gain); READ(landing_brake); READ(touchdown_speed); READ(max_fire_tilt_deg);
    READ(drift_kill_time); READ(max_lean_deg); READ(pad_return);
    READ(pad_return_height); READ(boost_max_lean_deg);

    READ(launch_tilt_deg); READ(wind_speed); READ(wind_direction_deg); READ(gust_strength); READ(wind_height); READ(wind_shear);

    READ(leg_span); READ(leg_drop); READ(leg_crush_force); READ(leg_stroke); READ(leg_stiffness); READ(max_touchdown_speed);
    READ(bounciness); READ(ground_friction); READ(ground_slope_deg); READ(ground_slope_direction_deg);
    READ(ground_bumps); READ(ground_bump_size); READ(ground_softness);

    READ(physics_dt); READ(max_time);

    READ(mc_dry_mass_percent); READ(mc_dry_cg); READ(mc_thrust_percent); READ(mc_ignition_delay); READ(mc_misalignment_deg);
    READ(mc_launch_tilt_deg); READ(mc_max_wind); READ(mc_paddle_effect); READ(mc_baro_airspeed); READ(mc_max_slope_deg);
    #undef READ

    // Anything left over is probably a typo
    for (auto& [name, value] : values)
        std::cerr << "Warning: unknown setting '" << name << "' in " << path << "\n";
    if (missing) std::exit(1);
    return c;
}
