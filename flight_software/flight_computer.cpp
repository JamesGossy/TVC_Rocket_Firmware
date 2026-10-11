#include "flight_computer.h"
#include "blocker_keeper_map.h"
#include "tvc_command_map.h"
#include "vehicle.h"

const char* stateName(FlightState state) {
    switch (state) {
        case FlightState::Pad: return "Pad";
        case FlightState::Boost: return "Boost";
        case FlightState::Coast: return "Coast";
        case FlightState::LandingBurn: return "LandingBurn";
    }
    return "?";
}

FlightComputer::FlightComputer(const Config& cfg, const ThrustCurve& curve) : cfg(cfg), curve(curve) {
    average_thrust = curve.totalImpulse() / curve.burnTime();
}

Commands FlightComputer::update(const SensorData& data) {
    double dt = data.time - last_time;
    last_time = data.time;
    Commands cmd;

    if (state == FlightState::Pad) {
        calibrateOnPad(data, cmd);
        return cmd;
    }
    estimate(data, dt);

    switch (state) {
        case FlightState::Boost:
            // Only a small lean: holding over the pad in wind means flying at an angle to the air,
            // which makes the coast tumble worse
            steer(data, padDirection(data, cfg.pad_return, cfg.boost_max_lean_deg), cmd);
            // Burnout: the push along the nose drops off
            if (data.time - state_start > 1.0 && data.accel.x < 5.0) changeState(FlightState::Coast, data.time);
            break;

        case FlightState::Coast:
            // Past the top and starting to fall: let the legs go while the air is still slow enough for their springs
            if (velocity.z < -cfg.leg_release_speed) releaseLegs();
            if (cfg.landing_motor && timeToLight(data)) {
                cmd.fire_landing = true;
                landing_lit_at = data.time + cfg.landing_ignition_delay;
                changeState(FlightState::LandingBurn, data.time);
            }
            break;

        case FlightState::LandingBurn:
            releaseLegs();  // in case the coast was too short to do it
            // Fly toward the pad up high, but arrive with no sideways speed
            steer(data, padDirection(data, cfg.pad_return * std::clamp(altitude / cfg.pad_return_height, 0.0, 1.0),
                                     cfg.max_lean_deg), cmd);
            cmd.throttle = landingThrottle(data);
            break;

        default:
            break;
    }
    servoCommands(cmd);
    return cmd;
}

void FlightComputer::releaseLegs() {
    if (legs_released) return;
    legs_released = true;
    massNumbers();
}

// Turn the wanted gimbal tilt and throttle into servo horn angles with the CAD maps
void FlightComputer::servoCommands(Commands& cmd) const {
    float yaw_servo = 0, pitch_servo = 0;
    tvc_command((float)(cmd.gimbal_z / DEG), (float)(cmd.gimbal_y / DEG), &yaw_servo, &pitch_servo);
    cmd.tvc_yaw_servo_deg = yaw_servo;
    cmd.tvc_pitch_servo_deg = pitch_servo;

    // Full close blocks 1 - throttle_min of the thrust, so the share of full close is the wanted loss over that
    double share = std::clamp((1 - cmd.throttle) / (1 - cfg.throttle_min), 0.0, 1.0);
    cmd.blocker_servo_deg = blocker_servo_for_share((float)share);
    cmd.release_legs = legs_released;
    cmd.keeper_servo_deg = legs_released ? KEEPER_RELEASE_DEG : KEEPER_HOLD_DEG;
}

// Sit still and average the sensors. That gives the gyro bias, which way is up, and the
// barometer reading at ground level. Then light the ascent motor.
void FlightComputer::calibrateOnPad(const SensorData& data, Commands& cmd) {
    gyro_sum += data.gyro;
    accel_sum += data.accel;
    baro_sum += data.baro_altitude;
    samples++;
    if (data.time < cfg.launch_time) return;

    gyro_bias = gyro_sum / samples;
    attitude = rotationBetween(accel_sum / samples, {0, 0, 1});
    ground_altitude = baro_sum / samples;
    cmd.fire_ascent = true;
    changeState(FlightState::Boost, data.time);
}

void FlightComputer::changeState(FlightState next, double time) {
    state = next;
    state_start = time;
    massNumbers();
}

// Rough mass numbers, pretending the burning motor is half used up. The spent ascent motor
// gets kicked out when the landing motor lights. The legs weigh in differently once they are out.
void FlightComputer::massNumbers() {
    double full = curve.totalMass();
    double half = full - curve.propellantMass() / 2;
    MassProps p = (state == FlightState::Boost) ? massProperties(cfg, half, full, legs_released)
                                                : massProperties(cfg, 0, half, legs_released);
    mass = p.mass;
    inertia = p.inertia_pitch;
    lever_arm = p.cg - cfg.gimbal_position;
}

void FlightComputer::estimate(const SensorData& data, double dt) {
    // Attitude: integrate the gyro. Flights are short, so the drift stays small.
    Vec3 rate = data.gyro - gyro_bias;
    attitude = normalize(attitude * fromAxisAngle(rate, length(rate) * dt));

    // Velocity: integrate the accelerometer, which feels every force except gravity
    velocity += (rotate(attitude, data.accel) - Vec3{0, 0, GRAVITY}) * dt;

    // Altitude: integrate the velocity, and gently pull both toward the barometer
    double w = cfg.estimator_gain;
    double error = data.baro_altitude - ground_altitude - altitude;
    altitude += (velocity.z + 2 * w * error) * dt;
    velocity.z += w * w * error * dt;
    position += Vec3{velocity.x, velocity.y, 0} * dt;

    // GPS velocity is good but arrives gps_latency late. Compare each fix with what we thought
    // back then, and nudge today's estimate by part of the difference.
    history.push_back({data.time, velocity});
    while (history.front().time < data.time - 1.0) history.pop_front();
    if (data.gps_new) {
        Vec3 then = history.front().velocity;
        for (const Snapshot& s : history)
            if (s.time <= data.time - cfg.gps_latency) then = s.velocity;
        velocity += (data.gps_velocity - then) * cfg.gps_velocity_gain;
    }
}

// Point the nose at target (a world direction) with PD control on the gimbal
void FlightComputer::steer(const SensorData& data, Vec3 target, Commands& cmd) const {
    if (!cfg.tvc_enabled || data.accel.x < 2.0) return;  // no motor burning, nothing to steer with

    // How far, and around which axis, the nose has to turn to reach the target. Using the real
    // angle (not just its sideways part) keeps this right even when we're tipped way over.
    Vec3 to = rotateBack(attitude, target);
    double angle = std::acos(std::clamp(to.x, -1.0, 1.0));
    Vec3 error = normalize(cross({1, 0, 0}, to)) * angle;
    Vec3 rate = data.gyro - gyro_bias;

    // The angular acceleration we want, then the gimbal angle that gives it
    Vec3 want = error * cfg.kp - rate * cfg.kd;
    double torque_per_radian = average_thrust * lever_arm;
    cmd.gimbal_y = -want.y * inertia / torque_per_radian;
    cmd.gimbal_z = -want.z * inertia / torque_per_radian;
}

// Which way to point while a motor burns: head back toward the pad (return_gain m/s per meter
// away), leaning to reach that sideways speed within drift_kill_time
Vec3 FlightComputer::padDirection(const SensorData& data, double return_gain, double max_lean_deg) const {
    double push = std::max(data.accel.x, 1.0);  // what the motor gives us right now, m/s^2
    Vec3 want = Vec3{-position.x, -position.y, 0} * return_gain;
    Vec3 side = (want - Vec3{velocity.x, velocity.y, 0}) * (1 / cfg.drift_kill_time);
    double most = push * std::sin(max_lean_deg * DEG);
    if (length(side) > most) side = normalize(side) * most;
    return normalize(Vec3{side.x, side.y, std::sqrt(push * push - dot(side, side))});
}

// Light the landing motor once we're falling fast enough that we need our planned braking to stop
// at the ground. Look ahead by the igniter delay, since that's where we'll be when it lights.
bool FlightComputer::timeToLight(const SensorData& data) const {
    if (velocity.z > -2) return false;  // not really falling yet

    double delay = cfg.landing_ignition_delay;
    double height = altitude + velocity.z * delay - 0.5 * GRAVITY * delay * delay;
    double speed = -velocity.z + GRAVITY * delay;
    double braking_needed = speed * speed / (2 * std::max(height, 0.1));
    if (braking_needed < cfg.landing_brake) return false;

    // If the nose will be tipped way over by then, wait for the tumble to swing back, unless
    // waiting would need much harder braking than planned
    Vec3 rate = data.gyro - gyro_bias;
    Quat soon = attitude * fromAxisAngle(rate, length(rate) * delay);
    bool tipped = tiltDegrees(soon) > cfg.max_fire_tilt_deg;
    bool urgent = braking_needed > 1.2 * cfg.landing_brake;
    return !tipped || urgent;
}

// Paddles: follow a braking curve. At each height there's a speed we should be falling at
// (braking evenly at landing_brake, easing to touchdown_speed at the ground). Falling too fast,
// open the paddles. Too slow, close them.
double FlightComputer::landingThrottle(const SensorData& data) const {
    double full_thrust = curve.thrust(data.time - landing_lit_at);
    if (full_thrust <= 0) return 1;  // motor isn't burning yet (as far as we know)

    double height = std::max(altitude, 0.0);
    double target_speed = -std::sqrt(2 * cfg.landing_brake * height) - cfg.touchdown_speed;
    double braking = height > 0.2 ? cfg.landing_brake : 0;
    double wanted_accel = braking + 3.0 * (target_speed - velocity.z);

    // Only the upward part of the thrust slows us down
    double pointing_up = std::max(rotate(attitude, {1, 0, 0}).z, 0.5);
    double wanted_thrust = (wanted_accel + GRAVITY) * mass / pointing_up;
    return std::clamp(wanted_thrust / full_thrust, cfg.throttle_min, 1.0);
}
