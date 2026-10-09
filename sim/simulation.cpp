#include "simulation.h"
#include <cmath>
#include <fstream>
#include <random>
#include "flight_computer.h"
#include "hardware.h"
#include "physics.h"
#include "vehicle.h"

// Heights of the field around the pad, so the viewer can draw it
static void writeTerrain(const Ground& ground, const std::string& path) {
    std::ofstream out(path);
    double size = 12, step = 0.25;
    out << "size,step,slope_x,slope_y\n" << size << "," << step << "," << ground.slope_x << "," << ground.slope_y << "\n";
    for (double y = -size; y <= size + 1e-9; y += step) {
        for (double x = -size; x <= size + 1e-9; x += step)
            out << ground.height(x, y) << (x + step <= size + 1e-9 ? "," : "\n");
    }
}

FlightResult runFlight(const Config& truth, const Config& nominal, const ThrustCurve& curve,
                       unsigned seed, const std::string& log_path) {
    std::mt19937 rng(seed);
    Sensors sensors(truth, rng);
    Wind wind(truth);
    Ground ground(truth, rng);
    FlightComputer computer(nominal, curve);
    Servo servo_y, servo_z;
    Paddles paddles;
    ThrustNoise flutter;

    // Which way things are off on this particular rocket: random directions across it
    std::normal_distribution<double> gauss(0.0, 1.0);
    auto sideways = [&]() { return normalize(Vec3{0, gauss(rng), gauss(rng)}); };
    Vec3 cg_shift = sideways() * truth.cg_offset;
    Vec3 paddle_push = sideways();
    Vec3 kick_direction = sideways();

    Motor ascent{&curve, truth.ascent_thrust_scale};
    Motor landing{&curve, truth.landing_thrust_scale};
    double exhaust_speed = curve.totalImpulse() / curve.propellantMass();  // thrust / exhaust_speed = kg/s burned

    // The spent ascent motor, once the landing motor has kicked it out
    EjectedCase spent;
    double case_mass = curve.totalMass() - curve.propellantMass();
    spent.drag = 0.5 * 1.225 * 1.0 * 0.029 * truth.motor_length / case_mass;

    // How heavy each motor is right now. Some test flights don't carry a landing motor.
    auto ascentMass = [&](double t) { return spent.out ? 0.0 : ascent.mass(t); };
    auto landingMass = [&](double t) { return truth.landing_motor ? landing.mass(t) : 0.0; };

    // Start on the pad, nose pointing at the sky, maybe tipped a little
    MassProps start = massProperties(truth, ascentMass(0), landingMass(0));
    RigidBodyState state;
    state.position = {0, 0, start.cg};
    state.attitude = fromAxisAngle({1, 0, 0}, truth.launch_tilt_deg * DEG) * rotationBetween({1, 0, 0}, {0, 0, 1});

    // The landing motor lighting kicks the spent ascent motor out the bottom. Pushing it away
    // nudges the rocket up a little (momentum has to balance). It never leaves perfectly
    // straight, and the sideways part pushes the bottom of the rocket the other way, twisting it.
    auto kickOutSpentCase = [&](double t) {
        MassProps before = massProperties(truth, ascent.mass(t), landingMass(t));
        MassProps after = massProperties(truth, 0, landingMass(t));
        double off_axis = std::sin(truth.eject_kick_deg * DEG);
        Vec3 shove = (Vec3{-1, 0, 0} + kick_direction * off_axis) * truth.eject_speed;  // body frame
        Vec3 recoil = shove * -case_mass;

        spent.out = true;
        spent.position = state.position + rotate(state.attitude, {truth.motor_position - before.cg, 0, 0});
        spent.velocity = state.velocity + rotate(state.attitude, shove);
        state.velocity += rotate(state.attitude, recoil) / after.mass;
        Vec3 twist = cross({truth.motor_position - after.cg, 0, 0}, recoil);
        state.angular_velocity += Vec3{0, twist.y, twist.z} / after.inertia_pitch;
    };

    std::ofstream log;
    if (!log_path.empty()) {
        writeTerrain(ground, log_path.substr(0, log_path.find_last_of("/\\") + 1) + "terrain.csv");
        log.open(log_path);
        log << "time,state,x,y,altitude,vx,vy,vz,tilt_deg,gimbal_y_deg,gimbal_z_deg,thrust,throttle,mass,cg,"
               "qw,qx,qy,qz,roll_rate_deg,wind_x,wind_y,est_altitude,est_vertical_speed,est_vx,est_vy,est_tilt_deg,"
               "case_out,case_x,case_y,case_z\n";
    }
    auto writeLog = [&](double t, const MassProps& mp) {
        if (!log.is_open()) return;
        Vec3 w = wind.velocity(state.position.z);
        log << t << "," << stateName(computer.state) << ","
            << state.position.x << "," << state.position.y << "," << state.position.z - start.cg << ","
            << state.velocity.x << "," << state.velocity.y << "," << state.velocity.z << ","
            << tiltDegrees(state.attitude) << "," << servo_y.angle / DEG << "," << servo_z.angle / DEG << ","
            << ascent.thrust(t) + landing.thrust(t) * paddles.throttle << "," << paddles.throttle << ","
            << mp.mass << "," << mp.cg << ","
            << state.attitude.w << "," << state.attitude.x << "," << state.attitude.y << "," << state.attitude.z << ","
            << state.angular_velocity.x / DEG << "," << w.x << "," << w.y << ","
            << computer.altitude << "," << computer.velocity.z << "," << computer.velocity.x << ","
            << computer.velocity.y << "," << tiltDegrees(computer.attitude) << ","
            << spent.out + spent.down << "," << spent.position.x << "," << spent.position.y << "," << spent.position.z << "\n";
    };

    FlightResult result;
    double dt = truth.physics_dt;
    int steps_per_control = std::max(1, (int)std::round(1.0 / (truth.control_rate * dt)));
    bool on_pad = true;
    bool falling = false;
    bool touching = false;
    LegTravel legs{};
    double end_time = truth.max_time;
    Commands cmd;       // what the hardware is doing now
    Commands next_cmd;  // what the flight computer just asked for, arriving after control_delay
    double next_cmd_at = -1;
    Vec3 acceleration;  // from the last physics step, the accelerometer needs it

    for (long step = 0; step * dt < end_time; step++) {
        double t = step * dt;
        if (!spent.out && landing.ignition_time >= 0 && t >= landing.ignition_time) kickOutSpentCase(t);
        spent.update(dt, ground);
        MassProps mp = massProperties(truth, ascentMass(t), landingMass(t));

        // The flight computer runs slower than the physics
        if (step % steps_per_control == 0) {
            wind.update(steps_per_control * dt, state.position.z, rng);
            flutter.update(truth.thrust_roughness, steps_per_control * dt, rng);
            bool burning = ascent.thrust(t) + landing.thrust(t) > 1.0;
            double airspeed = length(state.velocity - wind.velocity(state.position.z));
            next_cmd = computer.update(sensors.read(t, state, acceleration, burning, airspeed));
            next_cmd_at = t + truth.control_delay;

            if (next_cmd.fire_ascent) ascent.ignition_time = t + truth.ascent_ignition_delay;
            if (next_cmd.fire_landing) {
                landing.ignition_time = t + truth.landing_ignition_delay;
                result.landing_fire_time = t;
                result.landing_fire_altitude = state.position.z - start.cg;
                result.landing_fire_speed = -state.velocity.z;
                result.landing_fire_tilt_deg = tiltDegrees(state.attitude);
            }
            writeLog(t, mp);
        }
        if (next_cmd_at >= 0 && t >= next_cmd_at) {
            cmd = next_cmd;
            next_cmd_at = -1;
        }

        servo_y.update(cmd.gimbal_y, t, dt, truth);
        servo_z.update(cmd.gimbal_z, t, dt, truth);
        paddles.update(cmd.throttle, truth, dt);

        // Paddles never block exactly as planned, they erode as the motor burns, and they're never
        // perfectly even, so some of what they block gets pushed sideways
        double burned = (curve.totalMass() - landing.mass(t)) / curve.propellantMass();
        double blocked = (1 - paddles.throttle) * truth.paddle_effectiveness * (1 - truth.paddle_erosion * burned);
        double ascent_thrust = ascent.thrust(t) * (1 + flutter.value);
        double landing_thrust = landing.thrust(t) * (1 + flutter.value) * (1 - blocked);

        // Swinging the motors in the gimbal twists the rocket the other way
        double ascent_arm = truth.motor_position - truth.gimbal_position;
        double landing_arm = landingMotorPosition(truth) - truth.gimbal_position;
        double motor_len = truth.motor_length;
        double swinging = ascentMass(t) * (ascent_arm * ascent_arm + motor_len * motor_len / 12) +
                          landingMass(t) * (landing_arm * landing_arm + motor_len * motor_len / 12);

        StepInputs in;
        in.mass_props = mp;
        in.thrust = ascent_thrust + landing_thrust;
        in.side_force = paddle_push * (landing.thrust(t) * blocked * truth.paddle_side_force);
        in.mass_flow = in.thrust / exhaust_speed;
        in.roll_torque = in.thrust > 0 ? truth.roll_torque : 0;
        in.extra_torque = {0, -swinging * servo_y.accel, -swinging * servo_z.accel};
        in.cg_shift = cg_shift;
        in.gimbal_y = servo_y.angle + truth.gimbal_misalignment_y_deg * DEG;
        in.gimbal_z = servo_z.angle + truth.gimbal_misalignment_z_deg * DEG;
        in.wind = wind.velocity(state.position.z);
        in.legs_out = falling;  // the legs come out on the way down
        in.leg_travel = legs;
        in.ground = &ground;

        // On the pad nothing moves until the motor pushes up harder than gravity pulls down
        if (on_pad) {
            if (in.thrust * rotate(state.attitude, {1, 0, 0}).z < mp.mass * GRAVITY) continue;
            on_pad = false;
        }

        Vec3 old_velocity = state.velocity;
        state = rk4Step(truth, in, state, dt);
        acceleration = (state.velocity - old_velocity) / dt;

        result.apogee = std::max(result.apogee, state.position.z - start.cg);
        if (state.velocity.z < 0 && result.apogee > 1.0) falling = true;
        if (!falling) continue;

        strokeLegs(truth, ground, mp, state, legs);
        double lowest = lowestPoint(truth, ground, mp, state, legs);
        bool was_touching = touching;
        touching = lowest <= 0;

        // A new hit on the ground (the first touchdown, or coming back down after a bounce)
        if (touching && !was_touching) {
            result.hardest_impact = std::max(result.hardest_impact, -state.velocity.z);

            if (!result.touched_down) {
                result.touched_down = true;
                result.touchdown_time = t;
                result.touchdown_vertical_speed = -state.velocity.z;
                result.touchdown_horizontal_speed = std::hypot(state.velocity.x, state.velocity.y);
                result.touchdown_tilt_deg = tiltDegrees(state.attitude);
                result.landing_x = state.position.x;
                result.landing_y = state.position.y;
                result.drift = std::hypot(state.position.x, state.position.y);

                // Keep going so we see the bounce, until it settles and the motor is done
                double burnout = landing.ignition_time >= 0 ? landing.ignition_time + curve.burnTime() : t;
                result.landing_burn_left = std::max(0.0, burnout - t);
                end_time = std::min(truth.max_time, std::max(t + 3.0, burnout + 1.5));
            }
        }
        if (result.touched_down && !touching) result.hop_height = std::max(result.hop_height, lowest);
    }

    for (double travel : legs) result.leg_travel_used = std::max(result.leg_travel_used, travel);
    result.final_tilt_deg = tiltDegrees(state.attitude);
    result.tipped_over = result.final_tilt_deg > 45;
    return result;
}
