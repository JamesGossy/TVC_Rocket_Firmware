#include "simulation.h"
#include <cmath>
#include <fstream>
#include <random>
#include "flight_computer.h"
#include "hardware.h"
#include "mechanism.h"
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

// The carbon tube is 14 mm outside and 12 mm inside. Pushed sideways at the foot it bends like a cantilever:
// stiffness 3 E I / L^3.
static double tubeStiffness(const Config& cfg, const VehicleData& v) {
    double od = v.legs.tube_od, id = od - 2 * 0.001;  // 1 mm wall (CAD: 14 x 12 mm tube)
    double I = PI / 64 * (std::pow(od, 4) - std::pow(id, 4));
    double L = length(v.legs.leg[0].foot_stowed - v.legs.leg[0].hinge_origin);
    return 3 * cfg.tube_modulus * I / (L * L * L);
}

FlightResult runFlight(const Config& truth, const Config& nominal, const VehicleData& vehicle, const ThrustCurve& curve,
                       unsigned seed, const std::string& log_path) {
    std::mt19937 rng(seed);
    Sensors sensors(truth, rng);
    Wind wind(truth);
    Ground ground(truth, rng);
    FlightComputer computer(nominal, curve);
    Mechanism mech(vehicle, truth, nominal, rng);
    ThrustNoise flutter;

    // Which way things are off on this particular rocket: random directions across it
    std::normal_distribution<double> gauss(0.0, 1.0);
    auto sideways = [&]() { return normalize(Vec3{0, gauss(rng), gauss(rng)}); };
    Vec3 cg_shift = sideways() * truth.cg_offset;
    Vec3 kick_direction = sideways();

    Motor ascent{&curve, truth.ascent_thrust_scale};
    Motor landing{&curve, truth.landing_thrust_scale};
    double exhaust_speed = curve.totalImpulse() / curve.propellantMass();  // thrust / exhaust_speed = kg/s burned

    // The spent ascent motor, once the landing motor has kicked it out
    EjectedCase spent;
    double case_mass = vehicle.ascent.casing.mass;
    spent.drag = 0.5 * 1.225 * 1.0 * 0.029 * truth.motor_length / case_mass;

    // Things that can touch the ground besides the feet: the nozzle (the first point), the aft rim, the cheek lobes
    // and the nose tip, all from the CAD
    std::vector<Vec3> body_points;
    body_points.push_back(vehicle.contacts.nozzle_exit_ascent);
    for (const Vec3& p : vehicle.contacts.aft_rim) body_points.push_back(p);
    for (const Vec3& p : vehicle.contacts.cheek_lobes) body_points.push_back(p);
    body_points.push_back(vehicle.contacts.nose_tip);

    auto setMotors = [&](double t) {
        mech.setMotors(!spent.out, ascent.burned(t), truth.landing_motor != 0, landing.burned(t));
    };  // (the loop calls this every step; burned() is cheap enough but thrust() is cached below)
    setMotors(0);
    mech.refreshProps();

    // Start on the pad, nose pointing at the sky, maybe tipped a little
    double start_cg = mech.props().cg.x;
    RigidBodyState state;
    state.position = {0, 0, start_cg};
    state.attitude = fromAxisAngle({1, 0, 0}, truth.launch_tilt_deg * DEG) * rotationBetween({1, 0, 0}, {0, 0, 1});

    // The landing motor lighting kicks the spent ascent motor out the bottom. Pushing it away
    // nudges the rocket up a little (momentum has to balance). It never leaves perfectly
    // straight, and the sideways part pushes the bottom of the rocket the other way, twisting it.
    auto kickOutSpentCase = [&](double t) {
        BodyProps before = mech.props();
        spent.out = true;
        setMotors(t);
        mech.refreshProps();
        const BodyProps& after = mech.props();
        double gone = before.mass - after.mass;
        double off_axis = std::sin(truth.eject_kick_deg * DEG);
        Vec3 shove = (Vec3{-1, 0, 0} + kick_direction * off_axis) * truth.eject_speed;  // body frame
        Vec3 recoil = shove * -gone;

        Vec3 case_at = vehicle.links[vehicle.ascent_motor].origin + vehicle.ascent.loaded.cg;
        spent.position = state.position + rotate(state.attitude, case_at - before.cg);
        spent.velocity = state.velocity + rotate(state.attitude, shove);
        state.velocity += rotate(state.attitude, recoil) / after.mass;
        Vec3 twist = cross(case_at - after.cg, recoil);
        state.angular_velocity += after.inverse_inertia * Vec3{0, twist.y, twist.z};
        body_points[0] = vehicle.contacts.nozzle_exit_landing;
    };

    std::ofstream log;
    if (!log_path.empty()) {
        writeTerrain(ground, log_path.substr(0, log_path.find_last_of("/\\") + 1) + "terrain.csv");
        log.open(log_path);
        log << "time,state,x,y,altitude,vx,vy,vz,tilt_deg,gimbal_y_deg,gimbal_z_deg,thrust,throttle,mass,cg,"
               "qw,qx,qy,qz,roll_rate_deg,wind_x,wind_y,est_altitude,est_vertical_speed,est_vx,est_vy,est_tilt_deg,"
               "case_out,case_x,case_y,case_z,"
               "cmd_tvc_yaw_servo,cmd_tvc_pitch_servo,cmd_blocker_servo,cmd_keeper_servo,"
               "tvc_yaw_servo,tvc_pitch_servo,blocker_r_servo,blocker_l_servo,keeper_servo,"
               "gimbal_target_y_deg,gimbal_target_z_deg,gimbal_lash_y_deg,gimbal_lash_z_deg,"
               "claw_r_deg,claw_l_deg,blocked_share,keeper_ring_deg,"
               "leg1_deg,leg2_deg,leg3_deg,leg4_deg,legs_released,legs_latched,legs_failed,"
               "plunger1_n,plunger2_n,plunger3_n,plunger4_n,foot1_n,foot2_n,foot3_n,foot4_n,"
               "crush1_mm,crush2_mm,crush3_mm,crush4_mm,gimbal_at_stop\n";
    }
    ServoTargets targets;
    FootCrush crush{};
    FootLoads loads;
    auto writeLog = [&](double t, const BodyProps& mp, const Commands& c) {
        if (!log.is_open()) return;
        Vec3 w = wind.velocity(state.position.z);
        double blocked = (1 - truth.throttle_min) * mech.blockedShare() * truth.blocker_effectiveness *
                         (1 - truth.blocker_erosion * landing.burned(t));
        double thrust_share = 1 - blocked;
        int released = 0, latched = 0, failed = 0;
        for (int i = 0; i < 4; i++) {
            released |= mech.leg[i].released << i;
            latched |= mech.leg[i].latched << i;
            failed |= mech.leg[i].failed << i;
        }
        log << t << "," << stateName(computer.state) << ","
            << state.position.x << "," << state.position.y << "," << state.position.z - start_cg << ","
            << state.velocity.x << "," << state.velocity.y << "," << state.velocity.z << ","
            << tiltDegrees(state.attitude) << "," << mech.gimbal[1] / DEG << "," << mech.gimbal[0] / DEG << ","
            << ascent.thrust(t) + landing.thrust(t) * thrust_share << "," << thrust_share << ","
            << mp.mass << "," << mp.cg.x << ","
            << state.attitude.w << "," << state.attitude.x << "," << state.attitude.y << "," << state.attitude.z << ","
            << state.angular_velocity.x / DEG << "," << w.x << "," << w.y << ","
            << computer.altitude << "," << computer.velocity.z << "," << computer.velocity.x << ","
            << computer.velocity.y << "," << tiltDegrees(computer.attitude) << ","
            << spent.out + spent.down << "," << spent.position.x << "," << spent.position.y << "," << spent.position.z << ","
            << c.tvc_yaw_servo_deg << "," << c.tvc_pitch_servo_deg << "," << c.blocker_servo_deg << "," << c.keeper_servo_deg << ","
            << mech.servo_tvc[0].angle / DEG << "," << mech.servo_tvc[1].angle / DEG << ","
            << mech.servo_blocker[0].angle / DEG << "," << mech.servo_blocker[1].angle / DEG << ","
            << mech.servo_keeper.angle / DEG << ","
            << mech.gimbal_target[1] / DEG << "," << mech.gimbal_target[0] / DEG << ","
            << (mech.gimbal_link[1] - mech.gimbal_target[1]) / DEG << "," << (mech.gimbal_link[0] - mech.gimbal_target[0]) / DEG << ","
            << mech.claw[0] / DEG << "," << mech.claw[1] / DEG << "," << mech.blockedShare() << "," << mech.ring_angle / DEG << ","
            << mech.leg[0].angle / DEG << "," << mech.leg[1].angle / DEG << "," << mech.leg[2].angle / DEG << ","
            << mech.leg[3].angle / DEG << "," << released << "," << latched << "," << failed << ","
            << mech.leg[0].plunger_load << "," << mech.leg[1].plunger_load << "," << mech.leg[2].plunger_load << ","
            << mech.leg[3].plunger_load << ","
            << length(loads.force[0]) << "," << length(loads.force[1]) << "," << length(loads.force[2]) << ","
            << length(loads.force[3]) << ","
            << crush[0] * 1e3 << "," << crush[1] * 1e3 << "," << crush[2] * 1e3 << "," << crush[3] * 1e3 << ","
            << mech.tvc_at_stop << "\n";
    };

    FlightResult result;
    double dt = truth.physics_dt;
    int steps_per_control = std::max(1, (int)std::round(1.0 / (truth.control_rate * dt)));
    bool on_pad = true;
    bool falling = false;
    bool touching = false;
    double end_time = truth.max_time;
    Commands cmd;       // what the hardware is doing now
    Commands next_cmd;  // what the flight computer just asked for, arriving after control_delay
    double next_cmd_at = -1;
    Vec3 acceleration;  // from the last physics step, the accelerometer and the leg loads need it
    Vec3 angular_acceleration;
    double tube_k = tubeStiffness(truth, vehicle);

    for (long step = 0; step * dt < end_time; step++) {
        double t = step * dt;
        if (!spent.out && landing.ignition_time >= 0 && t >= landing.ignition_time) kickOutSpentCase(t);
        spent.update(dt, ground);
        setMotors(t);
        double ascent_now = ascent.thrust(t), landing_now = landing.thrust(t);  // N, before the flutter
        double landing_burned = landing.burned(t);
        const BodyProps& mp = mech.props();

        // The flight computer runs slower than the physics
        if (step % steps_per_control == 0) {
            wind.update(steps_per_control * dt, state.position.z, rng);
            flutter.update(truth.thrust_roughness, steps_per_control * dt, rng);
            bool burning = ascent_now + landing_now > 1.0;
            double airspeed = length(state.velocity - wind.velocity(state.position.z));
            next_cmd = computer.update(sensors.read(t, state, acceleration, burning, airspeed));
            next_cmd_at = t + truth.control_delay;

            if (next_cmd.fire_ascent) ascent.ignition_time = t + truth.ascent_ignition_delay;
            if (next_cmd.fire_landing) {
                landing.ignition_time = t + truth.landing_ignition_delay;
                result.landing_fire_time = t;
                result.landing_fire_altitude = state.position.z - start_cg;
                result.landing_fire_speed = -state.velocity.z;
                result.landing_fire_tilt_deg = tiltDegrees(state.attitude);
            }
            writeLog(t, mp, cmd);
        }
        if (next_cmd_at >= 0 && t >= next_cmd_at) {
            cmd = next_cmd;
            next_cmd_at = -1;
        }
        if (cmd.release_legs && result.leg_release_time < 0) result.leg_release_time = t;

        // The five servos take their horn angles; the linkages, claws, keeper ring and legs follow
        targets.tvc_yaw = cmd.tvc_yaw_servo_deg;
        targets.tvc_pitch = cmd.tvc_pitch_servo_deg;
        targets.blocker_r = targets.blocker_l = cmd.blocker_servo_deg;
        targets.keeper = cmd.keeper_servo_deg;

        Vec3 wind_now = wind.velocity(state.position.z);
        double rho = airDensity(state.position.z);
        MechanismMotion motion;
        motion.g_minus_a = rotateBack(state.attitude, Vec3{0, 0, -GRAVITY} - acceleration);
        motion.omega = state.angular_velocity;
        motion.alpha = angular_acceleration;
        motion.air = rotateBack(state.attitude, state.velocity - wind_now);
        motion.air_density = rho;
        motion.landing_burning = landing_now > 1.0;
        motion.landing_burned = landing_burned;
        for (int i = 0; i < 4; i++) {
            motion.foot_force[i] = loads.force[i];
            motion.foot_arm[i] = loads.point[i];
        }
        mech.update(t, dt, targets, motion);

        // The blockers: they erode as the motor burns, and an uneven pair twists the rocket and pushes some
        // of the blocked exhaust sideways
        double loss_per_share = (1 - truth.throttle_min) * truth.blocker_effectiveness * (1 - truth.blocker_erosion * landing_burned);
        double ascent_thrust = ascent_now * (1 + flutter.value);
        double landing_full = landing_now * (1 + flutter.value);
        double landing_thrust = landing_full * (1 - loss_per_share * mech.blockedShare());
        Vec3 blocker_push;
        Vec3 blocker_twist = mech.blockerForceTorque(landing_full * loss_per_share, blocker_push);

        StepInputs in;
        in.body = mp;
        in.body.cg += cg_shift;
        in.thrust = ascent_thrust + landing_thrust;
        in.thrust_dir = mech.thrustDirection();
        in.thrust_point = mech.thrustPoint();
        in.nozzle_x = (spent.out ? vehicle.contacts.nozzle_exit_landing : vehicle.contacts.nozzle_exit_ascent).x;
        in.mass_flow = in.thrust / exhaust_speed;
        in.roll_torque = in.thrust > 0 ? truth.roll_torque : 0;
        in.extra_torque = mech.gimbalReactionTorque() + blocker_twist;
        in.side_force = blocker_push;
        in.wind = wind_now;
        in.ground_active = falling;  // the legs come out on the way down
        in.ground = &ground;
        in.foot_crush = crush;
        in.foot_radius = vehicle.legs.foot_radius;
        in.tube_stiffness = tube_k;
        in.body_points = &body_points;
        for (int i = 0; i < 4; i++) {
            in.foot_centre[i] = mech.footCentre(i);
            in.leg_dir[i] = normalize(in.foot_centre[i] - vehicle.legs.leg[i].hinge_origin);
            if (mech.leg[i].released) {  // drag on the leg tubes and feet once they are out in the air
                Vec3 tq;
                in.leg_force += mech.legAeroForce(i, in.body.cg, motion.air, state.angular_velocity, rho, tq);
                in.leg_torque += tq;
            }
        }

        // On the pad nothing moves until the motor pushes up harder than gravity pulls down
        if (on_pad) {
            if (in.thrust * rotate(state.attitude, in.thrust_dir).z < mp.mass * GRAVITY) continue;
            on_pad = false;
        }

        Vec3 old_velocity = state.velocity;
        Vec3 old_rate = state.angular_velocity;
        state = rk4Step(truth, in, state, dt);
        acceleration = (state.velocity - old_velocity) / dt;
        angular_acceleration = (state.angular_velocity - old_rate) / dt;
        if (mech.tvc_at_stop) result.gimbal_stop_time += dt;

        result.apogee = std::max(result.apogee, state.position.z - start_cg);
        if (state.velocity.z < 0 && result.apogee > 1.0) falling = true;
        if (!falling) continue;

        footLoads(truth, in, state, loads);  // with the crush from before this step, the same forces the step used
        crushFeet(truth, ground, in, state, crush);
        double lowest = lowestPoint(truth, ground, in, state);
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
                for (int i = 0; i < 4; i++) result.legs_latched_at_touchdown += mech.leg[i].latched;

                // Keep going so we see the bounce, until it settles and the motor is done
                double burnout = landing.ignition_time >= 0 ? landing.ignition_time + curve.burnTime() : t;
                result.landing_burn_left = std::max(0.0, burnout - t);
                end_time = std::min(truth.max_time, std::max(t + 3.0, burnout + 1.5));
            }
        }
        if (result.touched_down && !touching) result.hop_height = std::max(result.hop_height, lowest);
    }

    for (int i = 0; i < 4; i++) {
        result.foot_crush_used = std::max(result.foot_crush_used, crush[i]);
        result.plunger_failed = result.plunger_failed || mech.leg[i].failed;
        result.peak_plunger_load = std::max(result.peak_plunger_load, mech.leg[i].peak_plunger_load);
        result.legs_out_time = std::max(result.legs_out_time, mech.leg[i].deploy_time);
    }
    result.final_tilt_deg = tiltDegrees(state.attitude);
    result.tipped_over = result.final_tilt_deg > 45;
    return result;
}
