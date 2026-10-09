#pragma once
#include <cmath>
#include <deque>
#include <random>
#include "config.h"
#include "flight_computer.h"
#include "math3d.h"
#include "physics.h"
#include "thrust_curve.h"

// Simple models of the real hardware: motors, servos, paddles, sensors and wind.

// One real motor. Knows when it was lit and how much propellant is left.
struct Motor {
    const ThrustCurve* curve = nullptr;
    double thrust_scale = 1.0;  // real motors are a few percent off the datasheet
    double ignition_time = -1;  // sim time it actually lights, -1 = not lit

    double thrust(double t) const {
        if (ignition_time < 0 || t < ignition_time) return 0;
        return thrust_scale * curve->thrust(t - ignition_time);
    }

    double mass(double t) const {
        double burned = 0;
        if (ignition_time >= 0 && t > ignition_time)
            burned = curve->impulseUpTo(t - ignition_time) / curve->totalImpulse();
        return curve->totalMass() - curve->propellantMass() * burned;
    }
};

// A hobby servo moving one gimbal axis through a linkage. Real servos only take a new
// position every 20 ms, ignore tiny changes, respond with a little lag, swing at a limited
// speed, the linkage has slop in it, and the printed mount is springy so the gimbal rings.
// So the gimbal never goes exactly where it's told.
struct Servo {
    double angle = 0;       // where the gimbal actually is
    double angle_rate = 0;
    double accel = 0;       // gimbal's angular acceleration (it twists the rocket the other way)
    double arm = 0;         // where the servo arm is
    double linkage = 0;     // where the linkage is pushing the gimbal, after the slop
    double goal = 0;        // last position the servo accepted
    double next_update = 0;

    void update(double target, double t, double dt, const Config& cfg) {
        if (t >= next_update) {
            next_update = t + 1.0 / cfg.servo_update_rate;
            target = std::clamp(target, -cfg.gimbal_max_deg * DEG, cfg.gimbal_max_deg * DEG);
            if (std::fabs(target - goal) > cfg.servo_deadband_deg * DEG) goal = target;
        }

        // The arm heads for the goal with a short lag, but no faster than the servo's top speed
        double max_speed = cfg.servo_rate_deg * DEG;
        arm += std::clamp((goal - arm) / std::max(cfg.servo_lag, dt), -max_speed, max_speed) * dt;

        // Backlash: the linkage only moves once the arm has taken up the slop
        double slop = cfg.gimbal_backlash_deg * DEG / 2;
        if (arm - linkage > slop) linkage = arm - slop;
        if (linkage - arm > slop) linkage = arm + slop;

        // The mount and linkage are springy, so the gimbal rings around where it's pushed
        if (cfg.gimbal_wobble_hz <= 0) {  // perfectly stiff
            double new_rate = (linkage - angle) / dt;
            accel = (new_rate - angle_rate) / dt;
            angle_rate = new_rate;
            angle = linkage;
            return;
        }
        double w = 2 * PI * cfg.gimbal_wobble_hz;
        accel = w * w * (linkage - angle) - 2 * cfg.gimbal_wobble_damping * w * angle_rate;
        angle_rate += accel * dt;
        angle += angle_rate * dt;
    }
};

// Paddles in the landing motor's exhaust. 1.0 = full thrust.
// They can't choke it below throttle_min, and their servo can only move them so fast.
struct Paddles {
    double throttle = 1.0;

    void update(double target, const Config& cfg, double dt) {
        target = std::clamp(target, cfg.throttle_min, 1.0);
        double max_step = cfg.throttle_rate * dt;
        throttle += std::clamp(target - throttle, -max_step, max_step);
    }
};

// The spent ascent motor after the landing motor kicks it out the bottom.
// Just a tube falling with gravity and drag, until it lands on the grass.
struct EjectedCase {
    bool out = false;
    bool down = false;  // lying on the ground
    Vec3 position;      // center of the tube
    Vec3 velocity;
    double drag = 0;    // 0.5 * air density * drag coefficient * area / mass

    void update(double dt, const Ground& ground) {
        if (!out || down) return;
        velocity += (Vec3{0, 0, -GRAVITY} - velocity * (drag * length(velocity))) * dt;
        position += velocity * dt;
        double grass = ground.height(position.x, position.y) + 0.015;
        if (position.z < grass) {
            position.z = grass;
            velocity = {};
            down = true;
        }
    }
};

// Black powder motors flutter instead of burning smoothly. This wobbles the thrust a few percent.
struct ThrustNoise {
    double value = 0;

    void update(double strength, double dt, std::mt19937& rng) {
        std::normal_distribution<double> gauss(0.0, 1.0);
        double keep = std::exp(-dt / 0.05);  // changes over about 50 ms
        value = value * keep + strength * std::sqrt(1 - keep * keep) * gauss(rng);
    }
};

// Steady wind that slows down near the ground, plus gusts that wander around smoothly
class Wind {
public:
    explicit Wind(const Config& cfg) : cfg(cfg) {
        double direction = cfg.wind_direction_deg * DEG;
        steady = Vec3{std::cos(direction), std::sin(direction), 0} * cfg.wind_speed;
        gust_size = cfg.gust_strength * cfg.wind_speed;
    }

    void update(double dt, double height, std::mt19937& rng) {
        // Each step the gust keeps most of its old value and gets a small random kick. Gusts change
        // over about a second, and falling through the air we also meet new ones.
        std::normal_distribution<double> gauss(0.0, 1.0);
        double moved = std::fabs(height - last_height);
        last_height = height;
        double keep = std::exp(-dt / 1.0 - moved / 10.0);
        double kick = gust_size * std::sqrt(1 - keep * keep);
        gust.x = gust.x * keep + kick * gauss(rng);
        gust.y = gust.y * keep + kick * gauss(rng);
        gust.z = gust.z * keep + 0.5 * kick * gauss(rng);  // up and down drafts are weaker
    }

    // The ground drags the air back, so the wind is slower low down
    Vec3 velocity(double height) const {
        double scale = std::pow(std::max(height, 0.5) / cfg.wind_height, cfg.wind_shear);
        return steady * scale + gust;
    }

private:
    const Config& cfg;
    Vec3 steady, gust;
    double gust_size = 0;
    double last_height = 0;
};

// Fake IMU, barometer and GPS: the true values plus everything that makes real sensors messy
class Sensors {
public:
    Sensors(const Config& cfg, std::mt19937& rng) : cfg(cfg), rng(rng) {
        gyro_bias = noise(cfg.gyro_bias_deg * DEG);
        gyro_scale = noise(cfg.gyro_scale_error);
        accel_bias = noise(cfg.accel_bias);
        crooked = fromAxisAngle(noise(1.0), cfg.imu_misalignment_deg * DEG);  // IMU mounted slightly off
    }

    SensorData read(double time, const RigidBodyState& s, Vec3 acceleration, bool burning, double airspeed) {
        SensorData d;
        d.time = time;
        double dt = last_time < 0 ? 0 : time - last_time;
        last_time = time;

        // Gyro: seen through the crooked mount, a bit off scale, biased, noisy, and shaken by the motor
        Vec3 rate = rotateBack(crooked, s.angular_velocity);
        rate = {rate.x * (1 + gyro_scale.x), rate.y * (1 + gyro_scale.y), rate.z * (1 + gyro_scale.z)};
        d.gyro = rate + gyro_bias + noise(cfg.gyro_noise_deg * DEG);
        if (burning) d.gyro += noise(cfg.vibration_gyro_deg * DEG);

        // Accelerometer: feels every force except gravity, so sitting still it reads 1 g up
        Vec3 felt = acceleration + Vec3{0, 0, GRAVITY};
        d.accel = rotateBack(crooked, rotateBack(s.attitude, felt)) + accel_bias + noise(cfg.accel_noise);
        if (burning) d.accel += noise(cfg.vibration_accel);

        // Barometer: airflow over the rocket lowers the pressure it sees, so it reads high when
        // moving fast, and its filtering makes it lag behind
        double reading = s.position.z + cfg.baro_airspeed_error * airspeed * airspeed / (2 * GRAVITY);
        if (dt == 0) baro = reading;
        else baro += (reading - baro) * dt / (cfg.baro_lag + dt);
        d.baro_altitude = baro + cfg.baro_noise * gauss(rng);

        readGps(time, s, acceleration, d);
        return d;
    }

private:
    // GPS velocity: a fix every 1/gps_rate seconds, saying how fast we were going about
    // gps_latency ago. Fixes go missing now and then (radio noise, the antenna facing the wrong
    // way), it's noisier when shaken hard or close to the ground, and once in a while it's just wrong.
    void readGps(double time, const RigidBodyState& s, Vec3 acceleration, SensorData& d) {
        history.push_back({time, s.velocity});
        while (history.front().time < time - 1.0) history.pop_front();
        if (time < next_fix) return;
        next_fix = time + 1.0 / cfg.gps_rate;

        if (time >= outage_until && chance(rng) < cfg.gps_outage_rate / cfg.gps_rate)
            outage_until = time + 1 + 2 * chance(rng);  // lost lock for 1-3 s
        if (time < outage_until || chance(rng) < cfg.gps_dropout) return;

        double age = cfg.gps_latency + cfg.gps_latency_jitter * (2 * chance(rng) - 1);
        Vec3 then = history.front().velocity;
        for (const Past& p : history)
            if (p.time <= time - age) then = p.velocity;

        double shaken = 1 + length(acceleration) / GRAVITY;
        double near_ground = s.position.z < cfg.gps_multipath_height ? 2 : 1;
        d.gps_new = true;
        d.gps_velocity = then + noise(cfg.gps_velocity_noise * shaken * near_ground);
        if (chance(rng) < cfg.gps_glitch) d.gps_velocity += noise(1.5);
    }

    Vec3 noise(double size) { return Vec3{gauss(rng), gauss(rng), gauss(rng)} * size; }

    const Config& cfg;
    std::mt19937& rng;
    std::normal_distribution<double> gauss{0.0, 1.0};
    std::uniform_real_distribution<double> chance{0.0, 1.0};

    Vec3 gyro_bias, gyro_scale, accel_bias;
    Quat crooked;
    double baro = 0;
    double last_time = -1;

    struct Past {
        double time;
        Vec3 velocity;
    };
    std::deque<Past> history;
    double next_fix = 0;
    double outage_until = 0;
};
