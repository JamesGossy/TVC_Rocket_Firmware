#pragma once
#include <deque>
#include "config.h"
#include "math3d.h"
#include "thrust_curve.h"

// The flight software. The sim feeds it fake sensor data, but the same code is meant to run on
// the real flight computer.

// What the flight computer reads each loop
struct SensorData {
    double time = 0;
    Vec3 gyro;   // rad/s, body frame
    Vec3 accel;  // m/s^2, body frame. Reads about +9.81 on x (the nose axis) sitting on the pad
    double baro_altitude = 0;
    bool gps_new = false;  // a fresh GPS fix arrived this loop
    Vec3 gps_velocity;     // m/s (east, north, up), gps_latency old
};

// What the flight computer tells the hardware to do
struct Commands {
    double gimbal_y = 0;  // radians
    double gimbal_z = 0;
    double throttle = 1;  // landing motor paddles, 1 = wide open
    bool fire_ascent = false;
    bool fire_landing = false;
};

enum class FlightState { Pad, Boost, Coast, LandingBurn };

const char* stateName(FlightState state);

class FlightComputer {
public:
    FlightComputer(const Config& cfg, const ThrustCurve& curve);
    Commands update(const SensorData& data);

    // What the flight computer believes. Public so the sim can log it.
    FlightState state = FlightState::Pad;
    Quat attitude;
    double altitude = 0;  // above the pad
    Vec3 velocity;        // m/s (east, north, up)

private:
    void calibrateOnPad(const SensorData& data, Commands& cmd);
    void estimate(const SensorData& data, double dt);
    void steer(const SensorData& data, Vec3 target, Commands& cmd) const;
    Vec3 landingDirection(const SensorData& data) const;
    bool timeToLight(const SensorData& data) const;
    double landingThrottle(const SensorData& data) const;
    void changeState(FlightState next, double time);

    Config cfg;
    ThrustCurve curve;
    double last_time = 0;
    double state_start = 0;

    // Measured on the pad
    Vec3 gyro_sum, accel_sum;
    double baro_sum = 0;
    int samples = 0;
    Vec3 gyro_bias;
    double ground_altitude = 0;

    // Recent velocity estimates, so a late GPS fix can be compared with what we thought back then
    struct Snapshot {
        double time;
        Vec3 velocity;
    };
    std::deque<Snapshot> history;

    // Rough numbers for the motor that's burning, for the steering and paddle math
    double mass = 0;
    double inertia = 0;
    double lever_arm = 0;  // gimbal pivot to CG
    double average_thrust = 0;

    double landing_lit_at = -1;  // when we expect the landing motor to light
};
