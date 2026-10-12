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
    double gimbal_y = 0;  // the gimbal tilt the steering wants, radians (pitch about the body y axis)
    double gimbal_z = 0;  // (yaw about the body z axis)
    double throttle = 1;  // share of the landing motor's thrust wanted, 1 = blockers wide open
    bool fire_ascent = false;
    bool fire_landing = false;
    bool release_legs = false;  // let the leg keeper go (stays true once sent)

    // What actually goes to the five KST X08H Plus servos: horn angles in degrees
    double tvc_yaw_servo_deg = 0;
    double tvc_pitch_servo_deg = 0;
    double blocker_servo_deg = -80;  // both blocker claws
    double keeper_servo_deg = 0;
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
    Vec3 position;        // m east and north of the pad, from adding up the velocity

private:
    void calibrateOnPad(const SensorData& data, Commands& cmd);
    void estimate(const SensorData& data, double dt);
    void steer(const SensorData& data, Vec3 target, Commands& cmd);
    Vec3 padDirection(const SensorData& data, double return_gain, double max_lean_deg) const;
    bool timeToLight(const SensorData& data) const;
    double landingThrottle(const SensorData& data) const;
    void changeState(FlightState next, double time);
    void massNumbers();
    void servoCommands(Commands& cmd) const;
    void releaseLegs();

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

    Vec3 rate_filtered;          // gyro rate after the low-pass, for the steering derivative term
    double rate_filter_time = -1; // when steer() last updated it (it restarts after a gap)

    double landing_lit_at = -1;  // when we expect the landing motor to light
    bool legs_released = false;  // we sent the leg release, so the legs are (about to be) out
};
