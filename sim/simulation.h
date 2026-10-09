#pragma once
#include <string>
#include "config.h"
#include "thrust_curve.h"

struct FlightResult {
    bool touched_down = false;
    double apogee = 0;

    double landing_fire_time = -1;  // when the flight computer sent the fire command
    double landing_fire_altitude = 0;
    double landing_fire_speed = 0;
    double landing_fire_tilt_deg = 0;

    double touchdown_time = 0;
    double touchdown_vertical_speed = 0;
    double touchdown_horizontal_speed = 0;
    double touchdown_tilt_deg = 0;
    double landing_burn_left = 0;  // seconds of landing burn still left at touchdown

    // After the first touchdown the sim keeps going until the rocket settles
    double hardest_impact = 0;  // fastest it hit the ground, counting bounces
    double hop_height = 0;      // highest it bounced or hopped back up
    double leg_travel_used = 0; // most any leg damper stroked
    double final_tilt_deg = 0;
    bool tipped_over = false;
    double drift = 0;              // distance from the pad
    double landing_x = 0;
    double landing_y = 0;
};

// Fly the rocket once.
// truth   = the real rocket and conditions
// nominal = what the flight computer thinks the rocket is
// Writes a CSV log if log_path isn't empty.
FlightResult runFlight(const Config& truth, const Config& nominal, const ThrustCurve& curve,
                       unsigned seed, const std::string& log_path);
