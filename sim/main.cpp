#include <algorithm>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <vector>
#include "config.h"
#include "simulation.h"
#include "thrust_curve.h"
#include "vehicle.h"

// Good landing: it came down, nothing broke, and it's still standing at the end
static bool goodLanding(const FlightResult& r, const Config& cfg) {
    return r.touched_down && r.hardest_impact < cfg.max_touchdown_speed && !r.tipped_over;
}

static void printRocketInfo(const Config& cfg, const ThrustCurve& curve) {
    double full = curve.totalMass();
    MassProps liftoff = massProperties(cfg, full, full);
    MassProps landing = massProperties(cfg, 0, full);  // the spent ascent motor is gone by then
    double average_thrust = curve.totalImpulse() / curve.burnTime();

    std::printf("Motor: %s, %.1f Ns, %.2f s burn, %.1f N average\n",
                curve.name.c_str(), curve.totalImpulse(), curve.burnTime(), average_thrust);
    std::printf("Liftoff: %.3f kg, CG %.3f m, average thrust/weight %.2f\n",
                liftoff.mass, liftoff.cg, average_thrust / (liftoff.mass * 9.81));
    std::printf("Landing: %.3f kg, CG %.3f m, average thrust/weight %.2f\n\n",
                landing.mass, landing.cg, average_thrust / (landing.mass * 9.81));
}

static void printResult(const FlightResult& r, const Config& cfg) {
    std::printf("Apogee:            %.1f m\n", r.apogee);
    if (r.landing_fire_time >= 0)
        std::printf("Landing motor:     fired at %.2f s, %.1f m up, falling at %.1f m/s\n",
                    r.landing_fire_time, r.landing_fire_altitude, r.landing_fire_speed);
    else
        std::printf("Landing motor:     never fired\n");

    if (!r.touched_down) {
        std::printf("Rocket never touched down (sim ran out of time)\n");
        return;
    }
    std::printf("Touchdown:         %.2f s, %.2f m/s down, %.2f m/s sideways, %.1f deg tilt\n",
                r.touchdown_time, r.touchdown_vertical_speed, r.touchdown_horizontal_speed, r.touchdown_tilt_deg);
    std::printf("Drift from pad:    %.1f m\n", r.drift);
    if (r.landing_burn_left > 0.1)
        std::printf("Landing motor:     still had %.2f s of burn left at touchdown\n", r.landing_burn_left);
    std::printf("Bounce:            up to %.2f m, hardest hit %.2f m/s\n", r.hop_height, r.hardest_impact);
    std::printf("Legs:              used %.1f of %.1f cm of damper travel\n", r.leg_travel_used * 100, cfg.leg_stroke * 100);
    std::printf("Ended:             %s (%.0f deg tilt)\n", r.tipped_over ? "tipped over" : "standing", r.final_tilt_deg);

    if (goodLanding(r, cfg)) std::printf("\nGOOD LANDING\n");
    else if (r.hardest_impact >= cfg.max_touchdown_speed) std::printf("\nCRASH (hit too hard for the legs)\n");
    else std::printf("\nCRASH (tipped over)\n");
}

// Make a slightly different rocket and a different day for one Monte Carlo run
static Config randomize(const Config& nominal, std::mt19937& rng) {
    std::normal_distribution<double> gauss(0.0, 1.0);
    std::uniform_real_distribution<double> uniform(0.0, 1.0);

    Config c = nominal;
    c.dry_mass *= 1 + nominal.mc_dry_mass_percent / 100 * gauss(rng);
    c.dry_cg += nominal.mc_dry_cg * gauss(rng);
    c.ascent_thrust_scale *= 1 + nominal.mc_thrust_percent / 100 * gauss(rng);
    c.landing_thrust_scale *= 1 + nominal.mc_thrust_percent / 100 * gauss(rng);
    c.ascent_ignition_delay = std::max(0.0, c.ascent_ignition_delay + nominal.mc_ignition_delay * gauss(rng));
    c.landing_ignition_delay = std::max(0.0, c.landing_ignition_delay + nominal.mc_ignition_delay * gauss(rng));
    c.gimbal_misalignment_y_deg += nominal.mc_misalignment_deg * gauss(rng);
    c.gimbal_misalignment_z_deg += nominal.mc_misalignment_deg * gauss(rng);
    c.launch_tilt_deg = nominal.mc_launch_tilt_deg * gauss(rng);
    c.wind_speed = nominal.mc_max_wind * uniform(rng);
    c.wind_direction_deg = 360 * uniform(rng);
    c.roll_torque = nominal.roll_torque * 2 * gauss(rng);
    c.paddle_effectiveness = std::max(0.3, nominal.paddle_effectiveness * (1 + nominal.mc_paddle_effect * gauss(rng)));
    c.baro_airspeed_error = std::max(0.0, nominal.baro_airspeed_error * (1 + nominal.mc_baro_airspeed * gauss(rng)));
    c.ground_slope_deg = nominal.mc_max_slope_deg * uniform(rng);
    c.ground_slope_direction_deg = 360 * uniform(rng);
    return c;
}

// Show what a random flight changed compared to rocket.cfg
static void printConditions(const Config& c, const Config& nominal, unsigned seed) {
    std::printf("Random flight, seed %u:\n", seed);
    std::printf("  Wind:            %.1f m/s toward %.0f deg\n", c.wind_speed, c.wind_direction_deg);
    std::printf("  Field slope:     %.1f deg, uphill toward %.0f deg\n", c.ground_slope_deg, c.ground_slope_direction_deg);
    std::printf("  Launch tilt:     %.1f deg\n", c.launch_tilt_deg);
    std::printf("  Motor thrust:    ascent %+.1f%%, landing %+.1f%%\n",
                (c.ascent_thrust_scale - 1) * 100, (c.landing_thrust_scale - 1) * 100);
    std::printf("  Landing igniter: %.2f s (normally %.2f s)\n", c.landing_ignition_delay, nominal.landing_ignition_delay);
    std::printf("  Dry mass:        %.0f g (normally %.0f g)\n", c.dry_mass * 1000, nominal.dry_mass * 1000);
    std::printf("  Gimbal off by:   %.2f / %.2f deg\n", c.gimbal_misalignment_y_deg, c.gimbal_misalignment_z_deg);
    std::printf("  Roll torque:     %.4f N m\n", c.roll_torque);
    std::printf("  Paddles:         %.0f%% as effective as expected\n\n", c.paddle_effectiveness * 100);
}

static double percentile(std::vector<double> values, double p) {
    if (values.empty()) return 0;
    std::sort(values.begin(), values.end());
    return values[(size_t)(p / 100 * (values.size() - 1))];
}

static void runMonteCarlo(const Config& nominal, const ThrustCurve& curve, int runs) {
    std::ofstream out("montecarlo.csv");
    out << "run,good,dry_mass,wind_speed,landing_ignition_delay,landing_thrust_scale,ascent_thrust_scale,"
           "apogee,fire_altitude,fire_speed,fire_tilt,touchdown_speed,sideways_speed,tilt_deg,burn_left,"
           "hardest_impact,hop_height,tipped_over,landing_x,landing_y\n";

    std::mt19937 rng(12345);
    std::vector<double> touchdown_speeds, apogees, drifts;
    int good = 0;

    for (int run = 0; run < runs; run++) {
        Config truth = randomize(nominal, rng);
        FlightResult r = runFlight(truth, nominal, curve, run + 1, "");

        if (goodLanding(r, nominal)) good++;
        apogees.push_back(r.apogee);
        if (r.touched_down) {
            touchdown_speeds.push_back(r.touchdown_vertical_speed);
            drifts.push_back(r.drift);
        }

        out << run << "," << goodLanding(r, nominal) << "," << truth.dry_mass << "," << truth.wind_speed << ","
            << truth.landing_ignition_delay << "," << truth.landing_thrust_scale << "," << truth.ascent_thrust_scale << ","
            << r.apogee << "," << r.landing_fire_altitude << "," << r.landing_fire_speed << "," << r.landing_fire_tilt_deg << ","
            << r.touchdown_vertical_speed << "," << r.touchdown_horizontal_speed << "," << r.touchdown_tilt_deg << ","
            << r.landing_burn_left << "," << r.hardest_impact << "," << r.hop_height << "," << r.tipped_over << ","
            << r.landing_x << "," << r.landing_y << "\n";
    }

    std::printf("Monte Carlo: %d runs\n", runs);
    std::printf("Good landings:     %d (%.0f%%)\n", good, 100.0 * good / runs);
    std::printf("Apogee:            %.1f to %.1f m (median %.1f)\n",
                percentile(apogees, 0), percentile(apogees, 100), percentile(apogees, 50));
    std::printf("Touchdown speed:   median %.2f m/s, 95%% under %.2f m/s\n",
                percentile(touchdown_speeds, 50), percentile(touchdown_speeds, 95));
    std::printf("Drift from pad:    median %.1f m, 95%% under %.1f m\n",
                percentile(drifts, 50), percentile(drifts, 95));
    std::printf("Results written to montecarlo.csv\n");
}

// Usage:
//   tvc_sim                          fly once with rocket.cfg, writes flight.csv
//   tvc_sim random 42                fly once with randomized conditions (seed 42), writes flight.csv
//   tvc_sim montecarlo 500           fly 500 randomized flights, writes montecarlo.csv
//   tvc_sim replay 180               fly run 180 of the Monte Carlo again, writes flight.csv
//   tvc_sim other.cfg montecarlo     any .cfg file name picks a different config
int main(int argc, char* argv[]) {
    std::string config_path = "rocket.cfg";
    bool monte_carlo = false;
    bool random_flight = false;
    bool replay = false;
    int number = -1;  // how many Monte Carlo runs, the seed for a random flight, or which run to replay

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg.size() > 4 && arg.substr(arg.size() - 4) == ".cfg") config_path = arg;
        else if (arg == "montecarlo") monte_carlo = true;
        else if (arg == "random") random_flight = true;
        else if (arg == "replay") replay = true;
        else number = std::atoi(arg.c_str());
    }

    Config cfg = loadConfig(config_path);

    ThrustCurve curve;
    if (!curve.load(cfg.motor_file)) {
        std::cerr << "Could not load motor file " << cfg.motor_file << "\n";
        return 1;
    }
    printRocketInfo(cfg, curve);

    if (monte_carlo) {
        runMonteCarlo(cfg, curve, number > 0 ? number : 200);
    } else if (replay) {
        // Fly one run from the Monte Carlo again (same randomness), with a full log
        std::mt19937 rng(12345);
        Config truth = cfg;
        for (int run = 0; run <= std::max(number, 0); run++) truth = randomize(cfg, rng);
        printConditions(truth, cfg, number + 1);
        FlightResult r = runFlight(truth, cfg, curve, number + 1, "flight.csv");
        printResult(r, cfg);
        std::printf("Flight log written to flight.csv\n");
    } else if (random_flight) {
        unsigned seed = number >= 0 ? number : std::random_device{}();
        std::mt19937 rng(seed);
        Config truth = randomize(cfg, rng);
        printConditions(truth, cfg, seed);
        FlightResult r = runFlight(truth, cfg, curve, seed, "flight.csv");
        printResult(r, cfg);
        std::printf("Flight log written to flight.csv\n");
    } else {
        FlightResult r = runFlight(cfg, cfg, curve, 1, "flight.csv");
        printResult(r, cfg);
        std::printf("Flight log written to flight.csv\n");
    }
    return 0;
}
