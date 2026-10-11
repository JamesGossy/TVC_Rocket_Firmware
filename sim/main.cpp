#include <algorithm>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <vector>
#include "config.h"
#include "mechanism.h"
#include "selftest.h"
#include "simulation.h"
#include "thrust_curve.h"
#include "vehicle.h"
#include "vehicle_data.h"

// Good landing: it came down, nothing broke, and it's still standing at the end
static bool goodLanding(const FlightResult& r, const Config& cfg) {
    return r.touched_down && r.hardest_impact < cfg.max_touchdown_speed && !r.tipped_over && !r.plunger_failed;
}

// The CAD rocket's mass, CG and inertia with the motors and legs as asked
static BodyProps cadProps(const VehicleData& vehicle, const Config& cfg, bool ascent, bool landing, bool legs_out) {
    std::mt19937 rng(1);
    Mechanism m(vehicle, cfg, cfg, rng);
    m.setMotors(ascent, 0, landing, 0);
    if (legs_out)
        for (int i = 0; i < 4; i++) m.leg[i].angle = vehicle.legs.latch_deg * DEG;
    m.refreshProps();
    return m.props();
}

// With vehicle_model = 1 the numbers the flight computer is told come from the CAD, the same data the sim flies.
// (With 0 it gets the dry_* numbers typed into rocket.cfg.)
static void fillNominalFromCad(Config& cfg, const VehicleData& vehicle) {
    if (!cfg.vehicle_model) return;
    BodyProps stowed = cadProps(vehicle, cfg, false, false, false);
    BodyProps out = cadProps(vehicle, cfg, false, false, true);
    cfg.dry_mass = stowed.mass;
    cfg.dry_cg = stowed.cg.x;
    cfg.dry_inertia_pitch = (stowed.inertia.m[1][1] + stowed.inertia.m[2][2]) / 2;
    cfg.dry_inertia_roll = stowed.inertia.m[0][0];
    cfg.dry_cg_legs_out = out.cg.x;
    cfg.dry_inertia_pitch_legs_out = (out.inertia.m[1][1] + out.inertia.m[2][2]) / 2;
    cfg.dry_inertia_roll_legs_out = out.inertia.m[0][0];
}

static void printRocketInfo(const Config& cfg, const VehicleData& vehicle, const ThrustCurve& curve) {
    double average_thrust = curve.totalImpulse() / curve.burnTime();
    BodyProps liftoff = cadProps(vehicle, cfg, true, true, false);
    BodyProps landing = cadProps(vehicle, cfg, false, true, true);

    std::printf("Rocket: %s\n", vehicle.source.c_str());
    std::printf("Motor: %s, %.1f Ns, %.2f s burn, %.1f N average\n",
                curve.name.c_str(), curve.totalImpulse(), curve.burnTime(), average_thrust);
    std::printf("Liftoff: %.3f kg, CG %.3f m, average thrust/weight %.2f\n",
                liftoff.mass, liftoff.cg.x, average_thrust / (liftoff.mass * 9.81));
    std::printf("Landing: %.3f kg, CG %.3f m (legs out), average thrust/weight %.2f\n\n",
                landing.mass, landing.cg.x, average_thrust / (landing.mass * 9.81));
}

static void printResult(const FlightResult& r, const Config& cfg) {
    std::printf("Apogee:            %.1f m\n", r.apogee);
    if (r.landing_fire_time >= 0)
        std::printf("Landing motor:     fired at %.2f s, %.1f m up, falling at %.1f m/s\n",
                    r.landing_fire_time, r.landing_fire_altitude, r.landing_fire_speed);
    else
        std::printf("Landing motor:     never fired\n");
    if (r.leg_release_time >= 0)
        std::printf("Legs:              released at %.2f s, last one locked out at %.2f s\n", r.leg_release_time, r.legs_out_time);

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
    std::printf("Legs:              %d of 4 locked out at touchdown, foot balls crushed up to %.1f mm, peak plunger load %.0f N (limit %.0f N)%s\n",
                r.legs_latched_at_touchdown, r.foot_crush_used * 1e3, r.peak_plunger_load, cfg.plunger_shear_limit,
                r.plunger_failed ? ", A PLUNGER SHEARED" : "");
    std::printf("Ended:             %s (%.0f deg tilt)\n", r.tipped_over ? "tipped over" : "standing", r.final_tilt_deg);

    if (goodLanding(r, cfg)) std::printf("\nGOOD LANDING\n");
    else if (r.plunger_failed) std::printf("\nCRASH (a leg plunger sheared)\n");
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
    c.blocker_effectiveness = std::max(0.3, nominal.blocker_effectiveness * (1 + nominal.mc_blocker_effect * gauss(rng)));
    c.baro_airspeed_error = std::max(0.0, nominal.baro_airspeed_error * (1 + nominal.mc_baro_airspeed * gauss(rng)));
    c.ground_slope_deg = nominal.mc_max_slope_deg * uniform(rng);
    c.ground_slope_direction_deg = 360 * uniform(rng);

    // Every joint's lost motion differs from rocket to rocket (printing and assembly tolerances), each its own way
    auto lash = [&](double& value) { value *= std::max(0.2, 1 + nominal.mc_backlash_spread * gauss(rng)); };
    lash(c.backlash_servo_deg);
    lash(c.backlash_tvc_horn_end_mm);
    lash(c.backlash_tvc_ujoint_mm);
    lash(c.backlash_tvc_drive_end_mm);
    lash(c.backlash_tvc_yaw_axle_deg);
    lash(c.backlash_tvc_pitch_axle_deg);
    lash(c.backlash_blocker_horn_pin_mm);
    lash(c.backlash_blocker_claw_pin_mm);
    lash(c.backlash_claw_pivot_mm);
    lash(c.backlash_keeper_slot_mm);
    lash(c.backlash_leg_latch_mm);

    // Blocker horns clocked a little differently on their splines, the leg springs and the foam differ
    c.blocker_servo_offset_r_deg = nominal.mc_servo_offset_deg * gauss(rng);
    c.blocker_servo_offset_l_deg = nominal.mc_servo_offset_deg * gauss(rng);
    double spring = std::max(0.3, 1 + nominal.mc_leg_spring_percent / 100 * gauss(rng));
    c.leg_spring_k *= spring;
    c.leg_spring_torque_deployed *= spring;
    c.foot_crush_stress *= std::max(0.3, 1 + nominal.mc_foot_crush_percent / 100 * gauss(rng));
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
    std::printf("  Blockers:        %.0f%% as effective as expected, horns clocked %+.2f / %+.2f deg\n\n",
                c.blocker_effectiveness * 100, c.blocker_servo_offset_r_deg, c.blocker_servo_offset_l_deg);
}

static double percentile(std::vector<double> values, double p) {
    if (values.empty()) return 0;
    std::sort(values.begin(), values.end());
    return values[(size_t)(p / 100 * (values.size() - 1))];
}

// Flies runs [first, last) of a Monte Carlo of `runs` flights. The random rockets and days are drawn for all of
// them in order, so a run is the same whichever chunk it is in (long Monte Carlos can be split up).
static void runMonteCarlo(const Config& nominal, const VehicleData& vehicle, const ThrustCurve& curve, int runs,
                          int first, int last) {
    std::ofstream out("montecarlo.csv");
    out << "run,good,dry_mass,wind_speed,landing_ignition_delay,landing_thrust_scale,ascent_thrust_scale,"
           "apogee,fire_altitude,fire_speed,fire_tilt,touchdown_speed,sideways_speed,tilt_deg,burn_left,"
           "hardest_impact,hop_height,tipped_over,landing_x,landing_y,"
           "legs_latched,plunger_failed,peak_plunger_load,foot_crush_mm,gimbal_stop_s\n";

    std::mt19937 rng(12345);
    std::vector<double> touchdown_speeds, apogees, drifts;
    int good = 0, tipped = 0, hard = 0, sheared = 0, legs_short = 0, no_touchdown = 0;
    auto started = std::chrono::steady_clock::now();

    for (int run = 0; run < runs; run++) {
        Config truth = randomize(nominal, rng);
        if (run < first || run >= last) continue;
        FlightResult r = runFlight(truth, nominal, vehicle, curve, run + 1, "");

        bool ok = goodLanding(r, nominal);
        if (ok) good++;
        else if (!r.touched_down) no_touchdown++;
        else if (r.plunger_failed) sheared++;
        else if (r.hardest_impact >= nominal.max_touchdown_speed) hard++;
        else tipped++;
        if (r.touched_down && r.legs_latched_at_touchdown < 4) legs_short++;
        apogees.push_back(r.apogee);
        if (r.touched_down) {
            touchdown_speeds.push_back(r.touchdown_vertical_speed);
            drifts.push_back(r.drift);
        }

        out << run << "," << ok << "," << truth.dry_mass << "," << truth.wind_speed << ","
            << truth.landing_ignition_delay << "," << truth.landing_thrust_scale << "," << truth.ascent_thrust_scale << ","
            << r.apogee << "," << r.landing_fire_altitude << "," << r.landing_fire_speed << "," << r.landing_fire_tilt_deg << ","
            << r.touchdown_vertical_speed << "," << r.touchdown_horizontal_speed << "," << r.touchdown_tilt_deg << ","
            << r.landing_burn_left << "," << r.hardest_impact << "," << r.hop_height << "," << r.tipped_over << ","
            << r.landing_x << "," << r.landing_y << ","
            << r.legs_latched_at_touchdown << "," << r.plunger_failed << "," << r.peak_plunger_load << ","
            << r.foot_crush_used * 1e3 << "," << r.gimbal_stop_time << "\n";
    }
    double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();

    int flown = std::min(last, runs) - first;
    std::printf("Monte Carlo: %d runs (%.1f s, %.0f ms a run)\n", flown, seconds, 1e3 * seconds / std::max(flown, 1));
    std::printf("Good landings:     %d (%.1f%%)\n", good, 100.0 * good / std::max(flown, 1));
    std::printf("Failures:          %d tipped over, %d hit too hard, %d plunger sheared, %d never touched down\n",
                tipped, hard, sheared, no_touchdown);
    std::printf("Legs short:        %d runs touched down with fewer than 4 legs locked out\n", legs_short);
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
//   tvc_sim montecarlo 2000 400 800  the same 2000-run Monte Carlo, but only flies runs 400 to 799
//   tvc_sim replay 180               fly run 180 of the Monte Carlo again, writes flight.csv
//   tvc_sim other.cfg montecarlo     any .cfg file name picks a different config
//   tvc_sim selftest tvc             check one part of the CAD model (tvc, blocker, legs, mass, hardware)
int main(int argc, char* argv[]) {
    std::string config_path = "rocket.cfg";
    bool monte_carlo = false;
    bool random_flight = false;
    bool replay = false;
    bool self_test = false;
    std::string test_name;
    int number = -1;  // how many Monte Carlo runs, the seed for a random flight, or which run to replay
    std::vector<int> numbers;

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg.size() > 4 && arg.substr(arg.size() - 4) == ".cfg") config_path = arg;
        else if (arg == "montecarlo") monte_carlo = true;
        else if (arg == "random") random_flight = true;
        else if (arg == "replay") replay = true;
        else if (arg == "selftest") self_test = true;
        else if (self_test && test_name.empty()) test_name = arg;
        else numbers.push_back(std::atoi(arg.c_str()));
    }
    if (!numbers.empty()) number = numbers[0];

    Config cfg = loadConfig(config_path);

    ThrustCurve curve;
    if (!curve.load(cfg.motor_file)) {
        std::cerr << "Could not load motor file " << cfg.motor_file << "\n";
        return 1;
    }
    VehicleData vehicle = loadVehicle(cfg.vehicle_file);
    fillNominalFromCad(cfg, vehicle);

    if (self_test) return runSelfTest(test_name, cfg, vehicle, curve);
    printRocketInfo(cfg, vehicle, curve);

    if (monte_carlo) {
        int runs = number > 0 ? number : 200;
        int first = numbers.size() > 1 ? numbers[1] : 0;                 // tvc_sim montecarlo 2000 400 800 flies runs 400 to 799
        int last = numbers.size() > 2 ? numbers[2] : runs;
        runMonteCarlo(cfg, vehicle, curve, runs, first, last);
    } else if (replay) {
        // Fly one run from the Monte Carlo again (same randomness), with a full log
        std::mt19937 rng(12345);
        Config truth = cfg;
        for (int run = 0; run <= std::max(number, 0); run++) truth = randomize(cfg, rng);
        printConditions(truth, cfg, number + 1);
        FlightResult r = runFlight(truth, cfg, vehicle, curve, number + 1, "flight.csv");
        printResult(r, cfg);
        std::printf("Flight log written to flight.csv\n");
    } else if (random_flight) {
        unsigned seed = number >= 0 ? number : std::random_device{}();
        std::mt19937 rng(seed);
        Config truth = randomize(cfg, rng);
        printConditions(truth, cfg, seed);
        FlightResult r = runFlight(truth, cfg, vehicle, curve, seed, "flight.csv");
        printResult(r, cfg);
        std::printf("Flight log written to flight.csv\n");
    } else {
        FlightResult r = runFlight(cfg, cfg, vehicle, curve, 1, "flight.csv");
        printResult(r, cfg);
        std::printf("Flight log written to flight.csv\n");
    }
    return 0;
}
