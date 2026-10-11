#pragma once
#include <string>
#include "config.h"
#include "thrust_curve.h"
#include "vehicle_data.h"

// Checks on the CAD model of the moving parts, run as: tvc_sim selftest <name>
//   mass      mass, CG, inertia and foot positions against the CAD numbers
//   tvc       the gimbal linkage against the TVC command map, and the hard stop
//   blocker   the claws: full close gives the thrust floor, a left/right mismatch gives a side force
//   legs      leg deploy time in still air and in airflow, the latch, and a leg that folds under load
//   hardware  all of the above
// Returns 0 if everything passed.
int runSelfTest(const std::string& name, const Config& cfg, const VehicleData& vehicle, const ThrustCurve& curve);
