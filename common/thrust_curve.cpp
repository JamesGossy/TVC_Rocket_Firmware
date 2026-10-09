#include "thrust_curve.h"
#include <algorithm>
#include <fstream>
#include <sstream>

bool ThrustCurve::load(const std::string& path) {
    std::ifstream file(path);
    if (!file) return false;

    // Every motor starts at zero thrust
    times = {0};
    thrusts = {0};
    bool got_header = false;

    std::string line;
    while (std::getline(file, line)) {
        size_t start = line.find_first_not_of(" \t\r");
        if (start == std::string::npos || line[start] == ';') continue;  // blank or comment

        std::stringstream ss(line);
        if (!got_header) {
            // Header: name diameter(mm) length(mm) delays propellant_mass(kg) total_mass(kg) maker
            std::string diameter, length, delays;
            ss >> name >> diameter >> length >> delays >> propellant_mass >> total_mass;
            got_header = true;
            continue;
        }

        double t, f;
        if (ss >> t >> f) {
            times.push_back(t);
            thrusts.push_back(f);
        }
    }
    return got_header && times.size() > 2;
}

double ThrustCurve::thrust(double t) const {
    if (t <= 0 || t >= times.back()) return 0;

    // Find the two points around t and draw a straight line between them
    for (size_t i = 1; i < times.size(); i++) {
        if (t <= times[i]) {
            double fraction = (t - times[i - 1]) / (times[i] - times[i - 1]);
            return thrusts[i - 1] + fraction * (thrusts[i] - thrusts[i - 1]);
        }
    }
    return 0;
}

double ThrustCurve::impulseUpTo(double t) const {
    // Area under the curve, one trapezoid at a time
    double impulse = 0;
    for (size_t i = 1; i < times.size(); i++) {
        if (t <= times[i - 1]) break;
        double end = std::min(t, times[i]);
        impulse += (thrusts[i - 1] + thrust(end)) / 2 * (end - times[i - 1]);
    }
    return impulse;
}
