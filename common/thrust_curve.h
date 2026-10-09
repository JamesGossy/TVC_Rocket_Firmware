#pragma once
#include <string>
#include <vector>

// A motor thrust curve loaded from a RASP .eng file (the format thrustcurve.org uses).
class ThrustCurve {
public:
    bool load(const std::string& path);

    double thrust(double t) const;       // newtons, t = seconds since ignition
    double impulseUpTo(double t) const;  // newton-seconds delivered by time t
    double burnTime() const { return times.back(); }
    double totalImpulse() const { return impulseUpTo(burnTime()); }
    double propellantMass() const { return propellant_mass; }
    double totalMass() const { return total_mass; }

    std::string name;

private:
    std::vector<double> times;
    std::vector<double> thrusts;
    double propellant_mass = 0;
    double total_mass = 0;
};
