#pragma once
#include <map>
#include <string>
#include <vector>
#include "math3d.h"

// The rocket as exported from the Fusion CAD (vehicle/vehicle.json, made by tools/build_vehicle.py).
// All points are in the body frame (x out the nose, measured up from the aft end of the ascent motor,
// metres) with the rocket in its neutral pose: gimbal 0, blocker claws closed, keeper ring 0, legs stowed.
// A link's own frame is the body frame moved to the link's joint origin.

struct MassItem {
    double mass = 0;
    Vec3 cg;        // in the link frame
    Mat3 inertia;   // about its own CG, body axes
};

struct LinkData {
    std::string name;
    int parent = -1;
    Vec3 origin;    // joint origin, body frame, neutral pose
    Vec3 axis;      // joint axis (unit), body frame. Zero for fixed links
    bool has_axis = false;
    MassItem item;  // mass, CG (link frame), inertia about the CG
    double neutral_deg = 0;
};

// One TVC axis: servo horn stud -> 33.5 mm pushrod -> drive stud on the cartridge
struct TvcAxisData {
    Vec3 shaft, shaft_axis;   // servo shaft point and axis
    Vec3 horn_stud;           // pushrod end on the horn (a universal joint)
    Vec3 cart_stud;           // pushrod end on the cartridge (a ball joint)
    double rod_length = 0, horn_radius = 0;
};

struct TvcData {
    TvcAxisData yaw, pitch;
    Vec3 pivot;
    double hard_stop_deg = 8.75, clip_deg = 8.0;
};

// One blocker claw: servo horn -> link -> claw (planar four bar), plus how much of the exhaust it covers
struct BlockerSide {
    Vec3 shaft, shaft_axis, horn_pin, claw_pin, claw_pivot, claw_axis;
    double link_length = 0, servo_neutral_deg = 0;
    std::vector<double> servo_deg, claw_deg;                       // linkage solution, servo from -0.5 down
    std::vector<double> table_claw, table_area, table_cy, table_cz; // claw angle (0 to -55), covered area mm^2, centroid mm
};

struct BlockerData {
    BlockerSide side[2];  // 0 = R, 1 = L
    double disk_area_mm2 = 1, plume_radius_mm = 11;
};

struct KeeperData {
    Vec3 servo_shaft, servo_axis, ring_origin, ring_axis;
    double ring_lo_deg = -9.5, ring_hi_deg = 1;
    double horn_centre = 0, stud_radius = 0.006;
    double slot_width = 0.0023, release_ring_deg = 9.1;
};

struct LegGeom {
    Vec3 hinge_origin, hinge_axis, foot_stowed;
    double lo_deg = 0, hi_deg = 119;
};

struct LegsData {
    double latch_deg = 119, ramp_start_deg = 91, plunger_arm = 0.022;
    double tube_length = 0.26, tube_od = 0.014, foot_radius = 0.0125;
    LegGeom leg[4];
};

struct ContactsData {
    std::vector<Vec3> cheek_lobes, aft_rim;
    Vec3 nose_tip, nozzle_exit_ascent, nozzle_exit_landing;
    double body_radius = 0, body_length = 0;
    std::vector<std::pair<double, double>> profile;  // (x, radius) of the shells
};

struct MotorData {
    int link = -1;
    double length = 0, radius = 0, aft_end = 0;
    MassItem loaded, casing, propellant;
};

struct VehicleData {
    std::string source;
    std::vector<LinkData> links;     // parents come before children
    std::map<std::string, int> index;
    int airframe = 0, outer_ring = 0, cartridge = 0, landing_motor = 0, ascent_motor = 0;
    int claw[2] = {0, 0}, blocker_horn[2] = {0, 0}, blocker_link[2] = {0, 0};
    int tvc_horn[2] = {0, 0}, tvc_rod[2] = {0, 0};   // 0 = yaw, 1 = pitch
    int keeper_horn = 0, keeper_ring = 0, leg[4] = {0, 0, 0, 0};
    TvcData tvc;
    BlockerData blocker;
    KeeperData keeper;
    LegsData legs;
    ContactsData contacts;
    MotorData landing, ascent;
    double total_length = 0;

    int link(const std::string& name) const;
};

VehicleData loadVehicle(const std::string& path);
