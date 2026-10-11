#include "vehicle_data.h"
#include <iostream>
#include "json.h"

static Vec3 vec(const Json& j) { return {j[0].num(), j[1].num(), j[2].num()}; }

static Mat3 matrix(const Json& j) {
    Mat3 m;
    if (j.size() == 9) {  // flat, row major
        for (int i = 0; i < 9; i++) m.m[i / 3][i % 3] = j[i].num();
    } else {              // 3 rows
        for (int i = 0; i < 3; i++)
            for (int k = 0; k < 3; k++) m.m[i][k] = j[i][k].num();
    }
    return m;
}

static MassItem massItem(const Json& j, const char* inertia_key) {
    MassItem it;
    it.mass = j["mass"].num();
    it.cg = vec(j["cg"]);
    it.inertia = matrix(j[inertia_key]);
    return it;
}

int VehicleData::link(const std::string& name) const {
    auto it = index.find(name);
    if (it == index.end()) {
        std::cerr << "vehicle.json has no link '" << name << "'\n";
        std::exit(1);
    }
    return it->second;
}

static TvcAxisData tvcAxis(const Json& j) {
    TvcAxisData a;
    a.shaft = vec(j["servo_shaft"]);
    a.shaft_axis = vec(j["servo_axis"]);
    a.horn_stud = vec(j["horn_stud"]);
    a.cart_stud = vec(j["cartridge_stud"]);
    a.rod_length = j["rod_length_m"].num();
    a.horn_radius = j["horn_radius_m"].num();
    return a;
}

VehicleData loadVehicle(const std::string& path) {
    Json root = loadJson(path);
    VehicleData v;
    v.source = root["source"]["document"].str() + " v" + std::to_string(root["source"]["version"].integer());

    const Json& links = root["links"];
    for (const Json& nameJ : root["link_order"].items) {
        const std::string& name = nameJ.str();
        const Json& l = links[name];
        LinkData d;
        d.name = name;
        d.origin = vec(l["origin"]);
        d.item.mass = l["mass"].num();
        d.item.cg = vec(l["cg"]);
        d.item.inertia = matrix(l["inertia"]);
        if (l.has("joint") && l["joint"].has("axis")) {
            d.axis = normalize(vec(l["joint"]["axis"]));
            d.has_axis = true;
            if (l["joint"].has("value_at_neutral_deg")) d.neutral_deg = l["joint"]["value_at_neutral_deg"].num();
        }
        if (!l["parent"].isNull()) d.parent = v.index.at(l["parent"].str());
        v.index[name] = (int)v.links.size();
        v.links.push_back(d);
    }
    v.airframe = v.link("airframe");
    v.outer_ring = v.link("outer_ring");
    v.cartridge = v.link("cartridge");
    v.landing_motor = v.link("landing_motor");
    v.ascent_motor = v.link("ascent_motor");
    const char* side[2] = {"R", "L"};
    for (int s = 0; s < 2; s++) {
        v.claw[s] = v.link(std::string("claw_") + side[s]);
        v.blocker_horn[s] = v.link(std::string("blocker_horn_") + side[s]);
        v.blocker_link[s] = v.link(std::string("blocker_link_") + side[s]);
    }
    const char* tvc[2] = {"yaw", "pitch"};
    for (int a = 0; a < 2; a++) {
        v.tvc_horn[a] = v.link(std::string("tvc_horn_") + tvc[a]);
        v.tvc_rod[a] = v.link(std::string("tvc_rod_") + tvc[a]);
    }
    v.keeper_horn = v.link("keeper_horn");
    v.keeper_ring = v.link("keeper_ring");
    for (int i = 0; i < 4; i++) v.leg[i] = v.link("leg_" + std::to_string(i + 1));

    // TVC
    const Json& t = root["tvc"];
    v.tvc.yaw = tvcAxis(t["yaw"]);
    v.tvc.pitch = tvcAxis(t["pitch"]);
    v.tvc.pivot = vec(t["pivot"]);
    v.tvc.hard_stop_deg = t["hard_stop_deg"].num();
    v.tvc.clip_deg = t["command_clip_deg"].num();

    // Blocker
    const Json& b = root["blocker"];
    v.blocker.disk_area_mm2 = b["disk_area_mm2"].num();
    v.blocker.plume_radius_mm = b["plume_radius_mm"].num();
    for (int s = 0; s < 2; s++) {
        const Json& j = b[side[s]];
        BlockerSide& o = v.blocker.side[s];
        o.shaft = vec(j["servo_shaft"]);
        o.shaft_axis = vec(j["servo_axis"]);
        o.horn_pin = vec(j["horn_pin"]);
        o.claw_pin = vec(j["claw_pin"]);
        o.claw_pivot = vec(j["claw_pivot"]);
        o.claw_axis = vec(j["claw_axis"]);
        o.link_length = j["link_length_m"].num();
        o.servo_neutral_deg = j["servo_neutral_deg"].num();
        o.servo_deg = j["servo_deg"].numbers();
        o.claw_deg = j["claw_deg"].numbers();
        for (const Json& row : j["blocked_table"]["rows"].items) {
            o.table_claw.push_back(row[0].num());
            o.table_area.push_back(row[1].num());
            o.table_cy.push_back(row[2].num());
            o.table_cz.push_back(row[3].num());
        }
    }

    // Keeper
    const Json& k = root["keeper"];
    v.keeper.servo_shaft = vec(k["servo_shaft"]);
    v.keeper.servo_axis = vec(k["servo_axis"]);
    v.keeper.ring_origin = vec(k["ring_origin"]);
    v.keeper.ring_axis = vec(k["ring_axis"]);
    v.keeper.ring_lo_deg = k["ring_limits_deg"][0].num();
    v.keeper.ring_hi_deg = k["ring_limits_deg"][1].num();
    v.keeper.horn_centre = k["horn_centre_offset_m"].num();
    v.keeper.stud_radius = k["stud_radius_m"].num();
    v.keeper.slot_width = k["slot_width_m"].num();
    v.keeper.release_ring_deg = k["release_ring_deg"].num();

    // Legs
    const Json& g = root["legs"];
    v.legs.latch_deg = g["latch_deg"].num();
    v.legs.ramp_start_deg = g["ramp_start_deg"].num();
    v.legs.plunger_arm = g["plunger_arm_m"].num();
    v.legs.tube_length = g["tube_length_m"].num();
    v.legs.tube_od = g["tube_od_m"].num();
    v.legs.foot_radius = g["foot_radius_m"].num();
    for (int i = 0; i < 4; i++) {
        const Json& l = g["legs"][i];
        v.legs.leg[i].hinge_origin = vec(l["hinge_origin"]);
        v.legs.leg[i].hinge_axis = normalize(vec(l["hinge_axis"]));
        v.legs.leg[i].foot_stowed = vec(l["foot_centre_stowed"]);
        v.legs.leg[i].lo_deg = l["limits_deg"][0].num();
        v.legs.leg[i].hi_deg = l["limits_deg"][1].num();
    }

    // Contacts and nose profile
    const Json& c = root["contacts"];
    for (const Json& p : c["cheek_lobes"].items) v.contacts.cheek_lobes.push_back(vec(p));
    for (const Json& p : c["aft_rim"].items) v.contacts.aft_rim.push_back(vec(p));
    v.contacts.nose_tip = vec(c["nose_tip"]);
    v.contacts.nozzle_exit_ascent = vec(c["nozzle_exit_ascent"]);
    v.contacts.nozzle_exit_landing = vec(c["nozzle_exit_landing"]);
    v.contacts.body_radius = c["body_radius_m"].num();
    v.contacts.body_length = c["body_length_m"].num();
    for (const Json& p : c["profile"].items) v.contacts.profile.push_back({p[0].num(), p[1].num()});
    v.total_length = v.contacts.body_length;

    // Motors
    const Json& m = root["motors"];
    const char* names[2] = {"landing_motor", "ascent_motor"};
    MotorData* out[2] = {&v.landing, &v.ascent};
    for (int i = 0; i < 2; i++) {
        const Json& j = m[names[i]];
        out[i]->link = v.link(names[i]);
        out[i]->length = j["length_m"].num();
        out[i]->radius = j["radius_m"].num();
        out[i]->aft_end = j["aft_end"][0].num();
        out[i]->loaded = massItem(j["loaded"], "I");
        out[i]->casing = massItem(j["case"], "I");
        out[i]->propellant = massItem(j["propellant"], "I");
    }
    return v;
}
