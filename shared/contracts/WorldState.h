#pragma once
#include <cstdint>

struct Pose {
    double   lat_deg     = 0.0;
    double   lon_deg     = 0.0;
    float    alt_m       = 0.0f;
    float    heading_deg = 0.0f;
    float    speed_ms    = 0.0f;
    uint64_t timestamp_us = 0;
    bool     valid       = false;
};

// Time-coherent snapshot of all observation domains for one tick.
// Built by ObservationAssembler::build() — do not write to this directly.
// BT nodes receive a const ref; nothing above the assembler writes here.
struct WorldState {
    Pose pose;
    // TODO: ObstacleMap  obstacles;
    // TODO: BatteryState battery;
};
