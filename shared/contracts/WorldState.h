#pragma once
#include <cstdint>
#include <vector>

struct Pose {
    double   lat_deg      = 0.0;
    double   lon_deg      = 0.0;
    float    alt_m        = 0.0f;
    float    heading_deg  = 0.0f;
    float    speed_ms     = 0.0f;
    uint64_t timestamp_us = 0;
    bool     valid        = false;
};

// Time-coherent snapshot of all observation domains for one tick.
// Built by ObservationAssembler::build() — do not write to this directly.
// BT nodes receive a const ref; nothing above the assembler writes here.
struct WorldState {
    Pose pose;
    // TODO: ObstacleMap  obstacles;
    // TODO: BatteryState battery;

    // Flat float vector consumed by learned policies (OnnxNavigateNode, SparEnv).
    // Field order is load-bearing: matches the observation space the SAC/Cosmos actor trains on.
    // [lat_deg, lon_deg, alt_m, heading_deg, speed_ms]
    std::vector<float> to_observation_vector() const {
        return {
            static_cast<float>(pose.lat_deg),
            static_cast<float>(pose.lon_deg),
            pose.alt_m,
            pose.heading_deg,
            pose.speed_ms,
        };
    }
};
