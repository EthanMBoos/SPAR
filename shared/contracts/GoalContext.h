#pragma once
#include <cstdint>

struct Waypoint {
    double lat_deg = 0.0;
    double lon_deg = 0.0;
    float  alt_m   = 0.0f;
};

struct GoalContext {
    uint64_t mission_id   = 0;
    uint64_t timestamp_us = 0;
    Waypoint target;
};
