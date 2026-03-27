#pragma once
#include <cstdint>

enum class BehaviorMode : uint8_t {
    Stop     = 0,
    Navigate = 1,
    Hold     = 2,
};

struct Waypoint {
    double lat_deg = 0.0;
    double lon_deg = 0.0;
    float  alt_m   = 0.0f;
};

struct GoalContext {
    uint64_t     mission_id    = 0;
    uint64_t     timestamp_us  = 0;
    Waypoint     target;
    BehaviorMode mode          = BehaviorMode::Stop;
};
