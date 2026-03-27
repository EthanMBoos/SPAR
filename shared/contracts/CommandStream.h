#pragma once
#include <cstdint>
#include <cmath>

struct RoverCommand {
    uint64_t timestamp_us  = 0;
    uint32_t seq           = 0;
    float    throttle      = 0.0f; // [-1.0, 1.0]  negative = reverse
    float    steering      = 0.0f; // [-1.0, 1.0]  negative = left
    float    heading_deg   = std::numeric_limits<float>::quiet_NaN(); // NaN = not commanded
};

struct CommandStream {
    RoverCommand cmd;
    uint64_t     source_id = 0; // BT node identity that produced this command
};
