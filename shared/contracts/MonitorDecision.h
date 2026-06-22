#pragma once
#include "CommandStream.h"
#include <cstdint>
#include <string>

struct MonitorDecision {
    enum class Outcome : uint8_t {
        Passed   = 0, // command passed through unchanged
        Fallback = 1, // invariant violated; fallback command substituted
        Halt     = 2, // critical invariant violated; zero-command + halt signal
    };

    // Bitmask encoding which invariant triggered (0 when Passed).
    static constexpr uint8_t kFlagBoundsNanInf        = 1 << 0;
    static constexpr uint8_t kFlagBoundsThrottleRange  = 1 << 1;
    static constexpr uint8_t kFlagBoundsSteeringRange  = 1 << 2;
    static constexpr uint8_t kFlagStalenessCmd         = 1 << 3;
    static constexpr uint8_t kFlagRateThrottle         = 1 << 4;
    static constexpr uint8_t kFlagRateSteering         = 1 << 5;
    static constexpr uint8_t kFlagStalenessPose        = 1 << 6; // assembler-level pose staleness

    Outcome     outcome;
    std::string triggered_invariant; // empty when Passed
    uint8_t     invariant_flags  = 0; // bit set for the triggered invariant
    uint64_t    timestamp_us     = 0;
    CommandStream input_cmd;
    CommandStream output_cmd;  // may differ from input on Fallback/Halt
};
