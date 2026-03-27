#pragma once
#include "CommandStream.h"
#include <cstdint>
#include <string>
#include <vector>

struct MonitorDecision {
    enum class Outcome : uint8_t {
        Passed   = 0, // command passed through unchanged
        Fallback = 1, // invariant violated; fallback command substituted
        Halt     = 2, // critical invariant violated; zero-command + halt signal
    };

    Outcome                  outcome;
    std::string              triggered_invariant; // empty when Passed
    std::vector<std::string> active_invariant_set;
    uint64_t                 timestamp_us = 0;
    CommandStream            input_cmd;
    CommandStream            output_cmd;  // may differ from input on Fallback/Halt
};
