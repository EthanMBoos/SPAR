#pragma once
#include "TransportDegradation.h"
#include <string_view>

// Maps the SPAR_SCENARIO env-var value to the DegradationParams for that scenario.
// All values are nominal; replace with hardware-measured distributions once
// real MAVLink timing data is collected from the rover (Phase 1 gate item).
//
// Scenario definitions match docs/eval-protocol.md:
//   baseline       — clean transport; establishes floor catch fraction (~0)
//   pose_dropout   — 80% dropout; exercises staleness path (high FALL rate expected)
//   pose_jitter    — 80 ms Gaussian jitter; exercises mixed fresh/stale snapshots
//   policy_exploit — clean transport; exploit is in the policy node, not transport
inline DegradationParams degradation_for_scenario(std::string_view scenario) {
    if (scenario == "pose_dropout")
        return {0.0, 0.0, 0.80};

    if (scenario == "pose_jitter")
        return {0.0, 80'000.0, 0.0};

    // baseline, policy_exploit, and any unknown name → clean transport
    return {};
}
