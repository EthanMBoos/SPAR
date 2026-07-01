#pragma once
#include "CommandStream.h"

// Interface for in-process simulation backends.
// Swapped in place of the controller adapter (Ros2Adapter) when SPAR_BACKEND=kinematic.
// The assembler, monitor, and node contract are unchanged — they do not know which backend is active.
//
// Contract:
//   reset()      — return world to a fixed start state; reset internal sim clock
//   step(cmd)    — advance world by one tick (dt = 50 ms); impl calls assembler.push_*()
//                  to inject the resulting observation before returning
class SimulatorBackend {
public:
    virtual ~SimulatorBackend() = default;

    virtual void reset() = 0;

    // Advance by one tick using the approved command from the monitor.
    // Implementations must push at least one observation into the assembler before returning
    // so that assembler.build(t) on the next tick finds a fresh sample.
    virtual void step(const CommandStream& approved_cmd) = 0;
};
