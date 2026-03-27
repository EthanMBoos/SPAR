#pragma once
#include "../../shared/contracts/CommandStream.h"

// Abstract interface for the single-writer controller adapter.
// Only the concrete implementation (e.g. ArdupilotAdapter) links against the
// controller transport. No other module in the system holds a pointer to this
// interface; it is owned exclusively by the rover process.
class ControllerAdapter {
public:
    virtual ~ControllerAdapter() = default;

    virtual bool connect()    = 0;
    virtual void disconnect() = 0;
    virtual bool is_connected() const = 0;

    // Write an approved command to the controller.
    // Called only with commands that have passed the runtime monitor.
    virtual bool write(const CommandStream& cmd) = 0;
};
