#pragma once
#include "CommandStream.h"

// Single-writer interface to a vehicle controller.
// The concrete adapter (e.g. Ros2Adapter) is the ONLY module in the system that
// links against the controller transport — bypass is architecturally impossible,
// not merely prohibited. Named by transport, not by vehicle: one Ros2Adapter
// serves any ROS 2 diff-drive base (Husky, Jackal, ...).
//
// The adapter is bidirectional at the transport layer: write() sends approved
// commands to the controller, and its inbound path feeds the external state
// estimate into the assembler (SPAR consumes estimated state, it never estimates).
//
// Contract:
//   connect()    — open the transport; start the inbound estimate feed. false on failure.
//   disconnect() — stop the feed and release the transport.
//   write(cmd)   — send one approved command to the controller. false if not connected.
class ControllerAdapter {
public:
    virtual ~ControllerAdapter() = default;

    virtual bool connect()                          = 0;
    virtual void disconnect()                       = 0;
    virtual bool is_connected() const               = 0;
    virtual bool write(const CommandStream& cmd)    = 0;
};
