#pragma once
#include "ControllerAdapter.h"
#include "../../shared/assembler/ObservationAssembler.h"
#include <string>
#include <cstdint>
#include <thread>
#include <atomic>

struct ArdupilotConfig {
    std::string host   = "127.0.0.1";
    uint16_t    port   = 14550;
    uint8_t     sysid  = 255;
    uint8_t     compid = 0;
};

// MAVLink adapter for ArduPilot Rover over UDP.
// Sends CommandStream as SET_POSITION_TARGET velocity set-points.
// Spawns a telemetry thread on connect() that reads GLOBAL_POSITION_INT
// messages and pushes them into ObservationAssembler (not WorldState directly).
//
// Requires: mavlink/c_library_v2 under third_party/mavlink (SPAR_ENABLE_MAVLINK).
class ArdupilotAdapter final : public ControllerAdapter {
public:
    explicit ArdupilotAdapter(ObservationAssembler& assembler, ArdupilotConfig cfg = {});
    ~ArdupilotAdapter() override;

    bool connect()              override;
    void disconnect()           override;
    bool is_connected() const   override { return connected_; }
    bool write(const CommandStream& cmd) override;

private:
    ArdupilotConfig       cfg_;
    ObservationAssembler& assembler_;
    int                   socket_fd_ = -1;
    bool                  connected_ = false;
    std::thread           telem_thread_;
    std::atomic<bool>     telem_running_{false};

    void telemetry_loop();
    bool send_set_position_target(const RoverCommand& cmd);
    bool send_heartbeat();
};
