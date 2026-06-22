#pragma once
#include "../../shared/contracts/CommandStream.h"
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
class ArdupilotAdapter {
public:
    explicit ArdupilotAdapter(ObservationAssembler& assembler, ArdupilotConfig cfg = {});
    ~ArdupilotAdapter();

    bool connect();
    void disconnect();
    bool is_connected() const { return connected_; }
    bool write(const CommandStream& cmd);

private:
    ArdupilotConfig       cfg_;
    ObservationAssembler& assembler_;
    int                   socket_fd_        = -1;
    bool                  connected_        = false;
    std::thread           telem_thread_;
    std::atomic<bool>     telem_running_{false};

    // Offset from ArduPilot boot clock to local monotonic clock (microseconds).
    // Measured on first received GLOBAL_POSITION_INT; used to convert gp.time_boot_ms
    // to local capture timestamps so the assembler's latest_at_or_before(t) is correct.
    uint64_t boot_ms_offset_    = 0;
    bool     boot_offset_set_   = false;

    void telemetry_loop();
    bool send_set_position_target(const RoverCommand& cmd);
    bool send_heartbeat();
};
