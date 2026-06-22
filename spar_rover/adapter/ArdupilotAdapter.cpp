#include "ArdupilotAdapter.h"
#include <cstring>
#include <cmath>
#include <cstdio>
#include <chrono>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

// MAVLink headers — requires third_party/mavlink submodule.
// Build will fail here (by design) if the submodule is absent.
#ifdef SPAR_HAVE_MAVLINK
#include <mavlink/v2.0/common/mavlink.h>
#endif

static uint64_t now_us() {
    using namespace std::chrono;
    return static_cast<uint64_t>(
        duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count());
}

ArdupilotAdapter::ArdupilotAdapter(ObservationAssembler& assembler, ArdupilotConfig cfg)
    : cfg_(cfg), assembler_(assembler) {}

ArdupilotAdapter::~ArdupilotAdapter() {
    disconnect();
}

bool ArdupilotAdapter::connect() {
    socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) return false;

    // Bind to any local port before connect() so recv() has a delivery address.
    // Without this, incoming datagrams are silently dropped.
    sockaddr_in local{};
    local.sin_family      = AF_INET;
    local.sin_addr.s_addr = INADDR_ANY;
    local.sin_port        = 0;  // OS assigns an ephemeral port
    if (::bind(socket_fd_, reinterpret_cast<sockaddr*>(&local), sizeof(local)) < 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(cfg_.port);
    addr.sin_addr.s_addr = ::inet_addr(cfg_.host.c_str());

    if (::connect(socket_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    // Set recv timeout so the telemetry thread can check telem_running_ periodically.
    struct timeval tv{};
    tv.tv_usec = 100'000;  // 100 ms
    ::setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    connected_     = true;
    telem_running_ = true;
    telem_thread_  = std::thread(&ArdupilotAdapter::telemetry_loop, this);
    send_heartbeat();
    return true;
}

void ArdupilotAdapter::disconnect() {
    telem_running_.store(false);
    if (telem_thread_.joinable()) telem_thread_.join();
    if (socket_fd_ >= 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
    }
    connected_ = false;
}

bool ArdupilotAdapter::write(const CommandStream& cmd) {
    if (!connected_) return false;
    return send_set_position_target(cmd.cmd);
}

bool ArdupilotAdapter::send_set_position_target(const RoverCommand& cmd) {
#ifdef SPAR_HAVE_MAVLINK
    mavlink_message_t msg;
    mavlink_set_position_target_local_ned_t tgt{};

    // Bits 0-2: ignore position (PX,PY,PZ). Bit 3: USE vx. Bits 4-9: ignore remaining vel/acc/yaw.
    // Bit 10: USE yaw_rate. Result: 0x3F7 = 0b11111110111.
    tgt.type_mask        = 0x3F7;
    tgt.coordinate_frame = MAV_FRAME_BODY_NED;
    tgt.time_boot_ms     = static_cast<uint32_t>(cmd.timestamp_us / 1000);
    tgt.vx               = cmd.throttle * 2.0f;  // m/s, max ~2 m/s for rover demo
    tgt.yaw_rate         = cmd.steering * 1.0f;  // rad/s
    tgt.target_system    = 1;
    tgt.target_component = 1;

    mavlink_msg_set_position_target_local_ned_encode(cfg_.sysid, cfg_.compid, &msg, &tgt);

    uint8_t  buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    return ::send(socket_fd_, buf, len, 0) == len;
#else
    // MAVLink submodule not present. Log and return true so the rover loop
    // continues in stub mode without crashing.
    (void)cmd;
    return true;
#endif
}

bool ArdupilotAdapter::send_heartbeat() {
#ifdef SPAR_HAVE_MAVLINK
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(cfg_.sysid, cfg_.compid, &msg,
                               MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID,
                               MAV_MODE_FLAG_SAFETY_ARMED, 0, MAV_STATE_ACTIVE);
    uint8_t  buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    return ::send(socket_fd_, buf, len, 0) == len;
#else
    return true;
#endif
}

void ArdupilotAdapter::telemetry_loop() {
#ifdef SPAR_HAVE_MAVLINK
    uint8_t           buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t msg;
    mavlink_status_t  status{};
    uint64_t          last_heartbeat_us = 0;

    while (telem_running_.load()) {
        // Send 1 Hz heartbeat so ArduPilot does not drop the GCS link after ~3 s.
        uint64_t t = now_us();
        if (t - last_heartbeat_us >= 1'000'000) {
            send_heartbeat();
            last_heartbeat_us = t;
        }

        ssize_t n = ::recv(socket_fd_, buf, sizeof(buf), 0);
        if (n <= 0) continue;  // timeout or error — check flag and loop

        for (ssize_t i = 0; i < n; ++i) {
            if (!mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status))
                continue;
            if (msg.msgid != MAVLINK_MSG_ID_GLOBAL_POSITION_INT)
                continue;

            mavlink_global_position_int_t gp{};
            mavlink_msg_global_position_int_decode(&msg, &gp);

            // On first message: measure the offset from ArduPilot boot clock to our
            // monotonic clock. Error is ~1 ms (loopback recv latency), well within
            // the 200 ms staleness bound. This converts gp.time_boot_ms to local time
            // for all subsequent samples — the assembler's core guarantee requires it.
            if (!boot_offset_set_) {
                boot_ms_offset_  = now_us() - static_cast<uint64_t>(gp.time_boot_ms) * 1000;
                boot_offset_set_ = true;
            }

            Pose p;
            p.lat_deg     = gp.lat / 1e7;
            p.lon_deg     = gp.lon / 1e7;
            p.alt_m       = gp.alt / 1000.0f;
            p.heading_deg = gp.hdg / 100.0f;  // cdeg → deg
            p.speed_ms    = std::sqrt(static_cast<float>(gp.vx * gp.vx +
                                                          gp.vy * gp.vy)) / 100.0f;
            p.valid        = true;
            p.timestamp_us = static_cast<uint64_t>(gp.time_boot_ms) * 1000 + boot_ms_offset_;

            assembler_.push_pose(p.timestamp_us, p);
        }
    }
#else
    // Stub: emit a fixed position on each loop iteration so the ring buffer
    // always has a fresh sample. In real SITL this is replaced by live MAVLink.
    Pose p;
    p.lat_deg     = 33.7756;
    p.lon_deg     = -84.3963;
    p.alt_m       = 0.0f;
    p.heading_deg = 0.0f;
    p.speed_ms    = 0.0f;
    p.valid       = true;

    while (telem_running_.load()) {
        p.timestamp_us = now_us();
        assembler_.push_pose(p.timestamp_us, p);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));  // ~20 Hz stub rate
    }
#endif
}
