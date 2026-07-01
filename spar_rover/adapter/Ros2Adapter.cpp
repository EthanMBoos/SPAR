#include "Ros2Adapter.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <optional>
#include <vector>

// Targets zenoh-cpp 1.x. spar_rover links zenoh-cpp only (no rclcpp / no DDS):
// the vehicle's ROS 2 stack publishes over rmw_zenoh, so messages arrive as raw
// CDR that we decode by hand. Keyexprs and per-field CDR layout must be validated
// on the robot before first drive — see Ros2Config.
#include <zenoh.hxx>

static uint64_t now_us() {
    using namespace std::chrono;
    return static_cast<uint64_t>(
        duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count());
}

// ── Minimal CDR (little-endian) reader/writer ──────────────────────────────────
// Offsets are measured from the start of the CDR body, i.e. after the 4-byte
// encapsulation header. Each primitive is aligned to its own size relative to
// that body start, per the OMG CDR rules ROS 2 serialization uses.
namespace {

class CdrWriter {
public:
    CdrWriter() {
        // Encapsulation header: PLAIN_CDR, little-endian (0x0001), no options.
        buf_ = {0x00, 0x01, 0x00, 0x00};
    }
    void f64(double v) {
        align(8);
        uint8_t tmp[8];
        std::memcpy(tmp, &v, 8);
        buf_.insert(buf_.end(), tmp, tmp + 8);
    }
    std::vector<uint8_t> take() { return std::move(buf_); }

private:
    void align(size_t a) {
        size_t body = buf_.size() - 4;
        while (body % a != 0) { buf_.push_back(0); ++body; }
    }
    std::vector<uint8_t> buf_;
};

class CdrReader {
public:
    CdrReader(const uint8_t* data, size_t n) : p_(data), n_(n) {
        // pos_ starts at the body; require little-endian encapsulation.
        little_ = (n_ >= 2 && p_[1] == 0x01);
        pos_    = 4;
    }
    bool little_endian() const { return little_; }

    double f64() {
        align(8);
        double v = 0.0;
        if (pos_ + 8 <= n_) std::memcpy(&v, p_ + pos_, 8);
        pos_ += 8;
        return v;
    }
    int32_t i32() {
        align(4);
        int32_t v = 0;
        if (pos_ + 4 <= n_) std::memcpy(&v, p_ + pos_, 4);
        pos_ += 4;
        return v;
    }
    uint32_t u32() {
        align(4);
        uint32_t v = 0;
        if (pos_ + 4 <= n_) std::memcpy(&v, p_ + pos_, 4);
        pos_ += 4;
        return v;
    }
    void skip_string() {
        uint32_t len = u32();          // length includes the null terminator
        pos_ += len;
    }
    void skip_f64(size_t count) {
        for (size_t i = 0; i < count; ++i) f64();
    }

private:
    void align(size_t a) {
        size_t body = pos_ - 4;
        while (body % a != 0) { ++pos_; ++body; }
    }
    const uint8_t* p_;
    size_t         n_;
    size_t         pos_;
    bool           little_;
};

} // namespace

// ── Impl ───────────────────────────────────────────────────────────────────────

struct Ros2Adapter::Impl {
    ObservationAssembler& assembler;
    Ros2Config            cfg;
    std::atomic<bool>     connected{false};

    std::optional<zenoh::Session>    session;
    std::optional<zenoh::Publisher>  cmd_pub;
    std::optional<zenoh::Subscriber<void>> odom_sub;

    // Align the estimator's message stamps to the local steady clock used by
    // ObservationAssembler::build(). Measured once on the first sample; preserves
    // relative jitter while removing the epoch offset, so staleness stays correct.
    uint64_t stamp_offset_us = 0;
    bool     offset_set      = false;

    explicit Impl(ObservationAssembler& a, Ros2Config c) : assembler(a), cfg(std::move(c)) {}

    void on_odom(const zenoh::Sample& sample) {
        std::vector<uint8_t> data = sample.get_payload().as_vector();
        CdrReader r(data.data(), data.size());
        if (!r.little_endian()) return;  // big-endian CDR not supported

        // TODO(review #2): CdrReader returns 0 for reads past the end without
        //   signaling, so a short or wrong-schema payload silently produces a
        //   valid (0,0,0) pose that the assembler accepts. Track an overrun flag
        //   in CdrReader and drop the sample (return before push_pose) if any read
        //   ran past the buffer. Validate on-robot alongside the keyexpr/CDR layout.

        // header.stamp
        int32_t  sec     = r.i32();
        uint32_t nanosec = r.u32();
        r.skip_string();                 // header.frame_id
        r.skip_string();                 // child_frame_id

        // pose.pose.position
        double px = r.f64();
        double py = r.f64();
        double pz = r.f64();
        // pose.pose.orientation (quaternion)
        double qx = r.f64();
        double qy = r.f64();
        double qz = r.f64();
        double qw = r.f64();
        r.skip_f64(36);                  // pose.covariance

        // twist.twist.linear.x is forward speed in the child (base_link) frame.
        double vx = r.f64();
        // remaining twist fields (linear.y/z, angular.x/y/z) + covariance unused

        uint64_t stamp_us = static_cast<uint64_t>(sec) * 1'000'000ull +
                            static_cast<uint64_t>(nanosec) / 1000ull;
        if (!offset_set) {
            stamp_offset_us = now_us() - stamp_us;
            offset_set      = true;
        }

        Pose p;
        p.x_m      = static_cast<float>(px);
        p.y_m      = static_cast<float>(py);
        p.z_m      = static_cast<float>(pz);
        // Planar yaw from quaternion (ENU, CCW from +x).
        p.yaw_rad  = static_cast<float>(
            std::atan2(2.0 * (qw * qz + qx * qy),
                       1.0 - 2.0 * (qy * qy + qz * qz)));
        p.speed_ms = static_cast<float>(vx);
        p.timestamp_us = stamp_us + stamp_offset_us;
        p.valid    = true;

        assembler.push_pose(p.timestamp_us, p);
    }
};

// ── ControllerAdapter interface ────────────────────────────────────────────────

Ros2Adapter::Ros2Adapter(ObservationAssembler& assembler, Ros2Config cfg)
    : impl_(std::make_unique<Impl>(assembler, std::move(cfg))) {}

Ros2Adapter::~Ros2Adapter() { disconnect(); }

bool Ros2Adapter::connect() {
    try {
        impl_->session.emplace(zenoh::Session::open(zenoh::Config::create_default()));
        impl_->cmd_pub.emplace(
            impl_->session->declare_publisher(zenoh::KeyExpr(impl_->cfg.cmd_vel_key)));
        impl_->odom_sub.emplace(impl_->session->declare_subscriber(
            zenoh::KeyExpr(impl_->cfg.odom_key),
            [this](const zenoh::Sample& s) { impl_->on_odom(s); },
            zenoh::closures::none));
        impl_->connected.store(true);
        return true;
    } catch (...) {
        disconnect();
        return false;
    }
}

void Ros2Adapter::disconnect() {
    impl_->connected.store(false);
    impl_->odom_sub.reset();
    impl_->cmd_pub.reset();
    impl_->session.reset();
}

bool Ros2Adapter::is_connected() const { return impl_->connected.load(); }

bool Ros2Adapter::write(const CommandStream& cmd) {
    if (!impl_->connected.load()) return false;

    const ControllerEnvelope& env = impl_->cfg.envelope;
    double vx    = static_cast<double>(cmd.cmd.throttle) * env.max_linear_mps;
    double wz    = static_cast<double>(cmd.cmd.steering) * env.max_angular_radps;

    // geometry_msgs/Twist: linear{x,y,z}, angular{x,y,z}.
    // TODO(review #3): some ROS 2 (Jazzy) diff-drive controllers subscribe to
    //   cmd_vel as geometry_msgs/TwistStamped, not plain Twist. If the vehicle
    //   ignores these, prepend a std_msgs/Header (stamp + frame_id) and switch
    //   the message type. Confirm the expected type on-robot.
    CdrWriter w;
    w.f64(vx); w.f64(0.0); w.f64(0.0);
    w.f64(0.0); w.f64(0.0); w.f64(wz);

    try {
        impl_->cmd_pub->put(zenoh::Bytes(w.take()));
        return true;
    } catch (...) {
        return false;
    }
}
