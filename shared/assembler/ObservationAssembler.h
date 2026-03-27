#pragma once
#include "../contracts/WorldState.h"
#include <array>
#include <mutex>
#include <cstdint>

// ── Standalone components (no SPAR/MAVLink/ArduPilot types) ─────────────────

// A single timestamped sample from one source.
// timestamp_us must be the sensor's capture time, not the callback receipt time.
template <typename T>
struct Sample {
    uint64_t timestamp_us{0};
    T        value{};
    bool     valid{false};
};

// Fixed-depth ring buffer for one async source.
// push() is called from the sensor thread; latest_at_or_before() from the tick thread.
template <typename T, std::size_t N = 8>
class SourceBuffer {
public:
    void push(uint64_t timestamp_us, T value) {
        std::lock_guard lock(mu_);
        buf_[head_] = Sample<T>{timestamp_us, std::move(value), true};
        head_       = (head_ + 1) % N;
        if (count_ < N) ++count_;
    }

    // Returns the most recent sample whose timestamp <= t.
    // Returns an invalid Sample{} if no such sample exists.
    Sample<T> latest_at_or_before(uint64_t t) const {
        std::lock_guard lock(mu_);
        // Walk newest-to-oldest; first hit at or before t is the answer.
        for (std::size_t i = 0; i < count_; ++i) {
            std::size_t   idx = (head_ + N - 1 - i) % N;
            const auto&   s   = buf_[idx];
            if (s.valid && s.timestamp_us <= t)
                return s;
        }
        return {};  // buffer empty or all samples newer than t
        // TODO: for larger N a binary search over a sorted snapshot is faster
    }

private:
    mutable std::mutex       mu_;
    std::array<Sample<T>, N> buf_{};
    std::size_t              head_{0};
    std::size_t              count_{0};
};

// ── SPAR-specific assembler ──────────────────────────────────────────────────

// Per-source verdict carried inside AssembledSnapshot.
struct SourceStatus {
    uint64_t age_us  {0};
    bool     degraded{false};  // true if age_us > bound, or source never pushed
};

// One time-coherent WorldState built per tick, plus per-source staleness.
struct AssembledSnapshot {
    WorldState   state;
    SourceStatus pose_status;
    bool         any_degraded{false};
    // TODO: add SourceStatus for each additional domain (obstacles, battery, …)
};

// Builds one AssembledSnapshot per tick via latest_at_or_before(t).
// Sensor/telemetry threads call push_*(); the tick thread calls build().
class ObservationAssembler {
public:
    // How old a pose sample may be before the source is marked degraded.
    // Derived from SR2_POSITION=10 Hz: worst-case inter-arrival ~100 ms plus jitter budget.
    // TODO: make runtime-configurable per deployment.
    static constexpr uint64_t kPoseStalenessLimitUs = 200'000;  // 200 ms

    // Called from the telemetry thread.
    // timestamp_us should be the sensor's capture time.
    // TODO: for MAVLink sources, convert gp.time_boot_ms to our monotonic clock
    //       (requires a one-time TIMESYNC-based offset measurement at connect).
    //       Currently set to receipt time (now_us()) as a stub.
    void push_pose(uint64_t timestamp_us, Pose pose) {
        pose_buf_.push(timestamp_us, std::move(pose));
    }

    // Called once per tick before bt.tick().
    // t is the tick's reference time (now_us() at tick start).
    AssembledSnapshot build(uint64_t t) const {
        AssembledSnapshot snap{};

        Sample<Pose> s = pose_buf_.latest_at_or_before(t);
        if (s.valid) {
            snap.state.pose           = s.value;
            snap.pose_status.age_us   = t - s.timestamp_us;
            snap.pose_status.degraded = snap.pose_status.age_us > kPoseStalenessLimitUs;
        } else {
            snap.pose_status.degraded = true;  // no sample has arrived yet
        }

        snap.any_degraded = snap.pose_status.degraded;

        // TODO: add additional sources following the same pattern:
        //   Sample<ObstacleMap> o = obstacle_buf_.latest_at_or_before(t);
        //   snap.state.obstacles          = o.value;
        //   snap.obstacles_status.age_us  = o.valid ? t - o.timestamp_us : UINT64_MAX;
        //   snap.obstacles_status.degraded = !o.valid || snap.obstacles_status.age_us > kObstacleStalenessLimitUs;
        //   snap.any_degraded |= snap.obstacles_status.degraded;

        return snap;
    }

private:
    SourceBuffer<Pose> pose_buf_;
    // TODO: SourceBuffer<ObstacleMap>  obstacle_buf_;
    // TODO: SourceBuffer<BatteryState> battery_buf_;
};
