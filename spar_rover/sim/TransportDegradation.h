#pragma once
#include <functional>
#include <cstdint>
#include <cmath>
#include <random>

// Per-stream degradation parameters.
// Populate from hardware measurement (MAVLink round-trip distributions on real rover).
// Zero-initialize for clean training (no degradation injected — pass-through).
struct DegradationParams {
    double mean_delay_us  = 0.0;  // added latency (mean)
    double jitter_stddev  = 0.0;  // Gaussian jitter stddev in microseconds
    double dropout_rate   = 0.0;  // fraction of samples dropped [0.0, 1.0]
};

// Wraps a push function and injects delay, jitter, and dropout.
// Sits between KinematicBackend::step() and assembler.push_*().
//
// Clean variant (all params zero): push_fn is called directly with no modification.
// Degraded variant: timestamp is shifted by (mean_delay + jitter sample),
// and dropout_rate fraction of calls are silently discarded.
//
// Note: this is a synchronous degradation model — delays shift the
// timestamp but do not actually defer the call. This is correct for
// the assembler's latest_at_or_before(t) discipline: a sample with a
// shifted (older) timestamp will appear stale relative to the tick's
// reference time, faithfully simulating what degraded transport looks like
// to the assembler.
template <typename T>
class DegradedSource {
public:
    using PushFn = std::function<void(uint64_t, const T&)>;

    DegradedSource(PushFn push_fn, DegradationParams params, uint64_t rng_seed = 42)
        : push_fn_(std::move(push_fn))
        , params_(params)
        , rng_(rng_seed)
        , dropout_dist_(0.0, 1.0)
        , jitter_dist_(0.0, params.jitter_stddev > 0.0 ? params.jitter_stddev : 1.0)
    {}

    void push(uint64_t capture_time_us, const T& value) {
        if (params_.dropout_rate > 0.0 && dropout_dist_(rng_) < params_.dropout_rate)
            return;  // packet dropped

        uint64_t degraded_time = capture_time_us;
        if (params_.mean_delay_us > 0.0 || params_.jitter_stddev > 0.0) {
            double shift = params_.mean_delay_us;
            if (params_.jitter_stddev > 0.0)
                shift += jitter_dist_(rng_);
            // Clamp so timestamp never goes backwards past origin.
            if (shift < 0.0 && static_cast<uint64_t>(-shift) > degraded_time)
                degraded_time = 0;
            else
                degraded_time = static_cast<uint64_t>(
                    static_cast<double>(degraded_time) + shift);
        }

        push_fn_(degraded_time, value);
    }

private:
    PushFn                             push_fn_;
    DegradationParams                  params_;
    std::mt19937_64                    rng_;
    std::uniform_real_distribution<double> dropout_dist_;
    std::normal_distribution<double>   jitter_dist_;
};
