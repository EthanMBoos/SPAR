#include "RuntimeMonitor.h"
#include <cmath>

static bool is_bad(float v) { return std::isnan(v) || std::isinf(v); }

static uint8_t flag_for(const std::string& v) {
    using D = MonitorDecision;
    if (v == "bounds.nan_inf")           return D::kFlagBoundsNanInf;
    if (v == "bounds.throttle_range")    return D::kFlagBoundsThrottleRange;
    if (v == "bounds.steering_range")    return D::kFlagBoundsSteeringRange;
    if (v == "staleness.cmd_too_old")    return D::kFlagStalenessCmd;
    if (v == "rate_of_change.throttle")  return D::kFlagRateThrottle;
    if (v == "rate_of_change.steering")  return D::kFlagRateSteering;
    return 0;
}

RuntimeMonitor::RuntimeMonitor(MonitorConfig cfg) : cfg_(cfg) {}

MonitorDecision RuntimeMonitor::evaluate(const CommandStream& cmd, uint64_t now_us) {
    MonitorDecision d;
    d.input_cmd    = cmd;
    d.timestamp_us = now_us;

    std::string violated;

    // Halt-class: malformed values — no fallback makes sense, stop immediately.
    if (!check_bounds(cmd.cmd, violated)) {
        d.outcome             = MonitorDecision::Outcome::Halt;
        d.triggered_invariant = violated;
        d.invariant_flags     = flag_for(violated);
        d.output_cmd          = zero_cmd(cmd);
        return d;
    }

    // Fallback-class: stale command — substitute zero and continue.
    if (!check_staleness(cmd.cmd, now_us, violated)) {
        d.outcome             = MonitorDecision::Outcome::Fallback;
        d.triggered_invariant = violated;
        d.invariant_flags     = flag_for(violated);
        d.output_cmd          = zero_cmd(cmd);
        return d;
    }

    // Fallback-class: excessive delta — clamp to max-delta window around last approved.
    // Reverts to last_approved_ on staleness, but clamps (rather than reverts) on rate
    // violation: a rate-of-change limit should limit the rate, not halt the vehicle.
    // last_approved_ is updated to the clamped output so subsequent ticks can ramp further.
    if (has_prev_ && !check_rate_of_change(cmd.cmd, violated)) {
        auto clamp_f = [](float v, float lo, float hi) {
            return v < lo ? lo : (v > hi ? hi : v);
        };
        CommandStream clamped   = cmd;
        clamped.cmd.throttle    = clamp_f(cmd.cmd.throttle,
                                          last_approved_.cmd.throttle - cfg_.max_throttle_delta,
                                          last_approved_.cmd.throttle + cfg_.max_throttle_delta);
        clamped.cmd.steering    = clamp_f(cmd.cmd.steering,
                                          last_approved_.cmd.steering - cfg_.max_steering_delta,
                                          last_approved_.cmd.steering + cfg_.max_steering_delta);
        d.outcome             = MonitorDecision::Outcome::Fallback;
        d.triggered_invariant = violated;
        d.invariant_flags     = flag_for(violated);
        d.output_cmd          = clamped;
        last_approved_        = clamped;
        has_prev_             = true;
        return d;
    }

    d.outcome    = MonitorDecision::Outcome::Passed;
    d.output_cmd = cmd;
    last_approved_ = cmd;
    has_prev_      = true;
    return d;
}

bool RuntimeMonitor::check_bounds(const RoverCommand& c, std::string& violated) const {
    if (is_bad(c.throttle) || is_bad(c.steering)) {
        violated = "bounds.nan_inf";
        return false;
    }
    if (c.throttle < cfg_.throttle_min || c.throttle > cfg_.throttle_max) {
        violated = "bounds.throttle_range";
        return false;
    }
    if (c.steering < cfg_.steering_min || c.steering > cfg_.steering_max) {
        violated = "bounds.steering_range";
        return false;
    }
    return true;
}

bool RuntimeMonitor::check_staleness(const RoverCommand& c, uint64_t now_us,
                                      std::string& violated) const {
    if (c.timestamp_us > 0 && now_us > c.timestamp_us &&
        (now_us - c.timestamp_us) > cfg_.max_cmd_age_us) {
        violated = "staleness.cmd_too_old";
        return false;
    }
    return true;
}

bool RuntimeMonitor::check_rate_of_change(const RoverCommand& c, std::string& violated) const {
    if (std::abs(c.throttle - last_approved_.cmd.throttle) >= cfg_.max_throttle_delta) {
        violated = "rate_of_change.throttle";
        return false;
    }
    if (std::abs(c.steering - last_approved_.cmd.steering) >= cfg_.max_steering_delta) {
        violated = "rate_of_change.steering";
        return false;
    }
    return true;
}

CommandStream RuntimeMonitor::zero_cmd(const CommandStream& src) const {
    CommandStream z   = src;
    z.cmd.throttle    = 0.0f;
    z.cmd.steering    = 0.0f;
    return z;
}
