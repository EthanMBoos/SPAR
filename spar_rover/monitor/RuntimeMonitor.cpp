#include "RuntimeMonitor.h"
#include <cmath>

static bool is_bad(float v) { return std::isnan(v) || std::isinf(v); }

RuntimeMonitor::RuntimeMonitor(MonitorConfig cfg) : cfg_(cfg) {}

MonitorDecision RuntimeMonitor::evaluate(const CommandStream& cmd, uint64_t now_us) {
    MonitorDecision d;
    d.input_cmd           = cmd;
    d.active_invariant_set = active_invariant_names();
    d.timestamp_us         = now_us;

    std::string violated;

    // Halt-class: malformed values — no fallback makes sense, stop immediately.
    if (!check_bounds(cmd.cmd, violated)) {
        d.outcome             = MonitorDecision::Outcome::Halt;
        d.triggered_invariant = violated;
        d.output_cmd          = zero_cmd(cmd);
        return d;
    }

    // Fallback-class: stale command — substitute zero and continue.
    if (!check_staleness(cmd.cmd, now_us, violated)) {
        d.outcome             = MonitorDecision::Outcome::Fallback;
        d.triggered_invariant = violated;
        d.output_cmd          = zero_cmd(cmd);
        return d;
    }

    // Fallback-class: excessive delta — clamp to last approved and continue.
    if (has_prev_ && !check_rate_of_change(cmd.cmd, violated)) {
        d.outcome             = MonitorDecision::Outcome::Fallback;
        d.triggered_invariant = violated;
        // Clamp to last approved rather than zeroing: smoother recovery.
        d.output_cmd          = last_approved_;
        d.output_cmd.cmd.timestamp_us = now_us;
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
    if (std::abs(c.throttle - last_approved_.cmd.throttle) > cfg_.max_throttle_delta) {
        violated = "rate_of_change.throttle";
        return false;
    }
    if (std::abs(c.steering - last_approved_.cmd.steering) > cfg_.max_steering_delta) {
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

std::vector<std::string> RuntimeMonitor::active_invariant_names() const {
    return {
        "bounds.nan_inf",
        "bounds.throttle_range",
        "bounds.steering_range",
        "staleness.cmd_too_old",
        "rate_of_change.throttle",
        "rate_of_change.steering",
    };
}
