#pragma once
#include "../../shared/contracts/CommandStream.h"
#include "../../shared/contracts/MonitorDecision.h"
#include <cstdint>
#include <string>

// TODO: these normalized bounds/rates should be derived from the active
//   ControllerEnvelope — the docs describe the monitor's invariants as coming
//   from the controller's accepted envelope, but currently the envelope only
//   feeds the adapter's command mapping. Wire it here so limits track the vehicle.
struct MonitorConfig {
    float    throttle_min        = -1.0f;
    float    throttle_max        =  1.0f;
    float    steering_min        = -1.0f;
    float    steering_max        =  1.0f;
    float    max_throttle_delta  =  0.30f; // per tick
    float    max_steering_delta  =  0.50f; // per tick
    uint64_t max_cmd_age_us      =  100'000; // 100 ms
};

class RuntimeMonitor {
public:
    explicit RuntimeMonitor(MonitorConfig cfg = {});

    MonitorDecision evaluate(const CommandStream& cmd, uint64_t now_us);

    const MonitorConfig& config() const { return cfg_; }

private:
    MonitorConfig cfg_;
    CommandStream last_approved_{};
    bool          has_prev_ = false;

    bool check_bounds(const RoverCommand& cmd, std::string& violated) const;
    bool check_staleness(const RoverCommand& cmd, uint64_t now_us, std::string& violated) const;
    bool check_rate_of_change(const RoverCommand& cmd, std::string& violated) const;

    CommandStream zero_cmd(const CommandStream& src) const;
};
