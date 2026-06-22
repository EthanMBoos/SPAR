#pragma once
#include "../../shared/contracts/SimulatorBackend.h"
#include "../../shared/assembler/ObservationAssembler.h"
#include "TransportDegradation.h"

struct KinematicParams {
    // Calibrate against measured ArduPilot GUIDED mode step responses.
    float wheelbase_m       = 0.30f;  // rover wheelbase
    float max_speed_ms      = 2.0f;   // full throttle forward speed
    float max_accel_ms2     = 1.5f;   // peak longitudinal acceleration
    float max_steer_rate_rs = 1.2f;   // max steering rate (rad/s)
    float dt_s              = 0.050f; // tick period (20 Hz)

    // Fixed start pose (GT campus)
    double start_lat_deg    = 33.7756;
    double start_lon_deg    = -84.3963;
    float  start_heading_deg = 0.0f;
};

// Bicycle-model kinematic simulator.
// Integrates throttle/steering by dt each tick and injects the resulting Pose
// into the assembler using a monotonically advancing simulated clock.
// No external process or network — runs entirely in-process.
class KinematicBackend final : public SimulatorBackend {
public:
    explicit KinematicBackend(ObservationAssembler& assembler,
                              KinematicParams       params      = {},
                              DegradationParams     degradation = {});

    void reset() override;
    void step(const CommandStream& approved_cmd) override;

    // Current simulated time in microseconds (starts at 0 on reset).
    uint64_t sim_time_us() const { return sim_time_us_; }

private:
    KinematicParams        params_;
    DegradedSource<Pose>   degraded_pose_;

    // Simulated state
    double   lat_deg_     = 0.0;
    double   lon_deg_     = 0.0;
    float    heading_deg_ = 0.0f;
    float    speed_ms_    = 0.0f;
    uint64_t sim_time_us_ = 0;

    void integrate(float throttle, float steering);
};
