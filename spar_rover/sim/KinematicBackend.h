#pragma once
#include "../../shared/contracts/SimulatorBackend.h"
#include "../../shared/contracts/ControllerEnvelope.h"
#include "../../shared/assembler/ObservationAssembler.h"
#include "TransportDegradation.h"

struct KinematicParams {
    // Vehicle dynamics only. The throttle/steering → physical-velocity mapping is
    // NOT here: it comes from the shared ControllerEnvelope so the sim and the
    // hardware adapter map identical normalized commands to identical motion.
    // Calibrate against measured Husky step responses.
    float wheelbase_m   = 0.30f;  // rover wheelbase
    float max_accel_ms2 = 1.5f;   // peak longitudinal acceleration
    float dt_s          = 0.050f; // tick period (20 Hz)

    // Fixed start pose in the local metric frame (ENU, meters / radians).
    float start_x_m    = 0.0f;
    float start_y_m    = 0.0f;
    float start_yaw_rad = 0.0f;
};

// Bicycle-model kinematic simulator.
// Integrates throttle/steering by dt each tick in the local metric frame and
// injects the resulting Pose into the assembler using a monotonically advancing
// simulated clock. No external process or network — runs entirely in-process.
class KinematicBackend final : public SimulatorBackend {
public:
    explicit KinematicBackend(ObservationAssembler& assembler,
                              KinematicParams       params      = {},
                              DegradationParams     degradation = {},
                              ControllerEnvelope    envelope    = {});

    void reset() override;
    void step(const CommandStream& approved_cmd) override;

    // Current simulated time in microseconds (starts at 0 on reset).
    uint64_t sim_time_us() const { return sim_time_us_; }

private:
    KinematicParams        params_;
    ControllerEnvelope     envelope_;
    DegradedSource<Pose>   degraded_pose_;

    // Simulated state (local metric frame, ENU).
    float    x_m_        = 0.0f;
    float    y_m_        = 0.0f;
    float    yaw_rad_    = 0.0f;
    float    speed_ms_   = 0.0f;
    uint64_t sim_time_us_ = 0;

    void integrate(float throttle, float steering);
};
