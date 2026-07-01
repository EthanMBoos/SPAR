#include "KinematicBackend.h"
#include <chrono>
#include <cmath>

static uint64_t wall_now_us() {
    using namespace std::chrono;
    return static_cast<uint64_t>(
        duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count());
}

static constexpr float kPi = 3.14159265358979323846f;

static float clamp(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

KinematicBackend::KinematicBackend(ObservationAssembler& assembler,
                                   KinematicParams       params,
                                   DegradationParams     degradation,
                                   ControllerEnvelope    envelope)
    : params_(params)
    , envelope_(envelope)
    , degraded_pose_([&assembler](uint64_t ts, const Pose& p) {
          assembler.push_pose(ts, p);
      }, degradation)
{
    reset();
}

void KinematicBackend::reset() {
    x_m_      = params_.start_x_m;
    y_m_      = params_.start_y_m;
    yaw_rad_  = params_.start_yaw_rad;
    speed_ms_ = 0.0f;
    // Anchor sim time to wall clock so assembler.build(now_us()) in main.cpp
    // sees pose ages near zero rather than enormous (sim epoch vs. wall epoch).
    sim_time_us_ = wall_now_us();
}

void KinematicBackend::step(const CommandStream& approved_cmd) {
    sim_time_us_ = wall_now_us();
    integrate(approved_cmd.cmd.throttle, approved_cmd.cmd.steering);

    Pose p;
    p.x_m          = x_m_;
    p.y_m          = y_m_;
    p.z_m          = 0.0f;
    p.yaw_rad      = yaw_rad_;
    p.speed_ms     = speed_ms_;
    p.timestamp_us = sim_time_us_;
    p.valid        = true;

    degraded_pose_.push(p.timestamp_us, p);
}

void KinematicBackend::integrate(float throttle, float steering) {
    const float dt = params_.dt_s;

    // Speed: first-order response toward throttle-commanded target.
    // Command → velocity mapping comes from the shared ControllerEnvelope, so the
    // sim and the Ros2Adapter turn the same normalized command into the same motion.
    float target_speed = throttle * envelope_.max_linear_mps;
    float delta_speed  = target_speed - speed_ms_;
    float max_delta    = params_.max_accel_ms2 * dt;
    speed_ms_ += clamp(delta_speed, -max_delta, max_delta);

    // Yaw integrates the commanded yaw rate; steering [-1, 1] → max yaw rate.
    float yaw_rate = steering * envelope_.max_angular_radps;
    yaw_rad_ += yaw_rate * dt;
    if (yaw_rad_ >  kPi) yaw_rad_ -= 2.0f * kPi;
    if (yaw_rad_ < -kPi) yaw_rad_ += 2.0f * kPi;

    // Position integration in the local metric ENU frame (yaw CCW from +x/east).
    x_m_ += speed_ms_ * dt * std::cos(yaw_rad_);
    y_m_ += speed_ms_ * dt * std::sin(yaw_rad_);
}
