#include "KinematicBackend.h"
#include <chrono>
#include <cmath>

static uint64_t wall_now_us() {
    using namespace std::chrono;
    return static_cast<uint64_t>(
        duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count());
}

static constexpr double kMetersPerDegreeLat = 111320.0;
static constexpr double kPi                 = 3.14159265358979323846;

static double deg_to_rad(double d) { return d * kPi / 180.0; }
static float  clamp(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

KinematicBackend::KinematicBackend(ObservationAssembler& assembler,
                                   KinematicParams       params,
                                   DegradationParams     degradation)
    : params_(params)
    , degraded_pose_([&assembler](uint64_t ts, const Pose& p) {
          assembler.push_pose(ts, p);
      }, degradation)
{
    reset();
}

void KinematicBackend::reset() {
    lat_deg_     = params_.start_lat_deg;
    lon_deg_     = params_.start_lon_deg;
    heading_deg_ = params_.start_heading_deg;
    speed_ms_    = 0.0f;
    // Anchor sim time to wall clock so assembler.build(now_us()) in main.cpp
    // sees pose ages near zero rather than enormous (sim epoch vs. wall epoch).
    sim_time_us_ = wall_now_us();
}

void KinematicBackend::step(const CommandStream& approved_cmd) {
    sim_time_us_ = wall_now_us();
    integrate(approved_cmd.cmd.throttle, approved_cmd.cmd.steering);

    Pose p;
    p.lat_deg      = lat_deg_;
    p.lon_deg      = lon_deg_;
    p.alt_m        = 0.0f;
    p.heading_deg  = heading_deg_;
    p.speed_ms     = speed_ms_;
    p.timestamp_us = sim_time_us_;
    p.valid        = true;

    degraded_pose_.push(p.timestamp_us, p);
}

void KinematicBackend::integrate(float throttle, float steering) {
    const float dt = params_.dt_s;

    // Speed: first-order response toward throttle-commanded target.
    float target_speed = throttle * params_.max_speed_ms;
    float delta_speed  = target_speed - speed_ms_;
    float max_delta    = params_.max_accel_ms2 * dt;
    speed_ms_ += clamp(delta_speed, -max_delta, max_delta);

    // Bicycle model: steering_angle → angular rate via wheelbase geometry.
    // Map steering [-1, 1] to max steer rate (rad/s).
    float angular_rate = steering * params_.max_steer_rate_rs;
    float dheading_deg = angular_rate * dt * (180.0f / static_cast<float>(kPi));
    heading_deg_ = std::fmod(heading_deg_ + dheading_deg, 360.0f);
    if (heading_deg_ < 0.0f) heading_deg_ += 360.0f;

    // Position integration in local flat-earth frame.
    float heading_rad = static_cast<float>(deg_to_rad(heading_deg_));
    float dx_north    = speed_ms_ * dt * std::cos(heading_rad);  // meters north
    float dx_east     = speed_ms_ * dt * std::sin(heading_rad);  // meters east

    double cos_lat = std::cos(deg_to_rad(lat_deg_));
    lat_deg_ += dx_north / kMetersPerDegreeLat;
    lon_deg_ += dx_east  / (kMetersPerDegreeLat * (cos_lat > 1e-9 ? cos_lat : 1e-9));
}
