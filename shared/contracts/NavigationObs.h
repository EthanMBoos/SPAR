#pragma once
#include "WorldState.h"
#include "GoalContext.h"
#include <vector>
#include <cmath>

// Goal-relative observation for learned navigation policies.
// Returns [dx_m, dy_m, heading_sin, heading_cos, speed_ms]
//
//   dx_m         — meters east  to goal (positive = goal is east)
//   dy_m         — meters north to goal (positive = goal is north)
//   heading_sin  — sin(heading_rad); avoids 0°/360° wrap discontinuity
//   heading_cos  — cos(heading_rad)
//   speed_ms     — current speed in m/s
//
// Field count matches WorldState::to_observation_vector() so model input
// shapes are unchanged. Use this instead of to_observation_vector() for all
// learned nodes — absolute lat/lon has no goal context and near-zero
// episode variance, making it untrainable for goal-directed policies.
inline std::vector<float> make_nav_obs(const WorldState& ws, const GoalContext& goal) {
    static constexpr double kR      = 6371000.0;
    static constexpr double kPi     = 3.14159265358979323846;
    static constexpr double kDegRad = kPi / 180.0;

    double dlat = (goal.target.lat_deg - ws.pose.lat_deg) * kDegRad;
    double dlon = (goal.target.lon_deg - ws.pose.lon_deg) * kDegRad;
    double cos_lat = std::cos(ws.pose.lat_deg * kDegRad);

    float dy_m = static_cast<float>(dlat * kR);
    float dx_m = static_cast<float>(dlon * kR * (cos_lat > 1e-9 ? cos_lat : 1e-9));

    float heading_rad = static_cast<float>(ws.pose.heading_deg * kDegRad);
    float heading_sin = std::sin(heading_rad);
    float heading_cos = std::cos(heading_rad);

    return {dx_m, dy_m, heading_sin, heading_cos, ws.pose.speed_ms};
}
