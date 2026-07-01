#pragma once
#include "WorldState.h"
#include "GoalContext.h"
#include <vector>
#include <cmath>

// Egocentric, goal-relative observation for learned navigation policies.
// Returns [gx_body, gy_body, speed_ms]
//
//   gx_body  — meters to goal, forward in the body frame (positive = ahead)
//   gy_body  — meters to goal, left in the body frame    (positive = to the left)
//   speed_ms — current body-forward speed (m/s)
//
// Body-frame goal displacement makes relative bearing implicit, so no separate
// heading channel is needed and the observation is invariant to absolute position
// and orientation — the standard point-goal navigation observation. Absolute
// pose has no goal context and near-zero episode variance; it is untrainable for
// goal-directed policies.
inline std::vector<float> make_nav_obs(const WorldState& ws, const GoalContext& goal) {
    float gx_w = goal.target.x_m - ws.pose.x_m;   // east
    float gy_w = goal.target.y_m - ws.pose.y_m;   // north

    float c = std::cos(ws.pose.yaw_rad);
    float s = std::sin(ws.pose.yaw_rad);

    float gx_body =  c * gx_w + s * gy_w;         // forward
    float gy_body = -s * gx_w + c * gy_w;         // left

    return {gx_body, gy_body, ws.pose.speed_ms};
}
