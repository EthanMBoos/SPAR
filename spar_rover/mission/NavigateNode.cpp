#include "NavigateNode.h"
#include <algorithm>
#include <cmath>

static constexpr float kHalfPi = 1.57079632679489661923f;

NodeStatus NavigateNode::tick(const GoalContext& goal,
                               const WorldState&  world,
                               CommandStream&     out_cmd)
{
    // WorldState is built by ObservationAssembler before bt.tick(); any_degraded
    // routes to the fallback before we get here, so this is a redundant backstop.
    const Pose& pose = world.pose;
    if (!pose.valid) {
        out_cmd.cmd = {};
        return NodeStatus::Failure;
    }
    // Staleness is already gated by ObservationAssembler::kPoseStalenessLimitUs;
    // any_degraded routes to the fallback before we get here. Re-checking with
    // now_us() adds one tick of drift (different reference time than the assembler)
    // and would cause false failures in degraded-transport scenarios.

    if (start_us_ == 0) start_us_ = goal.timestamp_us;
    if (goal.timestamp_us - start_us_ > cfg_.timeout_us) {
        out_cmd.cmd = {};
        return NodeStatus::Failure;
    }

    // Goal displacement in the world frame (meters), then rotated into the body frame.
    float gx_w = goal.target.x_m - pose.x_m;
    float gy_w = goal.target.y_m - pose.y_m;
    float dist = std::hypot(gx_w, gy_w);
    if (dist < cfg_.arrival_radius_m) {
        out_cmd.cmd = {};
        return NodeStatus::Success;
    }

    float c = std::cos(pose.yaw_rad);
    float s = std::sin(pose.yaw_rad);
    float gx_body =  c * gx_w + s * gy_w;   // forward
    float gy_body = -s * gx_w + c * gy_w;   // left

    // Bearing error to goal, relative to current heading: +left, in radians.
    float heading_err = std::atan2(gy_body, gx_body);

    out_cmd.source_id        = id_;
    out_cmd.cmd.timestamp_us = goal.timestamp_us;
    out_cmd.cmd.throttle     = cfg_.max_throttle;
    // Proportional steer: a 90° error saturates to full steering command.
    out_cmd.cmd.steering     = std::clamp(heading_err / kHalfPi,
                                          -cfg_.max_steering,
                                           cfg_.max_steering);
    return NodeStatus::Running;
}
