#include "NavigateNode.h"
#include <algorithm>
#include <cmath>

static constexpr double DEG2RAD = M_PI / 180.0;
static constexpr double EARTH_R  = 6371000.0; // metres

float NavigateNode::distance_m(double lat1, double lon1,
                                double lat2, double lon2) {
    double dlat = (lat2 - lat1) * DEG2RAD;
    double dlon = (lon2 - lon1) * DEG2RAD;
    double a = std::sin(dlat / 2) * std::sin(dlat / 2)
             + std::cos(lat1 * DEG2RAD) * std::cos(lat2 * DEG2RAD)
             * std::sin(dlon / 2) * std::sin(dlon / 2);
    return static_cast<float>(2.0 * EARTH_R * std::asin(std::sqrt(a)));
}

float NavigateNode::bearing_error_deg(double from_lat, double from_lon,
                                       double to_lat,   double to_lon) {
    double dlon = (to_lon - from_lon) * DEG2RAD;
    double x    = std::sin(dlon) * std::cos(to_lat * DEG2RAD);
    double y    = std::cos(from_lat * DEG2RAD) * std::sin(to_lat * DEG2RAD)
                - std::sin(from_lat * DEG2RAD) * std::cos(to_lat * DEG2RAD) * std::cos(dlon);
    return static_cast<float>(std::atan2(x, y) / DEG2RAD);
}

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

    float dist = distance_m(pose.lat_deg, pose.lon_deg,
                             goal.target.lat_deg, goal.target.lon_deg);
    if (dist < cfg_.arrival_radius_m) {
        out_cmd.cmd = {};
        return NodeStatus::Success;
    }

    // Bearing from current position to target, relative to current heading.
    float abs_bearing = bearing_error_deg(pose.lat_deg, pose.lon_deg,
                                          goal.target.lat_deg, goal.target.lon_deg);
    float heading_err = abs_bearing - pose.heading_deg;
    // Normalise to [-180, 180]
    while (heading_err >  180.0f) heading_err -= 360.0f;
    while (heading_err < -180.0f) heading_err += 360.0f;

    out_cmd.source_id        = id_;
    out_cmd.cmd.timestamp_us = goal.timestamp_us;
    out_cmd.cmd.throttle     = cfg_.max_throttle;
    out_cmd.cmd.steering     = std::clamp(heading_err / 90.0f,
                                          -cfg_.max_steering,
                                           cfg_.max_steering);
    return NodeStatus::Running;
}
