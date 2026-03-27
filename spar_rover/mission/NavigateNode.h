#pragma once
#include "../../shared/contracts/BTNode.h"
#include <cmath>

struct NavigateNodeConfig {
    float    max_throttle     = 0.6f;
    float    max_steering     = 0.8f;
    float    arrival_radius_m = 1.0f;
    uint64_t timeout_us       = 60'000'000; // 60 s
    uint64_t max_pose_age_us  = 200'000;    // 200 ms — reject stale EKF output
};

// Hand-coded navigate-to-waypoint BT node.
// Produces throttle/steering commands pointing the rover toward goal.target.
// Returns Success when within arrival_radius_m; Failure on timeout.
class NavigateNode final : public BTNode {
public:
    explicit NavigateNode(NavigateNodeConfig cfg = NavigateNodeConfig{})
        : cfg_(cfg), start_us_(0), id_(next_id_++) {}

    NodeStatus tick(const GoalContext& goal,
                    const WorldState&  world,
                    CommandStream&     out_cmd) override;

    void        reset()    override { start_us_ = 0; }
    const char* name()     const override { return "NavigateNode"; }
    uint64_t    node_id()  const override { return id_; }

private:
    NavigateNodeConfig cfg_;
    uint64_t start_us_;
    uint64_t id_;

    inline static uint64_t next_id_ = 1;

    static float bearing_error_deg(double from_lat, double from_lon,
                                   double to_lat,   double to_lon);
    static float distance_m(double lat1, double lon1,
                             double lat2, double lon2);
};
