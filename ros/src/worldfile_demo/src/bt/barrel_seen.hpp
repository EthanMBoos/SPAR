#pragma once

#include <geometry_msgs/msg/point.hpp>

#include <behaviortree_cpp/condition_node.h>

#include "bt/blackboard_keys.hpp"
#include "bt/stamped.hpp"

namespace worldfile_demo {

class BarrelSeen : public BT::ConditionNode {
 public:
  struct Params {
    double max_age_sec = 6.0;
    double cooldown_sec = 45.0;
  };

  BarrelSeen(const std::string& name, const BT::NodeConfig& config, Params params)
      : BT::ConditionNode(name, config), params_(params) {}

  BT::NodeStatus tick() override {
    double now = 0.0;
    (void)config().blackboard->get<double>(keys::kNowSec, now);
    Stamped<geometry_msgs::msg::Point> barrel;
    const bool have = config().blackboard->get<
        Stamped<geometry_msgs::msg::Point>>(keys::kBarrelPoint, barrel);
    if (!have || !fresh(barrel, now, params_.max_age_sec)) {
      return BT::NodeStatus::FAILURE;
    }
    double last_inspected = -1.0;
    (void)config().blackboard->get<double>(
        keys::kLastInspected, last_inspected);
    if (last_inspected >= 0.0 &&
        now - last_inspected < params_.cooldown_sec) {
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }

 private:
  Params params_;
};

}  // namespace worldfile_demo
