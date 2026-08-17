#pragma once

#include <string>

#include <behaviortree_cpp/action_node.h>

namespace worldfile_demo {

class Idle : public BT::StatefulActionNode {
 public:
  Idle(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus onStart() override { return onRunning(); }
  BT::NodeStatus onRunning() override { return BT::NodeStatus::RUNNING; }
  void onHalted() override {}
};

}  // namespace worldfile_demo
