#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>

namespace worldfile_demo {

class NavigateLeaf : public BT::StatefulActionNode {
 public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  NavigateLeaf(const std::string& name, const BT::NodeConfig& config,
               rclcpp::Node& node, double retry_cooldown_sec,
               double hold_success_sec = 0.0);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

 protected:
  virtual std::optional<geometry_msgs::msg::PoseStamped> next_goal() = 0;
  virtual void on_result(bool succeeded) { (void)succeeded; }
  rclcpp::Node& node_;

 private:
  void send_goal(const geometry_msgs::msg::PoseStamped& pose);
  void reset_goal_state();
  bool in_cooldown() const;
  bool in_success_hold() const;

  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  GoalHandle::SharedPtr goal_handle_;
  std::optional<rclcpp_action::ResultCode> result_;
  uint64_t goal_seq_ = 0;
  std::optional<rclcpp::Time> last_failure_;
  std::optional<rclcpp::Time> last_success_;
  rclcpp::Duration retry_cooldown_;
  rclcpp::Duration hold_success_;
};

class RoundsLeaf : public NavigateLeaf {
 public:
  struct Waypoint {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
  };

  RoundsLeaf(const std::string& name, const BT::NodeConfig& config,
             rclcpp::Node& node, std::vector<Waypoint> waypoints,
             std::string goal_frame, double retry_cooldown_sec,
             int max_retries);

 protected:
  std::optional<geometry_msgs::msg::PoseStamped> next_goal() override;
  void on_result(bool succeeded) override;

 private:
  std::vector<Waypoint> waypoints_;
  std::string goal_frame_;
  size_t index_ = 0;
  int consecutive_failures_ = 0;
  int max_retries_;
};

class DockLeaf : public NavigateLeaf {
 public:
  DockLeaf(const std::string& name, const BT::NodeConfig& config,
           rclcpp::Node& node, RoundsLeaf::Waypoint dock,
           std::string goal_frame, double retry_cooldown_sec);

 protected:
  std::optional<geometry_msgs::msg::PoseStamped> next_goal() override;

 private:
  RoundsLeaf::Waypoint dock_;
  std::string goal_frame_;
};

class InspectLeaf : public NavigateLeaf {
 public:
  struct Params {
    double standoff_m = 1.8;
  };

  InspectLeaf(const std::string& name, const BT::NodeConfig& config,
              rclcpp::Node& node, tf2_ros::Buffer& tf_buffer,
              std::string goal_frame, std::string base_frame,
              double retry_cooldown_sec, Params params);

 protected:
  std::optional<geometry_msgs::msg::PoseStamped> next_goal() override;
  void on_result(bool succeeded) override;

 private:
  tf2_ros::Buffer& tf_buffer_;
  std::string goal_frame_;
  std::string base_frame_;
  Params params_;
};

class HoldLeaf : public BT::StatefulActionNode {
 public:
  HoldLeaf(const std::string& name, const BT::NodeConfig& config,
           rclcpp::Node& node);
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override {}

 private:
  rclcpp::Node& node_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
};

}  // namespace worldfile_demo
