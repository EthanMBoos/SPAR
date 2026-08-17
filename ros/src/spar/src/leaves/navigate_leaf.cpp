#include "leaves/navigate_leaf.hpp"

#include <cmath>
#include <utility>

#include "bt/blackboard_keys.hpp"
#include "bt/stamped.hpp"

namespace spar {
namespace {

geometry_msgs::msg::PoseStamped make_pose(
    double x, double y, double yaw, const std::string& frame,
    const rclcpp::Time& stamp) {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame;
  pose.header.stamp = stamp;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation.z = std::sin(yaw / 2.0);
  pose.pose.orientation.w = std::cos(yaw / 2.0);
  return pose;
}

}  // namespace

NavigateLeaf::NavigateLeaf(
    const std::string& name, const BT::NodeConfig& config,
    rclcpp::Node& node, double retry_cooldown_sec, double hold_success_sec)
    : BT::StatefulActionNode(name, config),
      node_(node),
      retry_cooldown_(rclcpp::Duration::from_seconds(retry_cooldown_sec)),
      hold_success_(rclcpp::Duration::from_seconds(hold_success_sec)) {
  client_ = rclcpp_action::create_client<NavigateToPose>(
      &node, "navigate_to_pose");
}

BT::NodeStatus NavigateLeaf::onStart() {
  if (in_success_hold()) return BT::NodeStatus::SUCCESS;
  if (in_cooldown()) return BT::NodeStatus::FAILURE;
  if (!client_->action_server_is_ready()) {
    RCLCPP_WARN_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 5000,
        "%s: navigate_to_pose action server is not ready",
        registrationName().c_str());
    return BT::NodeStatus::FAILURE;
  }
  auto goal = next_goal();
  if (!goal) return BT::NodeStatus::FAILURE;
  send_goal(*goal);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateLeaf::onRunning() {
  if (!result_) return BT::NodeStatus::RUNNING;
  const bool succeeded = *result_ == rclcpp_action::ResultCode::SUCCEEDED;
  reset_goal_state();
  if (succeeded) {
    last_success_ = node_.now();
  } else {
    last_failure_ = node_.now();
  }
  on_result(succeeded);
  return succeeded ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void NavigateLeaf::onHalted() {
  if (goal_handle_) client_->async_cancel_goal(goal_handle_);
  reset_goal_state();
}

void NavigateLeaf::send_goal(
    const geometry_msgs::msg::PoseStamped& pose) {
  NavigateToPose::Goal goal;
  goal.pose = pose;
  const uint64_t sequence = ++goal_seq_;
  auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  options.goal_response_callback =
      [this, sequence](GoalHandle::SharedPtr handle) {
        if (sequence != goal_seq_) {
          if (handle) client_->async_cancel_goal(handle);
          return;
        }
        if (handle) {
          goal_handle_ = handle;
        } else {
          result_ = rclcpp_action::ResultCode::ABORTED;
        }
      };
  options.result_callback =
      [this, sequence](const GoalHandle::WrappedResult& result) {
        if (sequence == goal_seq_) result_ = result.code;
      };
  client_->async_send_goal(goal, options);
  goal_handle_.reset();
  result_.reset();
}

void NavigateLeaf::reset_goal_state() {
  goal_handle_.reset();
  result_.reset();
  ++goal_seq_;
}

bool NavigateLeaf::in_cooldown() const {
  return last_failure_ &&
         (node_.now() - *last_failure_) < retry_cooldown_;
}

bool NavigateLeaf::in_success_hold() const {
  return hold_success_ > rclcpp::Duration(0, 0) && last_success_ &&
         (node_.now() - *last_success_) < hold_success_;
}

RoundsLeaf::RoundsLeaf(
    const std::string& name, const BT::NodeConfig& config,
    rclcpp::Node& node, std::vector<Waypoint> waypoints,
    std::string goal_frame, double retry_cooldown_sec, int max_retries)
    : NavigateLeaf(name, config, node, retry_cooldown_sec),
      waypoints_(std::move(waypoints)),
      goal_frame_(std::move(goal_frame)),
      max_retries_(max_retries) {}

std::optional<geometry_msgs::msg::PoseStamped> RoundsLeaf::next_goal() {
  const Waypoint& waypoint = waypoints_[index_];
  return make_pose(waypoint.x, waypoint.y, waypoint.yaw,
                   goal_frame_, node_.now());
}

void RoundsLeaf::on_result(bool succeeded) {
  if (succeeded) {
    consecutive_failures_ = 0;
    index_ = (index_ + 1) % waypoints_.size();
    return;
  }
  if (++consecutive_failures_ >= max_retries_) {
    RCLCPP_WARN(node_.get_logger(),
                "Rounds: waypoint %zu failed %d times, skipping it",
                index_, consecutive_failures_);
    consecutive_failures_ = 0;
    index_ = (index_ + 1) % waypoints_.size();
  }
}

DockLeaf::DockLeaf(
    const std::string& name, const BT::NodeConfig& config,
    rclcpp::Node& node, RoundsLeaf::Waypoint dock,
    std::string goal_frame, double retry_cooldown_sec)
    : NavigateLeaf(name, config, node, retry_cooldown_sec, 5.0),
      dock_(dock),
      goal_frame_(std::move(goal_frame)) {}

std::optional<geometry_msgs::msg::PoseStamped> DockLeaf::next_goal() {
  return make_pose(dock_.x, dock_.y, dock_.yaw, goal_frame_, node_.now());
}

InspectLeaf::InspectLeaf(
    const std::string& name, const BT::NodeConfig& config,
    rclcpp::Node& node, tf2_ros::Buffer& tf_buffer,
    std::string goal_frame, std::string base_frame,
    double retry_cooldown_sec, Params params)
    : NavigateLeaf(name, config, node, retry_cooldown_sec),
      tf_buffer_(tf_buffer),
      goal_frame_(std::move(goal_frame)),
      base_frame_(std::move(base_frame)),
      params_(params) {}

std::optional<geometry_msgs::msg::PoseStamped> InspectLeaf::next_goal() {
  Stamped<geometry_msgs::msg::Point> barrel;
  if (!config().blackboard->get<Stamped<geometry_msgs::msg::Point>>(
          keys::kBarrelPoint, barrel)) {
    return std::nullopt;
  }
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform(
        goal_frame_, base_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException& error) {
    RCLCPP_WARN_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 5000,
        "Inspect: no %s->%s transform yet (%s)", goal_frame_.c_str(),
        base_frame_.c_str(), error.what());
    return std::nullopt;
  }
  const double robot_x = transform.transform.translation.x;
  const double robot_y = transform.transform.translation.y;
  const double dx = barrel.value.x - robot_x;
  const double dy = barrel.value.y - robot_y;
  const double distance = std::hypot(dx, dy);
  const double yaw = std::atan2(dy, dx);
  if (distance <= params_.standoff_m) {
    return make_pose(robot_x, robot_y, yaw, goal_frame_, node_.now());
  }
  const double scale = (distance - params_.standoff_m) / distance;
  return make_pose(robot_x + dx * scale, robot_y + dy * scale, yaw,
                   goal_frame_, node_.now());
}

void InspectLeaf::on_result(bool) {
  config().blackboard->set<double>(
      keys::kLastInspected, node_.now().seconds());
}

HoldLeaf::HoldLeaf(
    const std::string& name, const BT::NodeConfig& config,
    rclcpp::Node& node)
    : BT::StatefulActionNode(name, config), node_(node) {
  cmd_pub_ = node.create_publisher<geometry_msgs::msg::TwistStamped>(
      "cmd_vel", 10);
}

BT::NodeStatus HoldLeaf::onStart() { return onRunning(); }

BT::NodeStatus HoldLeaf::onRunning() {
  geometry_msgs::msg::TwistStamped stop;
  stop.header.stamp = node_.now();
  cmd_pub_->publish(stop);
  return BT::NodeStatus::RUNNING;
}

}  // namespace spar
