#pragma once

// The one pipe from the BT to PX4. Leaves set a map-frame target; a timer
// streams OffboardControlMode + TrajectorySetpoint at 10 Hz (PX4 drops out
// of offboard below 2 Hz, and wants the stream running BEFORE the mode
// switch, so this streams from node start, armed or not). Arm, mode, and
// land are one-shot VehicleCommands.

#include <cmath>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <rclcpp/rclcpp.hpp>

#include "frames.hpp"

namespace spar_air {

class OffboardLink {
public:
  explicit OffboardLink(rclcpp::Node& node) : node_(node) {
    mode_pub_ = node.create_publisher<px4_msgs::msg::OffboardControlMode>(
        "fmu/in/offboard_control_mode", 10);
    setpoint_pub_ = node.create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "fmu/in/trajectory_setpoint", 10);
    command_pub_ = node.create_publisher<px4_msgs::msg::VehicleCommand>(
        "fmu/in/vehicle_command", 10);
    target_ = {0.0, 0.0, 0.0};
    timer_ = rclcpp::create_timer(
        &node, node.get_clock(), rclcpp::Duration::from_seconds(0.1),
        [this] { stream(); });
  }

  // Target in the map frame (ENU); yaw ENU radians.
  void setTarget(double x, double y, double z, double yaw) {
    target_ = {x, y, z};
    yaw_ = yaw;
  }
  Vec3 target() const { return target_; }

  void arm() {
    command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.0);
  }
  void offboardMode() {
    command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
            1.0, 6.0);
  }
  void landCmd() {
    command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
  }
  // 21196 is PX4's force-disarm magic (bypasses the land detector).
  void disarmForce() {
    command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
            0.0, 21196.0);
  }

private:
  void stream() {
    const auto stamp =
        static_cast<uint64_t>(node_.get_clock()->now().nanoseconds() / 1000);

    px4_msgs::msg::OffboardControlMode mode;
    mode.timestamp = stamp;
    mode.position = true;
    mode_pub_->publish(mode);

    // The map origin and EKF origin are both the pad: only swap ENU -> NED.
    const auto ned = enuToNed(target_.x, target_.y, target_.z);
    px4_msgs::msg::TrajectorySetpoint sp;
    sp.timestamp = stamp;
    sp.position = {static_cast<float>(ned.x), static_cast<float>(ned.y),
                   static_cast<float>(ned.z)};
    sp.yaw = static_cast<float>(yawEnuNed(yaw_));
    setpoint_pub_->publish(sp);
  }

  void command(uint32_t cmd, double p1 = 0.0, double p2 = 0.0) {
    px4_msgs::msg::VehicleCommand msg;
    msg.timestamp =
        static_cast<uint64_t>(node_.get_clock()->now().nanoseconds() / 1000);
    msg.command = cmd;
    msg.param1 = static_cast<float>(p1);
    msg.param2 = static_cast<float>(p2);
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 255;
    msg.source_component = 1;
    msg.from_external = true;
    command_pub_->publish(msg);
  }

  rclcpp::Node& node_;
  Vec3 target_;
  double yaw_ = 0.0;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace spar_air
