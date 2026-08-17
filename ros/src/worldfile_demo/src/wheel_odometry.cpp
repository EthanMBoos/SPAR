// Integrate quantized Husky wheel encoders into a local odometry measurement.
//
// This node deliberately does not publish TF. robot_localization owns the
// odom -> base_link estimate after fusing this measurement with the IMU.

#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace worldfile_demo {

class WheelOdometry : public rclcpp::Node {
public:
  WheelOdometry() : rclcpp::Node("wheel_odometry") {
    declare_parameter("wheel_radius_m", 0.1651);
    declare_parameter("track_width_m", 0.555);
    wheel_radius_ = get_parameter("wheel_radius_m").as_double();
    track_width_ = get_parameter("track_width_m").as_double();
    if (wheel_radius_ <= 0.0 || track_width_ <= 0.0) {
      throw std::runtime_error("wheel geometry must be positive");
    }

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        "platform/wheel_odometry", rclcpp::QoS(10));
    joints_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "platform/joint_states", rclcpp::QoS(10),
        [this](const sensor_msgs::msg::JointState &msg) { onJoints(msg); });
  }

private:
  static constexpr std::array<const char *, 4> kJointNames = {
      "front_left_wheel_joint", "rear_left_wheel_joint",
      "front_right_wheel_joint", "rear_right_wheel_joint"};

  void onJoints(const sensor_msgs::msg::JointState &msg) {
    if (msg.name.size() != msg.position.size()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "joint-state names and positions differ in size");
      return;
    }
    std::unordered_map<std::string, double> positions;
    for (std::size_t index = 0; index < msg.name.size(); ++index) {
      positions[msg.name[index]] = msg.position[index];
    }
    std::array<double, 4> wheel{};
    for (std::size_t index = 0; index < kJointNames.size(); ++index) {
      const auto found = positions.find(kJointNames[index]);
      if (found == positions.end() || !std::isfinite(found->second)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "waiting for all four finite wheel encoders");
        return;
      }
      wheel[index] = found->second * wheel_radius_;
    }

    const rclcpp::Time stamp(msg.header.stamp, RCL_ROS_TIME);
    const double left = (wheel[0] + wheel[1]) / 2.0;
    const double right = (wheel[2] + wheel[3]) / 2.0;
    if (initialized_ && stamp < last_stamp_) {
      // A restarted simulator rewinds /clock and its wheel angles. Start a
      // fresh local odometry frame instead of joining two simulation runs.
      x_ = 0.0;
      y_ = 0.0;
      yaw_ = 0.0;
    }
    if (!initialized_ || stamp <= last_stamp_) {
      initialized_ = true;
      last_left_ = left;
      last_right_ = right;
      last_stamp_ = stamp;
      publish(stamp, 0.0, 0.0);
      return;
    }

    const double dt = (stamp - last_stamp_).seconds();
    const double delta_left = left - last_left_;
    const double delta_right = right - last_right_;
    const double distance = (delta_right + delta_left) / 2.0;
    const double delta_yaw = (delta_right - delta_left) / track_width_;
    x_ += distance * std::cos(yaw_ + delta_yaw / 2.0);
    y_ += distance * std::sin(yaw_ + delta_yaw / 2.0);
    yaw_ = std::atan2(std::sin(yaw_ + delta_yaw),
                     std::cos(yaw_ + delta_yaw));
    last_left_ = left;
    last_right_ = right;
    last_stamp_ = stamp;
    publish(stamp, distance / dt, delta_yaw / dt);
  }

  void publish(const rclcpp::Time &stamp, double linear, double angular) {
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_link";
    msg.pose.pose.position.x = x_;
    msg.pose.pose.position.y = y_;
    msg.pose.pose.orientation.z = std::sin(yaw_ / 2.0);
    msg.pose.pose.orientation.w = std::cos(yaw_ / 2.0);
    msg.twist.twist.linear.x = linear;
    msg.twist.twist.angular.z = angular;

    // Non-zero covariances make the measurement's simulated precision
    // explicit. The kinematic ground plant does not model wheel slip.
    msg.pose.covariance[0] = 0.0025;
    msg.pose.covariance[7] = 0.0025;
    msg.pose.covariance[35] = 0.01;
    msg.twist.covariance[0] = 0.0004;
    msg.twist.covariance[7] = 0.0004;
    msg.twist.covariance[35] = 0.0016;
    odom_pub_->publish(msg);
  }

  double wheel_radius_ = 0.0;
  double track_width_ = 0.0;
  bool initialized_ = false;
  double last_left_ = 0.0;
  double last_right_ = 0.0;
  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

}  // namespace worldfile_demo

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<worldfile_demo::WheelOdometry>());
  rclcpp::shutdown();
  return 0;
}
