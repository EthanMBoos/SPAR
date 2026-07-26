// Convert a hardware-shaped GPS fix into the REP-105 map -> odom correction.
//
// The simulator publishes NavSatFix, not pose truth. This node establishes a
// local ENU origin from the first second of fixes, then compares each GPS
// position with odom -> base_link at the same stamp. The resulting transform
// corrects position only: single-antenna GPS has no yaw, so heading remains
// owned by odometry.

#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace spar_ground {

class TfFromGps : public rclcpp::Node {
public:
  TfFromGps() : rclcpp::Node("tf_from_gps") {
    declare_parameter("map_frame", "map");
    declare_parameter("odom_frame", "odom");
    declare_parameter("base_frame", "base_link");
    declare_parameter("origin_average_sec", 1.0);
    declare_parameter("transform_tolerance", 0.5);

    map_frame_ = get_parameter("map_frame").as_string();
    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    origin_average_sec_ = get_parameter("origin_average_sec").as_double();
    transform_tolerance_ = get_parameter("transform_tolerance").as_double();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        "sensors/gps/fix", rclcpp::QoS(10),
        [this](const sensor_msgs::msg::NavSatFix &msg) { on_fix(msg); });
    retry_timer_ = create_wall_timer(
        std::chrono::milliseconds(20), [this] { process_fix(); });
  }

private:
  static constexpr double kEarthRadiusM = 6378137.0;

  void on_fix(const sensor_msgs::msg::NavSatFix &msg) {
    if (msg.status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX ||
        !std::isfinite(msg.latitude) || !std::isfinite(msg.longitude)) {
      return;
    }

    const rclcpp::Time fix_stamp(msg.header.stamp, RCL_ROS_TIME);
    if (!origin_ready_) {
      if (origin_samples_ == 0) {
        origin_start_ = fix_stamp;
      }
      lat_sum_ += msg.latitude;
      lon_sum_ += msg.longitude;
      ++origin_samples_;
      if ((fix_stamp - origin_start_).seconds() < origin_average_sec_) {
        return;
      }
      origin_lat_deg_ = lat_sum_ / static_cast<double>(origin_samples_);
      origin_lon_deg_ = lon_sum_ / static_cast<double>(origin_samples_);
      origin_ready_ = true;
      RCLCPP_INFO(get_logger(), "GPS origin initialized from %zu fixes",
                  origin_samples_);
    }

    // The odometry and GPS publishers share a sim loop but travel on
    // different ROS topics. Keep the stamped fix until the next odometry
    // sample arrives, so the TF buffer can interpolate at the exact time.
    pending_fix_ = msg;
  }

  void process_fix() {
    if (!pending_fix_) {
      return;
    }
    const rclcpp::Time fix_stamp(pending_fix_->header.stamp, RCL_ROS_TIME);
    if ((now() - fix_stamp).seconds() < 0.04) {
      return;
    }

    const auto msg = *pending_fix_;
    pending_fix_.reset();
    geometry_msgs::msg::TransformStamped odom_to_base;
    try {
      odom_to_base =
          tf_buffer_->lookupTransform(odom_frame_, base_frame_, fix_stamp);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "GPS correction skipped: %s", ex.what());
      return;
    }

    const double lat0 = origin_lat_deg_ * M_PI / 180.0;
    const double north =
        (msg.latitude - origin_lat_deg_) * M_PI / 180.0 * kEarthRadiusM;
    const double east =
        (msg.longitude - origin_lon_deg_) * M_PI / 180.0 * kEarthRadiusM *
        std::cos(lat0);

    geometry_msgs::msg::TransformStamped correction;
    correction.header.stamp =
        fix_stamp + rclcpp::Duration::from_seconds(transform_tolerance_);
    correction.header.frame_id = map_frame_;
    correction.child_frame_id = odom_frame_;
    correction.transform.translation.x =
        east - odom_to_base.transform.translation.x;
    correction.transform.translation.y =
        north - odom_to_base.transform.translation.y;
    correction.transform.rotation.w = 1.0;
    broadcaster_->sendTransform(correction);
  }

  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  double origin_average_sec_ = 1.0;
  double transform_tolerance_ = 0.5;

  bool origin_ready_ = false;
  std::size_t origin_samples_ = 0;
  double lat_sum_ = 0.0;
  double lon_sum_ = 0.0;
  double origin_lat_deg_ = 0.0;
  double origin_lon_deg_ = 0.0;
  rclcpp::Time origin_start_{0, 0, RCL_ROS_TIME};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
  std::optional<sensor_msgs::msg::NavSatFix> pending_fix_;
};

}  // namespace spar_ground

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<spar_ground::TfFromGps>());
  rclcpp::shutdown();
  return 0;
}
