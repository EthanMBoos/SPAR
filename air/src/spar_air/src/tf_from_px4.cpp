// Publish PX4's synchronized estimated odometry as ROS map -> base_link.
// VehicleLocalPosition supplies the GPS-backed WGS84 reference of PX4's
// local NED origin; VehicleOdometry supplies position and attitude from one
// estimator sample. Physical spawn coordinates are not localization inputs.

#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include "frames.hpp"

namespace spar_air {

class TfFromPx4 : public rclcpp::Node {
public:
  TfFromPx4() : Node("tf_from_px4") {
    declare_parameter("map_frame", "map");
    declare_parameter("base_frame", "base_link");
    declare_parameter("camera_frame", "camera_0_link");
    declare_parameter("camera_pitch_down_deg", 45.0);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    declare_parameter("datum_latitude_deg", nan);
    declare_parameter("datum_longitude_deg", nan);
    declare_parameter("datum_altitude_m", nan);

    map_frame_ = get_parameter("map_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    transform_ = std::make_unique<Px4MapTransform>(spar_geodesy::Geodetic{
        get_parameter("datum_latitude_deg").as_double(),
        get_parameter("datum_longitude_deg").as_double(),
        get_parameter("datum_altitude_m").as_double(),
    });

    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_broadcaster_ =
        std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

    // The camera mount matches x2_camera in sim/robots/x2.xml.
    const double half = 0.5 * get_parameter("camera_pitch_down_deg").as_double()
                        * spar_geodesy::kPi / 180.0;
    geometry_msgs::msg::TransformStamped camera;
    camera.header.frame_id = base_frame_;
    camera.child_frame_id = get_parameter("camera_frame").as_string();
    camera.transform.rotation.w = std::cos(half);
    camera.transform.rotation.y = std::sin(half);
    static_broadcaster_->sendTransform(camera);

    local_position_sub_ =
        create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "fmu/out/vehicle_local_position", rclcpp::SensorDataQoS(),
            [this](const px4_msgs::msg::VehicleLocalPosition& msg) {
              const auto update = transform_->updateReference(
                  msg.xy_valid, msg.z_valid, msg.xy_global, msg.z_global,
                  msg.ref_timestamp, msg.ref_lat, msg.ref_lon, msg.ref_alt,
                  msg.xy_reset_counter, msg.z_reset_counter);
              if (update.reference_changed) {
                RCLCPP_INFO(get_logger(),
                            "accepted PX4 global reference at timestamp %llu",
                            static_cast<unsigned long long>(msg.ref_timestamp));
              }
              if (update.xy_reset || update.z_reset) {
                RCLCPP_WARN(get_logger(),
                            "PX4 estimator reset observed (xy=%s, z=%s)",
                            update.xy_reset ? "yes" : "no",
                            update.z_reset ? "yes" : "no");
              }
            });

    odometry_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
        "fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
        [this](const px4_msgs::msg::VehicleOdometry& msg) { publish(msg); });
  }

private:
  void publish(const px4_msgs::msg::VehicleOdometry& msg) {
    if (msg.pose_frame != px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "ignoring PX4 odometry outside NED");
      return;
    }
    const auto map = transform_->localNedToMap(
        {msg.position[0], msg.position[1], msg.position[2]});
    const std::array<double, 4> q_ned{
        msg.q[0], msg.q[1], msg.q[2], msg.q[3]};
    if (!map || !std::isfinite(q_ned[0]) || !std::isfinite(q_ned[1]) ||
        !std::isfinite(q_ned[2]) || !std::isfinite(q_ned[3])) {
      return;
    }
    const auto q_enu = frdNedToFluEnu(q_ned);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = rclcpp::Time(
        static_cast<std::int64_t>(msg.timestamp_sample) * 1000,
        RCL_ROS_TIME);
    tf.header.frame_id = map_frame_;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = map->x;
    tf.transform.translation.y = map->y;
    tf.transform.translation.z = map->z;
    tf.transform.rotation.w = q_enu[0];
    tf.transform.rotation.x = q_enu[1];
    tf.transform.rotation.y = q_enu[2];
    tf.transform.rotation.z = q_enu[3];
    broadcaster_->sendTransform(tf);
  }

  std::string map_frame_;
  std::string base_frame_;
  std::unique_ptr<Px4MapTransform> transform_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr
      odometry_sub_;
};

}  // namespace spar_air

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<spar_air::TfFromPx4>());
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << "\n";
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
