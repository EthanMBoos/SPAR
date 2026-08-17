// Focused RGB-D perception for the worldgen demo: find the largest red blob,
// recover its range, and publish a map-frame point. It intentionally lives in
// the ground package and uses a standard ROS message; there is no perception
// framework or custom mission interface around it.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace spar {

class RedBarrelDetector : public rclcpp::Node {
 public:
  RedBarrelDetector() : Node("red_barrel_detector") {
    declare_parameter("color_topic", "sensors/camera_0/color/image");
    declare_parameter("depth_topic", "sensors/camera_0/depth/image");
    declare_parameter("info_topic", "sensors/camera_0/color/camera_info");
    declare_parameter("output_topic", "perception/red_barrel");
    declare_parameter("map_frame", "map");
    declare_parameter("camera_frame", "camera_0_link");
    declare_parameter("min_blob_area_px", 120);
    declare_parameter("max_range_m", 8.0);
    declare_parameter("hsv_low_1", std::vector<int64_t>{0, 150, 80});
    declare_parameter("hsv_high_1", std::vector<int64_t>{4, 255, 255});
    declare_parameter("hsv_low_2", std::vector<int64_t>{176, 150, 80});
    declare_parameter("hsv_high_2", std::vector<int64_t>{180, 255, 255});

    map_frame_ = get_parameter("map_frame").as_string();
    camera_frame_ = get_parameter("camera_frame").as_string();
    min_area_ = static_cast<int>(get_parameter("min_blob_area_px").as_int());
    max_range_ = get_parameter("max_range_m").as_double();
    auto hsv = [this](const std::string& name) {
      const auto values = get_parameter(name).as_integer_array();
      return cv::Scalar(values[0], values[1], values[2]);
    };
    hsv_low_1_ = hsv("hsv_low_1");
    hsv_high_1_ = hsv("hsv_high_1");
    hsv_low_2_ = hsv("hsv_low_2");
    hsv_high_2_ = hsv("hsv_high_2");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    detection_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
        get_parameter("output_topic").as_string(), 10);

    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        get_parameter("info_topic").as_string(), 10,
        [this](const sensor_msgs::msg::CameraInfo& message) {
          info_ = message;
          process_pair();
        });
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        get_parameter("depth_topic").as_string(), rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Image::ConstSharedPtr message) {
          depth_ = message;
          process_pair();
        });
    color_sub_ = create_subscription<sensor_msgs::msg::Image>(
        get_parameter("color_topic").as_string(), rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Image::ConstSharedPtr message) {
          color_ = message;
          process_pair();
        });
  }

 private:
  void process_pair() {
    if (!info_ || !color_ || !depth_) return;
    const auto color_stamp = rclcpp::Time(color_->header.stamp).nanoseconds();
    const auto depth_stamp = rclcpp::Time(depth_->header.stamp).nanoseconds();
    if (color_stamp < depth_stamp) {
      color_.reset();
      return;
    }
    if (depth_stamp < color_stamp) {
      depth_.reset();
      return;
    }
    const auto color = color_;
    const auto depth = depth_;
    color_.reset();
    depth_.reset();
    process(color, depth);
  }

  void process(const sensor_msgs::msg::Image::ConstSharedPtr& color,
               const sensor_msgs::msg::Image::ConstSharedPtr& depth) {
    cv::Mat bgr;
    try {
      bgr = cv_bridge::toCvShare(color, "bgr8")->image;
    } catch (const cv_bridge::Exception& error) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "cv_bridge: %s", error.what());
      return;
    }

    cv::Mat hsv;
    cv::Mat mask_1;
    cv::Mat mask_2;
    cv::Mat mask;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, hsv_low_1_, hsv_high_1_, mask_1);
    cv::inRange(hsv, hsv_low_2_, hsv_high_2_, mask_2);
    cv::bitwise_or(mask_1, mask_2, mask);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    const auto biggest = std::max_element(
        contours.begin(), contours.end(), [](const auto& left, const auto& right) {
          return cv::contourArea(left) < cv::contourArea(right);
        });
    if (biggest == contours.end() || cv::contourArea(*biggest) < min_area_) {
      return;
    }

    const cv::Moments moments = cv::moments(*biggest);
    if (moments.m00 == 0.0) return;
    const double pixel_x = moments.m10 / moments.m00;
    const double pixel_y = moments.m01 / moments.m00;
    const auto range = depth_at(
        depth, pixel_x / bgr.cols, pixel_y / bgr.rows);
    if (!range || *range <= 0.1 || *range > max_range_) return;

    const double optical_x =
        (pixel_x - info_->k[2]) / info_->k[0] * *range;
    const double optical_y =
        (pixel_y - info_->k[5]) / info_->k[4] * *range;

    geometry_msgs::msg::PointStamped camera_point;
    camera_point.header.stamp = color->header.stamp;
    camera_point.header.frame_id = camera_frame_;
    // Convert optical x-right/y-down/z-forward into x-forward/y-left/z-up.
    camera_point.point.x = *range;
    camera_point.point.y = -optical_x;
    camera_point.point.z = -optical_y;

    try {
      const auto map_point = tf_buffer_->transform(
          camera_point, map_frame_, tf2::durationFromSec(0.2));
      detection_pub_->publish(map_point);
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "red barrel at map (%.2f, %.2f), range %.2f m",
          map_point.point.x, map_point.point.y, *range);
    } catch (const tf2::TransformException& error) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "red barrel visible but not localizable: %s", error.what());
    }
  }

  std::optional<double> depth_at(
      const sensor_msgs::msg::Image::ConstSharedPtr& message,
      double x_fraction, double y_fraction) const {
    cv_bridge::CvImageConstPtr depth_image;
    try {
      depth_image = cv_bridge::toCvShare(message);
    } catch (const cv_bridge::Exception&) {
      return std::nullopt;
    }

    const cv::Mat& image = depth_image->image;
    std::vector<double> samples;
    for (int dy = -2; dy <= 2; ++dy) {
      for (int dx = -2; dx <= 2; ++dx) {
        const int x = static_cast<int>(x_fraction * image.cols) + dx;
        const int y = static_cast<int>(y_fraction * image.rows) + dy;
        if (x < 0 || y < 0 || x >= image.cols || y >= image.rows) continue;
        double depth = 0.0;
        if (image.type() == CV_32FC1) {
          depth = image.at<float>(y, x);
        } else if (image.type() == CV_16UC1) {
          depth = image.at<uint16_t>(y, x) / 1000.0;
        }
        if (std::isfinite(depth) && depth > 0.0) samples.push_back(depth);
      }
    }
    if (samples.empty()) return std::nullopt;
    const auto middle = samples.begin() + samples.size() / 2;
    std::nth_element(samples.begin(), middle, samples.end());
    return *middle;
  }

  std::string map_frame_;
  std::string camera_frame_;
  int min_area_ = 120;
  double max_range_ = 8.0;
  cv::Scalar hsv_low_1_;
  cv::Scalar hsv_high_1_;
  cv::Scalar hsv_low_2_;
  cv::Scalar hsv_high_2_;
  std::optional<sensor_msgs::msg::CameraInfo> info_;
  sensor_msgs::msg::Image::ConstSharedPtr color_;
  sensor_msgs::msg::Image::ConstSharedPtr depth_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr detection_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
};

}  // namespace spar

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<spar::RedBarrelDetector>());
  rclcpp::shutdown();
  return 0;
}
