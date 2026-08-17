#include <cmath>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "bt/barrel_seen.hpp"
#include "bt/battery_low.hpp"
#include "bt/blackboard_keys.hpp"
#include "bt/idle.hpp"
#include "bt/mission_active.hpp"
#include "bt/stamped.hpp"
#include "leaves/navigate_leaf.hpp"

namespace spar {

class BtExecutive : public rclcpp::Node {
 public:
  BtExecutive() : rclcpp::Node("bt_executive") {
    declare_parameter("tick_rate_hz", 10.0);
    declare_parameter("goal_frame", "map");
    declare_parameter<std::vector<double>>(
        "patrol_waypoints", std::vector<double>{});
    declare_parameter("dock_x", 0.0);
    declare_parameter("dock_y", 0.0);
    declare_parameter("dock_yaw", 0.0);
    declare_parameter("battery_low_percent", 30.0);
    declare_parameter("battery_resume_percent", 90.0);
    declare_parameter("battery_stale_sec", 2.0);
    declare_parameter("nav_retry_cooldown_sec", 2.0);
    declare_parameter("waypoint_max_retries", 3);
    declare_parameter("base_frame", "base_link");
    declare_parameter("barrel_stale_sec", 6.0);
    declare_parameter("inspect_cooldown_sec", 45.0);
    declare_parameter("inspect_standoff_m", 1.8);
  }

  void init() {
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<MissionActive>("MissionActive");
    factory.registerNodeType<Idle>("Idle");

    BatteryLow::Params battery_params;
    battery_params.low_percent =
        get_parameter("battery_low_percent").as_double();
    battery_params.resume_percent =
        get_parameter("battery_resume_percent").as_double();
    battery_params.max_age_sec =
        get_parameter("battery_stale_sec").as_double();
    factory.registerBuilder<BatteryLow>(
        "BatteryLow",
        [battery_params](const std::string& name,
                         const BT::NodeConfig& config) {
          return std::make_unique<BatteryLow>(
              name, config, battery_params);
        });

    BarrelSeen::Params barrel_params;
    barrel_params.max_age_sec =
        get_parameter("barrel_stale_sec").as_double();
    barrel_params.cooldown_sec =
        get_parameter("inspect_cooldown_sec").as_double();
    factory.registerBuilder<BarrelSeen>(
        "BarrelSeen",
        [barrel_params](const std::string& name,
                        const BT::NodeConfig& config) {
          return std::make_unique<BarrelSeen>(
              name, config, barrel_params);
        });

    const auto goal_frame = get_parameter("goal_frame").as_string();
    const auto base_frame = get_parameter("base_frame").as_string();
    const double retry_cooldown =
        get_parameter("nav_retry_cooldown_sec").as_double();
    RoundsLeaf::Waypoint dock{
        get_parameter("dock_x").as_double(),
        get_parameter("dock_y").as_double(),
        get_parameter("dock_yaw").as_double()};

    const auto values = get_parameter("patrol_waypoints").as_double_array();
    if (values.size() < 9 || values.size() % 3 != 0) {
      throw std::runtime_error(
          "patrol_waypoints must contain at least three x, y, yaw triples");
    }
    std::vector<RoundsLeaf::Waypoint> waypoints;
    for (size_t index = 0; index < values.size(); index += 3) {
      if (!std::isfinite(values[index]) ||
          !std::isfinite(values[index + 1]) ||
          !std::isfinite(values[index + 2])) {
        throw std::runtime_error("patrol_waypoints values must be finite");
      }
      waypoints.push_back(
          {values[index], values[index + 1], values[index + 2]});
    }
    RCLCPP_INFO(get_logger(), "using %zu configured round waypoints",
                waypoints.size());
    const int max_retries = static_cast<int>(
        get_parameter("waypoint_max_retries").as_int());

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    InspectLeaf::Params inspect_params;
    inspect_params.standoff_m =
        get_parameter("inspect_standoff_m").as_double();

    factory.registerBuilder<RoundsLeaf>(
        "Rounds",
        [this, waypoints, goal_frame, retry_cooldown, max_retries](
            const std::string& name, const BT::NodeConfig& config) {
          return std::make_unique<RoundsLeaf>(
              name, config, *this, waypoints, goal_frame,
              retry_cooldown, max_retries);
        });
    factory.registerBuilder<DockLeaf>(
        "ReturnToDock",
        [this, dock, goal_frame, retry_cooldown](
            const std::string& name, const BT::NodeConfig& config) {
          return std::make_unique<DockLeaf>(
              name, config, *this, dock, goal_frame, retry_cooldown);
        });
    factory.registerBuilder<InspectLeaf>(
        "Inspect",
        [this, goal_frame, base_frame, retry_cooldown, inspect_params](
            const std::string& name, const BT::NodeConfig& config) {
          return std::make_unique<InspectLeaf>(
              name, config, *this, *tf_buffer_, goal_frame, base_frame,
              retry_cooldown, inspect_params);
        });
    factory.registerBuilder<HoldLeaf>(
        "HoldPosition",
        [this](const std::string& name, const BT::NodeConfig& config) {
          return std::make_unique<HoldLeaf>(name, config, *this);
        });

    tree_ = factory.createTreeFromFile(
        ament_index_cpp::get_package_share_directory("spar") +
        "/behavior_trees/main_tree.xml");
    auto blackboard = tree_.rootBlackboard();
    blackboard->set<bool>(keys::kMissionActive, false);

    battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
        "battery/state", 10,
        [this, blackboard](const sensor_msgs::msg::BatteryState& message) {
          blackboard->set<Stamped<double>>(
              keys::kBatteryPercent,
              {message.percentage * 100.0, now().seconds()});
        });
    barrel_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        "perception/red_barrel", 10,
        [blackboard](const geometry_msgs::msg::PointStamped& message) {
          blackboard->set<Stamped<geometry_msgs::msg::Point>>(
              keys::kBarrelPoint,
              {message.point, rclcpp::Time(message.header.stamp).seconds()});
        });
    mission_sub_ = create_subscription<std_msgs::msg::String>(
        "mission/command", 10,
        [this, blackboard](const std_msgs::msg::String& message) {
          if (message.data == "start" || message.data == "stop") {
            blackboard->set<bool>(
                keys::kMissionActive, message.data == "start");
            RCLCPP_INFO(get_logger(), "mission command: %s",
                        message.data.c_str());
          } else {
            RCLCPP_WARN(get_logger(),
                        "unknown mission command '%s' (want start|stop)",
                        message.data.c_str());
          }
        });
    status_pub_ = create_publisher<std_msgs::msg::String>("bt/status", 10);

    const double tick_rate = get_parameter("tick_rate_hz").as_double();
    timer_ = rclcpp::create_timer(
        this, get_clock(), rclcpp::Duration::from_seconds(1.0 / tick_rate),
        [this] { tick_once(); });
    RCLCPP_INFO(get_logger(), "bt_executive ticking at %.1f Hz", tick_rate);
  }

 private:
  void tick_once() {
    auto blackboard = tree_.rootBlackboard();
    blackboard->set<double>(keys::kNowSec, now().seconds());
    const BT::NodeStatus root_status = tree_.tickExactlyOnce();

    double now_sec = 0.0;
    (void)blackboard->get<double>(keys::kNowSec, now_sec);
    Stamped<double> battery;
    const bool have_battery = blackboard->get<Stamped<double>>(
        keys::kBatteryPercent, battery);
    Stamped<geometry_msgs::msg::Point> barrel;
    const bool have_barrel = blackboard->get<
        Stamped<geometry_msgs::msg::Point>>(keys::kBarrelPoint, barrel);
    bool mission = false;
    (void)blackboard->get<bool>(keys::kMissionActive, mission);

    std_msgs::msg::String status;
    std::ostringstream output;
    output << "{\"root_status\":\"" << BT::toStr(root_status) << "\""
           << ",\"mission_active\":" << (mission ? "true" : "false")
           << ",\"barrel_age_sec\":"
           << (have_barrel ? std::to_string(now_sec - barrel.stamp) : "null")
           << ",\"active_leaf\":\"" << active_leaf() << "\""
           << ",\"battery_percent\":"
           << (have_battery ? std::to_string(battery.value) : "null")
           << ",\"battery_age_sec\":"
           << (have_battery
                   ? std::to_string(now_sec - battery.stamp)
                   : "null")
           << "}";
    status.data = output.str();
    status_pub_->publish(status);
  }

  std::string active_leaf() const {
    std::string active = "none";
    tree_.applyVisitor([&active](const BT::TreeNode* node) {
      if (node->type() == BT::NodeType::ACTION &&
          node->status() != BT::NodeStatus::IDLE) {
        active = node->registrationName();
      }
    });
    return active;
  }

  BT::Tree tree_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr barrel_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace spar

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<spar::BtExecutive>();
    node->init();
    rclcpp::spin(node);
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << "\n";
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
