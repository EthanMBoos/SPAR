#include <gtest/gtest.h>

#include <geometry_msgs/msg/point.hpp>

#include "bt/barrel_seen.hpp"
#include "bt/battery_low.hpp"
#include "bt/stamped.hpp"

namespace worldfile_demo {
namespace {

BT::NodeConfig config_at(double now) {
  BT::NodeConfig config;
  config.blackboard = BT::Blackboard::create();
  config.blackboard->set<double>("now_sec", now);
  return config;
}

TEST(Stamped, FreshnessUsesTheProvidedClock) {
  EXPECT_FALSE(fresh(Stamped<double>{}, 100.0, 5.0));
  EXPECT_TRUE(fresh(Stamped<double>{50.0, 100.0}, 105.0, 5.0));
  EXPECT_FALSE(fresh(Stamped<double>{50.0, 100.0}, 106.0, 5.0));
  EXPECT_FALSE(fresh(Stamped<double>{50.0, 101.0}, 100.0, 5.0));
}

TEST(BatteryLow, MissingOrStaleReadingIsLow) {
  auto config = config_at(100.0);
  BatteryLow node("BatteryLow", config, BatteryLow::Params{});
  EXPECT_EQ(node.tick(), BT::NodeStatus::SUCCESS);
  config.blackboard->set<Stamped<double>>(
      "battery_percent", {80.0, 50.0});
  EXPECT_EQ(node.tick(), BT::NodeStatus::SUCCESS);
}

TEST(BatteryLow, HysteresisLatchesUntilResumeThreshold) {
  auto config = config_at(0.0);
  BatteryLow node("BatteryLow", config, BatteryLow::Params{});
  config.blackboard->set<Stamped<double>>(
      "battery_percent", {50.0, 0.0});
  EXPECT_EQ(node.tick(), BT::NodeStatus::FAILURE);
  config.blackboard->set<Stamped<double>>(
      "battery_percent", {25.0, 0.0});
  EXPECT_EQ(node.tick(), BT::NodeStatus::SUCCESS);
  config.blackboard->set<Stamped<double>>(
      "battery_percent", {50.0, 0.0});
  EXPECT_EQ(node.tick(), BT::NodeStatus::SUCCESS);
  config.blackboard->set<Stamped<double>>(
      "battery_percent", {95.0, 0.0});
  EXPECT_EQ(node.tick(), BT::NodeStatus::FAILURE);
}

TEST(BarrelSeen, RequiresFreshPointOutsideCooldown) {
  auto config = config_at(100.0);
  BarrelSeen::Params params;
  params.max_age_sec = 6.0;
  params.cooldown_sec = 30.0;
  BarrelSeen node("BarrelSeen", config, params);

  config.blackboard->set<Stamped<geometry_msgs::msg::Point>>(
      "barrel_point", {geometry_msgs::msg::Point{}, 90.0});
  EXPECT_EQ(node.tick(), BT::NodeStatus::FAILURE);

  config.blackboard->set<Stamped<geometry_msgs::msg::Point>>(
      "barrel_point", {geometry_msgs::msg::Point{}, 100.0});
  config.blackboard->set<double>("last_inspected", 90.0);
  EXPECT_EQ(node.tick(), BT::NodeStatus::FAILURE);

  config.blackboard->set<double>("last_inspected", 50.0);
  EXPECT_EQ(node.tick(), BT::NodeStatus::SUCCESS);
}

}  // namespace
}  // namespace worldfile_demo
