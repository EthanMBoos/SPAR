#pragma once
#include "../../shared/contracts/ControllerAdapter.h"
#include "../../shared/contracts/ControllerEnvelope.h"
#include "../../shared/assembler/ObservationAssembler.h"
#include <memory>
#include <string>

struct Ros2Config {
    // Zenoh keyexprs. On a vehicle running rmw_zenoh these are the structured
    // keyexprs for its ROS 2 topics (nail them down on-robot by printing
    // sample.get_keyexpr() — do not guess). Defaults are placeholders.
    std::string cmd_vel_key = "spar/cmd_vel";  // publish  geometry_msgs/Twist
    std::string odom_key    = "spar/odom";     // subscribe nav_msgs/Odometry
    ControllerEnvelope envelope{};
};

// ROS 2 controller adapter over Zenoh (rmw_zenoh). The single writer to the
// controller: write() publishes approved commands as geometry_msgs/Twist on
// cmd_vel. Its inbound path subscribes to nav_msgs/Odometry from the external
// state estimator (robot_localization) and feeds pose into the assembler —
// SPAR consumes the estimate, it never estimates. No rclcpp: CDR is encoded and
// decoded directly so middleware stays out of the tick-loop process.
class Ros2Adapter final : public ControllerAdapter {
public:
    explicit Ros2Adapter(ObservationAssembler& assembler, Ros2Config cfg = {});
    ~Ros2Adapter() override;

    bool connect() override;
    void disconnect() override;
    bool is_connected() const override;
    bool write(const CommandStream& cmd) override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
