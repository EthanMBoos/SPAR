#!/usr/bin/env python3
"""Send one generated world goal directly through Nav2's standard action."""

import argparse
import math
from pathlib import Path
import sys

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
import yaml


class GoalClient(Node):
    def __init__(self):
        super().__init__("spar_nav_goal", namespace="husky")
        self.positions = []
        self.create_subscription(
            Odometry, "platform/odom", self._on_odom, 10)
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def _on_odom(self, message):
        position = message.pose.pose.position
        self.positions.append((position.x, position.y))


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", default="utility_depot_40_v2")
    parser.add_argument("--index", type=int, default=0)
    return parser.parse_args()


def load_goal(world, index):
    path = Path("/ws/src/spar/config/worlds") / f"{world}.yaml"
    if not path.is_file():
        raise RuntimeError(f"world navigation config does not exist: {path}")
    document = yaml.safe_load(path.read_text())
    goals = document.get("navigation_goals", [])
    if index < 0 or index >= len(goals):
        raise RuntimeError(f"goal index {index} is outside 0..{len(goals) - 1}")
    return goals[index]


def main():
    args = arguments()
    try:
        goal_data = load_goal(args.world, args.index)
    except (OSError, TypeError, yaml.YAMLError, RuntimeError) as exc:
        sys.exit(str(exc))

    rclpy.init()
    node = GoalClient()
    try:
        if not node.client.wait_for_server(timeout_sec=30.0):
            raise RuntimeError("NavigateToPose action server is unavailable")
        while not node.positions:
            rclpy.spin_once(node, timeout_sec=1.0)
        initial = node.positions[-1]

        yaw = float(goal_data["yaw"])
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = float(goal_data["x"])
        goal.pose.pose.position.y = float(goal_data["y"])
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        sent = node.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, sent, timeout_sec=30.0)
        handle = sent.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("Nav2 rejected the generated goal")
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result, timeout_sec=300.0)
        response = result.result()
        if response is None or response.status != GoalStatus.STATUS_SUCCEEDED:
            status = None if response is None else response.status
            raise RuntimeError(f"NavigateToPose did not succeed (status={status})")

        final = node.positions[-1]
        distance = math.hypot(final[0] - initial[0], final[1] - initial[1])
        if distance < 0.5:
            raise RuntimeError(f"odometry moved only {distance:.3f} m")
        print(
            f"[nav-goal] reached {goal_data['name']} at "
            f"({goal_data['x']}, {goal_data['y']}); moved {distance:.2f} m"
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (KeyError, TypeError, ValueError, RuntimeError) as exc:
        sys.exit(str(exc))
