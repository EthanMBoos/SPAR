"""Launch the complete Husky stack: Nav2, localization, perception, and BT."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from nav2_common.launch import RewrittenYaml


def launch_stack(context):
    namespace = LaunchConfiguration("namespace").perform(context)
    package = get_package_share_directory("spar_ground")
    autonomy_params = os.path.join(package, "config", "autonomy.yaml")
    nav2_params = RewrittenYaml(
        source_file=os.path.join(package, "config", "nav2.yaml"),
        root_key=namespace,
        param_rewrites={
            "topic": f"/{namespace}/sensors/lidar2d_0/scan",
        },
        convert_types=True,
    )

    nav2_remaps = [("/tf", "tf"), ("/tf_static", "tf_static")]

    def nav2_node(package_name, executable, remappings=()):
        return Node(
            package=package_name,
            executable=executable,
            parameters=[nav2_params, {"use_sim_time": True}],
            remappings=nav2_remaps + list(remappings),
            output="screen",
        )

    lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "behavior_server",
        "velocity_smoother",
        "collision_monitor",
        "bt_navigator",
    ]
    nav2 = GroupAction([
        PushRosNamespace(namespace),
        SetRemap(f"/{namespace}/odom", f"/{namespace}/platform/odom"),
        nav2_node(
            "nav2_controller", "controller_server",
            [("cmd_vel", "cmd_vel_nav")]),
        nav2_node("nav2_planner", "planner_server"),
        nav2_node(
            "nav2_behaviors", "behavior_server",
            [("cmd_vel", "cmd_vel_nav")]),
        nav2_node(
            "nav2_velocity_smoother", "velocity_smoother",
            [("cmd_vel", "cmd_vel_nav")]),
        nav2_node("nav2_collision_monitor", "collision_monitor"),
        nav2_node("nav2_bt_navigator", "bt_navigator"),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            parameters=[
                {"use_sim_time": True},
                {"autostart": True},
                {"node_names": lifecycle_nodes},
            ],
            output="screen",
        ),
    ])

    def node(executable, package_name="spar_ground"):
        return Node(
            package=package_name,
            executable=executable,
            namespace=namespace,
            parameters=[autonomy_params, {"use_sim_time": True}],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            output="screen",
        )

    return [
        nav2,
        node("tf_from_gps"),
        node("battery_sim"),
        node("anomaly_detector", "spar_perception"),
        node("bt_executive"),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="husky"),
        OpaqueFunction(function=launch_stack),
    ])
