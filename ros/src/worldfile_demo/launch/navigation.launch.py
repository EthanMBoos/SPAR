"""Launch the complete ground autonomy stack for one generated world."""

import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from nav2_common.launch import RewrittenYaml


def load_world_config(
    package: str, world: str
) -> tuple[str, list[float], list[float], list[float]]:
    path = os.path.join(package, "config", "worlds", f"{world}.yaml")
    if not os.path.isfile(path):
        raise RuntimeError(f"world navigation config does not exist: {path}")
    with open(path, encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    datum = document.get("navsat_datum") if isinstance(document, dict) else None
    dock = document.get("dock_pose") if isinstance(document, dict) else None
    goals = document.get("navigation_goals") if isinstance(document, dict) else None
    if (
        not isinstance(datum, list)
        or len(datum) != 3
        or not all(type(value) is float and math.isfinite(value) for value in datum)
    ):
        raise RuntimeError(f"invalid navsat_datum in {path}")
    if not isinstance(goals, list) or not goals:
        raise RuntimeError(f"navigation_goals must be a non-empty list in {path}")
    if (
        not isinstance(dock, list)
        or len(dock) != 3
        or not all(type(value) is float and math.isfinite(value) for value in dock)
    ):
        raise RuntimeError(f"dock_pose must be three finite floats in {path}")
    patrol = []
    for index, goal in enumerate(goals):
        if not isinstance(goal, dict):
            raise RuntimeError(f"navigation goal {index} must be a mapping in {path}")
        values = [goal.get("x"), goal.get("y"), goal.get("yaw")]
        if not all(type(value) is float and math.isfinite(value) for value in values):
            raise RuntimeError(
                f"navigation goal {index} must have finite float x/y/yaw in {path}"
            )
        patrol.extend(values)
    if len(patrol) < 9:
        raise RuntimeError(f"at least three navigation goals are required in {path}")
    return path, datum, dock, patrol


def launch_stack(context):
    namespace = LaunchConfiguration("namespace").perform(context)
    world = LaunchConfiguration("world").perform(context)
    if not world or not world.replace("_", "").isalnum():
        raise RuntimeError("world must contain only letters, numbers, and underscores")

    package = get_package_share_directory("worldfile_demo")
    _, navsat_datum, dock_pose, patrol_waypoints = load_world_config(
        package, world
    )
    localization_params = os.path.join(package, "config", "localization.yaml")
    perception_params = os.path.join(package, "config", "perception.yaml")
    mission_params = os.path.join(package, "config", "mission.yaml")
    nav2_params = RewrittenYaml(
        source_file=os.path.join(package, "config", "nav2.yaml"),
        root_key=namespace,
        param_rewrites={"topic": f"/{namespace}/sensors/lidar2d_0/scan"},
        convert_types=True,
    )
    tf_remaps = [("/tf", "tf"), ("/tf_static", "tf_static")]

    def nav2_node(package_name, executable, remappings=()):
        return Node(
            package=package_name,
            executable=executable,
            parameters=[nav2_params, {"use_sim_time": True}],
            remappings=tf_remaps + list(remappings),
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
    navigation = GroupAction([
        PushRosNamespace(namespace),
        SetRemap(f"/{namespace}/odom", f"/{namespace}/platform/odom"),
        nav2_node(
            "nav2_controller", "controller_server", [("cmd_vel", "cmd_vel_nav")]
        ),
        nav2_node("nav2_planner", "planner_server"),
        nav2_node(
            "nav2_behaviors", "behavior_server", [("cmd_vel", "cmd_vel_nav")]
        ),
        nav2_node(
            "nav2_velocity_smoother",
            "velocity_smoother",
            [("cmd_vel", "cmd_vel_nav")],
        ),
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

    def localization_node(executable, name, remappings=(), parameters=()):
        return Node(
            package="robot_localization",
            executable=executable,
            name=name,
            namespace=namespace,
            parameters=[localization_params, *parameters, {"use_sim_time": True}],
            remappings=tf_remaps + list(remappings),
            output="screen",
        )

    return [
        navigation,
        Node(
            package="worldfile_demo",
            executable="wheel_odometry",
            namespace=namespace,
            parameters=[localization_params, {"use_sim_time": True}],
            remappings=tf_remaps,
            output="screen",
        ),
        Node(
            package="worldfile_demo",
            executable="red_barrel_detector",
            namespace=namespace,
            parameters=[perception_params, {"use_sim_time": True}],
            remappings=tf_remaps,
            output="screen",
        ),
        Node(
            package="worldfile_demo",
            executable="battery_sim",
            namespace=namespace,
            parameters=[
                mission_params,
                {
                    "use_sim_time": True,
                    "dock_x": dock_pose[0],
                    "dock_y": dock_pose[1],
                },
            ],
            remappings=tf_remaps,
            output="screen",
        ),
        Node(
            package="worldfile_demo",
            executable="bt_executive",
            namespace=namespace,
            parameters=[
                mission_params,
                {
                    "use_sim_time": True,
                    "dock_x": dock_pose[0],
                    "dock_y": dock_pose[1],
                    "dock_yaw": dock_pose[2],
                    "patrol_waypoints": patrol_waypoints,
                },
            ],
            remappings=tf_remaps,
            output="screen",
        ),
        localization_node(
            "ekf_node", "ekf_local", [("odometry/filtered", "platform/odom")]
        ),
        localization_node(
            "ekf_node", "ekf_global", [("odometry/filtered", "localization/global")]
        ),
        localization_node(
            "navsat_transform_node",
            "navsat_transform",
            [
                ("imu/data", "sensors/imu/data"),
                ("gps/fix", "sensors/gps/fix"),
                ("odometry/filtered", "localization/global"),
                ("odometry/gps", "localization/gps"),
            ],
            [{"datum": navsat_datum}],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="husky"),
        DeclareLaunchArgument(
            "world",
            default_value="utility_depot_40_v2",
            description="world selecting datum, dock, and mission route",
        ),
        OpaqueFunction(function=launch_stack),
    ])
