"""Launch the complete Husky stack: Nav2, localization, perception, and BT."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from nav2_common.launch import RewrittenYaml


def launch_stack(context):
    namespace = LaunchConfiguration("namespace").perform(context)
    package = get_package_share_directory("spar_ground")
    nav2_package = get_package_share_directory("nav2_bringup")
    autonomy_params = os.path.join(package, "config", "autonomy.yaml")
    nav2_params = RewrittenYaml(
        source_file=os.path.join(package, "config", "nav2.yaml"),
        param_rewrites={
            "topic": f"/{namespace}/sensors/lidar2d_0/scan",
        },
        convert_types=True,
    )

    nav2 = GroupAction([
        PushRosNamespace(namespace),
        SetRemap(f"/{namespace}/odom", f"/{namespace}/platform/odom"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_package, "launch", "navigation_launch.py")
            ),
            launch_arguments=[
                ("use_sim_time", "true"),
                ("params_file", nav2_params),
                ("use_composition", "False"),
                ("namespace", namespace),
            ],
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
