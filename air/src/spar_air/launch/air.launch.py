"""The air stack: the TF publisher, the detector, and the behavior tree,
all under the drone's namespace. PX4 SITL runs separately (started by the
smoke script or by hand, see the README); this launch is only the ROS
side, so a BT or config change is a Ctrl-C and rerun away, PX4 untouched.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_stack(context):
    namespace = LaunchConfiguration("namespace").perform(context)
    world = LaunchConfiguration("world").perform(context)
    if not world:
        raise RuntimeError("world is required for air datum, home, and route config")
    if not world.replace("_", "").isalnum():
        raise RuntimeError("world must contain only letters, numbers, and underscores")

    package = get_package_share_directory("spar_air")
    autonomy_params = os.path.join(package, "config", "autonomy.yaml")
    world_params = os.path.join(package, "config", "worlds", f"{world}.yaml")
    if not os.path.isfile(world_params):
        raise RuntimeError(f"world autonomy parameters do not exist: {world_params}")
    node_params = [autonomy_params, world_params]

    def behavior_node(executable, package="spar_air"):
        return Node(
            package=package,
            executable=executable,
            namespace=namespace,
            parameters=[*node_params, {"use_sim_time": True}],
            # TransformListener subscribes to the absolute /tf; this stack
            # publishes TF namespaced. Remap it in.
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            output="screen",
        )

    return [
        behavior_node("tf_from_px4"),
        behavior_node("anomaly_detector", "spar_perception"),
        behavior_node("bt_executive"),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="skydio"),
        DeclareLaunchArgument(
            "world", default_value="utility_depot_40_v2",
            description="world name selecting air datum, home, and route config"),
        OpaqueFunction(function=launch_stack),
    ])
