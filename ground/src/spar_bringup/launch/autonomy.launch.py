"""The whole robot stack: GPS localization, Nav2, and the behavior layer
(bt_executive + battery_sim + anomaly_detector). The MuJoCo sim node
publishing sensors lives in the sim container (sim/spar_sim), not here.

One `ros2 launch` process, launched by hand from a `make ros2_container`/
`make shell` session, see the Makefile and README. Ctrl-C and rerun it to
pick up any change, whether that's a code rebuild, a costmap tweak, or a
bridge/robot change; there's no reason to protect one layer from another
when nothing here restarts anything automatically anyway.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")

    autonomy_params = PathJoinSubstitution(
        [FindPackageShare("spar_bringup"), "config", "autonomy.yaml"]
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("spar_bringup"), "launch", "nav2.launch.py"]
            )
        ),
        launch_arguments=[
            ("namespace", namespace),
            ("use_sim_time", "true"),
        ],
    )

    def behavior_node(executable):
        return Node(
            package="spar_ground",
            executable=executable,
            namespace=namespace,
            parameters=[autonomy_params, {"use_sim_time": True}],
            # TransformListener subscribes to the absolute /tf; this stack
            # publishes TF namespaced. Remap it in.
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            output="screen",
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="husky"),
            nav2,
            behavior_node("tf_from_gps"),
            behavior_node("battery_sim"),
            behavior_node("anomaly_detector"),
            behavior_node("bt_executive"),
        ]
    )
