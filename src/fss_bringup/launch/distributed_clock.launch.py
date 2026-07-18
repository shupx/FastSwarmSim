from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("participant_id", default_value="clock_bridge"),
        DeclareLaunchArgument("max_speed_ratio", default_value="1.0"),
        DeclareLaunchArgument("lease_timeout_ms", default_value="1000"),
        SetParameter(name="use_sim_time", value=True),
        Node(
            package="fss_time",
            executable="clock_bridge_node",
            name="fss_clock_bridge",
            output="screen",
            parameters=[{
                "participant_id": LaunchConfiguration("participant_id"),
                "max_speed_ratio": LaunchConfiguration("max_speed_ratio"),
                "lease_timeout_ms": LaunchConfiguration("lease_timeout_ms"),
            }],
        ),
    ])
