from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("participant_id", default_value="clock_bridge"),
        DeclareLaunchArgument("max_speed_ratio", default_value="1.0"),
        DeclareLaunchArgument("lease_timeout_ms", default_value="1000"),
        DeclareLaunchArgument("helics_core_type", default_value="zmq"),
        DeclareLaunchArgument("helics_broker_address", default_value="127.0.0.1"),
        DeclareLaunchArgument("helics_broker_port", default_value="23404"),
        DeclareLaunchArgument("helics_broker_federates", default_value="0"),
        DeclareLaunchArgument("helics_time_delta_ns", default_value="1000000"),
        DeclareLaunchArgument("start_helics_broker", default_value="true"),
        SetParameter(name="use_sim_time", value=True),
        Node(
            package="fss_time",
            executable="helics_broker_node",
            name="fss_helics_broker",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_helics_broker")),
            parameters=[{
                "helics_core_type": LaunchConfiguration("helics_core_type"),
                "helics_broker_address": LaunchConfiguration("helics_broker_address"),
                "helics_broker_port": LaunchConfiguration("helics_broker_port"),
                "helics_broker_federates": LaunchConfiguration("helics_broker_federates"),
            }],
        ),
        Node(
            package="fss_time",
            executable="clock_bridge_node",
            name="fss_clock_bridge",
            output="screen",
            parameters=[{
                "participant_id": LaunchConfiguration("participant_id"),
                "max_speed_ratio": LaunchConfiguration("max_speed_ratio"),
                "lease_timeout_ms": LaunchConfiguration("lease_timeout_ms"),
                "helics_core_type": LaunchConfiguration("helics_core_type"),
                "helics_broker_address": LaunchConfiguration("helics_broker_address"),
                "helics_broker_port": LaunchConfiguration("helics_broker_port"),
                "helics_time_delta_ns": LaunchConfiguration("helics_time_delta_ns"),
            }],
        ),
    ])
