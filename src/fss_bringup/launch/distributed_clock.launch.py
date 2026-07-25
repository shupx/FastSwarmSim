from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("max_real_time_factor", default_value="1.0"),
        DeclareLaunchArgument("helics_core_type", default_value="zmq"),
        DeclareLaunchArgument("broker_address", default_value="127.0.0.1"),
        DeclareLaunchArgument("broker_port", default_value="23404"),
        DeclareLaunchArgument("helics_broker_federates", default_value="0"),
        DeclareLaunchArgument("helics_time_delta_ns", default_value="1000000"),
        DeclareLaunchArgument("speed_regulator_tick_ns", default_value="1000000"),
        DeclareLaunchArgument("start_broker", default_value="true"),
        SetParameter(name="use_sim_time", value=True),
        Node(
            package="fss_time",
            executable="sim_time_broker_node",
            name="fss_sim_time_broker",
            output="screen",
            parameters=[{
                "helics_core_type": LaunchConfiguration("helics_core_type"),
                "broker_address": LaunchConfiguration("broker_address"),
                "broker_port": LaunchConfiguration("broker_port"),
                "helics_broker_federates": LaunchConfiguration("helics_broker_federates"),
                "helics_time_delta_ns": LaunchConfiguration("helics_time_delta_ns"),
                "speed_regulator_tick_ns": LaunchConfiguration("speed_regulator_tick_ns"),
                "start_broker": LaunchConfiguration("start_broker"),
                "max_real_time_factor": LaunchConfiguration("max_real_time_factor"),
            }],
        ),
        Node(
            package="fss_time",
            executable="ros_clock_publisher_node",
            name="fss_ros_clock_publisher",
            output="screen",
            parameters=[{
            }],
        ),
    ])
