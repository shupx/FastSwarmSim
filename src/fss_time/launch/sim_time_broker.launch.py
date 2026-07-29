from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("max_real_time_factor", default_value="1.0"),
        DeclareLaunchArgument("auto_start", default_value="true"),
        DeclareLaunchArgument("speed_regulator_step_ns", default_value="1000000"),
        DeclareLaunchArgument("start_broker_ui", default_value="true"),
        SetParameter(name="use_sim_time", value=True),
        Node(
            package="fss_time",
            executable="sim_time_broker",
            name="fss_sim_time_broker",
            output="screen",
            parameters=[{
                "max_real_time_factor": LaunchConfiguration("max_real_time_factor"),
                "auto_start": LaunchConfiguration("auto_start"),
                "speed_regulator_step_ns": LaunchConfiguration("speed_regulator_step_ns"),
            }],
        ),
        Node(
            package="fss_time",
            executable="sim_time_broker_ui.py",
            name="fss_sim_time_broker_ui",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_broker_ui")),
        ),
    ])
