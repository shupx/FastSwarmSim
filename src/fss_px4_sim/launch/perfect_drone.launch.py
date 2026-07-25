from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetParameter


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="uav1"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("init_x", default_value="0.0"),
        DeclareLaunchArgument("init_y", default_value="0.0"),
        DeclareLaunchArgument("init_z", default_value="1.0"),
        DeclareLaunchArgument("init_yaw", default_value="0.0"),
        DeclareLaunchArgument("helics_core_type", default_value="zmq"),
        DeclareLaunchArgument("broker_address", default_value="127.0.0.1"),
        DeclareLaunchArgument("broker_port", default_value="23404"),
        DeclareLaunchArgument("helics_time_delta_ns", default_value="1000000"),
        SetParameter(name="use_sim_time", value=LaunchConfiguration("use_sim_time")),
        PushRosNamespace(namespace),
        Node(
            package="fss_px4_sim",
            executable="perfect_mavros_quadrotor_sim_node",
            name="perfect_mavros_quadrotor_sim",
            output="screen",
            parameters=[{
                "init_x": LaunchConfiguration("init_x"),
                "init_y": LaunchConfiguration("init_y"),
                "init_z": LaunchConfiguration("init_z"),
                "init_yaw": LaunchConfiguration("init_yaw"),
                "helics_core_type": LaunchConfiguration("helics_core_type"),
                "broker_address": LaunchConfiguration("broker_address"),
                "broker_port": LaunchConfiguration("broker_port"),
                "helics_time_delta_ns": LaunchConfiguration("helics_time_delta_ns"),
            }],
        ),
    ])
