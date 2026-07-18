from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("config_path", default_value=""),
        SetParameter(name="use_sim_time", value=LaunchConfiguration("use_sim_time")),
        Node(
            package="fastswarm_sensing",
            executable="local_pointcloud_sim_node",
            name="local_pointcloud_sim",
            output="screen",
            parameters=[{
                "config_path": LaunchConfiguration("config_path"),
            }],
        ),
    ])
