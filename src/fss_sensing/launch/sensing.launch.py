from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config_path = PathJoinSubstitution([
        FindPackageShare("fss_sensing"),
        "config",
        "local_pointcloud_sim.yaml",
    ])
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("config_path", default_value=default_config_path),
        SetParameter(name="use_sim_time", value=LaunchConfiguration("use_sim_time")),
        Node(
            package="fss_sensing",
            executable="local_pointcloud_sim_node",
            name="local_pointcloud_sim",
            output="screen",
            parameters=[{
                "config_path": LaunchConfiguration("config_path"),
            }],
        ),
    ])
