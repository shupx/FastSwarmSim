from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
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
        DeclareLaunchArgument(
            "use_fss_sim_time",
            default_value="true",
            choices=["true", "false"],
            description="Use the FastSwarmSim coordinated simulation clock.",
        ),
        DeclareLaunchArgument(
            "config_path",
            default_value=default_config_path,
            description="Path to the local pointcloud simulator configuration file.",
        ),
        GroupAction(
            scoped=True,
            actions=[
                SetParameter(
                    name="use_fss_sim_time",
                    value=LaunchConfiguration("use_fss_sim_time"),
                ),
                SetParameter(
                    name="use_sim_time",
                    value=LaunchConfiguration("use_fss_sim_time"),
                ),
                Node(
                    package="fss_sensing",
                    executable="local_pointcloud_sim_node",
                    name="local_pointcloud_sim",
                    output="screen",
                    parameters=[{
                        "config_path": LaunchConfiguration("config_path"),
                    }],
                ),
            ],
        ),
    ])
