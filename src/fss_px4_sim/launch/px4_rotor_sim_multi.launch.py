from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def make_drones(context):
    count = int(LaunchConfiguration("num_drones").perform(context))
    drones_per_row = int(LaunchConfiguration("drones_per_row").perform(context))
    if drones_per_row < 1:
        raise ValueError("drones_per_row must be at least 1")
    launch_file = PathJoinSubstitution([
        FindPackageShare("fss_px4_sim"), "launch", "px4_rotor_sim_single.launch.py"])
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments={
                "namespace": f"uav{index}",
                "mav_sys_id": str(index),
                "mav_comp_id": "1",
                "mavlink_udp_local_port": "0",
                "mavlink_udp_remote_port": str(24540 + index - 1),
                "init_x_East_metre": str((index - 1) % drones_per_row),
                "init_y_North_metre": str((index - 1) // drones_per_row),
                "use_fss_sim_time": LaunchConfiguration("use_fss_sim_time"),
                "fss_time_coordinator_endpoint": LaunchConfiguration("fss_time_coordinator_endpoint"),
                "enable_mavros": LaunchConfiguration("enable_mavros"),
                "enable_visualizer": LaunchConfiguration("enable_visualizer"),
                "enable_rviz": "false",
            }.items(),
        )
        for index in range(1, count + 1)
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("num_drones", default_value="5"),
        DeclareLaunchArgument("drones_per_row", default_value="10"),
        DeclareLaunchArgument("use_fss_sim_time", default_value="true"),
        DeclareLaunchArgument("fss_time_coordinator_endpoint", default_value="ipc:///tmp/fss_time_coordinator.ipc"),
        DeclareLaunchArgument("enable_mavros", default_value="true"),
        DeclareLaunchArgument("enable_visualizer", default_value="true"),
        DeclareLaunchArgument("enable_rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=PathJoinSubstitution([
            FindPackageShare("fss_px4_sim"), "rviz", "multi_px4_rotor.rviz"])),
        SetParameter(name="use_fss_sim_time", value=LaunchConfiguration("use_fss_sim_time")),
        SetParameter(name="use_sim_time", value=LaunchConfiguration("use_fss_sim_time")),
        Node(
            package="rviz2", executable="rviz2", name="rviz2", output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            condition=IfCondition(LaunchConfiguration("enable_rviz")),
        ),
        OpaqueFunction(function=make_drones),
    ])
