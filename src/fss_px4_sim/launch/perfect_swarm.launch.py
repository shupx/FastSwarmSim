from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def make_drones(context):
    count = int(LaunchConfiguration("num_drones").perform(context))
    drones_per_row = int(LaunchConfiguration("drones_per_row").perform(context))
    if drones_per_row < 1:
        raise ValueError("drones_per_row must be at least 1")
    launch_file = PathJoinSubstitution([
        FindPackageShare("fss_px4_sim"), "launch", "perfect_drone.launch.py"])
    return [
        GroupAction(
            scoped=True,
            actions=[
                # Recommended: scope each included launch to isolate its parameters and actions and prevent leakage.
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(launch_file),
                    launch_arguments={
                        "namespace": f"uav{index}",
                        "init_x": f"{(index - 1) % drones_per_row:.1f}",
                        "init_y": f"{(index - 1) // drones_per_row:.1f}",
                        "use_fss_sim_time": LaunchConfiguration("use_fss_sim_time"),
                        "fss_time_coordinator_endpoint": LaunchConfiguration("fss_time_coordinator_endpoint"),
                        "enable_mavros": LaunchConfiguration("enable_mavros"),
                        "enable_visualizer": LaunchConfiguration("enable_visualizer"),
                        "enable_rviz": "false",
                        "enable_time_coordinator": "false",
                    }.items(),
                ),
            ],
        )
        for index in range(1, count + 1)
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("num_drones", default_value="3"),
        DeclareLaunchArgument("drones_per_row", default_value="10"),
        DeclareLaunchArgument("use_fss_sim_time", default_value="true"),
        DeclareLaunchArgument("fss_time_coordinator_endpoint", default_value="ipc:///tmp/fss_time_coordinator.ipc"),
        DeclareLaunchArgument("enable_time_coordinator", default_value="true"),
        DeclareLaunchArgument("enable_mavros", default_value="true"),
        DeclareLaunchArgument("enable_visualizer", default_value="true"),
        DeclareLaunchArgument("enable_rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=PathJoinSubstitution([
            FindPackageShare("fss_px4_sim"), "rviz", "multi_px4_rotor.rviz"])),
        SetParameter(name="use_fss_sim_time", value=LaunchConfiguration("use_fss_sim_time")),
        SetParameter(name="use_sim_time", value=LaunchConfiguration("use_fss_sim_time")),
        # Recommended: scope each included launch to isolate its parameters and actions and prevent leakage.
        GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(PathJoinSubstitution([
                        FindPackageShare("fss_time"), "launch", "time_coordinator.launch.py"])),
                    condition=IfCondition(PythonExpression([
                        "'", LaunchConfiguration("use_fss_sim_time"),
                        "' == 'true' and '", LaunchConfiguration("enable_time_coordinator"), "' == 'true'",
                    ])),
                    launch_arguments={
                        "fss_time_coordinator_endpoint": LaunchConfiguration("fss_time_coordinator_endpoint"),
                        "publish_clock": "true",
                    }.items(),
                ),
            ],
        ),
        Node(
            package="rviz2", executable="rviz2", name="rviz2", output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            condition=IfCondition(LaunchConfiguration("enable_rviz")),
        ),
        OpaqueFunction(function=make_drones),
    ])
