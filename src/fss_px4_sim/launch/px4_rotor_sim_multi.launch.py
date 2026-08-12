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
        FindPackageShare("fss_px4_sim"), "launch", "px4_rotor_sim_single.launch.py"])
    return [
        # Recommended: scope each included launch to isolate its parameters and actions and prevent leakage.
        GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(launch_file),
                    launch_arguments={
                        "namespace": f"uav{index}",
                        "mav_sys_id": str(index),
                        "mav_comp_id": "1",
                        "px4.mavlink_transport": LaunchConfiguration("px4.mavlink_transport"),
                        "px4.mavlink_udp_local_port": "0", # only used when transport is udp; 0 lets the OS choose a port
                        "px4.mavlink_udp_remote_port": str(24540 + index - 1), # MAVROS receive port for UDP transport
                        "init_x_East_metre": f"{(index - 1) % drones_per_row:.1f}",
                        "init_y_North_metre": f"{(index - 1) // drones_per_row:.1f}",
                        "use_fss_sim_time": LaunchConfiguration("use_fss_sim_time"),
                        "fss_time_coordinator_endpoint": LaunchConfiguration("fss_time_coordinator_endpoint"),
                        "enable_mavros_node": LaunchConfiguration("enable_mavros_node"),
                        "mavros_node_type": LaunchConfiguration("mavros_node_type"),
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
        DeclareLaunchArgument("num_drones", default_value="5"),
        DeclareLaunchArgument("drones_per_row", default_value="10"),
        DeclareLaunchArgument("use_fss_sim_time", default_value="true"),
        DeclareLaunchArgument("fss_time_coordinator_endpoint", default_value="ipc:///tmp/fss_time_coordinator.ipc"),
        DeclareLaunchArgument("enable_time_coordinator", default_value="true"),
        DeclareLaunchArgument("enable_mavros_node", default_value="false"),
        DeclareLaunchArgument("mavros_node_type", default_value="lite"),
        DeclareLaunchArgument("px4.mavlink_transport", default_value="direct_ros"),
        DeclareLaunchArgument("enable_visualizer", default_value="true"),
        DeclareLaunchArgument("enable_rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=PathJoinSubstitution([
            FindPackageShare("fss_px4_sim"), "rviz", "multi_px4_rotor.rviz"])),
        SetParameter(name="use_fss_sim_time", value=LaunchConfiguration("use_fss_sim_time")),
        SetParameter(name="use_sim_time", value=LaunchConfiguration("use_fss_sim_time")),

        ## Time coordinator (only when use_fss_sim_time is true and enable_time_coordinator is true)
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

        ## Rviz2
        Node(
            package="rviz2", executable="rviz2", name="rviz2", output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            condition=IfCondition(LaunchConfiguration("enable_rviz")),
        ),

        ## Make drones
        OpaqueFunction(function=make_drones),
    ])
