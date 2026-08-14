from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def make_drones(context):
    count = int(LaunchConfiguration("num_drones").perform(context))
    drones_per_row = int(LaunchConfiguration("drones_per_row").perform(context))
    startup_batch_size = int(LaunchConfiguration("startup_batch_size").perform(context))
    startup_batch_delay_ms = int(LaunchConfiguration("startup_batch_delay_ms").perform(context))
    if count < 1:
        raise ValueError("num_drones must be at least 1")
    if drones_per_row < 1:
        raise ValueError("drones_per_row must be at least 1")
    if startup_batch_size < 1:
        raise ValueError("startup_batch_size must be at least 1")
    if startup_batch_delay_ms < 0:
        raise ValueError("startup_batch_delay_ms must be non-negative")

    single_launch = PathJoinSubstitution([
        FindPackageShare("fss_bringup"),
        "launch",
        "sim_px4_drone_lidar_single.launch.py",
    ])
    drone_actions = [
        GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(single_launch),
                    launch_arguments={
                        "namespace": f"uav{index}",
                        "use_perfect_drone": LaunchConfiguration("use_perfect_drone"),
                        "use_fss_sim_time": LaunchConfiguration("use_fss_sim_time"),
                        "enable_time_coordinator": "false",
                        "enable_sensing": LaunchConfiguration("sensing_enable"),
                        "enable_rviz": "false",
                        "enable_visualizer": LaunchConfiguration("enable_visualizer"),
                        "vis.visualize_max_freq": LaunchConfiguration("vis.visualize_max_freq"),
                        "vis.enable_history_path": LaunchConfiguration("vis.enable_history_path"),
                        "vis.visualize_path_time": LaunchConfiguration("vis.visualize_path_time"),
                        "vis.color": LaunchConfiguration("vis.color"),
                        "init_x_metre": f"{(index - 1) % drones_per_row:.1f}",
                        "init_y_metre": f"{(index - 1) // drones_per_row:.1f}",
                    }.items(),
                ),
            ],
        )
        for index in range(1, count + 1)
    ]
    if startup_batch_delay_ms == 0:
        return drone_actions

    return [
        TimerAction(
            period=(index - 1) // startup_batch_size * startup_batch_delay_ms / 1000.0,
            actions=[drone_action],
        )
        for index, drone_action in enumerate(drone_actions, start=1)
    ]


def generate_launch_description():
    time_coordinator_launch = PathJoinSubstitution([
        FindPackageShare("fss_time"),
        "launch",
        "time_coordinator.launch.py",
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare("fss_bringup"),
        "rviz",
        "sim_px4_drone_lidar_multi.rviz",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "num_drones",
            default_value="5",
            description="Number of vehicles to launch.",
        ),
        DeclareLaunchArgument(
            "drones_per_row",
            default_value="10",
            description="Number of vehicles in each formation row; must be at least 1.",
        ),
        DeclareLaunchArgument(
            "startup_batch_size",
            default_value="10",
            description="Number of vehicles to start per batch; must be at least 1.",
        ),
        DeclareLaunchArgument(
            "startup_batch_delay_ms",
            default_value="0",
            description="Delay in milliseconds before starting each vehicle batch.",
        ),
        DeclareLaunchArgument(
            "use_perfect_drone",
            default_value="true",
            choices=["true", "false"],
            description="Use perfect MAVROS-compatible drones instead of PX4 SITL.",
        ),
        DeclareLaunchArgument(
            "use_fss_sim_time",
            default_value="true",
            choices=["true", "false"],
            description="Use the FastSwarmSim coordinated simulation clock.",
        ),
        DeclareLaunchArgument(
            "enable_time_coordinator",
            default_value="true",
            choices=["true", "false"],
            description="Start the FastSwarmSim time coordinator.",
        ),
        DeclareLaunchArgument(
            "sensing_enable",
            default_value="true",
            choices=["true", "false"],
            description="Start the local LiDAR pointcloud simulator for each vehicle.",
        ),
        DeclareLaunchArgument(
            "rviz_enable",
            default_value="true",
            choices=["true", "false"],
            description="Start RViz.",
        ),
        DeclareLaunchArgument(
            "enable_visualizer",
            default_value="true",
            choices=["true", "false"],
            description="Start vehicle visualizers.",
        ),
        DeclareLaunchArgument(
            "vis.visualize_max_freq",
            default_value="20.0",
            description="Maximum visualization update frequency in Hz.",
        ),
        DeclareLaunchArgument(
            "vis.enable_history_path",
            default_value="true",
            choices=["true", "false"],
            description="Publish vehicle history paths.",
        ),
        DeclareLaunchArgument(
            "vis.visualize_path_time",
            default_value="10.0",
            description="History path duration in seconds.",
        ),
        DeclareLaunchArgument(
            "vis.color",
            default_value="red",
            choices=["red", "green", "blue", "grey"],
            description="Vehicle color used to select the Iris URDF model.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=rviz_config,
            description="RViz configuration file used when RViz is enabled.",
        ),

        SetParameter(name="use_fss_sim_time", value=LaunchConfiguration("use_fss_sim_time")),
        SetParameter(name="use_sim_time", value=LaunchConfiguration("use_fss_sim_time")),
        GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(time_coordinator_launch),
                    condition=IfCondition(PythonExpression([
                        "'", LaunchConfiguration("use_fss_sim_time"),
                        "' == 'true' and '", LaunchConfiguration("enable_time_coordinator"),
                        "' == 'true'",
                    ])),
                    launch_arguments={"publish_clock": "true"}.items(),
                ),
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            condition=IfCondition(LaunchConfiguration("rviz_enable")),
        ),
        OpaqueFunction(function=make_drones),
    ])
