from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, PushRosNamespace, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_perfect_drone = LaunchConfiguration("use_perfect_drone")
    use_fss_sim_time = LaunchConfiguration("use_fss_sim_time")

    sensing_launch = PathJoinSubstitution([
        FindPackageShare("fss_sensing"), "launch", "fss_local_pointcloud_sim.launch.py"])
    perfect_drone_launch = PathJoinSubstitution([
        FindPackageShare("fss_px4_sim"), "launch", "perfect_mavros_drone_single.launch.py"])
    px4_drone_launch = PathJoinSubstitution([
        FindPackageShare("fss_px4_sim"), "launch", "px4_rotor_sim_single.launch.py"])
    rviz_config = PathJoinSubstitution([
        FindPackageShare("fss_bringup"), "rviz", "sim_px4_drone_lidar_single.rviz"])

    return LaunchDescription([
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"), # force rcl log color

        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="ROS namespace for this vehicle; empty uses the root namespace.",
        ),
        DeclareLaunchArgument(
            "use_perfect_drone",
            default_value="false",
            choices=["true", "false"],
            description="Use the perfect MAVROS-compatible drone instead of PX4 SITL.",
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
            "enable_sensing",
            default_value="true",
            choices=["true", "false"],
            description="Start the local LiDAR pointcloud simulator.",
        ),
        DeclareLaunchArgument(
            "enable_rviz",
            default_value="true",
            choices=["true", "false"],
            description="Start RViz.",
        ),
        DeclareLaunchArgument(
            "enable_visualizer",
            default_value="true",
            choices=["true", "false"],
            description="Start the drone visualizer.",
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
            description="Publish the vehicle history path.",
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
            "init_x_metre",
            default_value="1.0",
            description="Initial east position in metres.",
        ),
        DeclareLaunchArgument(
            "init_y_metre",
            default_value="0.0",
            description="Initial north position in metres.",
        ),
        DeclareLaunchArgument(
            "init_z_metre",
            default_value="1.0",
            description="Initial up position in metres.",
        ),
        DeclareLaunchArgument(
            "init_yaw_deg",
            default_value="0.0",
            description="Initial yaw in degrees.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=rviz_config,
            description="RViz configuration file used when RViz is enabled.",
        ),

        GroupAction(
            scoped=True,
            actions=[
                PushRosNamespace(namespace),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(sensing_launch),
                    condition=IfCondition(LaunchConfiguration("enable_sensing")),
                    launch_arguments={
                        "use_fss_sim_time": use_fss_sim_time,
                    }.items(),
                ),
            ],
        ),
        GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(perfect_drone_launch),
                    condition=IfCondition(use_perfect_drone),
                    launch_arguments={
                        "namespace": namespace,
                        "use_fss_sim_time": use_fss_sim_time,
                        "enable_time_coordinator": LaunchConfiguration("enable_time_coordinator"),
                        "init_x": LaunchConfiguration("init_x_metre"),
                        "init_y": LaunchConfiguration("init_y_metre"),
                        "init_z": LaunchConfiguration("init_z_metre"),
                        "init_yaw": PythonExpression([
                            "float('", LaunchConfiguration("init_yaw_deg"), "') * 3.141592653589793 / 180.0"]),
                        "enable_visualizer": LaunchConfiguration("enable_visualizer"),
                        "vis.visualize_max_freq": LaunchConfiguration("vis.visualize_max_freq"),
                        "vis.enable_history_path": LaunchConfiguration("vis.enable_history_path"),
                        "vis.visualize_path_time": LaunchConfiguration("vis.visualize_path_time"),
                        "vis.color": LaunchConfiguration("vis.color"),
                        "enable_rviz": "false",
                    }.items(),
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(px4_drone_launch),
                    condition=IfCondition(PythonExpression([
                        "'", use_perfect_drone, "' == 'false'"])),
                    launch_arguments={
                        "namespace": namespace,
                        "use_fss_sim_time": use_fss_sim_time,
                        "enable_time_coordinator": LaunchConfiguration("enable_time_coordinator"),
                        "init_x_East_metre": LaunchConfiguration("init_x_metre"),
                        "init_y_North_metre": LaunchConfiguration("init_y_metre"),
                        "init_z_Up_metre": LaunchConfiguration("init_z_metre"),
                        "init_yaw_deg": LaunchConfiguration("init_yaw_deg"),
                        "enable_visualizer": LaunchConfiguration("enable_visualizer"),
                        "vis.visualize_max_freq": LaunchConfiguration("vis.visualize_max_freq"),
                        "vis.enable_history_path": LaunchConfiguration("vis.enable_history_path"),
                        "vis.visualize_path_time": LaunchConfiguration("vis.visualize_path_time"),
                        "vis.color": LaunchConfiguration("vis.color"),
                        "enable_rviz": "false",
                    }.items(),
                ),
            ],
        ),

        GroupAction(
            scoped=True,
            actions=[
                PushRosNamespace(namespace),
                SetParameter(name="use_sim_time", value=LaunchConfiguration("use_fss_sim_time")),
                Node(
                    package="rviz2", executable="rviz2", name="rviz2", output="screen",
                    arguments=["-d", LaunchConfiguration("rviz_config")],
                    condition=IfCondition(LaunchConfiguration("enable_rviz")),
                ),
            ],
        ),
    ])
