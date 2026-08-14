from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    return LaunchDescription([
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"), # force rcl log color
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="ROS namespace for this vehicle; empty uses the root namespace.",
        ),
        DeclareLaunchArgument(
            "use_fss_sim_time",
            default_value="true",
            choices=["true", "false"],
            description="Use the FastSwarmSim coordinated simulation clock.",
        ),
        DeclareLaunchArgument(
            "init_x",
            default_value="0.0",
            description="Initial x position in metres.",
        ),
        DeclareLaunchArgument(
            "init_y",
            default_value="0.0",
            description="Initial y position in metres.",
        ),
        DeclareLaunchArgument(
            "init_z",
            default_value="1.0",
            description="Initial z position in metres.",
        ),
        DeclareLaunchArgument(
            "init_yaw",
            default_value="0.0",
            description="Initial yaw in radians.",
        ),
        DeclareLaunchArgument(
            "enable_time_coordinator",
            default_value="true",
            choices=["true", "false"],
            description="Start the time coordinator when FastSwarmSim simulation time is enabled.",
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
            default_value="30.0",
            description="History path duration in seconds.",
        ),
        DeclareLaunchArgument(
            "vis.color",
            default_value="red",
            choices=["red", "green", "blue", "grey"],
            description="Vehicle color used to select the Iris URDF model.",
        ),
        DeclareLaunchArgument(
            "enable_rviz",
            default_value="true",
            choices=["true", "false"],
            description="Start RViz.",
        ),
        DeclareLaunchArgument("rviz_config", default_value=PathJoinSubstitution([
            FindPackageShare("fss_px4_sim"), "rviz", "single_px4_rotor.rviz"]), description="RViz configuration file used when RViz is enabled."),

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
                        "publish_clock": "true",
                    }.items(),
                ),
            ],
        ),

        ## Visualizer
        # Recommended: scope each included launch to isolate its parameters and actions and prevent leakage.
        GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(PathJoinSubstitution([
                        FindPackageShare("fss_px4_sim"), "launch", "drone_visualizer_single.launch.py"])),
                    condition=IfCondition(LaunchConfiguration("enable_visualizer")),
                    launch_arguments={
                        "namespace": namespace,
                        "use_sim_time": LaunchConfiguration("use_fss_sim_time"),
                        "visualize_max_freq": LaunchConfiguration("vis.visualize_max_freq"),
                        "enable_history_path": LaunchConfiguration("vis.enable_history_path"),
                        "visualize_path_time": LaunchConfiguration("vis.visualize_path_time"),
                        "vis.color": LaunchConfiguration("vis.color"),
                    }.items(),
                ),
            ],
        ),

        ## Rviz
        Node(
            package="rviz2", executable="rviz2", name="rviz2", output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            condition=IfCondition(LaunchConfiguration("enable_rviz")),
        ),

        ## perfect drone dynamics + mavros like interface
        GroupAction(
            scoped=True,
            actions=[
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
            }],
        ),
            ],
        ),
    ])
