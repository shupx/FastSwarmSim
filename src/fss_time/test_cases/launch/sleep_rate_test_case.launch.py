from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    time_coordinator_launch = PathJoinSubstitution(
        [FindPackageShare("fss_time"), "launch", "time_coordinator.launch.py"]
    )

    return LaunchDescription([
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"), # force rcl log color
        DeclareLaunchArgument(
            "rate.topic_name",
            default_value="fss_rate",
            description="Topic name used by the rate test.",
        ),
        DeclareLaunchArgument(
            "rate.period_ms",
            default_value="10",
            description="Rate test period in milliseconds.",
        ),
        DeclareLaunchArgument(
            "rate.enable_loop_sleep_for",
            default_value="false",
            choices=["true", "false"],
            description="Enable deliberate sleep in the rate test loop.",
        ),
        DeclareLaunchArgument(
            "rate.loop_sleep_for_ms",
            default_value="1",
            description="Deliberate rate-loop sleep duration in milliseconds.",
        ),
        DeclareLaunchArgument(
            "timer.topic_name",
            default_value="timer_sleep",
            description="Topic name used by the timer test.",
        ),
        DeclareLaunchArgument(
            "timer.period_ms",
            default_value="10",
            description="Timer test period in milliseconds.",
        ),
        DeclareLaunchArgument(
            "timer.callback_sleep_for_ms",
            default_value="5",
            description="Deliberate timer callback sleep duration in milliseconds.",
        ),
        DeclareLaunchArgument(
            "node.log_every_n",
            default_value="100",
            description="Log every N callbacks.",
        ),
        DeclareLaunchArgument(
            "coordinator.max_real_time_factor",
            default_value="1.0",
            description="Maximum real-time factor passed to the coordinator.",
        ),
        DeclareLaunchArgument(
            "coordinator.auto_start",
            default_value="true",
            choices=["true", "false"],
            description="Automatically start the coordinator.",
        ),
        
        DeclareLaunchArgument(
            "fss_time_coordinator_endpoint",
            default_value="ipc:///tmp/fss_time_coordinator.ipc",
            description="IPC endpoint of the FastSwarmSim time coordinator.",
        ),
        SetParameter(name="fss_time_coordinator_endpoint", value=LaunchConfiguration("fss_time_coordinator_endpoint")),

        DeclareLaunchArgument(
            "use_fss_sim_time",
            default_value="true",
            choices=["true", "false"],
            description="Use the FastSwarmSim coordinated simulation clock.",
        ),
        SetParameter(name="use_fss_sim_time", value=LaunchConfiguration("use_fss_sim_time")),
        SetParameter(name="use_sim_time", value=LaunchConfiguration("use_fss_sim_time")),

        TimerAction(
            period=0.2,
            actions=[
                # Recommended: scope each included launch to isolate its parameters and actions and prevent leakage.
                GroupAction(
                    scoped=True,
                    actions=[
                        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(time_coordinator_launch),
                            launch_arguments={
                                "max_real_time_factor": LaunchConfiguration("coordinator.max_real_time_factor"),
                                "auto_start": LaunchConfiguration("coordinator.auto_start"),
                                "fss_time_coordinator_endpoint": LaunchConfiguration("fss_time_coordinator_endpoint"),
                            }.items(),
                        ),
                    ],
                ),
            ],
        ),
        Node(
            package="fss_time",
            executable="fss_rate_test_node",
            namespace="test",
            name="fss_rate_node",
            output="screen",
            parameters=[{
                "topic_name": LaunchConfiguration("rate.topic_name"),
                "rate_period_ms": LaunchConfiguration("rate.period_ms"),
                "enable_loop_sleep_for": LaunchConfiguration("rate.enable_loop_sleep_for"),
                "loop_sleep_for_ms": LaunchConfiguration("rate.loop_sleep_for_ms"),
                "log_every_n": LaunchConfiguration("node.log_every_n"),
            }],
        ),
        Node(
            package="fss_time",
            executable="timer_sleep_test_node",
            namespace="test",
            name="timer_sleep_node",
            output="screen",
            parameters=[{
                "topic_name": LaunchConfiguration("timer.topic_name"),
                "timer_period_ms": LaunchConfiguration("timer.period_ms"),
                "callback_sleep_for_ms": LaunchConfiguration("timer.callback_sleep_for_ms"),
                "log_every_n": LaunchConfiguration("node.log_every_n"),
            }],
        ),
    ])
