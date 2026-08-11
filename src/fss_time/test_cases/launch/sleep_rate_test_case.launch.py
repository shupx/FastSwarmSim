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
        DeclareLaunchArgument("rate.topic_name", default_value="fss_rate"),
        DeclareLaunchArgument("rate.period_ms", default_value="10"),
        DeclareLaunchArgument("rate.enable_loop_sleep_for", default_value="false"),
        DeclareLaunchArgument("rate.loop_sleep_for_ms", default_value="1"),
        DeclareLaunchArgument("timer.topic_name", default_value="timer_sleep"),
        DeclareLaunchArgument("timer.period_ms", default_value="10"),
        DeclareLaunchArgument("timer.callback_sleep_for_ms", default_value="5"),
        DeclareLaunchArgument("node.log_every_n", default_value="100"),
        DeclareLaunchArgument("coordinator.max_real_time_factor", default_value="1.0"),
        DeclareLaunchArgument("coordinator.auto_start", default_value="true"),
        
        DeclareLaunchArgument("fss_time_coordinator_endpoint", default_value="ipc:///tmp/fss_time_coordinator.ipc"),
        SetParameter(name="fss_time_coordinator_endpoint", value=LaunchConfiguration("fss_time_coordinator_endpoint")),

        DeclareLaunchArgument("use_fss_sim_time", default_value="true"),
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
