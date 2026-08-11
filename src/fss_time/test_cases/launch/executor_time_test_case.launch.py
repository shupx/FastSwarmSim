from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch.actions import SetEnvironmentVariable


def _create_executor_nodes(context):
    spin_count = int(LaunchConfiguration("spin.node_count").perform(context))
    single_count = int(LaunchConfiguration("single.node_count").perform(context))
    multi_count = int(LaunchConfiguration("multi.node_count").perform(context))
    timer_period_ms = LaunchConfiguration("node.timer_period_ms")
    timer_count = LaunchConfiguration("node.timer_count")
    topic_prefix = LaunchConfiguration("node.topic_prefix")
    log_every_n = LaunchConfiguration("node.log_every_n")
    multi_thread_count = LaunchConfiguration("multi.thread_count")

    actions = []

    def append_nodes(executor_type, count):
        for index in range(count):
            actions.append(
                Node(
                    package="fss_time",
                    executable="executor_time_test_node",
                    namespace=f"{executor_type}_{index}",
                    name=f"{executor_type}_node_{index}",
                    output="screen",
                    parameters=[{
                        "executor_type": executor_type,
                        "timer_period_ms": timer_period_ms,
                        "timer_count": timer_count,
                        "topic_prefix": topic_prefix,
                        "log_every_n": log_every_n,
                        "multi_thread_count": multi_thread_count,
                    }],
                )
            )

    append_nodes("fss_spin", spin_count)
    append_nodes("single_threaded", single_count)
    append_nodes("multi_threaded", multi_count)
    return actions


def generate_launch_description():
    time_coordinator_launch = PathJoinSubstitution(
        [FindPackageShare("fss_time"), "launch", "time_coordinator.launch.py"]
    )

    return LaunchDescription([
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"), # force rcl log color
        DeclareLaunchArgument("spin.node_count", default_value="1"),
        DeclareLaunchArgument("single.node_count", default_value="1"),
        DeclareLaunchArgument("multi.node_count", default_value="1"),
        DeclareLaunchArgument("multi.thread_count", default_value="2"),
        DeclareLaunchArgument("node.timer_period_ms", default_value="10"),
        DeclareLaunchArgument("node.timer_count", default_value="2"),
        DeclareLaunchArgument("node.topic_prefix", default_value="executor_time"),
        DeclareLaunchArgument("node.log_every_n", default_value="100"),
        DeclareLaunchArgument("coordinator.max_real_time_factor", default_value="1.0"),
        DeclareLaunchArgument("coordinator.auto_start", default_value="true"),
        
        DeclareLaunchArgument("fss_time_coordinator_endpoint", default_value="ipc:///tmp/fss_time_coordinator.ipc"),

        DeclareLaunchArgument("use_fss_sim_time", default_value="true"),
        SetParameter(name="use_fss_sim_time", value=LaunchConfiguration("use_fss_sim_time")),
        SetParameter(name="use_sim_time", value=LaunchConfiguration("use_fss_sim_time")),

        # for all time participants
        SetParameter(name="fss_time_coordinator_endpoint", value=LaunchConfiguration("fss_time_coordinator_endpoint")),

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
        OpaqueFunction(function=_create_executor_nodes),
    ])
