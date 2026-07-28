from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def _create_executor_nodes(context):
    spin_count = int(LaunchConfiguration("spin.node_count").perform(context))
    single_count = int(LaunchConfiguration("single.node_count").perform(context))
    multi_count = int(LaunchConfiguration("multi.node_count").perform(context))
    timer_period_ms = LaunchConfiguration("node.timer_period_ms")
    topic_prefix = LaunchConfiguration("node.topic_prefix")
    log_every_n = LaunchConfiguration("node.log_every_n")
    multi_thread_count = LaunchConfiguration("multi.thread_count")
    use_fss_sim_time = LaunchConfiguration("node.use_fss_sim_time")

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
                        "topic_prefix": topic_prefix,
                        "log_every_n": log_every_n,
                        "multi_thread_count": multi_thread_count,
                        "use_fss_sim_time": use_fss_sim_time,
                    }],
                )
            )

    append_nodes("fss_spin", spin_count)
    append_nodes("single_threaded", single_count)
    append_nodes("multi_threaded", multi_count)
    return actions


def generate_launch_description():
    sim_time_broker_launch = PathJoinSubstitution(
        [FindPackageShare("fss_time"), "launch", "sim_time_broker.launch.py"]
    )

    return LaunchDescription([
        DeclareLaunchArgument("spin.node_count", default_value="1"),
        DeclareLaunchArgument("single.node_count", default_value="1"),
        DeclareLaunchArgument("multi.node_count", default_value="1"),
        DeclareLaunchArgument("multi.thread_count", default_value="2"),
        DeclareLaunchArgument("node.timer_period_ms", default_value="10"),
        DeclareLaunchArgument("node.topic_prefix", default_value="executor_time"),
        DeclareLaunchArgument("node.log_every_n", default_value="100"),
        DeclareLaunchArgument("node.use_fss_sim_time", default_value="true"),
        DeclareLaunchArgument("broker.max_real_time_factor", default_value="1.0"),
        DeclareLaunchArgument("broker.auto_start", default_value="true"),
        SetParameter(name="use_sim_time", value=True),
        TimerAction(
            period=0.2,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(sim_time_broker_launch),
                    launch_arguments={
                        "max_real_time_factor": LaunchConfiguration("broker.max_real_time_factor"),
                        "auto_start": LaunchConfiguration("broker.auto_start"),
                    }.items(),
                ),
            ],
        ),
        OpaqueFunction(function=_create_executor_nodes),
    ])
