from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def _create_load_nodes(context):
    node_count = int(LaunchConfiguration("test.node_count").perform(context))
    thread_count = LaunchConfiguration("node.thread_count")
    base_period_ms = LaunchConfiguration("node.base_period_ms")
    period_step_ms = LaunchConfiguration("node.period_step_ms")
    wait_poll_ms = LaunchConfiguration("node.wait_poll_ms")
    enable_fss_time = LaunchConfiguration("node.enable_fss_time")
    topic_prefix = LaunchConfiguration("node.topic_prefix")

    actions = []
    for index in range(node_count):
      actions.append(
          Node(
              package="fss_time",
              executable="sim_time_test_load_node",
              namespace=f"node_{index}",
              name=f"load_node_{index}",
              output="screen",
              parameters=[{
                  "thread_count": thread_count,
                  "base_period_ms": base_period_ms,
                  "period_step_ms": period_step_ms,
                  "wait_poll_ms": wait_poll_ms,
                  "enable_fss_time": enable_fss_time,
                  "topic_prefix": topic_prefix,
              }],
          )
      )
    return actions


def generate_launch_description():
    sim_time_broker_launch = PathJoinSubstitution(
        [FindPackageShare("fss_time"), "launch", "sim_time_broker.launch.py"]
    )

    return LaunchDescription([
        DeclareLaunchArgument("test.node_count", default_value="3"),
        DeclareLaunchArgument("node.thread_count", default_value="4"),
        DeclareLaunchArgument("node.base_period_ms", default_value="10"),
        DeclareLaunchArgument("node.period_step_ms", default_value="5"),
        DeclareLaunchArgument("node.wait_poll_ms", default_value="1"),
        DeclareLaunchArgument("node.topic_prefix", default_value="load"),
        DeclareLaunchArgument("node.enable_fss_time", default_value="true"),
        DeclareLaunchArgument("broker.max_real_time_factor", default_value="1.0"),
        DeclareLaunchArgument("broker.auto_start", default_value="true"),
        SetParameter(name="use_sim_time", value=True),
        TimerAction(
            period=3.0,
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
        OpaqueFunction(function=_create_load_nodes),
    ])
