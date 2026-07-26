from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
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
    broker_address = LaunchConfiguration("broker.address")
    broker_port = LaunchConfiguration("broker.port")
    helics_core_type = LaunchConfiguration("helics.core_type")
    helics_time_delta_ns = LaunchConfiguration("helics.time_delta_ns")
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
                  "broker_address": broker_address,
                  "broker_port": broker_port,
                  "helics_core_type": helics_core_type,
                  "helics_time_delta_ns": helics_time_delta_ns,
                  "topic_prefix": topic_prefix,
              }],
          )
      )
    return actions


def generate_launch_description():
    distributed_clock_launch = PathJoinSubstitution(
        [FindPackageShare("fss_time"), "launch", "distributed_clock.launch.py"]
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
        DeclareLaunchArgument("helics.core_type", default_value="zmq"),
        DeclareLaunchArgument("broker.address", default_value="127.0.0.1"),
        DeclareLaunchArgument("broker.port", default_value="23404"),
        DeclareLaunchArgument("helics.broker_federates", default_value="0"),
        DeclareLaunchArgument("helics.time_delta_ns", default_value="1000000"),
        DeclareLaunchArgument("helics.speed_regulator_tick_ns", default_value="1000000"),
        DeclareLaunchArgument("broker.participant_query_period_ns", default_value="500000000"),
        DeclareLaunchArgument("broker.start", default_value="true"),
        SetParameter(name="use_sim_time", value=True),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(distributed_clock_launch),
            launch_arguments={
                "max_real_time_factor": LaunchConfiguration("broker.max_real_time_factor"),
                "helics_core_type": LaunchConfiguration("helics.core_type"),
                "broker_address": LaunchConfiguration("broker.address"),
                "broker_port": LaunchConfiguration("broker.port"),
                "helics_broker_federates": LaunchConfiguration("helics.broker_federates"),
                "helics_time_delta_ns": LaunchConfiguration("helics.time_delta_ns"),
                "speed_regulator_tick_ns": LaunchConfiguration("helics.speed_regulator_tick_ns"),
                "participant_query_period_ns": LaunchConfiguration("broker.participant_query_period_ns"),
                "start_broker": LaunchConfiguration("broker.start"),
            }.items(),
        ),
        OpaqueFunction(function=_create_load_nodes),
    ])
