from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def _create_load_nodes(context):
    node_count = int(LaunchConfiguration("node_count").perform(context))
    config_file = LaunchConfiguration("config_file")
    thread_count = LaunchConfiguration("thread_count")
    base_period_ms = LaunchConfiguration("base_period_ms")
    period_step_ms = LaunchConfiguration("period_step_ms")
    wait_poll_ms = LaunchConfiguration("wait_poll_ms")
    enable_fss_time = LaunchConfiguration("enable_fss_time")
    broker_address = LaunchConfiguration("broker_address")
    broker_port = LaunchConfiguration("broker_port")
    helics_core_type = LaunchConfiguration("helics_core_type")
    helics_time_delta_ns = LaunchConfiguration("helics_time_delta_ns")
    topic_prefix = LaunchConfiguration("topic_prefix")

    actions = []
    for index in range(node_count):
      actions.append(
          Node(
              package="fss_time",
              executable="sim_time_test_load_node",
              namespace=f"sim_time_test/node_{index}",
              name=f"load_node_{index}",
              output="screen",
              parameters=[
                  config_file,
                  {
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
                  },
              ],
          )
      )
    return actions


def generate_launch_description():
    distributed_clock_launch = PathJoinSubstitution(
        [FindPackageShare("fss_time"), "launch", "distributed_clock.launch.py"]
    )
    default_config = PathJoinSubstitution(
        [FindPackageShare("fss_time"), "test_cases", "config", "multi_node_sim_time_test.yaml"]
    )

    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=default_config),
        DeclareLaunchArgument("node_count", default_value="3"),
        DeclareLaunchArgument("thread_count", default_value="4"),
        DeclareLaunchArgument("base_period_ms", default_value="10"),
        DeclareLaunchArgument("period_step_ms", default_value="5"),
        DeclareLaunchArgument("wait_poll_ms", default_value="1"),
        DeclareLaunchArgument("topic_prefix", default_value="load"),
        DeclareLaunchArgument("enable_fss_time", default_value="true"),
        DeclareLaunchArgument("max_real_time_factor", default_value="1.0"),
        DeclareLaunchArgument("helics_core_type", default_value="zmq"),
        DeclareLaunchArgument("broker_address", default_value="127.0.0.1"),
        DeclareLaunchArgument("broker_port", default_value="23404"),
        DeclareLaunchArgument("helics_broker_federates", default_value="0"),
        DeclareLaunchArgument("helics_time_delta_ns", default_value="1000000"),
        DeclareLaunchArgument("speed_regulator_tick_ns", default_value="1000000"),
        DeclareLaunchArgument("start_broker", default_value="true"),
        SetParameter(name="use_sim_time", value=True),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(distributed_clock_launch),
            launch_arguments={
                "max_real_time_factor": LaunchConfiguration("max_real_time_factor"),
                "helics_core_type": LaunchConfiguration("helics_core_type"),
                "broker_address": LaunchConfiguration("broker_address"),
                "broker_port": LaunchConfiguration("broker_port"),
                "helics_broker_federates": LaunchConfiguration("helics_broker_federates"),
                "helics_time_delta_ns": LaunchConfiguration("helics_time_delta_ns"),
                "speed_regulator_tick_ns": LaunchConfiguration("speed_regulator_tick_ns"),
                "start_broker": LaunchConfiguration("start_broker"),
            }.items(),
        ),
        OpaqueFunction(function=_create_load_nodes),
    ])
