from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def _create_load_nodes(context):
    node_count = int(LaunchConfiguration("test.node_count").perform(context))
    thread_count = LaunchConfiguration("node.thread_count")
    base_period_ms = LaunchConfiguration("node.base_period_ms")
    period_step_ms = LaunchConfiguration("node.period_step_ms")
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
                  "enable_fss_time": enable_fss_time,
                  "topic_prefix": topic_prefix,
              }],
          )
      )
    return actions


def generate_launch_description():
    time_coordinator_launch = PathJoinSubstitution(
        [FindPackageShare("fss_time"), "launch", "time_coordinator.launch.py"]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "test.node_count",
            default_value="3",
            description="Number of load-test nodes.",
        ),
        DeclareLaunchArgument(
            "node.thread_count",
            default_value="4",
            description="Thread count for each load-test node.",
        ),
        DeclareLaunchArgument(
            "node.base_period_ms",
            default_value="10",
            description="Base timer period in milliseconds.",
        ),
        DeclareLaunchArgument(
            "node.period_step_ms",
            default_value="5",
            description="Timer period increment between nodes in milliseconds.",
        ),
        DeclareLaunchArgument(
            "node.topic_prefix",
            default_value="load",
            description="Topic prefix used by load-test nodes.",
        ),
        DeclareLaunchArgument(
            "node.enable_fss_time",
            default_value="true",
            choices=["true", "false"],
            description="Enable FastSwarmSim time in load-test nodes.",
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
        
        SetParameter(name="use_sim_time", value=True),
        TimerAction(
            period=2.0,
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
        OpaqueFunction(function=_create_load_nodes),
    ])
