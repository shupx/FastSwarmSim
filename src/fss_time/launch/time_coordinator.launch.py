from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter, PushRosNamespace
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    return LaunchDescription([
        GroupAction(
            scoped=True,
            actions=[
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"), # force rcl log color

        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Namespace for the coordinator and optional UI.",
        ),
        DeclareLaunchArgument(
            "max_real_time_factor",
            default_value="1.0",
            description="Maximum allowed real-time factor.",
        ),
        DeclareLaunchArgument(
            "auto_start",
            default_value="true",
            choices=["true", "false"],
            description="Start the coordinator on the first participant connection automatically, otherwise manually.",
        ),
        DeclareLaunchArgument(
            "start_coordinator_ui",
            default_value="true",
            choices=["true", "false"],
            description="Start the time coordinator UI.",
        ),
        DeclareLaunchArgument(
            "ui_always_on_top",
            default_value="true",
            choices=["true", "false"],
            description="Keep the time coordinator UI above other windows.",
        ),

        # The bind endpoint for the time coordinator's ROUTER socket. (zeroMQ url format, e.g. "tcp://*:5545" or "ipc:///tmp/fss_time_coordinator.ipc"). 
        # The default IPC path includes the namespace to avoid collisions:
        # "" -> ipc:///tmp/fss_time_coordinator.ipc
        # "parent1" -> ipc:///tmp/fss_time_coordinator_parent1.ipc
        # "/parent/child" -> ipc:///tmp/fss_time_coordinator_parent_child.ipc
        DeclareLaunchArgument("fss_time_coordinator_endpoint", default_value=PythonExpression([
            "'ipc:///tmp/fss_time_coordinator' + ('_' + '", LaunchConfiguration("namespace"),
            "'.strip('/').replace('/', '_') if '", LaunchConfiguration("namespace"),
            "' else '') + '.ipc'",
        ]), description="Bind endpoint for the coordinator ROUTER socket; the default IPC path includes the namespace to avoid collisions."),
        # NOTE: use LaunchConfiguration for default value is dangerous, since it may not exactly use the LaunchArgument declared in this file, but use the LaunchArgument from the parent launch file, which may have a different default value. So we use GroupAction(scoped=True) at the beginning to force the LaunchConfiguration to use the LaunchArgument declared in this file scope.

        DeclareLaunchArgument(
            "fss_time_coordinator_pub_endpoint",
            default_value="",
            description="Bind endpoint for the zeromq PUB socket if it is a parent coordinator; child coordinators discover it automatically. tcp://*:0 binds to a random idle TCP port for example. Make sure it is accessible to the child coordinators.",
        ),
        DeclareLaunchArgument(
            "fss_time_parent_coordinator_endpoint",
            default_value="",
            description="Parent fss_time_coordinator_endpoint to connect to; leave empty for a root coordinator. tcp://parent_IP:5545 for example.",
        ),
        DeclareLaunchArgument(
            "publish_clock",
            default_value="true",
            choices=["true", "false"],
            description="Publish the /clock topic required by simulation-time participants.",
        ),
        DeclareLaunchArgument(
            "speed_regulator_step_ns",
            default_value="10000000",
            description="Clock regulator step in nanoseconds; use at least 1e5 and no greater than the smallest participant step.",
        ),# Set between [1e5, min step of all time participants], and a larger value improves RTF. REASON: If all time participants announce infinite safe time (a possible case when using fss_time::executors), the step of the coordinator /clock will be equal to this value. Set to the minimum step of all time participants to avoid time jumps. And do not set it too low to avoid high CPU usage of the coordinator, which may lower the real time factor (RTF). A reasonable value is 5ms (5000000ns) for a coordinator, which supports time participants with a maximum frequency of 200Hz, and a RTF of 50x.
        DeclareLaunchArgument(
            "follows_real_time",
            default_value="true",
            choices=["true", "false"],
            description="Apply a wall-time catch-up floor; false uses only participant requests and the speed regulator.",
        ),# If true, the coordinator applies a wall-time catch-up floor to participant requests so long-running callbacks do not leave observed RTF below 1x when max_real_time_factor allows it. Set false to use only participant requests and the speed regulator.
        DeclareLaunchArgument(
            "zombie_participant_timeout_ms",
            default_value="500",
            description="Time without a safe-time announcement before a participant can be considered zombie, in milliseconds.",
        ),

        GroupAction(
            scoped=True,
            actions=[
            
            PushRosNamespace(namespace=LaunchConfiguration("namespace")), # set the namespace for the coordinator and its UI, if any.

            SetParameter(name="use_sim_time", value=True),
            
            Node(
                package="fss_time",
                executable="time_coordinator",
                name="fss_time_coordinator",
                output="screen",
                parameters=[{
                    "max_real_time_factor": LaunchConfiguration("max_real_time_factor"),
                    "zombie_participant_timeout_ms": LaunchConfiguration("zombie_participant_timeout_ms"),
                    "auto_start": LaunchConfiguration("auto_start"),
                    "publish_clock": LaunchConfiguration("publish_clock"),
                    "speed_regulator_step_ns": LaunchConfiguration("speed_regulator_step_ns"),
                    "follows_real_time": LaunchConfiguration("follows_real_time"),
                    "fss_time_coordinator_endpoint": LaunchConfiguration("fss_time_coordinator_endpoint"),
                    "fss_time_coordinator_pub_endpoint": LaunchConfiguration("fss_time_coordinator_pub_endpoint"),
                    "fss_time_parent_coordinator_endpoint": LaunchConfiguration("fss_time_parent_coordinator_endpoint"),
                }],
            ),
            Node(
                package="fss_time",
                executable="time_coordinator_ui.py",
                name="fss_time_coordinator_ui",
                output="screen",
                parameters=[{
                    "always_on_top": LaunchConfiguration("ui_always_on_top"),
                }],
                condition=IfCondition(LaunchConfiguration("start_coordinator_ui")),
            ),
                ],
            ),
        ]),
    ])
