from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


'''
### Unique changes for parent time coordinator:
- The `fss_time_coordinator_endpoint` is set to "tcp://*:5545" to bind to a specific TCP port for the parent coordinator so that child coordinators can connect to it.
- The `fss_time_coordinator_pub_endpoint` is set to "tcp://*:0" to bind to a random idle TCP port for the parent coordinator's PUB socket so that child coordinators can connect to it.
- The `publish_clock` parameter is set to "false" because the parent coordinator does not need to publish the /clock topic.
- The `namespace` parameter is set to "parent1" to give the parent coordinator a unique namespace.
'''

'''
### For child coordinators, 
- set `fss_time_parent_coordinator_endpoint` to the `fss_time_coordinator_endpoint` of the parent coordinator (e.g., "tcp://parent_IP:5545") so that the child coordinator can connect to the parent and follow its time.
- (optional) set `namespace` to a unique value for the child coordinator to avoid name collisions with the parent coordinator.
'''


def generate_launch_description():
    """Launch a parent coordinator with TCP endpoints for child coordinators."""
    return LaunchDescription([
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"), # force rcl log color
        DeclareLaunchArgument(
            "namespace",
            default_value="parent1",
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
        DeclareLaunchArgument(
            "fss_time_coordinator_endpoint",
            default_value="tcp://*:5545",
            description="Bind endpoint for the coordinator ROUTER socket; the default IPC path includes the namespace to avoid collisions.",
        ),
        DeclareLaunchArgument(
            "fss_time_coordinator_pub_endpoint",
            default_value="tcp://*:0",
            description="Bind endpoint for the zeromq PUB socket if it is a parent coordinator; child coordinators discover it automatically. tcp://*:0 binds to a random idle TCP port for example. Make sure it is accessible to the child coordinators.",
        ),
        DeclareLaunchArgument(
            "fss_time_parent_coordinator_endpoint",
            default_value="",
            description="Parent fss_time_coordinator_endpoint to connect to; leave empty for a root coordinator. tcp://parent_IP:5545 for example.",
        ),
        DeclareLaunchArgument(
            "publish_clock",
            default_value="false",
            choices=["true", "false"],
            description="Publish the /clock topic required by simulation-time participants.",
        ),
        DeclareLaunchArgument(
            "speed_regulator_step_ns",
            default_value="10000000",
            description="Clock regulator step in nanoseconds; use at least 1e5 and no greater than the smallest participant step.",
        ),
        DeclareLaunchArgument(
            "follows_real_time",
            default_value="true",
            choices=["true", "false"],
            description="Apply a wall-time catch-up floor; false uses only participant requests and the speed regulator.",
        ),
        DeclareLaunchArgument(
            "zombie_participant_timeout_ms",
            default_value="500",
            description="Time without a safe-time announcement before a participant can be considered zombie, in milliseconds.",
        ),
        GroupAction(
            scoped=True,
            actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare("fss_time"), "launch", "time_coordinator.launch.py",
                ])),
                launch_arguments={
                    "namespace": LaunchConfiguration("namespace"),
                    "max_real_time_factor": LaunchConfiguration("max_real_time_factor"),
                    "auto_start": LaunchConfiguration("auto_start"),
                    "start_coordinator_ui": LaunchConfiguration("start_coordinator_ui"),
                    "ui_always_on_top": LaunchConfiguration("ui_always_on_top"),
                    "fss_time_coordinator_endpoint": LaunchConfiguration("fss_time_coordinator_endpoint"),
                    "fss_time_coordinator_pub_endpoint": LaunchConfiguration("fss_time_coordinator_pub_endpoint"),
                    "fss_time_parent_coordinator_endpoint": LaunchConfiguration("fss_time_parent_coordinator_endpoint"),
                    "publish_clock": LaunchConfiguration("publish_clock"),
                    "speed_regulator_step_ns": LaunchConfiguration("speed_regulator_step_ns"),
                    "follows_real_time": LaunchConfiguration("follows_real_time"),
                    "zombie_participant_timeout_ms": LaunchConfiguration("zombie_participant_timeout_ms"),
                }.items(),
            ),
        ]),
    ])
