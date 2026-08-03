from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"), # force rcl log color
        DeclareLaunchArgument("max_real_time_factor", default_value="1.0"),
        DeclareLaunchArgument("auto_start", default_value="true"),
        DeclareLaunchArgument("start_coordinator_ui", default_value="true"),

        DeclareLaunchArgument("speed_regulator_step_ns", default_value="5000000"), # Set between [1e5, min step of all time participants], and a larger value improves RTF. REASON: If all time participants announce infinite safe time (a possible case when using fss_time::executors), the step of the coordinator /clock will be equal to this value. Set to the minimum step of all time participants to avoid time jumps. And do not set it too low to avoid high CPU usage of the coordinator, which may lower the real time factor (RTF). A reasonable value is 5ms (5000000ns) for a coordinator, which supports time participants with a maximum frequency of 200Hz, and a RTF of 50x.

        DeclareLaunchArgument("follows_real_time", default_value="true"), # If true, the coordinator applies a wall-time catch-up floor to participant requests so long-running callbacks do not leave observed RTF below 1x when max_real_time_factor allows it. Set false to use only participant requests and the speed regulator.

        # DeclareLaunchArgument("fss_time_coordinator_endpoint", default_value="ipc:///tmp/fss_time_coordinator.ipc"),  # leave for other launch file that includes this one to set, so that time coordinator and time participants can be launched using the same fss_time_coordinator_endpoint.
        
        SetParameter(name="use_sim_time", value=True),
        Node(
            package="fss_time",
            executable="time_coordinator",
            name="fss_time_coordinator",
            output="screen",
            parameters=[{
                "max_real_time_factor": LaunchConfiguration("max_real_time_factor"),
                "auto_start": LaunchConfiguration("auto_start"),
                "speed_regulator_step_ns": LaunchConfiguration("speed_regulator_step_ns"),
                "follows_real_time": LaunchConfiguration("follows_real_time"),
                # "fss_time_coordinator_endpoint": LaunchConfiguration("fss_time_coordinator_endpoint"),
            }],
        ),
        Node(
            package="fss_time",
            executable="time_coordinator_ui.py",
            name="fss_time_coordinator_ui",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_coordinator_ui")),
        ),
    ])
