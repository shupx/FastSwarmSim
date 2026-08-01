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
        DeclareLaunchArgument("speed_regulator_step_ns", default_value="5000000"), # Set between [1e5, min step of all time participants], and a larger value improves RTF. REASON: If all time participants announce infinite safe time (a possible case when using fss_time::executors), the step of the broker /clock will be equal to this value. Set to the minimum step of all time participants to avoid time jumps. And do not set it too low to avoid high CPU usage of the broker, which may lower the real time factor (RTF). A reasonable value is 5ms (5000000ns) for a broker, which supports time participants with a maximum frequency of 200Hz, and a RTF of 50x.
        DeclareLaunchArgument("start_broker_ui", default_value="true"),
        # DeclareLaunchArgument("fss_time_broker_endpoint", default_value="ipc:///tmp/fss_time_broker.ipc"),
        SetParameter(name="use_sim_time", value=True),
        Node(
            package="fss_time",
            executable="sim_time_broker",
            name="fss_sim_time_broker",
            output="screen",
            parameters=[{
                "max_real_time_factor": LaunchConfiguration("max_real_time_factor"),
                "auto_start": LaunchConfiguration("auto_start"),
                "speed_regulator_step_ns": LaunchConfiguration("speed_regulator_step_ns"),
                # "fss_time_broker_endpoint": LaunchConfiguration("fss_time_broker_endpoint"),
            }],
        ),
        Node(
            package="fss_time",
            executable="sim_time_broker_ui.py",
            name="fss_sim_time_broker_ui",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_broker_ui")),
        ),
    ])
