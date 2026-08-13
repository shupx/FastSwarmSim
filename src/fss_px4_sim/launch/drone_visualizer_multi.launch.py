from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter
from ament_index_python.packages import get_package_share_directory


def make_visualizers(context):
    count = int(LaunchConfiguration("num_drones").perform(context))
    single_visualizer_launch = (
        get_package_share_directory("fss_px4_sim")
        + "/launch/drone_visualizer_single.launch.py"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    return [
        # Recommended: scope each included launch to isolate its parameters and actions and prevent leakage.
        GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(single_visualizer_launch),
                    launch_arguments={
                        "namespace": f"uav{index}",
                        "use_sim_time": use_sim_time,
                        "local_pos_source": LaunchConfiguration("local_pos_source"),
                        "world_origin_latitude_deg": LaunchConfiguration("world_origin_latitude_deg"),
                        "world_origin_longitude_deg": LaunchConfiguration("world_origin_longitude_deg"),
                        "world_origin_AMSL_alt_metre": LaunchConfiguration("world_origin_AMSL_alt_metre"),
                    }.items(),
                ),
            ],
        )
        for index in range(1, count + 1)
    ]


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"), # force rcl log color
        DeclareLaunchArgument(
            "num_drones",
            default_value="5",
            description="Number of vehicle visualizers to launch.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            choices=["true", "false"],
            description="Use ROS simulation time.",
        ),
        DeclareLaunchArgument(
            "local_pos_source",
            default_value="0",
            choices=["0", "1"],
            description="Position source: 0 for mocap/local position, 1 for GPS/global position.",
        ),
        DeclareLaunchArgument(
            "world_origin_latitude_deg",
            default_value="39.978861",
            description="Geographic origin latitude in degrees.",
        ),
        DeclareLaunchArgument(
            "world_origin_longitude_deg",
            default_value="116.339803",
            description="Geographic origin longitude in degrees.",
        ),
        DeclareLaunchArgument(
            "world_origin_AMSL_alt_metre",
            default_value="53.0",
            description="Geographic origin AMSL altitude in metres.",
        ),
        SetParameter(name="use_sim_time", value=LaunchConfiguration("use_sim_time")),
        OpaqueFunction(function=make_visualizers),
    ])
