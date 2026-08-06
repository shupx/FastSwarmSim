from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="uav1"),
        DeclareLaunchArgument("use_fss_sim_time", default_value="true"),
        DeclareLaunchArgument("init_x_East_metre", default_value="0.0"),
        DeclareLaunchArgument("init_y_North_metre", default_value="0.0"),
        DeclareLaunchArgument("init_z_Up_metre", default_value="0.0"),
        DeclareLaunchArgument("init_roll_deg", default_value="0.0"),
        DeclareLaunchArgument("init_pitch_deg", default_value="0.0"),
        DeclareLaunchArgument("init_yaw_deg", default_value="0.0"),
        DeclareLaunchArgument("local_pos_source", default_value="0"),
        DeclareLaunchArgument("world_origin_latitude_deg", default_value="39.978861"),
        DeclareLaunchArgument("world_origin_longitude_deg", default_value="116.339803"),
        DeclareLaunchArgument("world_origin_AMSL_alt_metre", default_value="53.0"),
        DeclareLaunchArgument("fss_time_coordinator_endpoint", default_value="ipc:///tmp/fss_time_coordinator.ipc"),
        SetParameter(name="use_fss_sim_time", value=LaunchConfiguration("use_fss_sim_time")),
        SetParameter(name="use_sim_time", value=LaunchConfiguration("use_fss_sim_time")),
        PushRosNamespace(namespace),
        Node(
            package="fss_px4_sim",
            executable="mavros_px4_quadrotor_sim_node",
            name="mavros_px4_quadrotor_sim_node",
            output="screen",
            parameters=[
                PathJoinSubstitution([FindPackageShare("fss_px4_sim"), "config", "px4_rotor_sim_ros2.yaml"]),
                PathJoinSubstitution([FindPackageShare("fss_px4_sim"), "config", "px4_rotor_sim_px4_params_ros2.yaml"]),
                PathJoinSubstitution([FindPackageShare("fss_px4_sim"), "config", "mavros_px4_config_ros2.yaml"]),
                {
                "init_x_East_metre": LaunchConfiguration("init_x_East_metre"),
                "init_y_North_metre": LaunchConfiguration("init_y_North_metre"),
                "init_z_Up_metre": LaunchConfiguration("init_z_Up_metre"),
                "init_roll_deg": LaunchConfiguration("init_roll_deg"),
                "init_pitch_deg": LaunchConfiguration("init_pitch_deg"),
                "init_yaw_deg": LaunchConfiguration("init_yaw_deg"),
                "local_pos_source": LaunchConfiguration("local_pos_source"),
                "fss_time_coordinator_endpoint": LaunchConfiguration("fss_time_coordinator_endpoint"),
                "use_fss_sim_time": LaunchConfiguration("use_fss_sim_time"),
                "use_sim_time": LaunchConfiguration("use_fss_sim_time"),
                "world_origin_latitude_deg": LaunchConfiguration("world_origin_latitude_deg"),
                "world_origin_longitude_deg": LaunchConfiguration("world_origin_longitude_deg"),
                "world_origin_AMSL_alt_metre": LaunchConfiguration("world_origin_AMSL_alt_metre"),
                },
            ],
        ),
    ])
