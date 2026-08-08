from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription([
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
        # Official MAVROS plugins communicate directly with the simulator's
        # PX4-core MAVLink UDP endpoint.
        Node(
            package="mavros", executable="mavros_node", name="mavros", output="screen",
            # Standard MAVLink UDP FCU endpoint provided by PX4SITL.
            parameters=[{
                "fcu_url": "udp://127.0.0.1:14557@127.0.0.1:14540",
                "plugin_allowlist": ["setpoint_raw", "local_position", "imu", "sys_status", "command", "global_position"],
                "plugin_denylist": ["*"],
                # FSS_SIM_MODIFICATION: the copied PX4 stream set has no
                # COMMAND_ACK stream and reported queued commands as accepted.
                "fss_simulate_no_command_ack": True,
                "use_sim_time": LaunchConfiguration("use_fss_sim_time"),
            }],
        ),
        Node(
            package="fss_px4_sim", executable="mavros_px4_quadrotor_sim_node",
            name="mavros_px4_quadrotor_sim_node", output="screen",
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
                "world_origin_latitude_deg": LaunchConfiguration("world_origin_latitude_deg"),
                "world_origin_longitude_deg": LaunchConfiguration("world_origin_longitude_deg"),
                "world_origin_AMSL_alt_metre": LaunchConfiguration("world_origin_AMSL_alt_metre"),
                "mavlink_udp_local_port": 14540,
                "mavlink_udp_remote_port": 14557,
                "fss_time_coordinator_endpoint": LaunchConfiguration("fss_time_coordinator_endpoint"),
                "use_fss_sim_time": LaunchConfiguration("use_fss_sim_time"),
                "use_sim_time": LaunchConfiguration("use_fss_sim_time"),
                },
            ],
        ),
    ])
