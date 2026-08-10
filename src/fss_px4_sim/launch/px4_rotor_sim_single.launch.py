from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    return LaunchDescription([
        # ROS namespace for this vehicle. Empty means use the root namespace.
        DeclareLaunchArgument("namespace", default_value=""),
        # Use the FastSwarmSim coordinated simulation clock when true. If ture, run 'ros2 launch fss_time time_coordinator.launch.py ' first.
        DeclareLaunchArgument("use_fss_sim_time", default_value="true"),
        # MAVLink system identifiers; valid values are 1-255.
        DeclareLaunchArgument("mav_sys_id", default_value="1"),
        # MAVLink component identifiers, shoulb be fixed to 1 for PX4 SITL.
        DeclareLaunchArgument("mav_comp_id", default_value="1"),
        # PX4 SITL UDP bind port. 0 lets the operating system choose a port.
        DeclareLaunchArgument("mavlink_udp_local_port", default_value="0"),
        # MAVROS UDP port receiving MAVLink packets from PX4 SITL. (for example, mavros binds this port to receive telemetry from PX4 SITL)
        DeclareLaunchArgument("mavlink_udp_remote_port", default_value="24540"),
        # Initial position in the local East/North/Up frame, in metres.
        DeclareLaunchArgument("init_x_East_metre", default_value="1.0"),
        DeclareLaunchArgument("init_y_North_metre", default_value="0.0"),
        DeclareLaunchArgument("init_z_Up_metre", default_value="0.0"),
        # Initial vehicle attitude in degrees.
        DeclareLaunchArgument("init_roll_deg", default_value="0.0"),
        DeclareLaunchArgument("init_pitch_deg", default_value="0.0"),
        DeclareLaunchArgument("init_yaw_deg", default_value="0.0"),
        # Position source: 0 uses mocap/local position; 1 uses GPS/global position.
        DeclareLaunchArgument("local_pos_source", default_value="0"),
        # Geographic origin used to convert GPS coordinates to the local frame.
        DeclareLaunchArgument("world_origin_latitude_deg", default_value="39.978861"),
        DeclareLaunchArgument("world_origin_longitude_deg", default_value="116.339803"),
        DeclareLaunchArgument("world_origin_AMSL_alt_metre", default_value="53.0"),
        # IPC endpoint of the FastSwarmSim time coordinator.
        DeclareLaunchArgument("fss_time_coordinator_endpoint", default_value="ipc:///tmp/fss_time_coordinator.ipc"),
        # Start the time coordinator only when use_fss_sim_time is enabled.
        DeclareLaunchArgument("enable_time_coordinator", default_value="true"),
        # Enable or disable the corresponding runtime component.
        DeclareLaunchArgument("enable_mavros", default_value="true"),
        DeclareLaunchArgument("enable_visualizer", default_value="true"),
        DeclareLaunchArgument("enable_rviz", default_value="true"),
        # RViz configuration file used when enable_rviz is true.
        DeclareLaunchArgument("rviz_config", default_value=PathJoinSubstitution([
            FindPackageShare("fss_px4_sim"), "rviz", "single_px4_rotor.rviz"])),

        SetParameter(name="use_fss_sim_time", value=LaunchConfiguration("use_fss_sim_time")),
        SetParameter(name="use_sim_time", value=LaunchConfiguration("use_fss_sim_time")),

        ### Time coordinator (only when use_fss_sim_time is true and enable_time_coordinator is true)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("fss_time"), "launch", "time_coordinator.launch.py"])),
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration("use_fss_sim_time"),
                "' == 'true' and '", LaunchConfiguration("enable_time_coordinator"), "' == 'true'",
            ])),
            launch_arguments={
                "fss_time_coordinator_endpoint": LaunchConfiguration("fss_time_coordinator_endpoint"),
                "publish_clock": "true",
            }.items(),
        ),

        ### Visualizer
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("fss_px4_sim"), "launch", "drone_visualizer_single.launch.py"])),
            condition=IfCondition(LaunchConfiguration("enable_visualizer")),
            launch_arguments={
                "namespace": namespace,
                "use_sim_time": LaunchConfiguration("use_fss_sim_time"),
                "local_pos_source": LaunchConfiguration("local_pos_source"),
                "world_origin_latitude_deg": LaunchConfiguration("world_origin_latitude_deg"),
                "world_origin_longitude_deg": LaunchConfiguration("world_origin_longitude_deg"),
                "world_origin_AMSL_alt_metre": LaunchConfiguration("world_origin_AMSL_alt_metre"),
            }.items(),
        ),

        ### Rviz
        Node(
            package="rviz2", executable="rviz2", name="rviz2", output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            condition=IfCondition(LaunchConfiguration("enable_rviz")),
        ),

        GroupAction(
            scoped=True,
            actions=[
        PushRosNamespace(namespace),

        ### mavros
        Node(
            package="mavros", executable="mavros_node", name="mavros", output="screen",
            parameters=[
                {
                "fcu_url": PythonExpression([
                    "'udp://127.0.0.1:' + str(",
                    LaunchConfiguration("mavlink_udp_remote_port"), ") + '@'"]), # mavros url rules: https://blog.csdn.net/benchuspx/article/details/156449911?spm=1001.2014.3001.5501
                "gcs_url": "udp://@127.0.0.1:14550", # for local QGC which listens on 14550, MAVROS will forward telemetry to it.
                "tgt_system": LaunchConfiguration("mav_sys_id"),
                "tgt_component": LaunchConfiguration("mav_comp_id"),
                "plugin_allowlist": ["setpoint_raw", "local_position", "imu",
                                     "sys_status", "command", "global_position"],
                "plugin_denylist": ["*"],
                },
            ],
            condition=IfCondition(LaunchConfiguration("enable_mavros")),
        ),
        ### PX4 sitl
        Node(
            package="fss_px4_sim",
            executable="mavros_px4_quadrotor_sim_node",
            name="mavros_px4_quadrotor_sim_node",
            output="screen",
            parameters=[
                PathJoinSubstitution([FindPackageShare("fss_px4_sim"), "config", "px4_params.yaml"]),
                {
                "init_x_East_metre": LaunchConfiguration("init_x_East_metre"),
                "init_y_North_metre": LaunchConfiguration("init_y_North_metre"),
                "init_z_Up_metre": LaunchConfiguration("init_z_Up_metre"),
                "init_roll_deg": LaunchConfiguration("init_roll_deg"),
                "init_pitch_deg": LaunchConfiguration("init_pitch_deg"),
                "init_yaw_deg": LaunchConfiguration("init_yaw_deg"),
                "mavlink_udp_local_port": LaunchConfiguration("mavlink_udp_local_port"), # px4 sitl binds to this port
                "mavlink_udp_remote_port": LaunchConfiguration("mavlink_udp_remote_port"), # px4 sitl connect to this port
                "MAV_SYS_ID": LaunchConfiguration("mav_sys_id"),
                "MAV_COMP_ID": LaunchConfiguration("mav_comp_id"),
                "local_pos_source": LaunchConfiguration("local_pos_source"),
                "fss_time_coordinator_endpoint": LaunchConfiguration("fss_time_coordinator_endpoint"),
                "world_origin_latitude_deg": LaunchConfiguration("world_origin_latitude_deg"),
                "world_origin_longitude_deg": LaunchConfiguration("world_origin_longitude_deg"),
                "world_origin_AMSL_alt_metre": LaunchConfiguration("world_origin_AMSL_alt_metre"),
                },
            ],
        ),
            ],
        ),
    ])
