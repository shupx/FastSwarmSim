from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")

    def validate_mavlink_configuration(context):
        transport = LaunchConfiguration("px4.mavlink_transport").perform(context)
        mavros_node_type = LaunchConfiguration("mavros_node_type").perform(context)
        enable_mavros_node = (
            LaunchConfiguration("enable_mavros_node").perform(context).lower() == "true")
        if transport not in ("udp", "direct_ros", "empty"):
            raise ValueError("px4.mavlink_transport must be udp, direct_ros, or empty")
        if mavros_node_type not in ("official", "lite"):
            raise ValueError("mavros_node_type must be official or lite")
        if enable_mavros_node and transport == "direct_ros":
            raise ValueError(
                "enable_mavros_node=true conflicts with "
                "px4.mavlink_transport=direct_ros since direct_ros already uses an embedded MAVROS Lite node")
        if enable_mavros_node and transport == "empty":
            get_logger("px4_rotor_sim_single").warning(
                "enable_mavros_node=true requested an external MAVROS node, but "
                f"px4.mavlink_transport={transport}; no external MAVROS node will be started")
        return []

    return LaunchDescription([
        # ROS namespace for this vehicle. Empty means use the root namespace.
        DeclareLaunchArgument("namespace", default_value=""),
        # Use the FastSwarmSim coordinated simulation clock when true. If ture, run 'ros2 launch fss_time time_coordinator.launch.py ' first.
        DeclareLaunchArgument("use_fss_sim_time", default_value="true"),
        # MAVLink system identifiers; valid values are 1-255.
        DeclareLaunchArgument("mav_sys_id", default_value="1"),
        # MAVLink component identifiers, shoulb be fixed to 1 for PX4 SITL.
        DeclareLaunchArgument("mav_comp_id", default_value="1"),
        # PX4 MAVLink transport: udp, direct_ros, or empty.
        DeclareLaunchArgument("px4.mavlink_transport", default_value="direct_ros"),
        # PX4 SITL UDP bind port (only when mavlink_transport="udp"). 
        # 0 lets the operating system choose a port.
        DeclareLaunchArgument("px4.mavlink_udp_local_port", default_value="0"),
        # MAVROS UDP port receiving MAVLink packets from PX4 SITL (only when mavlink_transport="udp"). 
        # For example, mavros binds this port to receive telemetry from PX4 SITL
        DeclareLaunchArgument("px4.mavlink_udp_remote_port", default_value="24540"),
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
        DeclareLaunchArgument("enable_mavros_node", default_value="false"),
        # External MAVROS node implementation: official or lite.
        DeclareLaunchArgument("mavros_node_type", default_value="lite"),
        DeclareLaunchArgument("enable_visualizer", default_value="true"),
        DeclareLaunchArgument("enable_rviz", default_value="true"),
        # RViz configuration file used when enable_rviz is true.
        DeclareLaunchArgument("rviz_config", default_value=PathJoinSubstitution([
            FindPackageShare("fss_px4_sim"), "rviz", "single_px4_rotor.rviz"])),

        SetParameter(name="use_fss_sim_time", value=LaunchConfiguration("use_fss_sim_time")),
        SetParameter(name="use_sim_time", value=LaunchConfiguration("use_fss_sim_time")),
        OpaqueFunction(function=validate_mavlink_configuration),

        ### Time coordinator (only when use_fss_sim_time is true and enable_time_coordinator is true)
        # Recommended: scope each included launch to isolate its parameters and actions and prevent leakage.
        GroupAction(
            scoped=True,
            actions=[
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
            ],
        ),

        ### Visualizer
        # Recommended: scope each included launch to isolate its parameters and actions and prevent leakage.
        GroupAction(
            scoped=True,
            actions=[
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
            ],
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

        ### MAVROS Lite node over UDP
        Node(
            package="fss_px4_sim", executable="mavros_lite_node", namespace="mavros", output="screen",
            parameters=[
                {
                "udp.bind_port": LaunchConfiguration("px4.mavlink_udp_remote_port"),
                "target_system": LaunchConfiguration("mav_sys_id"),
                "target_component": LaunchConfiguration("mav_comp_id"),
                },
            ],
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration("enable_mavros_node"), "' == 'true' and '",
                LaunchConfiguration("px4.mavlink_transport"), "' == 'udp' and '",
                LaunchConfiguration("mavros_node_type"), "' == 'lite'",
            ])),
        ),
        ### Official MAVROS node over UDP
        Node(
            package="mavros", executable="mavros_node", namespace="mavros", output="screen",
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare("fss_px4_sim"), "config", "mavros_official_plugins.yaml"]),
                {
                    "fcu_url": PythonExpression([
                        "'udp://:' + '", LaunchConfiguration("px4.mavlink_udp_remote_port"), "' + '@'",
                    ]),
                    "gcs_url": "",
                    "tgt_system": LaunchConfiguration("mav_sys_id"),
                    "tgt_component": LaunchConfiguration("mav_comp_id"),
                    "fcu_protocol": "v2.0",
                },
            ],
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration("enable_mavros_node"), "' == 'true' and '",
                LaunchConfiguration("px4.mavlink_transport"), "' == 'udp' and '",
                LaunchConfiguration("mavros_node_type"), "' == 'official'",
            ])),
        ),
        ### PX4 sitl
        Node(
            package="fss_px4_sim",
            executable="px4_rotor_sim_node",
            name="px4_rotor_sim_node",
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
                "px4.mavlink_udp_local_port": LaunchConfiguration("px4.mavlink_udp_local_port"),
                "px4.mavlink_udp_remote_port": LaunchConfiguration("px4.mavlink_udp_remote_port"),
                "px4.mavlink_transport": LaunchConfiguration("px4.mavlink_transport"),
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
