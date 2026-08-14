from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
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
        mavros_type = LaunchConfiguration("mavros_type").perform(context)
        if mavros_type != "disabled" and transport != "udp":
            get_logger("px4_rotor_sim_single").warning(
                "\033[33m"
                f"mavros_type={mavros_type} requested a MAVROS node, but "
                f"px4.mavlink_transport={transport}; no MAVROS node will be started. "
                "px4.mavlink_transport must be udp"
                "\033[0m")
        return []

    return LaunchDescription([
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"), # force rcl log color
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="ROS namespace for this vehicle; empty uses the root namespace.",
        ),
        DeclareLaunchArgument(
            "use_fss_sim_time",
            default_value="true",
            choices=["true", "false"],
            description="Use the FastSwarmSim coordinated simulation clock.",
        ),
        DeclareLaunchArgument(
            "mav_sys_id",
            default_value="1",
            description="MAVLink system identifier (1-255).",
        ),
        DeclareLaunchArgument(
            "mav_comp_id",
            default_value="1",
            description="MAVLink component identifier; PX4 SITL normally uses 1.",
        ),
        DeclareLaunchArgument(
            "px4.mavlink_transport",
            default_value="direct_ros",
            choices=["udp", "direct_ros", "empty"],
            description="PX4 MAVLink transport.",
        ),
        DeclareLaunchArgument(
            "px4.mavlink_udp_local_port",
            default_value="0",
            description="PX4 SITL UDP bind port when transport is udp; 0 selects an available port.",
        ),
        DeclareLaunchArgument(
            "px4.mavlink_udp_remote_port",
            default_value="24540",
            description="MAVROS UDP receive port when transport is udp.",
        ),
        DeclareLaunchArgument(
            "init_x_East_metre",
            default_value="1.0",
            description="Initial east position in metres.",
        ),
        DeclareLaunchArgument(
            "init_y_North_metre",
            default_value="0.0",
            description="Initial north position in metres.",
        ),
        DeclareLaunchArgument(
            "init_z_Up_metre",
            default_value="0.0",
            description="Initial up position in metres.",
        ),
        DeclareLaunchArgument(
            "init_roll_deg",
            default_value="0.0",
            description="Initial roll in degrees.",
        ),
        DeclareLaunchArgument(
            "init_pitch_deg",
            default_value="0.0",
            description="Initial pitch in degrees.",
        ),
        DeclareLaunchArgument(
            "init_yaw_deg",
            default_value="0.0",
            description="Initial yaw in degrees.",
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
        DeclareLaunchArgument(
            "enable_time_coordinator",
            default_value="true",
            choices=["true", "false"],
            description="Start the time coordinator when FastSwarmSim simulation time is enabled.",
        ),
        DeclareLaunchArgument(
            "mavros_type",
            default_value="disabled",
            choices=["disabled", "official", "lite"],
            description="External MAVROS bridge: disabled starts none, official starts MAVROS, lite starts MAVROS Lite.",
        ),
        DeclareLaunchArgument(
            "enable_visualizer",
            default_value="true",
            choices=["true", "false"],
            description="Start the drone visualizer.",
        ),
        DeclareLaunchArgument(
            "vis.visualize_max_freq",
            default_value="20.0",
            description="Maximum visualization update frequency in Hz.",
        ),
        DeclareLaunchArgument(
            "vis.enable_history_path",
            default_value="true",
            choices=["true", "false"],
            description="Publish the vehicle history path.",
        ),
        DeclareLaunchArgument(
            "vis.visualize_path_time",
            default_value="30.0",
            description="History path duration in seconds.",
        ),
        DeclareLaunchArgument(
            "vis.color",
            default_value="red",
            choices=["red", "green", "blue", "grey"],
            description="Vehicle color used to select the Iris URDF model.",
        ),
        DeclareLaunchArgument(
            "enable_rviz",
            default_value="true",
            choices=["true", "false"],
            description="Start RViz.",
        ),
        DeclareLaunchArgument("rviz_config", default_value=PathJoinSubstitution([
            FindPackageShare("fss_px4_sim"), "rviz", "single_px4_rotor.rviz"]), description="RViz configuration file used when RViz is enabled."),

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
                        "visualize_max_freq": LaunchConfiguration("vis.visualize_max_freq"),
                        "enable_history_path": LaunchConfiguration("vis.enable_history_path"),
                        "visualize_path_time": LaunchConfiguration("vis.visualize_path_time"),
                        "vis.color": LaunchConfiguration("vis.color"),
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
                "'", LaunchConfiguration("mavros_type"), "' == 'lite' and '",
                LaunchConfiguration("px4.mavlink_transport"), "' == 'udp'",
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
                "'", LaunchConfiguration("mavros_type"), "' == 'official' and '",
                LaunchConfiguration("px4.mavlink_transport"), "' == 'udp'",
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
                "world_origin_latitude_deg": LaunchConfiguration("world_origin_latitude_deg"), # python only has float (float64 = double precision), so it is safe to use float for latitude/longitude
                "world_origin_longitude_deg": LaunchConfiguration("world_origin_longitude_deg"),
                "world_origin_AMSL_alt_metre": LaunchConfiguration("world_origin_AMSL_alt_metre"),
                },
            ],
        ),
            ],
        ),
    ])
