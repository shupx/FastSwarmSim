'''
This file composes multiple PX4 rotor sim components into a single multi-threaded executor container for each vehicle.

It will decrease the launch time and memory usage when launching 100+ vehicles, 
but it will also make the real RTF lower because nodes share a single executor thread pool instead of each node having its own executor thread.
'''

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import ComposableNodeContainer, Node, SetParameter
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"), # force rcl log color
        DeclareLaunchArgument(
            "num_drones",
            default_value="5",
            description="Number of vehicles to launch.",
        ),
        DeclareLaunchArgument(
            "drones_per_row",
            default_value="10",
            description="Number of vehicles in each formation row; must be at least 1.",
        ),
        DeclareLaunchArgument(
            "executor_threads",
            default_value="0",
            description="Number of component container executor threads; 0 keeps the container default, which uses the detected CPU core count.",
        ),
        DeclareLaunchArgument(
            "use_fss_sim_time",
            default_value="true",
            choices=["true", "false"],
            description="Use the FastSwarmSim coordinated simulation clock.",
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
            "px4.mavlink_transport",
            default_value="direct_ros",
            choices=["udp", "direct_ros", "empty"],
            description="PX4 MAVLink transport.",
        ),
        DeclareLaunchArgument(
            "enable_visualizer",
            default_value="true",
            choices=["true", "false"],
            description="Start vehicle visualizers.",
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
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("fss_px4_sim"), "rviz", "multi_px4_rotor.rviz"
            ]),
            description="RViz configuration file used when RViz is enabled.",
        ),
        SetParameter(
            name="use_fss_sim_time",
            value=LaunchConfiguration("use_fss_sim_time"),
        ),
        SetParameter(
            name="use_sim_time",
            value=LaunchConfiguration("use_fss_sim_time"),
        ),

        ## Time coordinator (only when use_fss_sim_time is true and enable_time_coordinator is true)
        # Recommended: scope each included launch to isolate its parameters and actions and prevent leakage.
        GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(PathJoinSubstitution([
                        FindPackageShare("fss_time"), "launch", "time_coordinator.launch.py"
                    ])),
                    condition=IfCondition(PythonExpression([
                        "'", LaunchConfiguration("use_fss_sim_time"),
                        "' == 'true' and '", LaunchConfiguration("enable_time_coordinator"),
                        "' == 'true'",
                    ])),
                    launch_arguments={"publish_clock": "true"}.items(),
                ),
            ],
        ),

        ## Rviz2
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            condition=IfCondition(LaunchConfiguration("enable_rviz")),
        ),

        ## Make drones
        OpaqueFunction(function=make_drones),
    ])


def make_drones(context):
    count = int(LaunchConfiguration("num_drones").perform(context))
    drones_per_row = int(LaunchConfiguration("drones_per_row").perform(context))
    executor_threads = int(LaunchConfiguration("executor_threads").perform(context))
    if drones_per_row < 1:
        raise ValueError("drones_per_row must be at least 1")
    if executor_threads < 0:
        raise ValueError("executor_threads must be non-negative")

    mavros_type = LaunchConfiguration("mavros_type").perform(context)
    mavlink_transport = LaunchConfiguration("px4.mavlink_transport").perform(context)
    enable_visualizer = LaunchConfiguration("enable_visualizer").perform(context) == "true"
    visualize_max_freq = float(LaunchConfiguration("vis.visualize_max_freq").perform(context))
    enable_history_path = LaunchConfiguration("vis.enable_history_path").perform(context) == "true"
    visualize_path_time = float(LaunchConfiguration("vis.visualize_path_time").perform(context))
    color = LaunchConfiguration("vis.color").perform(context)

    if mavros_type != "disabled" and mavlink_transport != "udp":
        get_logger("px4_rotor_sim_components_multi").warning(
            "\033[33m"
            f"mavros_type={mavros_type} requested a MAVROS node, but "
            f"px4.mavlink_transport={mavlink_transport}; no MAVROS node will be started. "
            "px4.mavlink_transport must be udp"
            "\033[0m"
        )

    package_share = Path(get_package_share_directory("fss_px4_sim"))
    px4_parameters = str(package_share / "config" / "px4_params.yaml")
    mavros_parameters = str(package_share / "config" / "mavros_official_plugins.yaml")
    robot_description = (package_share / "model" / f"iris_{color}.urdf").read_text()

    drone_containers = []
    external_mavros_nodes = []
    container_parameters = []
    if executor_threads > 0:
        container_parameters.append({"thread_num": executor_threads})
    for index in range(1, count + 1):
        namespace = f"uav{index}"
        mav_sys_id = index
        mav_comp_id = 1
        mavlink_remote_port = 24540 + index - 1
        components = []

        ### PX4 sitl
        components.append(
            ComposableNode(
                package="fss_px4_sim",
                plugin="fss_px4_sim::MavrosPx4QuadrotorSim",
                name="px4_rotor_sim_node",
                namespace=namespace,
                extra_arguments=[{"use_intra_process_comms": True}],
                parameters=[
                    px4_parameters,
                    {
                        "init_x_East_metre": float((index - 1) % drones_per_row),
                        "init_y_North_metre": float((index - 1) // drones_per_row),
                        "init_z_Up_metre": 0.0,
                        "init_roll_deg": 0.0,
                        "init_pitch_deg": 0.0,
                        "init_yaw_deg": 0.0,
                        "px4.mavlink_udp_local_port": 0, # only used when transport is udp; 0 lets the OS choose a port
                        "px4.mavlink_udp_remote_port": mavlink_remote_port, # MAVROS receive port for UDP transport
                        "px4.mavlink_transport": mavlink_transport,
                        "MAV_SYS_ID": mav_sys_id,
                        "MAV_COMP_ID": mav_comp_id,
                        "local_pos_source": 0,
                        "world_origin_latitude_deg": 39.978861, # python only has float (float64 = double precision), so it is safe to use float for latitude/longitude
                        "world_origin_longitude_deg": 116.339803,
                        "world_origin_AMSL_alt_metre": 53.0,
                    },
                ],
            )
        )

        ### MAVROS Lite node over UDP
        if mavros_type == "lite" and mavlink_transport == "udp":
            external_mavros_nodes.append(
                Node(
                    package="fss_px4_sim",
                    executable="mavros_lite_node",
                    name="mavros_lite_node",
                    namespace=f"{namespace}/mavros",
                    output="screen",
                    parameters=[{
                        "udp.bind_port": mavlink_remote_port,
                        "target_system": mav_sys_id,
                        "target_component": mav_comp_id,
                    }],
                )
            )
        ### Official MAVROS node over UDP
        elif mavros_type == "official" and mavlink_transport == "udp":
            external_mavros_nodes.append(
                Node(
                    package="mavros",
                    executable="mavros_node",
                    name="mavros",
                    namespace=f"{namespace}/mavros",
                    output="screen",
                    parameters=[
                        mavros_parameters,
                        {
                            "fcu_url": f"udp://:{mavlink_remote_port}@",
                            "gcs_url": "",
                            "tgt_system": mav_sys_id,
                            "tgt_component": mav_comp_id,
                            "fcu_protocol": "v2.0",
                        },
                    ],
                )
            )

        ### Visualizer
        if enable_visualizer:
            components.extend([
                ### Robot description publisher
                ComposableNode(
                    package="robot_state_publisher",
                    plugin="robot_state_publisher::RobotStatePublisher",
                    name="robot_state_publisher",
                    namespace=f"{namespace}/visualizer",
                    parameters=[{
                        "robot_description": robot_description,
                        # The visualizer emits the namespaced base-link transform.
                        "frame_prefix": f"{namespace}/",
                    }],
                ),
                ### Drone visualizer
                ComposableNode(
                    package="fss_px4_sim",
                    plugin="fss_px4_sim::DroneVisualizer",
                    name="px4_rotor_visualizer_node",
                    namespace=namespace,
                    extra_arguments=[{"use_intra_process_comms": True}],
                    parameters=[{
                        "visualize_max_freq": visualize_max_freq,
                        "enable_history_path": enable_history_path,
                        "visualize_path_time": visualize_path_time,
                        "visualize_tf_frame": "map",
                        "visualize_marker_name": namespace,
                        "base_link_name": "base_link",
                        "base_link_tf_prefix": namespace,
                        "rotor_0_joint_name": "rotor_0_joint",
                        "rotor_1_joint_name": "rotor_1_joint",
                        "rotor_2_joint_name": "rotor_2_joint",
                        "rotor_3_joint_name": "rotor_3_joint",
                        "local_pos_source": 0,
                        "world_origin_latitude_deg": 39.978861, # python only has float (float64 = double precision), so it is safe to use float for latitude/longitude
                        "world_origin_longitude_deg": 116.339803,
                        "world_origin_AMSL_alt_metre": 53.0,
                    }],
                ),
            ])

        ### Composable node container for this vehicle
        drone_containers.append(
            # ComposableNodeContainer can decrease process number and thus reduce memory usage and launch time.
            # But all nodes will share a single executor thread pool, 
            # which may reduce performance at run time.
            ComposableNodeContainer(
                package="rclcpp_components",
                executable="component_container_mt",
                name="px4_rotor_sim_container",
                namespace=namespace,
                output="screen",
                parameters=container_parameters,
                composable_node_descriptions=components,
            )
        )

    return [*drone_containers, *external_mavros_nodes]
