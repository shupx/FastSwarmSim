from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def make_nodes(context):
    count = int(LaunchConfiguration("num_drones").perform(context))
    nodes = []
    for index in range(1, count + 1):
        namespace = f"uav{index}"
        bridge_port = 14540 + 2 * (index - 1)
        mavros_port = bridge_port + 1
        # FSS_SIM_MODIFICATION: each vehicle gets one unmodified official
        # MAVROS runtime and a namespace-local PX4 MAVLink UDP endpoint.
        nodes.append(Node(
            package="mavros", executable="mavros_node", namespace=namespace,
            name="mavros", output="screen",
            parameters=[{
                "fcu_url": f"udp://127.0.0.1:{mavros_port}@127.0.0.1:{bridge_port}",
                "plugin_allowlist": ["setpoint_raw", "local_position", "imu",
                                     "sys_status", "command", "global_position"],
                "plugin_denylist": ["*"],
                # FSS_SIM_MODIFICATION: no COMMAND_ACK stream exists in the
                # copied ROS 1 PX4 simulator MAVLink stream set.
                "fss_simulate_no_command_ack": True,
                "use_sim_time": LaunchConfiguration("use_fss_sim_time"),
            }],
        ))
        nodes.append(Node(
            package="fss_px4_sim", executable="mavros_px4_quadrotor_sim_node",
            namespace=namespace, name="mavros_px4_quadrotor_sim_node", output="screen",
            parameters=[
                PathJoinSubstitution([FindPackageShare("fss_px4_sim"), "config", "px4_rotor_sim_ros2.yaml"]),
                PathJoinSubstitution([FindPackageShare("fss_px4_sim"), "config", "px4_rotor_sim_px4_params_ros2.yaml"]),
                PathJoinSubstitution([FindPackageShare("fss_px4_sim"), "config", "mavros_px4_config_ros2.yaml"]),
                {
                    "init_x_East_metre": float(index), "init_y_North_metre": 0.0,
                    "init_z_Up_metre": 0.0,
                    "mavlink_udp_local_port": bridge_port,
                    "mavlink_udp_remote_port": mavros_port,
                    "fss_time_coordinator_endpoint": LaunchConfiguration("fss_time_coordinator_endpoint"),
                    "use_fss_sim_time": LaunchConfiguration("use_fss_sim_time"),
                    "use_sim_time": LaunchConfiguration("use_fss_sim_time"),
                },
            ],
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("num_drones", default_value="5"),
        DeclareLaunchArgument("use_fss_sim_time", default_value="true"),
        DeclareLaunchArgument("fss_time_coordinator_endpoint", default_value="ipc:///tmp/fss_time_coordinator.ipc"),
        SetParameter(name="use_fss_sim_time", value=LaunchConfiguration("use_fss_sim_time")),
        SetParameter(name="use_sim_time", value=LaunchConfiguration("use_fss_sim_time")),
        OpaqueFunction(function=make_nodes),
    ])
