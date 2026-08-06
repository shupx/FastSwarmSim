from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def make_nodes(context):
    count = int(LaunchConfiguration("num_drones").perform(context))
    return [Node(
        package="fss_px4_sim", executable="mavros_px4_quadrotor_sim_node",
        namespace=f"uav{index}", name="mavros_px4_quadrotor_sim_node", output="screen",
        parameters=[
            PathJoinSubstitution([FindPackageShare("fss_px4_sim"), "config", "px4_rotor_sim_ros2.yaml"]),
                PathJoinSubstitution([FindPackageShare("fss_px4_sim"), "config", "px4_rotor_sim_px4_params_ros2.yaml"]),
            PathJoinSubstitution([FindPackageShare("fss_px4_sim"), "config", "mavros_px4_config_ros2.yaml"]),
            {
            "init_x_East_metre": float(index), "init_y_North_metre": 0.0,
            "init_z_Up_metre": 0.0, "fss_time_coordinator_endpoint": LaunchConfiguration("fss_time_coordinator_endpoint"),
            "use_fss_sim_time": LaunchConfiguration("use_fss_sim_time"),
            "use_sim_time": LaunchConfiguration("use_fss_sim_time"),
            },
        ],
    ) for index in range(1, count + 1)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("num_drones", default_value="5"),
        DeclareLaunchArgument("use_fss_sim_time", default_value="true"),
        DeclareLaunchArgument("fss_time_coordinator_endpoint", default_value="ipc:///tmp/fss_time_coordinator.ipc"),
        SetParameter(name="use_fss_sim_time", value=LaunchConfiguration("use_fss_sim_time")),
        SetParameter(name="use_sim_time", value=LaunchConfiguration("use_fss_sim_time")),
        OpaqueFunction(function=make_nodes),
    ])
