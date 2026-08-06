from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetParameter


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="uav1"),
        DeclareLaunchArgument("use_fss_sim_time", default_value="true"),
        DeclareLaunchArgument("init_x", default_value="0.0"),
        DeclareLaunchArgument("init_y", default_value="0.0"),
        DeclareLaunchArgument("init_z", default_value="1.0"),
        DeclareLaunchArgument("init_yaw", default_value="0.0"),
        DeclareLaunchArgument("fss_time_coordinator_endpoint", default_value="ipc:///tmp/fss_time_coordinator.ipc"),
        SetParameter(name="use_fss_sim_time", value=LaunchConfiguration("use_fss_sim_time")),
        SetParameter(name="use_sim_time", value=LaunchConfiguration("use_fss_sim_time")),
        PushRosNamespace(namespace),
        Node(
            package="fss_px4_sim",
            executable="perfect_mavros_quadrotor_sim_node",
            name="perfect_mavros_quadrotor_sim",
            output="screen",
            parameters=[{
                "init_x": LaunchConfiguration("init_x"),
                "init_y": LaunchConfiguration("init_y"),
                "init_z": LaunchConfiguration("init_z"),
                "init_yaw": LaunchConfiguration("init_yaw"),
                "fss_time_coordinator_endpoint": LaunchConfiguration("fss_time_coordinator_endpoint"),
            }],
        ),
    ])
