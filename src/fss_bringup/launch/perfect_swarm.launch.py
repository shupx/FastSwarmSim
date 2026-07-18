from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def make_drones(context):
    num_drones = int(LaunchConfiguration("num_drones").perform(context))
    return [
        Node(
            package="fss_px4_sim",
            executable="perfect_mavros_quadrotor_sim_node",
            namespace=f"uav{index}",
            name="perfect_mavros_quadrotor_sim",
            output="screen",
            parameters=[{
                "init_x": float(index - 1),
                "init_y": 0.0,
                "init_z": 1.0,
                "init_yaw": 0.0,
            }],
        )
        for index in range(1, num_drones + 1)
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("num_drones", default_value="3"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        SetParameter(name="use_sim_time", value=LaunchConfiguration("use_sim_time")),
        OpaqueFunction(function=make_drones),
    ])
