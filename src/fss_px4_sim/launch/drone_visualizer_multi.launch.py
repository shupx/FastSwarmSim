from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def make_nodes(context):
    count = int(LaunchConfiguration("num_drones").perform(context))
    urdf = get_package_share_directory("fss_px4_sim") + "/model/iris.urdf"
    nodes = []
    for index in range(1, count + 1):
        namespace = f"uav{index}"
        nodes.extend([
            Node(
                package="robot_state_publisher", executable="robot_state_publisher",
                namespace=namespace, name="robot_state_publisher", output="screen",
                parameters=[{
                    "robot_description": ParameterValue(Command(["cat ", urdf]), value_type=str),
                    "frame_prefix": namespace + "/",
                }],
            ),
            Node(
                package="fss_px4_sim", executable="px4_rotor_visualizer_node",
                namespace=namespace, name="px4_rotor_visualizer_node", output="screen",
                parameters=[{"base_link_tf_prefix": namespace}],
            ),
        ])
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("num_drones", default_value="5"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        SetParameter(name="use_sim_time", value=LaunchConfiguration("use_sim_time")),
        OpaqueFunction(function=make_nodes),
    ])
