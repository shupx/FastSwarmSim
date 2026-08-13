from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    default_urdf = get_package_share_directory("fss_px4_sim") + "/model/iris.urdf"
    return LaunchDescription([
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"), # force rcl log color
        GroupAction(
            scoped=True, 
            actions=[
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="ROS namespace for this vehicle; empty uses the root namespace.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                choices=["true", "false"],
                description="Use ROS simulation time.",
            ),
            DeclareLaunchArgument(
                "visualize_max_freq",
                default_value="20.0",
                description="Maximum visualization update frequency in Hz.",
            ),
            DeclareLaunchArgument(
                "enable_history_path",
                default_value="true",
                choices=["true", "false"],
                description="Publish the vehicle history path.",
            ),
            DeclareLaunchArgument(
                "visualize_path_time",
                default_value="30.0",
                description="History path duration in seconds.",
            ),
            DeclareLaunchArgument(
                "visualize_tf_frame",
                default_value="map",
                description="TF frame used for visualization markers.",
            ),
            DeclareLaunchArgument(
                "visualize_marker_name",
                default_value=PythonExpression([
                    "'", LaunchConfiguration("namespace"),
                    "' if '", LaunchConfiguration("namespace"), "' else 'uav'",
                ]), description="Visualization marker name; defaults to the namespace or uav.",
            ),
            DeclareLaunchArgument(
                "base_link_name",
                default_value="base_link",
                description="Base-link frame name in the vehicle URDF.",
            ),
            DeclareLaunchArgument(
                "base_link_tf_prefix",
                default_value=LaunchConfiguration("namespace"),
                description="Prefix for the base-link TF frame; defaults to the namespace.",
            ),
            DeclareLaunchArgument(
                "rotor_0_joint_name",
                default_value="rotor_0_joint",
                description="URDF joint name for rotor 0.",
            ),
            DeclareLaunchArgument(
                "rotor_1_joint_name",
                default_value="rotor_1_joint",
                description="URDF joint name for rotor 1.",
            ),
            DeclareLaunchArgument(
                "rotor_2_joint_name",
                default_value="rotor_2_joint",
                description="URDF joint name for rotor 2.",
            ),
            DeclareLaunchArgument(
                "rotor_3_joint_name",
                default_value="rotor_3_joint",
                description="URDF joint name for rotor 3.",
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
                "urdf_model_path",
                default_value=default_urdf,
                description="Path to the vehicle URDF model.",
            ),

            SetParameter(name="use_sim_time", value=LaunchConfiguration("use_sim_time")),
            GroupAction(
                scoped=True, # specify the PushRosNamespace scope
                actions=[
                PushRosNamespace(LaunchConfiguration("namespace")),
                Node(
                    package="robot_state_publisher", executable="robot_state_publisher",
                    name="robot_state_publisher", output="screen",
                    parameters=[{
                        "robot_description": ParameterValue(
                            Command(["cat ", LaunchConfiguration("urdf_model_path")]), value_type=str),
                        # The visualizer emits the namespaced base-link transform.
                        "frame_prefix": PythonExpression([
                            "'", LaunchConfiguration("base_link_tf_prefix"),
                            "' + '/' if '", LaunchConfiguration("base_link_tf_prefix"), "' else ''"]),
                    }],
                    namespace="visualizer",
                ),
                Node(
                    package="fss_px4_sim", executable="px4_rotor_visualizer_node",
                    name="px4_rotor_visualizer_node", output="screen",
                    parameters=[{
                        "visualize_max_freq": LaunchConfiguration("visualize_max_freq"),
                        "enable_history_path": LaunchConfiguration("enable_history_path"),
                        "visualize_path_time": LaunchConfiguration("visualize_path_time"),
                        "visualize_tf_frame": LaunchConfiguration("visualize_tf_frame"),
                        "visualize_marker_name": LaunchConfiguration("visualize_marker_name"),
                        "base_link_name": LaunchConfiguration("base_link_name"),
                        "base_link_tf_prefix": LaunchConfiguration("base_link_tf_prefix"),
                        "rotor_0_joint_name": LaunchConfiguration("rotor_0_joint_name"),
                        "rotor_1_joint_name": LaunchConfiguration("rotor_1_joint_name"),
                        "rotor_2_joint_name": LaunchConfiguration("rotor_2_joint_name"),
                        "rotor_3_joint_name": LaunchConfiguration("rotor_3_joint_name"),
                        "local_pos_source": LaunchConfiguration("local_pos_source"),
                        "world_origin_latitude_deg": LaunchConfiguration("world_origin_latitude_deg"), # python only has float (float64 = double precision), so it is safe to use float for latitude/longitude
                        "world_origin_longitude_deg": LaunchConfiguration("world_origin_longitude_deg"),
                        "world_origin_AMSL_alt_metre": LaunchConfiguration("world_origin_AMSL_alt_metre"),
                    }],
                ),
                ],
            ),
        ]),
    ])
