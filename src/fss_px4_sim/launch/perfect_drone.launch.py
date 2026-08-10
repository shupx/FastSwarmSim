from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace, SetParameter
from launch_ros.substitutions import FindPackageShare


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
        DeclareLaunchArgument("enable_time_coordinator", default_value="true"),
        DeclareLaunchArgument("enable_mavros", default_value="true"),
        DeclareLaunchArgument("enable_visualizer", default_value="true"),
        DeclareLaunchArgument("enable_rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=PathJoinSubstitution([
            FindPackageShare("fss_px4_sim"), "rviz", "single_px4_rotor.rviz"])),

        SetParameter(name="use_fss_sim_time", value=LaunchConfiguration("use_fss_sim_time")),
        SetParameter(name="use_sim_time", value=LaunchConfiguration("use_fss_sim_time")),

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

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("fss_px4_sim"), "launch", "drone_visualizer_single.launch.py"])),
            condition=IfCondition(LaunchConfiguration("enable_visualizer")),
            launch_arguments={
                "namespace": namespace,
                "use_sim_time": LaunchConfiguration("use_fss_sim_time"),
            }.items(),
        ),

        Node(
            package="rviz2", executable="rviz2", name="rviz2", output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            condition=IfCondition(LaunchConfiguration("enable_rviz")),
        ),

        GroupAction(
            scoped=True,
            actions=[
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
            condition=IfCondition(LaunchConfiguration("enable_mavros")),
        ),
            ],
        ),
    ])
