from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("hairoom_control")
    moveit_py_config = PathJoinSubstitution([package_share, "config", "moveit_py.yaml"])
    points_file = PathJoinSubstitution([package_share, "data", "points.csv"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="0.0.0.0"),
            DeclareLaunchArgument("port", default_value="8080"),
            DeclareLaunchArgument("planning_group", default_value="fr5_arm"),
            DeclareLaunchArgument("joint_names", default_value="j1,j2,j3,j4,j5,j6"),
            DeclareLaunchArgument("base_degrees", default_value="0,0,0,0,0,0"),
            DeclareLaunchArgument("controllers", default_value=""),
            Node(
                package="hairoom_control",
                executable="wash_server",
                name="hairoom_moveit_py",
                output="screen",
                parameters=[moveit_py_config],
                arguments=[
                    "--host",
                    LaunchConfiguration("host"),
                    "--port",
                    LaunchConfiguration("port"),
                    "--points-file",
                    points_file,
                    "--planning-group",
                    LaunchConfiguration("planning_group"),
                    "--joint-names",
                    LaunchConfiguration("joint_names"),
                    "--base-degrees",
                    LaunchConfiguration("base_degrees"),
                    "--controllers",
                    LaunchConfiguration("controllers"),
                ],
            ),
        ]
    )
