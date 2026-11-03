# HAIROOM BRINGUP — official driver path (2026-07-21).
#
# Primary launch file for the real fairino_hardware/FairinoHardwareInterface
# ros2_control driver: robot_state_publisher, move_group, RViz, the real
# controller_manager (ros2_control_node) with the real driver, controller
# spawners, and the head+sink planning scene — the same scene the bridge uses.
#
# demo.launch.py is untouched and remains the bridge's launch file (it
# deliberately omits ros2_control_node/spawners — see the comment at the top
# of that file). hairoom_action_bridge.py is untouched and remains the
# fallback control path; nothing here starts or depends on it.
#
# Promoted from demo_real_driver_TEST.launch.py once the driver connection
# was verified live (2026-07-21: SDK handshake succeeds in ~3s, no crash).
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg
import os


# Hairoom's own scripts live in this repo, not in the ROS workspace. Point at
# them directly so there is exactly one copy of hairoom_scene.py /
# hairoom_head_tf.py -- editing a constant like HEAD_BASE_POSE must not have to
# be mirrored into ~/hairoom_ws/src to take effect.
HAIROOM_SCRIPTS = os.environ.get(
    "HAIROOM_SCRIPTS",
    os.path.expanduser("~/Desktop/Hairoom/v2/scripts"))


def _script(name):
    return os.path.join(HAIROOM_SCRIPTS, name)



def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "fairino5_v6_robot", package_name="fairino5_v6_moveit2_config"
    ).to_moveit_configs()
    launch_package_path = moveit_config.package_path

    ld = LaunchDescription()
    ld.add_action(
        DeclareBooleanLaunchArg(
            "db",
            default_value=False,
            description="By default, we do not start a database (it can be large)",
        )
    )
    ld.add_action(
        DeclareBooleanLaunchArg(
            "debug",
            default_value=False,
            description="By default, we are not in debug mode",
        )
    )
    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))
    ld.add_action(DeclareBooleanLaunchArg("use_scene", default_value=True))
    ld.add_action(DeclareBooleanLaunchArg("use_head_tf", default_value=True))

    virtual_joints_launch = (
        launch_package_path / "launch/static_virtual_joint_tfs.launch.py"
    )
    if virtual_joints_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch)),
            )
        )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/rsp.launch.py")
            ),
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/move_group.launch.py")
            ),
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/moveit_rviz.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/warehouse_db.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("db")),
        )
    )

    # Real driver: controller_manager loads fairino_hardware/FairinoHardwareInterface
    # per fairino5_v6_robot.ros2_control.xacro.
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                str(moveit_config.package_path / "config/ros2_controllers.yaml"),
            ],
            output="screen",
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/spawn_controllers.launch.py")
            ),
        )
    )

    # Head + sink planning scene — same script the bridge uses
    # (scripts/hairoom_scene.py). One-shot: waits on
    # /apply_planning_scene internally (up to 10s), applies the scene, exits.
    ld.add_action(
        ExecuteProcess(
            cmd=["python3", _script("hairoom_scene.py")],
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_scene")),
        )
    )

    # head_link TF frame (base_link -> head_link), matching the head object's
    # pose from hairoom_scene.py exactly (scripts/hairoom_head_tf.py
    # imports the pose directly, can't drift out of sync). Long-running, not
    # one-shot: static-transform durability only serves subscribers connected
    # while the publisher is alive, so this stays up for the life of the
    # launch, same as ros2_control_node/move_group/rviz2.
    ld.add_action(
        ExecuteProcess(
            cmd=["python3", _script("hairoom_head_tf.py")],
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_head_tf")),
        )
    )
    # NOTE 2026-07-21: tcp_tool is now a real URDF link (hairoom_tool.xacro,
    # included from fairino5_v6_robot.urdf.xacro) — robot_state_publisher
    # publishes wrist3_link -> tool_link -> tcp_tool natively from the
    # kinematic tree. The standalone hairoom_tcp_tool_tf.py broadcaster used
    # before this is retired (would now double-publish/conflict with it).

    return ld
