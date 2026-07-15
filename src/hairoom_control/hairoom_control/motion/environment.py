from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject

from hairoom_control.motion.stl_mesh import load_stl_as_mesh


@dataclass(frozen=True)
class CollisionMeshSpec:
    object_id: str
    path: Path
    frame_id: str = "world"
    pose: Pose | None = None


def make_identity_pose() -> Pose:
    pose = Pose()
    pose.orientation.w = 1.0
    return pose


class PlanningSceneEnvironment:
    def __init__(self, moveit):
        self._moveit = moveit

    def apply_meshes(self, meshes: list[CollisionMeshSpec]) -> int:
        if not meshes:
            return 0

        planning_scene_monitor = self._moveit.get_planning_scene_monitor()
        with planning_scene_monitor.read_write() as scene:
            for mesh_spec in meshes:
                collision_object = CollisionObject()
                collision_object.header.frame_id = mesh_spec.frame_id
                collision_object.id = mesh_spec.object_id
                collision_object.meshes.append(load_stl_as_mesh(mesh_spec.path))
                collision_object.mesh_poses.append(mesh_spec.pose or make_identity_pose())
                collision_object.operation = CollisionObject.ADD
                scene.apply_collision_object(collision_object)
            scene.current_state.update()

        return len(meshes)
