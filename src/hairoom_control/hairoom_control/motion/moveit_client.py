from __future__ import annotations

import threading
from dataclasses import dataclass

from hairoom_control.motion.environment import CollisionMeshSpec, PlanningSceneEnvironment
from hairoom_control.motion.points import JointWaypoint


@dataclass(frozen=True)
class MotionExecutionResult:
    executed_waypoints: list[str]


class MoveItExecutionError(RuntimeError):
    pass


class MoveIt2Client:
    def __init__(
        self,
        *,
        node_name: str,
        planning_group: str,
        controllers: list[str],
        environment_meshes: list[CollisionMeshSpec],
    ):
        from moveit.core.robot_state import RobotState
        from moveit.planning import MoveItPy

        self._robot_state_class = RobotState
        self._moveit = MoveItPy(node_name=node_name)
        self._planning_group = planning_group
        self._planning_component = self._moveit.get_planning_component(planning_group)
        self._controllers = controllers
        self._lock = threading.Lock()
        self._environment_meshes = environment_meshes
        self._environment_applied = False

    def execute_joint_waypoints(
        self,
        waypoints: list[JointWaypoint],
        *,
        return_to_base: JointWaypoint,
    ) -> MotionExecutionResult:
        with self._lock:
            self._apply_environment_once()

            executed: list[str] = []
            for waypoint in [*waypoints, return_to_base]:
                self._plan_and_execute(waypoint)
                executed.append(waypoint.zone)

            return MotionExecutionResult(executed_waypoints=executed)

    def _apply_environment_once(self) -> None:
        if self._environment_applied:
            return
        PlanningSceneEnvironment(self._moveit).apply_meshes(self._environment_meshes)
        self._environment_applied = True

    def _plan_and_execute(self, waypoint: JointWaypoint) -> None:
        robot_model = self._moveit.get_robot_model()
        robot_state = self._robot_state_class(robot_model)
        robot_state.joint_positions = waypoint.joint_positions

        self._planning_component.set_start_state_to_current_state()
        self._planning_component.set_goal_state(robot_state=robot_state)

        plan_result = self._planning_component.plan()
        if not plan_result:
            raise MoveItExecutionError(f"MoveIt2 planning failed for waypoint {waypoint.zone}")

        self._moveit.execute(plan_result.trajectory, controllers=self._controllers)
