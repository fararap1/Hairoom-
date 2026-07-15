from __future__ import annotations

import argparse
import os
import signal
import sys
import threading
from pathlib import Path
from typing import Sequence

import rclpy
from rclpy.utilities import remove_ros_args

from hairoom_control.http.json_server import ProcedureBusyError, make_server
from hairoom_control.motion.environment import CollisionMeshSpec
from hairoom_control.motion.moveit_client import MoveIt2Client
from hairoom_control.motion.points import JointWaypoint, PointsRepository


DEFAULT_JOINT_NAMES = ["j1", "j2", "j3", "j4", "j5", "j6"]


class WashProcedureService:
    def __init__(
        self,
        *,
        points_file: Path,
        joint_names: list[str],
        base_joint_positions: dict[str, float],
        motion_client: MoveIt2Client,
    ):
        self._points_file = points_file
        self._joint_names = joint_names
        self._base_joint_positions = base_joint_positions
        self._motion_client = motion_client
        self._busy_lock = threading.Lock()

    def run_wash(self) -> dict:
        if not self._busy_lock.acquire(blocking=False):
            raise ProcedureBusyError("wash procedure is already running")

        try:
            waypoints = PointsRepository(self._points_file, self._joint_names).load_waypoints()
            base = JointWaypoint(zone="base", joint_positions=self._base_joint_positions)
            result = self._motion_client.execute_joint_waypoints(
                waypoints,
                return_to_base=base,
            )
            return {
                "status": "completed",
                "procedure": "wash",
                "points_file": str(self._points_file),
                "executed_waypoints": result.executed_waypoints,
                "waypoint_count": len(result.executed_waypoints),
            }
        finally:
            self._busy_lock.release()


def main(argv: Sequence[str] | None = None) -> None:
    raw_args = list(argv) if argv is not None else sys.argv[1:]
    args = _parse_args(raw_args)

    rclpy.init(args=raw_args)
    try:
        motion_client = MoveIt2Client(
            node_name=args.moveit_node_name,
            planning_group=args.planning_group,
            controllers=_split_csv(args.controllers),
            environment_meshes=_default_environment_meshes(args),
        )
        service = WashProcedureService(
            points_file=Path(args.points_file).expanduser().resolve(),
            joint_names=_split_csv(args.joint_names),
            base_joint_positions=_parse_base_positions(args.base_degrees, _split_csv(args.joint_names)),
            motion_client=motion_client,
        )

        http_server = make_server(args.host, args.port, service)
        stop_event = threading.Event()

        def stop_server(*_):
            stop_event.set()
            http_server.shutdown()

        signal.signal(signal.SIGINT, stop_server)
        signal.signal(signal.SIGTERM, stop_server)

        print(f"Hairoom wash server listening on http://{args.host}:{args.port}")
        print("POST /wash executes all points and returns to base")
        http_server.serve_forever()
        stop_event.wait(timeout=1.0)
    finally:
        rclpy.shutdown()


def _parse_args(argv: Sequence[str] | None):
    parser = argparse.ArgumentParser(description="Hairoom HTTP wash server backed by MoveIt2")
    parser.add_argument("--host", default=os.getenv("HAIROOM_HTTP_HOST", "0.0.0.0"))
    parser.add_argument("--port", type=int, default=int(os.getenv("HAIROOM_HTTP_PORT", "8080")))
    parser.add_argument("--points-file", default=str(_default_points_file()))
    parser.add_argument("--planning-group", default=os.getenv("HAIROOM_PLANNING_GROUP", "fr5_arm"))
    parser.add_argument("--moveit-node-name", default="hairoom_moveit_py")
    parser.add_argument("--joint-names", default=",".join(DEFAULT_JOINT_NAMES))
    parser.add_argument("--base-degrees", default=os.getenv("HAIROOM_BASE_DEGREES", "0,0,0,0,0,0"))
    parser.add_argument("--controllers", default=os.getenv("HAIROOM_CONTROLLERS", ""))
    parser.add_argument(
        "--mesh",
        action="append",
        default=[],
        help="Collision mesh as object_id=path[:frame_id]. Can be repeated.",
    )
    return parser.parse_args(remove_ros_args(args=argv))


def _default_points_file() -> Path:
    share_data = _package_share() / "data" / "points.csv"
    if share_data.exists():
        return share_data
    return _repo_root() / "data" / "points.csv"


def _default_environment_meshes(args) -> list[CollisionMeshSpec]:
    mesh_specs = [_parse_mesh_arg(mesh_arg) for mesh_arg in args.mesh]
    if mesh_specs:
        return mesh_specs

    mesh_dir = _package_share() / "meshes"
    if not mesh_dir.exists():
        mesh_dir = _repo_root() / "src" / "meshes"

    default_meshes = [
        CollisionMeshSpec("head", mesh_dir / "head_centered.stl"),
        CollisionMeshSpec("sink", mesh_dir / "sink_centered.stl"),
    ]
    return [mesh for mesh in default_meshes if mesh.path.exists()]


def _parse_mesh_arg(value: str) -> CollisionMeshSpec:
    if "=" not in value:
        raise ValueError("--mesh must use object_id=path[:frame_id]")
    object_id, rest = value.split("=", maxsplit=1)
    parts = rest.split(":")
    path = Path(parts[0]).expanduser().resolve()
    frame_id = parts[1] if len(parts) > 1 else "world"
    return CollisionMeshSpec(object_id=object_id, path=path, frame_id=frame_id)


def _parse_base_positions(value: str, joint_names: list[str]) -> dict[str, float]:
    import math

    degrees = [float(part.strip()) for part in value.split(",") if part.strip()]
    if len(degrees) != len(joint_names):
        raise ValueError(
            f"--base-degrees must contain {len(joint_names)} values, got {len(degrees)}"
        )
    return {
        joint_name: math.radians(degrees[index])
        for index, joint_name in enumerate(joint_names)
    }


def _split_csv(value: str) -> list[str]:
    return [part.strip() for part in value.split(",") if part.strip()]


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _package_share() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("hairoom_control"))
    except Exception:
        return Path()


if __name__ == "__main__":
    main()
