from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class JointWaypoint:
    zone: str
    joint_positions: dict[str, float]
    xyz: tuple[float, float, float] | None = None


def _to_float(value: str) -> float:
    return float(value.strip().replace("−", "-"))


class PointsFileError(ValueError):
    pass


class PointsRepository:
    def __init__(self, path: str | Path, joint_names: list[str]):
        self.path = Path(path)
        self.joint_names = joint_names

    def load_waypoints(self) -> list[JointWaypoint]:
        if not self.path.exists():
            raise PointsFileError(f"Points file does not exist: {self.path}")

        lines = [
            line
            for line in self.path.read_text(encoding="utf-8").splitlines()
            if line.strip()
        ]
        header_index = self._find_header(lines)
        delimiter = "\t" if "\t" in lines[header_index] else ","
        reader = csv.DictReader(lines[header_index:], delimiter=delimiter)

        waypoints: list[JointWaypoint] = []
        for row in reader:
            zone = (row.get("Зона") or row.get("zone") or row.get("Zone") or "").strip()
            if not zone:
                continue

            joint_positions = {
                joint_name: math.radians(_to_float(row[f"j{index}"]))
                for index, joint_name in enumerate(self.joint_names, start=1)
            }
            xyz = None
            if all(key in row and row[key] for key in ("X", "Y", "Z")):
                xyz = (_to_float(row["X"]), _to_float(row["Y"]), _to_float(row["Z"]))

            waypoints.append(
                JointWaypoint(
                    zone=zone,
                    joint_positions=joint_positions,
                    xyz=xyz,
                )
            )

        if not waypoints:
            raise PointsFileError(f"No waypoints found in points file: {self.path}")

        return waypoints

    @staticmethod
    def _find_header(lines: list[str]) -> int:
        for index, line in enumerate(lines):
            if "j1" in line and "j6" in line:
                return index
        raise PointsFileError("Could not find a header with j1..j6 columns")
