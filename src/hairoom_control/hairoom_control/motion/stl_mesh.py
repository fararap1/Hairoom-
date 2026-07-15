from __future__ import annotations

import struct
from pathlib import Path

from geometry_msgs.msg import Point
from shape_msgs.msg import Mesh, MeshTriangle


class StlMeshError(ValueError):
    pass


def load_stl_as_mesh(path: str | Path) -> Mesh:
    stl_path = Path(path)
    if not stl_path.exists():
        raise StlMeshError(f"STL file does not exist: {stl_path}")

    data = stl_path.read_bytes()
    if _looks_like_binary_stl(data):
        return _load_binary_stl(data)
    return _load_ascii_stl(data.decode("utf-8", errors="ignore"))


def _looks_like_binary_stl(data: bytes) -> bool:
    if len(data) < 84:
        return False
    triangle_count = struct.unpack_from("<I", data, 80)[0]
    return len(data) == 84 + triangle_count * 50


def _load_binary_stl(data: bytes) -> Mesh:
    triangle_count = struct.unpack_from("<I", data, 80)[0]
    mesh = Mesh()

    offset = 84
    for triangle_index in range(triangle_count):
        vertex_start = len(mesh.vertices)
        offset += 12  # normal vector
        for _ in range(3):
            x, y, z = struct.unpack_from("<fff", data, offset)
            mesh.vertices.append(Point(x=float(x), y=float(y), z=float(z)))
            offset += 12
        mesh.triangles.append(
            MeshTriangle(vertex_indices=[vertex_start, vertex_start + 1, vertex_start + 2])
        )
        offset += 2  # attribute byte count

    return mesh


def _load_ascii_stl(text: str) -> Mesh:
    vertices: list[Point] = []
    mesh = Mesh()

    for line in text.splitlines():
        stripped = line.strip()
        if not stripped.startswith("vertex "):
            continue
        _, x, y, z = stripped.split(maxsplit=3)
        vertices.append(Point(x=float(x), y=float(y), z=float(z)))

        if len(vertices) == 3:
            vertex_start = len(mesh.vertices)
            mesh.vertices.extend(vertices)
            mesh.triangles.append(
                MeshTriangle(vertex_indices=[vertex_start, vertex_start + 1, vertex_start + 2])
            )
            vertices = []

    if not mesh.triangles:
        raise StlMeshError("ASCII STL did not contain any triangles")

    return mesh
