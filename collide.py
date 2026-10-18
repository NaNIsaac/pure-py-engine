"""
Only Works for counter clockwise oriented vertices
"""
import math
from typing import NamedTuple

from seltools import circular_pairwise

from gmath import Vec2


class Contact(NamedTuple):
    """`normal` is unit length and points from the first shape to the second."""
    normal: Vec2
    depth: float

def point_in_triangle(p: Vec2, a: Vec2, b: Vec2, c: Vec2) -> bool:
    return all((v - u).cross(p - u) > 0 for u, v in circular_pairwise((a, b, c)))

def point_in_convex(p: Vec2, vertices: list[Vec2]) -> bool:
    
    origin = vertices[0]
    segments = [vertices[i] - origin for i in range(1, len(vertices))]

    point = p - origin
    if segments[0].cross(point) < 0:
        return False
    if segments[-1].cross(point) > 0:
        return False
    low = 0
    high = len(segments) - 1
    while low + 1 < high:
        mid = (low + high) // 2
        if segments[mid].cross(point) >= 0:
            low = mid
        else:
            high = mid
    
    return (segments[high] - segments[low]).cross(point - segments[low]) >= 0


def _project(vertices: list[Vec2], axis: Vec2) -> tuple[float, float]:
    dots = [v.dot(axis) for v in vertices]
    return min(dots), max(dots)


def _mean(vertices: list[Vec2]) -> Vec2:
    return sum(vertices, Vec2.zero()) / len(vertices)


def _min_overlap(axes, project_a, project_b, a_center: Vec2, b_center: Vec2) -> Contact | None:
    """Separating axis test: None if any axis separates, else the shallowest axis."""
    best = Contact(Vec2.zero(), math.inf)
    for axis in axes:
        a_min, a_max = project_a(axis)
        b_min, b_max = project_b(axis)
        overlap = min(a_max, b_max) - max(a_min, b_min)
        if overlap <= 0:
            return None
        if overlap < best.depth:
            best = Contact(axis, overlap)
    if (b_center - a_center).dot(best.normal) < 0:
        best = Contact(-best.normal, best.depth)
    return best


def _edge_normals(vertices: list[Vec2]):
    return ((b - a).perp().normalized() for a, b in circular_pairwise(vertices))


def polygons_collide(a: list[Vec2], b: list[Vec2]) -> Contact | None:
    """Convex polygons in world space."""
    return _min_overlap(
        [*_edge_normals(a), *_edge_normals(b)],
        lambda axis: _project(a, axis),
        lambda axis: _project(b, axis),
        _mean(a), _mean(b),
    )


def circle_polygon_collide(center: Vec2, radius: float, polygon: list[Vec2]) -> Contact | None:
    """Circle and convex polygon in world space; the normal points from the circle."""
    closest = min(polygon, key=center.dist_square)
    axes = [*_edge_normals(polygon)]
    if closest.dist_square(center) > 0:
        axes.append((closest - center).normalized())
    return _min_overlap(
        axes,
        lambda axis: (center.dot(axis) - radius, center.dot(axis) + radius),
        lambda axis: _project(polygon, axis),
        center, _mean(polygon),
    )


def circles_collide(a: Vec2, ra: float, b: Vec2, rb: float) -> Contact | None:
    gap = b - a
    dist = gap.length()
    if dist >= ra + rb:
        return None
    normal = gap / dist if dist > 0 else Vec2(1, 0)
    return Contact(normal, ra + rb - dist)
