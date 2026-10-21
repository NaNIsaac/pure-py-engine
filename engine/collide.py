"""
Only Works for counter clockwise oriented vertices
"""

import math
from typing import NamedTuple

from .gmath import Vec2, circular_pairwise

# clipped points this close outside the reference face are kept, so a resting
# face keeps both points while it rocks by a hair
CONTACT_MARGIN = 0.02


class Contact(NamedTuple):
    """`normal` is unit length and points from the first shape to the second.

    `points` holds one or two world-space contact points; `depths[i]` is how far
    `points[i]` sits inside the other shape (negative: just outside).
    """

    normal: Vec2
    depth: float
    points: tuple[Vec2, ...] = ()
    depths: tuple[float, ...] = ()


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


def _min_overlap(
    axes, project_a, project_b, a_center: Vec2, b_center: Vec2, margin: float
) -> Contact | None:
    """Separating axis test: None if any axis separates by `margin` or more, else the shallowest axis.

    A negative depth is the gap of shapes closer than `margin` (a speculative contact).
    """
    best = Contact(Vec2.zero(), math.inf)
    for axis in axes:
        a_min, a_max = project_a(axis)
        b_min, b_max = project_b(axis)
        overlap = min(a_max, b_max) - max(a_min, b_min)
        if overlap <= -margin:
            return None
        if overlap < best.depth:
            best = Contact(axis, overlap)
    if (b_center - a_center).dot(best.normal) < 0:
        best = Contact(-best.normal, best.depth)
    return best


def _edge_normals(vertices: list[Vec2]):
    return ((b - a).perp().normalized() for a, b in circular_pairwise(vertices))


def _facing_edge(vertices: list[Vec2], direction: Vec2) -> tuple[Vec2, Vec2]:
    """The edge whose outward normal points most along `direction`."""
    return max(
        circular_pairwise(vertices), key=lambda e: (e[1] - e[0]).perp().normalized().dot(direction)
    )


def _clip(points: list[Vec2], axis: Vec2, offset: float) -> list[Vec2]:
    """Keep the part of a point or segment where axis . x >= offset."""
    dist = [axis.dot(x) - offset for x in points]
    kept = [x for x, d in zip(points, dist) if d >= 0]
    if len(points) == 2 and dist[0] * dist[1] < 0:
        p, q = points
        kept.append(p + (q - p) * (dist[0] / (dist[0] - dist[1])))
    return kept


def _clip_points(a: list[Vec2], b: list[Vec2], hit: Contact, margin: float) -> Contact:
    """Box2D-style: clip the incident edge against the reference edge's side planes."""
    normal = hit.normal
    edge_a, edge_b = _facing_edge(a, normal), _facing_edge(b, -normal)

    def alignment(e, n):
        return abs((e[1] - e[0]).normalized().dot(n))

    # the reference edge is the one more perpendicular to the normal; ties go to `a`
    if alignment(edge_a, normal) <= alignment(edge_b, normal) + 1e-9:
        ref, inc = edge_a, edge_b
    else:
        ref, inc = edge_b, edge_a
    tangent = (ref[1] - ref[0]).normalized()
    points = _clip(inc, tangent, tangent.dot(ref[0]))
    points = _clip(points, -tangent, -tangent.dot(ref[1]))
    face = tangent.perp()
    kept = [
        (x, depth) for x in points if (depth := face.dot(ref[0] - x)) >= -CONTACT_MARGIN - margin
    ]
    if not kept:
        # ponytail: degenerate clip falls back to the deepest vertex of b
        kept = [(min(b, key=normal.dot), hit.depth)]
    points, depths = zip(*kept)
    return hit._replace(points=points, depths=depths)


def polygons_collide(a: list[Vec2], b: list[Vec2], margin: float = 0.0) -> Contact | None:
    """Convex polygons in world space, reported up to `margin` apart."""
    hit = _min_overlap(
        [*_edge_normals(a), *_edge_normals(b)],
        lambda axis: _project(a, axis),
        lambda axis: _project(b, axis),
        _mean(a),
        _mean(b),
        margin,
    )
    return None if hit is None else _clip_points(a, b, hit, margin)


def circle_polygon_collide(
    center: Vec2, radius: float, polygon: list[Vec2], margin: float = 0.0
) -> Contact | None:
    """Circle and convex polygon in world space; the normal points from the circle."""
    closest = min(polygon, key=center.dist_square)
    axes = [*_edge_normals(polygon)]
    if closest.dist_square(center) > 0:
        axes.append((closest - center).normalized())
    hit = _min_overlap(
        axes,
        lambda axis: (center.dot(axis) - radius, center.dot(axis) + radius),
        lambda axis: _project(polygon, axis),
        center,
        _mean(polygon),
        margin,
    )
    return (
        None
        if hit is None
        else hit._replace(points=(center + hit.normal * radius,), depths=(hit.depth,))
    )


def circles_collide(a: Vec2, ra: float, b: Vec2, rb: float, margin: float = 0.0) -> Contact | None:
    gap = b - a
    dist = gap.length()
    if dist >= ra + rb + margin:
        return None
    normal = gap / dist if dist > 0 else Vec2(1, 0)
    depth = ra + rb - dist
    return Contact(normal, depth, (a + normal * ra,), (depth,))
