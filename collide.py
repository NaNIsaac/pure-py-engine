"""
Only Works for counter clockwise oriented vertices
"""
from seltools import circular_pairs

from gmath import Vec2

def point_in_triangle(p: Vec2, a: Vec2, b: Vec2, c: Vec2) -> bool:
    return all((u - p).cross(u - v) > 0 for u, v in circular_pairs((a, b, c)))

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
