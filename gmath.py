import math
from dataclasses import dataclass

from seltools import circular_pairwise, circular_windowed


def sign(x: float) -> int:
    return (x > 0) - (x < 0)

def near_zero(n: float, tol: float = 1e-12) -> bool:
    return -tol < n < tol

class AABB:
    def __init__(self, min_x: float, min_y: float, max_x: float, max_y: float) -> None:
        self.min_x: float = min_x
        self.min_y: float = min_y
        self.max_x: float = max_x
        self.max_y: float = max_y
    
    def is_point_inside(self, point: Vec2) -> bool:
        return self.min_x <= point.x <= self.max_x and self.min_y <= point.y <= self.max_y
    
    def is_overlap(self, aabb: AABB) -> bool:
        return not (
            aabb.min_x > self.max_x or 
            aabb.min_y > self.max_y or 
            aabb.max_x < self.min_x or 
            aabb.max_y < self.min_y
            )
    
    @classmethod
    def from_circle(cls, center: Vec2, radius: float) -> AABB:
        return cls(center.x - radius, center.y - radius, center.x + radius, center.y + radius)
    
    @classmethod
    def from_vertices(cls, vertices: list[Vec2]):
        min_x: float = float("inf")
        min_y: float = float("inf")
        max_x: float = float("-inf")
        max_y: float = float("-inf")

        for vertex in vertices:
            min_x = min(min_x, vertex.x)
            min_y = min(min_y, vertex.y)
            max_x = max(max_x, vertex.x)
            max_y = max(max_y, vertex.y)
        
        return cls(min_x, min_y, max_x, max_y)


@dataclass(frozen=True, slots=True)
class Vec2:
    x: float
    y: float
    
    def __mul__(self, a: float) -> Vec2:
        return Vec2(a * self.x, a * self.y)

    def __rmul__(self, a: float) -> Vec2:
        return Vec2(a * self.x, a * self.y)
    
    def __truediv__(self, a: float) -> Vec2:
        return Vec2(self.x / a, self.y / a)

    def __add__(self, other: Vec2) -> Vec2:
        return Vec2(self.x + other.x, self.y + other.y)
    
    def __sub__(self, other: Vec2) -> Vec2:
        return Vec2(self.x - other.x, self.y - other.y)

    def __neg__(self) -> Vec2:
        return Vec2(-self.x, -self.y)

    def length(self) -> float:
        return math.hypot(self.x, self.y)

    def normalized(self) -> Vec2:
        return self / self.length()

    def perp(self) -> Vec2:
        """Rotated -90 degrees: the outward normal of a counter-clockwise edge."""
        return Vec2(self.y, -self.x)

    def square(self) -> float:
        return self.x**2 + self.y**2

    def dist_square(self, other: Vec2) -> float:
        return (self.x - other.x)**2 + (self.y - other.y)**2

    def cross(self, other: Vec2) -> float:
        return self.x * other.y - other.x * self.y

    def dot(self, other: Vec2) -> float:
        return self.x * other.x + self.y * other.y

    def rotate(self, angle: float) -> Vec2:
        ang_rad = math.radians(angle)
        sin = math.sin(ang_rad)
        cos = math.cos(ang_rad)
        return Vec2(self.x * cos - self.y * sin, self.x * sin + self.y * cos)

    def to_tuple(self) -> tuple[float, float]:
        return (self.x, self.y)

    @classmethod
    def zero(cls) -> Vec2:
        return cls(0, 0)
    

def polygon_signed_area(vertices: list[Vec2]) -> float:
    """Positive for counter-clockwise vertices."""
    return 0.5 * sum(a.cross(b) for a, b in circular_pairwise(vertices))


def polygon_centroid(vertices: list[Vec2]) -> Vec2:
    weighted = sum(((a + b) * a.cross(b) for a, b in circular_pairwise(vertices)), Vec2.zero())
    return weighted / (6 * polygon_signed_area(vertices))


def is_convex(vertices: list[Vec2]) -> bool:
    """For counter-clockwise vertices; collinear points count as convex."""
    return all((b - a).cross(c - b) >= 0 for a, b, c in circular_windowed(vertices, 3))


def centered_convex(vertices: list[Vec2]) -> list[Vec2]:
    """Counter-clockwise copy with the centroid moved to the origin.

    The collision tests and the inertia formula assume both.
    """
    area = polygon_signed_area(vertices)
    if near_zero(area):
        raise ValueError("polygon has no area")
    if area < 0:
        vertices = vertices[::-1]
    if not is_convex(vertices):
        raise ValueError("polygon is not convex")
    c = polygon_centroid(vertices)
    return [v - c for v in vertices]


def regular_polygon(sides: int, radius: float) -> list[Vec2]:
    """Counter-clockwise, centered on the origin, first vertex on +x."""
    step = 2 * math.pi / sides
    return [Vec2(radius * math.cos(k * step), radius * math.sin(k * step)) for k in range(sides)]


def rectangle(width: float, height: float) -> list[Vec2]:
    """Counter-clockwise, centered on the origin."""
    w, h = width / 2, height / 2
    return [Vec2(-w, -h), Vec2(w, -h), Vec2(w, h), Vec2(-w, h)]
