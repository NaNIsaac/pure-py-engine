import math


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
            aabb.min_y < self.max_y or 
            aabb.max_x < self.min_x or 
            aabb.max_y < self.min_y
            )
    
    @classmethod
    def from_circle(cls, center: Vec2, radius: float) -> AABB:
        return cls(center.x - radius, center.y - radius, center.x + radius, center.y + radius)
    
    @classmethod
    def from_vertices(cls, vertices: list[Vec2]):
        min_x: float = float("-inf")
        min_y: float = float("-inf")
        max_x: float = float("inf")
        max_y: float = float("inf")

        for vertex in vertices:
            min_x = min(min_x, vertex.x)
            min_y = min(min_y, vertex.y)
            max_x = max(max_x, vertex.x)
            max_y = max(max_y, vertex.y)
        
        return cls(min_x, min_y, max_x, max_y)


class Vec2:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    
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

    def square(self) -> float:
        return self.x**2 + self.y**2

    def dist_squre(self, other: Vec2) -> float:
        return (self.x - other.x)**2 + (self.y - other.y)**2

    def cross(self, other: Vec2) -> float:
        return self.x * other.y - self.y * other.x

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
    def lex_comp(cls, a, b):
        return a.x > b.x or (a.x == b.x and a.y > b.y)

    @classmethod
    def zero(cls) -> Vec2:
        return cls(0, 0)
