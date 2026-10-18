import math

from seltools import circular_pairwise

from .collide import point_in_convex
from .gmath import AABB, Vec2, centered_convex, rectangle, regular_polygon


def _unit_inertia(radius: float, vertices: list[Vec2] | None) -> float:
    """Moment of inertia per unit mass about the centroid."""
    if vertices is None:
        return 0.5 * radius * radius
    numerator = denominator = 0.0
    for a, b in circular_pairwise(vertices):
        cross = abs(a.cross(b))
        numerator += (a.square() + a.dot(b) + b.square()) * cross
        denominator += cross
    return numerator / denominator / 6


class Body:
    """A rigid circle (`radius`) or convex polygon (`vertices`).

    Build one with `Body.circle`, `Body.polygon`, `Body.box` or `Body.regular`.
    Polygon vertices may be in either winding and anywhere around the origin;
    they are stored counter-clockwise around their centroid. Angles are in
    radians. `static=True` (or `mass=math.inf`) pins the body in place.
    Points and vectors may be given as `Vec2` or `(x, y)`.
    """

    def __init__(
        self,
        mass: float = 1.0,
        *,
        radius: float = 0.0,
        vertices: list[Vec2] | None = None,
        pos=(0, 0),
        angle: float = 0.0,
        v=(0, 0),
        w: float = 0.0,
        friction: float = 0.4,
        restitution: float = 0.2,
        static: bool = False,
    ):
        static = static or math.isinf(mass)
        self.radius = radius
        self.local_vertices = None if vertices is None else centered_convex(vertices)
        self.pos = Vec2(*pos)
        self.angle = angle
        self.v = Vec2(*v)
        self.w = w
        self.friction = friction
        self.restitution = restitution
        self.inv_mass = 0.0 if static else 1 / mass
        self.inv_inertia = 0.0 if static else 1 / (mass * _unit_inertia(radius, self.local_vertices))

        self.force = Vec2.zero()
        self.torque = 0.0
        self.awake = not static
        self.sleep_time = 0.0
        self._pose = None

    @classmethod
    def circle(cls, radius: float, **kwargs) -> Body:
        return cls(radius=radius, **kwargs)

    @classmethod
    def polygon(cls, vertices, **kwargs) -> Body:
        return cls(vertices=[Vec2(*v) for v in vertices], **kwargs)

    @classmethod
    def box(cls, width: float, height: float, **kwargs) -> Body:
        return cls(vertices=rectangle(width, height), **kwargs)

    @classmethod
    def regular(cls, sides: int, radius: float, **kwargs) -> Body:
        return cls(vertices=regular_polygon(sides, radius), **kwargs)

    @property
    def is_static(self) -> bool:
        return self.inv_mass == 0

    def _refresh(self) -> None:
        # world vertices and AABB are rebuilt only when the pose changed (sleeping and static bodies never do)
        if self._pose == (self.pos, self.angle):
            return
        self._pose = (self.pos, self.angle)
        if self.local_vertices is None:
            self._vertices = None
            self._aabb = AABB.from_circle(self.pos, self.radius)
        else:
            cos, sin = math.cos(self.angle), math.sin(self.angle)
            px, py = self.pos
            self._vertices = [Vec2(px + v.x * cos - v.y * sin, py + v.x * sin + v.y * cos) for v in self.local_vertices]
            self._aabb = AABB.from_vertices(self._vertices)

    @property
    def vertices(self) -> list[Vec2] | None:
        """World-space vertices, counter-clockwise; None for a circle."""
        self._refresh()
        return self._vertices

    @property
    def aabb(self) -> AABB:
        self._refresh()
        return self._aabb

    def to_world(self, local) -> Vec2:
        return self.pos + Vec2(*local).rotate(self.angle)

    def to_local(self, point) -> Vec2:
        return (Vec2(*point) - self.pos).rotate(-self.angle)

    def contains(self, point) -> bool:
        point = Vec2(*point)
        if self.local_vertices is None:
            return self.pos.dist_square(point) <= self.radius ** 2
        return point_in_convex(point, self.vertices)

    def velocity_at(self, point) -> Vec2:
        return self._velocity(Vec2(*point) - self.pos)

    def apply_force(self, force, point=None) -> None:
        """Accumulated until the next step; `point` (world) adds torque."""
        force = Vec2(*force)
        self.force += force
        if point is not None:
            self.torque += (Vec2(*point) - self.pos).cross(force)

    def apply_impulse(self, impulse, point=None) -> None:
        """Instant velocity change; `point` (world) adds spin. Wakes the body."""
        self.wake()
        r = Vec2.zero() if point is None else Vec2(*point) - self.pos
        self._push(r, Vec2(*impulse))

    def wake(self) -> None:
        if not self.is_static:
            self.awake = True
            self.sleep_time = 0.0

    def sleep(self) -> None:
        self.awake = False
        self.v = Vec2.zero()
        self.w = 0.0

    # solver hot path: `r` is the arm from the center to the point, in world orientation
    def _velocity(self, r: Vec2) -> Vec2:
        return self.v + r.perp() * -self.w  # v + w x r

    def _push(self, r: Vec2, impulse: Vec2) -> None:
        self.v += impulse * self.inv_mass
        self.w += r.cross(impulse) * self.inv_inertia


def inv_effective_mass(a: Body, ra: Vec2, b: Body, rb: Vec2, direction: Vec2) -> float:
    """1 / effective mass of the two bodies along `direction` at arms `ra`, `rb`."""
    return (a.inv_mass + b.inv_mass
            + a.inv_inertia * ra.cross(direction) ** 2
            + b.inv_inertia * rb.cross(direction) ** 2)
