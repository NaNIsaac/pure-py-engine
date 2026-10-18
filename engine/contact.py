import math
from dataclasses import dataclass, field

from .body import Body, inv_effective_mass
from .collide import Contact, circle_polygon_collide, circles_collide, polygons_collide
from .gmath import Vec2

BAUMGARTE = 0.2  # fraction of the penetration fixed per step
SLOP = 0.01  # penetration left alone so resting contacts do not jitter
BOUNCE_THRESHOLD = 1.0  # slower impacts do not bounce, so stacks can come to rest
# ponytail: warm-start points are matched by distance in a's frame; add feature ids if fast spinning contacts mismatch
MATCH_DISTANCE = 0.1


def collide(a: Body, b: Body) -> Contact | None:
    """Overlap of two bodies; the normal points from `a` to `b`."""
    match a.vertices, b.vertices:
        case None, None:
            return circles_collide(a.pos, a.radius, b.pos, b.radius)
        case None, _:
            return circle_polygon_collide(a.pos, a.radius, b.vertices)
        case _, None:
            hit = circle_polygon_collide(b.pos, b.radius, a.vertices)
            return None if hit is None else hit._replace(normal=-hit.normal)
        case _:
            return polygons_collide(a.vertices, b.vertices)


@dataclass(slots=True)
class _Point:
    world: Vec2
    depth: float
    local: Vec2  # in a's frame, to recognise the same point next step
    normal_impulse: float = 0.0
    tangent_impulse: float = 0.0
    ra: Vec2 = field(default_factory=Vec2.zero)
    rb: Vec2 = field(default_factory=Vec2.zero)
    normal_mass: float = 0.0
    tangent_mass: float = 0.0
    bias: float = 0.0


class Arbiter:
    """Contact between two bodies, kept across steps so impulses can warm start."""

    def __init__(self, a: Body, b: Body):
        self.a, self.b = a, b
        self.friction = math.sqrt(a.friction * b.friction)
        self.restitution = max(a.restitution, b.restitution)
        self.points: list[_Point] = []
        self.normal = Vec2(1, 0)

    def update(self, hit: Contact) -> None:
        old = self.points
        self.normal = hit.normal
        self.points = []
        for world, depth in zip(hit.points, hit.depths):
            point = _Point(world, depth, self.a.to_local(world))
            match = min(old, key=lambda o: o.local.dist_square(point.local), default=None)
            if match is not None and match.local.dist_square(point.local) < MATCH_DISTANCE ** 2:
                point.normal_impulse, point.tangent_impulse = match.normal_impulse, match.tangent_impulse
            self.points.append(point)

    def prestep(self, inv_dt: float) -> None:
        a, b, n = self.a, self.b, self.normal
        t = n.perp()
        for p in self.points:
            p.ra, p.rb = p.world - a.pos, p.world - b.pos
            p.normal_mass = 1 / inv_effective_mass(a, p.ra, b, p.rb, n)
            p.tangent_mass = 1 / inv_effective_mass(a, p.ra, b, p.rb, t)
            closing = (b._velocity(p.rb) - a._velocity(p.ra)).dot(n)
            bounce = -self.restitution * closing if closing < -BOUNCE_THRESHOLD else 0.0
            # a bounce already separates the bodies; adding the push on top would add energy
            if p.depth < 0:
                push = p.depth * inv_dt  # speculative: may close the gap, not more
            else:
                push = BAUMGARTE * inv_dt * max(0.0, p.depth - SLOP)
            p.bias = bounce or push

    def warm_start(self) -> None:
        """Reapply last step's impulses; run after every arbiter's prestep has read the velocities."""
        a, b, n = self.a, self.b, self.normal
        t = n.perp()
        for p in self.points:
            impulse = n * p.normal_impulse + t * p.tangent_impulse
            a._push(p.ra, -impulse)
            b._push(p.rb, impulse)

    def solve(self) -> None:
        a, b, n = self.a, self.b, self.normal
        t = n.perp()
        for p in self.points:
            dv = b._velocity(p.rb) - a._velocity(p.ra)
            total = max(p.normal_impulse + p.normal_mass * (p.bias - dv.dot(n)), 0.0)
            impulse = n * (total - p.normal_impulse)
            p.normal_impulse = total
            a._push(p.ra, -impulse)
            b._push(p.rb, impulse)

            dv = b._velocity(p.rb) - a._velocity(p.ra)
            limit = self.friction * p.normal_impulse
            total = min(max(p.tangent_impulse - p.tangent_mass * dv.dot(t), -limit), limit)
            impulse = t * (total - p.tangent_impulse)
            p.tangent_impulse = total
            a._push(p.ra, -impulse)
            b._push(p.rb, impulse)
