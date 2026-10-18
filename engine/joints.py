"""Joints solved as velocity constraints alongside the contacts.

Every joint keeps its accumulated impulse between steps for warm starting.
Rigid constraints correct drift with Baumgarte stabilisation; `hertz > 0`
makes a constraint soft (Box2D's spring formulation) instead.
"""
import math

from .body import Body, inv_effective_mass
from .gmath import Vec2

BAUMGARTE = 0.2


def _softness(hertz: float, damping_ratio: float, mass: float, dt: float) -> tuple[float, float]:
    """(gamma, bias rate) so that impulse = -(Cdot + rate * C + gamma * total) / (1 / mass + gamma)."""
    if hertz <= 0:
        return 0.0, BAUMGARTE / dt
    omega = math.tau * hertz
    damping = 2 * mass * damping_ratio * omega
    stiffness = mass * omega * omega
    gamma = 1 / (dt * (damping + dt * stiffness))
    return gamma, dt * stiffness * gamma


def _point_matrix(*arms: tuple[Body, Vec2], gamma: float = 0.0) -> tuple[float, float, float]:
    """Symmetric 2x2 inverse effective mass (k11, k12, k22) of a point constraint."""
    k11 = k22 = gamma
    k12 = 0.0
    for body, r in arms:
        m, i = body.inv_mass, body.inv_inertia
        k11 += m + i * r.y * r.y
        k12 -= i * r.x * r.y
        k22 += m + i * r.x * r.x
    return k11, k12, k22


def _solve2(k: tuple[float, float, float], rhs: Vec2) -> Vec2:
    k11, k12, k22 = k
    det = k11 * k22 - k12 * k12
    return Vec2((k22 * rhs.x - k12 * rhs.y) / det, (k11 * rhs.y - k12 * rhs.x) / det)


class Joint:
    """Two bodies tied at anchors given in world space when the joint is made.

    Connected bodies do not collide unless `collide_connected` is set.
    """

    def __init__(self, a: Body, b: Body, anchor_a, anchor_b, collide_connected: bool = False):
        self.a, self.b = a, b
        self.local_a, self.local_b = a.to_local(anchor_a), b.to_local(anchor_b)
        self.collide_connected = collide_connected

    @property
    def bodies(self) -> tuple[Body, ...]:
        return self.a, self.b

    def anchors(self) -> tuple[Vec2, Vec2]:
        return self.a.to_world(self.local_a), self.b.to_world(self.local_b)

    def _arms(self) -> tuple[Vec2, Vec2]:
        return self.local_a.rotate(self.a.angle), self.local_b.rotate(self.b.angle)

    def prestep(self, dt: float) -> None: ...

    def solve(self) -> None: ...


class _AngleLock:
    """Keeps b.angle - a.angle at its starting value."""

    def __init__(self, a: Body, b: Body):
        self.a, self.b = a, b
        self.reference = b.angle - a.angle
        self.total = 0.0

    def prestep(self, dt: float) -> None:
        a, b = self.a, self.b
        inv = a.inv_inertia + b.inv_inertia
        self.mass = 1 / inv if inv else 0.0
        self.bias = -BAUMGARTE / dt * (b.angle - a.angle - self.reference)
        a.w -= self.total * a.inv_inertia
        b.w += self.total * b.inv_inertia

    def solve(self) -> None:
        a, b = self.a, self.b
        impulse = self.mass * (self.bias - (b.w - a.w))
        self.total += impulse
        a.w -= impulse * a.inv_inertia
        b.w += impulse * b.inv_inertia


class RevoluteJoint(Joint):
    """Pins `a` and `b` together at the world point `anchor`; they still turn freely.

    With `motor_speed` (rad/s) set, b turns relative to a at that speed, using
    at most `max_motor_torque`.
    """

    def __init__(self, a: Body, b: Body, anchor, *, motor_speed: float | None = None,
                 max_motor_torque: float = math.inf, collide_connected: bool = False):
        super().__init__(a, b, anchor, anchor, collide_connected)
        self.motor_speed = motor_speed
        self.max_motor_torque = max_motor_torque
        self._total = Vec2.zero()
        self._motor_total = 0.0

    def prestep(self, dt: float) -> None:
        a, b = self.a, self.b
        self._ra, self._rb = ra, rb = self._arms()
        self._k = _point_matrix((a, ra), (b, rb))
        self._bias = ((b.pos + rb) - (a.pos + ra)) * (-BAUMGARTE / dt)
        a._push(ra, -self._total)
        b._push(rb, self._total)
        if self.motor_speed is not None:
            inv = a.inv_inertia + b.inv_inertia
            self._motor_mass = 1 / inv if inv else 0.0
            self._motor_limit = self.max_motor_torque * dt
            a.w -= self._motor_total * a.inv_inertia
            b.w += self._motor_total * b.inv_inertia

    def solve(self) -> None:
        a, b = self.a, self.b
        if self.motor_speed is not None:
            old = self._motor_total
            wanted = old - self._motor_mass * (b.w - a.w - self.motor_speed)
            self._motor_total = min(max(wanted, -self._motor_limit), self._motor_limit)
            impulse = self._motor_total - old
            a.w -= impulse * a.inv_inertia
            b.w += impulse * b.inv_inertia
        dv = b._velocity(self._rb) - a._velocity(self._ra)
        impulse = _solve2(self._k, self._bias - dv)
        self._total += impulse
        a._push(self._ra, -impulse)
        b._push(self._rb, impulse)


class WeldJoint(RevoluteJoint):
    """Glues `a` and `b` together at `anchor`: no relative motion or turning."""

    def __init__(self, a: Body, b: Body, anchor, *, collide_connected: bool = False):
        super().__init__(a, b, anchor, collide_connected=collide_connected)
        self._lock = _AngleLock(a, b)

    def prestep(self, dt: float) -> None:
        super().prestep(dt)
        self._lock.prestep(dt)

    def solve(self) -> None:
        self._lock.solve()
        super().solve()


class DistanceJoint(Joint):
    """Keeps `anchor_a` on a and `anchor_b` on b at `length` (default: as made).

    `hertz > 0` turns the rod into a spring with that natural frequency.
    """

    def __init__(self, a: Body, b: Body, anchor_a, anchor_b, *, length: float | None = None,
                 hertz: float = 0.0, damping_ratio: float = 0.0, collide_connected: bool = False):
        super().__init__(a, b, anchor_a, anchor_b, collide_connected)
        self.length = (Vec2(*anchor_b) - Vec2(*anchor_a)).length() if length is None else length
        self.hertz, self.damping_ratio = hertz, damping_ratio
        self._total = 0.0

    def prestep(self, dt: float) -> None:
        a, b = self.a, self.b
        self._ra, self._rb = ra, rb = self._arms()
        gap = (b.pos + rb) - (a.pos + ra)
        distance = gap.length()
        self._n = n = gap / distance if distance > 0 else Vec2(1, 0)
        inv = inv_effective_mass(a, ra, b, rb, n)
        self._gamma, rate = _softness(self.hertz, self.damping_ratio, 1 / inv, dt)
        self._mass = 1 / (inv + self._gamma)
        self._bias = rate * (distance - self.length)
        a._push(ra, n * -self._total)
        b._push(rb, n * self._total)

    def solve(self) -> None:
        a, b, n = self.a, self.b, self._n
        speed = (b._velocity(self._rb) - a._velocity(self._ra)).dot(n)
        impulse = -self._mass * (speed + self._bias + self._gamma * self._total)
        self._total += impulse
        a._push(self._ra, n * -impulse)
        b._push(self._rb, n * impulse)


class PrismaticJoint(Joint):
    """`b` slides along `axis` (fixed in a's frame) through the world point `anchor`, without turning."""

    # ponytail: no translation limits or motor; add them like RevoluteJoint's motor when a slider needs stops
    def __init__(self, a: Body, b: Body, anchor, axis, *, collide_connected: bool = False):
        super().__init__(a, b, anchor, anchor, collide_connected)
        self.local_axis = Vec2(*axis).normalized().rotate(-a.angle)
        self._lock = _AngleLock(a, b)
        self._total = 0.0

    def prestep(self, dt: float) -> None:
        a, b = self.a, self.b
        ra, rb = self._arms()
        gap = (b.pos + rb) - (a.pos + ra)
        self._perp = perp = self.local_axis.rotate(a.angle).perp()
        self._sa, self._sb = (gap + ra).cross(perp), rb.cross(perp)
        inv = a.inv_mass + b.inv_mass + a.inv_inertia * self._sa ** 2 + b.inv_inertia * self._sb ** 2
        self._mass = 1 / inv
        self._bias = -BAUMGARTE / dt * gap.dot(perp)
        self._apply(self._total)
        self._lock.prestep(dt)

    def _apply(self, impulse: float) -> None:
        a, b, perp = self.a, self.b, self._perp
        a.v -= perp * (impulse * a.inv_mass)
        a.w -= self._sa * impulse * a.inv_inertia
        b.v += perp * (impulse * b.inv_mass)
        b.w += self._sb * impulse * b.inv_inertia

    def solve(self) -> None:
        a, b = self.a, self.b
        self._lock.solve()
        speed = self._perp.dot(b.v - a.v) + self._sb * b.w - self._sa * a.w
        impulse = self._mass * (self._bias - speed)
        self._total += impulse
        self._apply(impulse)


class MouseJoint(Joint):
    """Pulls the world point `anchor` of `body` toward `target` with a soft spring.

    Set `target` every frame while dragging; the pull is capped at `max_force`
    (default 1000 x the body's mass).
    """

    def __init__(self, body: Body, anchor=None, *, hertz: float = 5.0, damping_ratio: float = 0.7,
                 max_force: float | None = None):
        anchor = body.pos if anchor is None else Vec2(*anchor)
        self.a, self.b = None, body
        self.local_b = body.to_local(anchor)
        self.collide_connected = True
        self._target = anchor
        self.hertz, self.damping_ratio = hertz, damping_ratio
        self.max_force = 1000 / body.inv_mass if max_force is None else max_force
        self._total = Vec2.zero()

    @property
    def bodies(self) -> tuple[Body, ...]:
        return (self.b,)

    @property
    def target(self) -> Vec2:
        return self._target

    @target.setter
    def target(self, point) -> None:
        self._target = Vec2(*point)
        self.b.wake()

    def anchors(self) -> tuple[Vec2, Vec2]:
        return self._target, self.b.to_world(self.local_b)

    def prestep(self, dt: float) -> None:
        b = self.b
        self._rb = rb = self.local_b.rotate(b.angle)
        self._gamma, rate = _softness(self.hertz, self.damping_ratio, 1 / b.inv_mass, dt)
        self._k = _point_matrix((b, rb), gamma=self._gamma)
        self._bias = ((b.pos + rb) - self._target) * rate
        self._limit = self.max_force * dt
        b._push(rb, self._total)

    def solve(self) -> None:
        b = self.b
        old = self._total
        total = old + _solve2(self._k, -(b._velocity(self._rb) + self._bias + old * self._gamma))
        if total.square() > self._limit ** 2:
            total = total.normalized() * self._limit
        self._total = total
        b._push(self._rb, total - old)
