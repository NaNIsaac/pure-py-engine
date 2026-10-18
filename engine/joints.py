"""Joints solved as velocity constraints alongside the contacts.

Every joint keeps its accumulated impulse between steps for warm starting.
Rigid constraints correct drift with Baumgarte stabilisation; `hertz > 0`
makes a constraint soft (Box2D's spring formulation) instead.
"""
import math

from .body import Body
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


class _Row:
    """One scalar velocity constraint, speed = d . (vb - va) + sb * wb - sa * wa.

    Keeps its accumulated impulse for warm starting. `solve` drives
    speed + bias toward zero, clamping the total impulse to [lo, hi].
    """

    def __init__(self, a: Body, b: Body):
        self.a, self.b = a, b
        self.total = 0.0

    def setup(self, dt: float, d: Vec2, sa: float, sb: float,
              hertz: float = 0.0, damping_ratio: float = 0.0) -> float:
        """Prepare for this step and warm start; returns the bias rate for the position error."""
        a, b = self.a, self.b
        self.d, self.sa, self.sb = d, sa, sb
        inv = (a.inv_mass + b.inv_mass) * d.square() + a.inv_inertia * sa * sa + b.inv_inertia * sb * sb
        self.gamma, rate = _softness(hertz, damping_ratio, 1 / inv, dt)
        self.mass = 1 / (inv + self.gamma)
        self._apply(self.total)
        return rate

    def _apply(self, impulse: float) -> None:
        a, b = self.a, self.b
        a.v -= self.d * (impulse * a.inv_mass)
        a.w -= self.sa * impulse * a.inv_inertia
        b.v += self.d * (impulse * b.inv_mass)
        b.w += self.sb * impulse * b.inv_inertia

    def solve(self, bias: float, lo: float = -math.inf, hi: float = math.inf) -> None:
        a, b = self.a, self.b
        old = self.total
        speed = self.d.dot(b.v - a.v) + self.sb * b.w - self.sa * a.w
        self.total = min(max(old - self.mass * (speed + bias + self.gamma * old), lo), hi)
        self._apply(self.total - old)


def _limit_bias(room: float, dt: float) -> float:
    """Bias for an inequality `room >= 0`: speculative while clear, Baumgarte once violated."""
    return room / dt if room > 0 else BAUMGARTE * room / dt


_TURN = Vec2(0, 0)  # `d` of a purely angular row


class _Limits:
    """Optional lower / upper stops and a motor on one joint coordinate."""

    def __init__(self, a: Body, b: Body, lower: float | None, upper: float | None,
                 motor_speed: float | None, max_motor: float):
        self.lower, self.upper = lower, upper
        self.motor_speed, self.max_motor = motor_speed, max_motor
        self._lower, self._upper, self._motor = _Row(a, b), _Row(a, b), _Row(a, b)

    def prestep(self, dt: float, value: float, d: Vec2, sa: float, sb: float) -> None:
        if self.motor_speed is not None:
            self._motor.setup(dt, d, sa, sb)
            self._motor_limit = self.max_motor * dt
        if self.lower is not None:
            self._lower.setup(dt, d, sa, sb)
            self._lower_bias = _limit_bias(value - self.lower, dt)
        if self.upper is not None:
            self._upper.setup(dt, -d, -sa, -sb)
            self._upper_bias = _limit_bias(self.upper - value, dt)

    def solve(self) -> None:
        if self.motor_speed is not None:
            self._motor.solve(-self.motor_speed, -self._motor_limit, self._motor_limit)
        if self.lower is not None:
            self._lower.solve(self._lower_bias, lo=0.0)
        if self.upper is not None:
            self._upper.solve(self._upper_bias, lo=0.0)


class RevoluteJoint(Joint):
    """Pins `a` and `b` together at the world point `anchor`; they still turn freely.

    `angle` is b's turn relative to a since the joint was made. `lower` /
    `upper` (radians) stop it; with `motor_speed` (rad/s) set, b turns at that
    speed using at most `max_motor_torque`.
    """

    def __init__(self, a: Body, b: Body, anchor, *, lower: float | None = None, upper: float | None = None,
                 motor_speed: float | None = None, max_motor_torque: float = math.inf,
                 collide_connected: bool = False):
        super().__init__(a, b, anchor, anchor, collide_connected)
        self.reference = b.angle - a.angle
        self.limits = _Limits(a, b, lower, upper, motor_speed, max_motor_torque)
        self._total = Vec2.zero()

    @property
    def angle(self) -> float:
        return self.b.angle - self.a.angle - self.reference

    def prestep(self, dt: float) -> None:
        a, b = self.a, self.b
        self._ra, self._rb = ra, rb = self._arms()
        self._k = _point_matrix((a, ra), (b, rb))
        self._bias = ((b.pos + rb) - (a.pos + ra)) * (-BAUMGARTE / dt)
        a._push(ra, -self._total)
        b._push(rb, self._total)
        self.limits.prestep(dt, self.angle, _TURN, 1.0, 1.0)

    def solve(self) -> None:
        a, b = self.a, self.b
        self.limits.solve()
        dv = b._velocity(self._rb) - a._velocity(self._ra)
        impulse = _solve2(self._k, self._bias - dv)
        self._total += impulse
        a._push(self._ra, -impulse)
        b._push(self._rb, impulse)


class WeldJoint(RevoluteJoint):
    """Glues `a` and `b` together at `anchor`: a revolute joint with both stops at 0."""

    def __init__(self, a: Body, b: Body, anchor, *, collide_connected: bool = False):
        super().__init__(a, b, anchor, lower=0.0, upper=0.0, collide_connected=collide_connected)


class DistanceJoint(Joint):
    """Keeps `anchor_a` on a and `anchor_b` on b at `length` (default: as made).

    `hertz > 0` turns the rod into a spring with that natural frequency.
    """

    def __init__(self, a: Body, b: Body, anchor_a, anchor_b, *, length: float | None = None,
                 hertz: float = 0.0, damping_ratio: float = 0.0, collide_connected: bool = False):
        super().__init__(a, b, anchor_a, anchor_b, collide_connected)
        self.length = (Vec2(*anchor_b) - Vec2(*anchor_a)).length() if length is None else length
        self.hertz, self.damping_ratio = hertz, damping_ratio
        self._row = _Row(a, b)

    def prestep(self, dt: float) -> None:
        a, b = self.a, self.b
        ra, rb = self._arms()
        gap = (b.pos + rb) - (a.pos + ra)
        distance = gap.length()
        n = gap / distance if distance > 0 else Vec2(1, 0)
        rate = self._row.setup(dt, n, ra.cross(n), rb.cross(n), self.hertz, self.damping_ratio)
        self._bias = rate * (distance - self.length)

    def solve(self) -> None:
        self._row.solve(self._bias)


class PrismaticJoint(Joint):
    """`b` slides along `axis` (fixed in a's frame) through the world point `anchor`, without turning.

    `translation` is b's travel along the axis since the joint was made.
    `lower` / `upper` stop it; with `motor_speed` set, b slides at that speed
    using at most `max_motor_force`.
    """

    def __init__(self, a: Body, b: Body, anchor, axis, *, lower: float | None = None,
                 upper: float | None = None, motor_speed: float | None = None,
                 max_motor_force: float = math.inf, collide_connected: bool = False):
        super().__init__(a, b, anchor, anchor, collide_connected)
        self.local_axis = Vec2(*axis).normalized().rotate(-a.angle)
        self.reference = b.angle - a.angle
        self.limits = _Limits(a, b, lower, upper, motor_speed, max_motor_force)
        self._perp, self._turn = _Row(a, b), _Row(a, b)

    @property
    def translation(self) -> float:
        start, end = self.anchors()
        return (end - start).dot(self.local_axis.rotate(self.a.angle))

    def prestep(self, dt: float) -> None:
        a, b = self.a, self.b
        ra, rb = self._arms()
        gap = (b.pos + rb) - (a.pos + ra)
        axis = self.local_axis.rotate(a.angle)
        perp = axis.perp()
        rate = self._perp.setup(dt, perp, (gap + ra).cross(perp), rb.cross(perp))
        self._perp_bias = rate * gap.dot(perp)
        rate = self._turn.setup(dt, _TURN, 1.0, 1.0)
        self._turn_bias = rate * (b.angle - a.angle - self.reference)
        self.limits.prestep(dt, gap.dot(axis), axis, (gap + ra).cross(axis), rb.cross(axis))

    def solve(self) -> None:
        self.limits.solve()
        self._turn.solve(self._turn_bias)
        self._perp.solve(self._perp_bias)


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
