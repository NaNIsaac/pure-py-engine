from collections import defaultdict

from .body import Body
from .contact import Arbiter, collide
from .gmath import Vec2
from .joints import Joint


def _pair(a: Body, b: Body) -> tuple[Body, Body]:
    """Stable order, so a pair keeps its arbiter (and normal direction) across steps."""
    return (a, b) if id(a) < id(b) else (b, a)


class World:
    """Bodies and joints stepped together.

    Each step: sweep-and-prune broadphase, SAT narrowphase, island sleeping,
    then `iterations` rounds of sequential impulses, warm started from the
    previous step. Islands whose bodies all stayed below `sleep_linear` and
    `sleep_angular` for `time_to_sleep` seconds stop simulating until touched,
    jointed to something awake, or woken; `time_to_sleep = math.inf` disables it.
    """

    def __init__(self, gravity=(0, 0), *, iterations: int = 10):
        self.gravity = Vec2(*gravity)
        self.iterations = iterations
        self.time_to_sleep = 0.5
        self.sleep_linear = 0.05
        self.sleep_angular = 0.05
        self.bodies: list[Body] = []
        self.joints: list[Joint] = []
        self.arbiters: dict[tuple[Body, Body], Arbiter] = {}
        self._sweep: list[Body] = []

    def add[T: Body | Joint](self, item: T) -> T:
        if isinstance(item, Body):
            self.bodies.append(item)
            self._sweep.append(item)
        else:
            self.joints.append(item)
            for body in item.bodies:
                body.wake()
        return item

    def remove(self, item: Body | Joint) -> None:
        if isinstance(item, Joint):
            self.joints.remove(item)
            for body in item.bodies:
                body.wake()
            return
        self.bodies.remove(item)
        self._sweep.remove(item)
        self.joints = [j for j in self.joints if item not in j.bodies]
        self.arbiters = {k: arb for k, arb in self.arbiters.items() if item not in k}

    def body_at(self, point) -> Body | None:
        """Topmost (last added) body containing the world point."""
        return next((b for b in reversed(self.bodies) if b.contains(point)), None)

    def step(self, dt: float) -> None:
        self._collide(dt)
        self._sleep_islands()

        awake = [b for b in self.bodies if b.awake]
        for b in awake:
            b.v += (self.gravity + b.force * b.inv_mass) * dt
            b.w += b.torque * b.inv_inertia * dt
        for b in self.bodies:
            b.force, b.torque = Vec2.zero(), 0.0

        arbiters = [arb for arb in self.arbiters.values() if arb.a.awake or arb.b.awake]
        joints = [j for j in self.joints if any(b.awake for b in j.bodies)]
        inv_dt = 1 / dt
        for arb in arbiters:
            arb.prestep(inv_dt)
        for joint in joints:
            joint.prestep(dt)
        for arb in arbiters:
            arb.warm_start()
        for _ in range(self.iterations):
            for joint in joints:
                joint.solve()
            for arb in arbiters:
                arb.solve()

        linear, angular = self.sleep_linear**2, self.sleep_angular**2
        for b in awake:
            b.pos += b.v * dt
            b.angle += b.w * dt
            moving = b.v.square() > linear or b.w * b.w > angular
            b.sleep_time = 0.0 if moving else b.sleep_time + dt

    def _collide(self, dt: float) -> None:
        # Speculative contacts stand in for continuous collision: each awake body's
        # box grows by how far its surface can travel this step, and pairs within
        # reach get contacts with negative depth that stop them at first touch.
        fall = self.gravity.length() * dt
        reach = {
            b: (b.v.length() + fall + abs(b.w) * b.extent) * dt if b.awake else 0.0
            for b in self.bodies
        }
        boxes = {b: b.aabb.expanded(r) if r else b.aabb for b, r in reach.items()}
        # bodies barely reorder between steps, so this in-place sort runs in near linear time
        self._sweep.sort(key=lambda b: boxes[b].min_x)
        skip = {
            _pair(*j.bodies) for j in self.joints if not j.collide_connected and len(j.bodies) == 2
        }
        arbiters = {}
        sweep = self._sweep
        for i, a in enumerate(sweep):
            box = boxes[a]
            for j in range(i + 1, len(sweep)):
                b = sweep[j]
                other = boxes[b]
                if other.min_x > box.max_x:
                    break
                if not (a.awake or b.awake) or other.min_y > box.max_y or other.max_y < box.min_y:
                    continue
                pair = _pair(a, b)
                if pair in skip or (hit := collide(*pair, reach[a] + reach[b])) is None:
                    continue
                arbiter = self.arbiters.get(pair) or Arbiter(*pair)
                arbiter.update(hit)
                arbiters[pair] = arbiter
        self.arbiters = arbiters

    def _sleep_islands(self) -> None:
        """Group dynamic bodies linked by contacts or joints; an island sleeps or wakes as one."""
        parent = {b: b for b in self.bodies if not b.is_static}

        def find(b: Body) -> Body:
            while parent[b] is not b:
                parent[b] = parent[parent[b]]
                b = parent[b]
            return b

        links = [*self.arbiters, *(j.bodies for j in self.joints)]
        for bodies in links:
            dynamic = [b for b in bodies if not b.is_static]
            for b in dynamic[1:]:
                parent[find(b)] = find(dynamic[0])

        islands = defaultdict(list)
        for b in parent:
            islands[find(b)].append(b)
        for island in islands.values():
            if all(b.sleep_time >= self.time_to_sleep for b in island):
                for b in island:
                    b.sleep()
            elif not all(b.awake for b in island):
                for b in island:
                    b.wake()
