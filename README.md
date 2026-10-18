# pure-py-engine

A 2D rigid-body physics engine in plain Python. The `engine` package has no dependencies besides `seltools`; the pygame-ce demo lives in `main.py`.

- Shapes: circles and convex polygons, separating-axis collision with Box2D-style edge clipping (up to two contact points, each with its own depth).
- Solver: sequential impulses with accumulated clamping, warm started from the previous step; Coulomb friction, restitution above an impact-speed threshold, Baumgarte position correction.
- Continuous collision: speculative contacts. Each awake body's reach this step (speed, spin, gravity) widens its broadphase box and the SAT margin, so a fast body is stopped at the first surface instead of tunnelling.
- Joints: `RevoluteJoint` and `PrismaticJoint` (optional `lower` / `upper` stops and motor), `WeldJoint`, `DistanceJoint` (rigid, or a spring with `hertz` / `damping_ratio`), `MouseJoint`.
- Performance: sweep-and-prune broadphase, world vertices and AABBs cached until a body moves, sleeping islands.

Status: experiment. Speculative contacts can stop a body a fraction of a step early; distance joints have no min / max length.

## Requirements

Python 3.14+, pygame-ce (demo only), `../seltools` (path dependency).

## Usage

```python
from engine import Body, RevoluteJoint, World

world = World(gravity=(0, -10))
ground = world.add(Body.box(40, 2, pos=(0, -1), static=True))
crate = world.add(Body.box(2, 2, pos=(0, 5), angle=0.3))
ball = world.add(Body.circle(0.5, pos=(4, 3), restitution=0.8))
world.add(RevoluteJoint(ground, ball, (4, 5)))  # pendulum pinned 2 above the ball

for _ in range(300):
    world.step(1 / 60)
print(f"{crate.pos.y:.2f} {crate.awake}")  # 1.00 False: landed and fell asleep
```

Bodies are built with `Body.circle`, `Body.polygon`, `Body.box` or `Body.regular`, with keyword options `mass`, `pos`, `angle` (radians), `v`, `w`, `friction`, `restitution`, `static`. Points and vectors may be `Vec2` or `(x, y)`. Polygon vertices can be in either winding and anywhere around the origin; they are stored counter-clockwise around their centroid, and non-convex or zero-area input raises `ValueError`.

Joint anchors are world points at the moment the joint is made; `RevoluteJoint.angle` and `PrismaticJoint.translation` count from that pose, and the stops use the same zero. Jointed bodies do not collide unless `collide_connected=True`.

`World` tuning: `iterations` (default 10), `time_to_sleep` (0.5 s, `math.inf` disables sleeping), `sleep_linear`, `sleep_angular`. `apply_impulse`, `MouseJoint.target` and new joints wake a body.

## Demo

```bash
uv run main.py
```

Loose shapes, a stack that falls asleep (drawn dimmed), a hanging chain, a motorised windmill, a welded L, a spring, and a slider on a diagonal rail with end stops. Drag any body with the left mouse button (a `MouseJoint`), B fires a 400 units/s bullet, Space kicks everything upward, arrow keys pan, mouse wheel zooms.

## Test

```bash
uv run --with pytest pytest
```

## Structure

```
engine/gmath.py      Vec2, AABB, polygon area / centroid / convexity, regular_polygon, rectangle
engine/collide.py    point tests, SAT for polygons and circles, contact point clipping
engine/body.py       Body: shape, mass, cached world vertices / AABB, impulses, sleep state
engine/contact.py    narrowphase dispatch, Arbiter (persistent contact, friction, warm start)
engine/joints.py     revolute, weld, distance, prismatic, mouse joints; limits and motors
engine/world.py      World: broadphase, islands and sleeping, solver loop
main.py              pygame demo: Camera, scene, drawing, input
test_engine.py       geometry, SAT, response, conservation, stacking, sleeping, tunnelling, each joint, the demo scene
```
