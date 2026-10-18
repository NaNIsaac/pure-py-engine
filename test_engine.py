import math
import os

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

from itertools import combinations

import pytest

from engine import (
    Body,
    Contact,
    DistanceJoint,
    MouseJoint,
    PrismaticJoint,
    RevoluteJoint,
    Vec2,
    WeldJoint,
    World,
)
from engine.collide import (
    circle_polygon_collide,
    circles_collide,
    point_in_convex,
    point_in_triangle,
    polygons_collide,
)
from engine.contact import collide
from engine.gmath import polygon_centroid, polygon_signed_area, rectangle
from main import HEIGHT, WIDTH, Camera, make_scene

# equilateral, side 2, centroid at origin, counter-clockwise
TRIANGLE = [Vec2(-1, -math.sqrt(3) / 3), Vec2(1, -math.sqrt(3) / 3), Vec2(0, math.sqrt(3) * 2 / 3)]


def run(world: World, seconds: float, dt: float = 1 / 60) -> None:
    for _ in range(round(seconds / dt)):
        world.step(dt)


def test_point_in_triangle():
    assert point_in_triangle(Vec2(0, 0), *TRIANGLE)
    assert not point_in_triangle(Vec2(5, 5), *TRIANGLE)
    assert not point_in_triangle(Vec2(0.9, 0.5), *TRIANGLE)


def test_point_in_convex():
    square = [Vec2(0, 0), Vec2(2, 0), Vec2(2, 2), Vec2(0, 2)]
    assert point_in_convex(Vec2(1, 1), square)
    assert not point_in_convex(Vec2(3, 1), square)
    assert not point_in_convex(Vec2(1, -0.1), square)


def test_inertia_matches_equilateral_formula():
    # I = m * s^2 / 12 about the centroid
    assert math.isclose(1 / Body.polygon(TRIANGLE).inv_inertia, 4 / 12)


def test_world_vertices_follow_pose():
    body = Body.polygon(TRIANGLE)
    body.angle = math.pi / 2
    assert math.isclose(body.vertices[0].y, TRIANGLE[0].x)
    body.angle = 0
    body.pos = Vec2(5, 0)
    assert math.isclose(body.vertices[0].x, TRIANGLE[0].x + 5)
    assert math.isclose(body.aabb.min_x, 4)


def test_contains_and_camera_round_trip():
    camera = Camera(3, -2, WIDTH, HEIGHT)
    point = Vec2(1.5, 2.5)
    assert camera.to_world(camera.to_screen(point)) == point
    for body in (Body.circle(2), Body.polygon(TRIANGLE)):
        assert body.contains((0, 0))
        assert not body.contains((5, 5))


def test_polygon_normalized_ccw_and_centered():
    shifted_cw = [Vec2(v.x + 7, v.y - 3) for v in reversed(TRIANGLE)]
    body = Body.polygon(shifted_cw)
    assert polygon_signed_area(body.local_vertices) > 0
    c = polygon_centroid(body.local_vertices)
    assert math.isclose(c.x, 0, abs_tol=1e-12) and math.isclose(c.y, 0, abs_tol=1e-12)
    assert math.isclose(body.inv_inertia, 3.0)
    with pytest.raises(ValueError):
        Body.polygon([(0, 0), (2, 0), (1, 0.2), (1, 2)])
    with pytest.raises(ValueError):
        Body.polygon([(0, 0), (1, 0), (2, 0)])


def test_initial_angle_applied():
    assert math.isclose(Body.box(4, 2, angle=math.pi / 2).aabb.max_y, 2)


def square(x, y):
    return [Vec2(x - 1, y - 1), Vec2(x + 1, y - 1), Vec2(x + 1, y + 1), Vec2(x - 1, y + 1)]


def test_sat():
    assert polygons_collide(square(0, 0), square(1.5, 0)) == Contact(Vec2(1, 0), 0.5, (Vec2(0.5, 1), Vec2(0.5, -1)), (0.5, 0.5))
    assert polygons_collide(square(0, 0), square(2.5, 0)) is None
    assert polygons_collide(square(1.5, 0.2), square(0, 0)).normal == Vec2(-1, 0)
    assert circle_polygon_collide(Vec2(0, 1.5), 1, square(0, 0)) == Contact(Vec2(0, -1), 0.5, (Vec2(0, 0.5),), (0.5,))
    assert circle_polygon_collide(Vec2(2, 2), 1, square(0, 0)) is None  # corner gap: sqrt(2) - 1
    hit = circle_polygon_collide(Vec2(1.6, 1.6), 1, square(0, 0))
    assert math.isclose(hit.depth, 1 - 0.6 * math.sqrt(2))
    assert circles_collide(Vec2(0, 0), 1, Vec2(1.5, 0), 1) == Contact(Vec2(1, 0), 0.5, (Vec2(1, 0),), (0.5,))


def test_contact_points_clip_to_overlap():
    # half-overlapping faces: the incident edge is clipped to the reference edge
    hit = polygons_collide(square(0, 0), square(1.8, 1))
    assert sorted(p.to_tuple() for p in hit.points) == [(0.8, 0), (0.8, 1)]
    # corner into a face: a single point at the corner
    corner = [Vec2(2.3, 0.5) + v.rotate(math.pi / 4) for v in rectangle(2, 2)]
    (point,) = polygons_collide(square(0, 0), corner).points
    assert math.isclose(point.x, 2.3 - math.sqrt(2)) and math.isclose(point.y, 0.5)


def test_elastic_head_on():
    world = World()
    a = world.add(Body.circle(1, pos=(-0.9, 0), v=(3, 0), restitution=1))
    b = world.add(Body.circle(1, pos=(0.9, 0), v=(-1, 0), restitution=1))
    world.step(1 / 60)
    assert math.isclose(a.v.x, -1) and math.isclose(b.v.x, 3)


def test_momentum_conserved():
    world = World()
    a = world.add(Body.box(2, 2, mass=2, v=(2, 0)))
    b = world.add(Body.circle(1, pos=(1.8, 0.3), v=(-4, 0)))
    world.step(1 / 60)
    momentum = a.v * 2 + b.v
    assert math.isclose(momentum.x, 0, abs_tol=1e-12) and math.isclose(momentum.y, 0, abs_tol=1e-12)


def test_static_wall_does_not_move():
    world = World()
    wall = world.add(Body.box(2, 10, static=True))
    ball = world.add(Body.circle(1, pos=(1.5, 0), v=(-5, 0), restitution=1))
    world.step(1 / 60)
    assert wall.pos == Vec2(0, 0) and wall.v == Vec2(0, 0)
    assert math.isclose(ball.v.x, 5)


def corner_hit(friction=0.3, restitution=0.5):
    """A diamond's corner strikes the upper half of a resting square's right face."""
    world = World()
    a = world.add(Body.box(2, 2, friction=friction, restitution=restitution))
    b = world.add(Body.box(2, 2, mass=2, angle=math.pi / 4, pos=(2.3, 0.5), v=(-4, 1),
                           friction=friction, restitution=restitution))
    world.step(1 / 60)
    return a, b


def test_corner_hit_spins():
    a, b = corner_hit()
    assert a.w > 0  # pushed left above its center: counter-clockwise
    assert b.w != 0  # friction along the face torques the diamond


def test_corner_hit_conserves_momentum_and_energy():
    a, b = corner_hit(friction=0, restitution=1)
    ma, mb, ia, ib = 1, 2, 1 / a.inv_inertia, 1 / b.inv_inertia

    def totals(pa, va, wa, pb, vb, wb):
        momentum = va * ma + vb * mb
        spin = ma * pa.cross(va) + ia * wa + mb * pb.cross(vb) + ib * wb  # about the origin
        energy = (ma * va.square() + ia * wa * wa + mb * vb.square() + ib * wb * wb) / 2
        return momentum.x, momentum.y, spin, energy

    assert a.w != 0
    # free flight keeps pos x v constant, so the start pose pairs with the start velocities
    before = totals(Vec2(0, 0), Vec2(0, 0), 0, Vec2(2.3, 0.5), Vec2(-4, 1), 0)
    after = totals(a.pos, a.v, a.w, b.pos, b.v, b.w)
    assert all(math.isclose(x, y, abs_tol=1e-9) for x, y in zip(before, after))


def test_broadphase_matches_brute_force():
    world = World()
    for k in range(40):
        x, y = (k * 7.3) % 20, (k * 3.1) % 12
        world.add(Body.box(2, 1.5, pos=(x, y), angle=k) if k % 2 else Body.circle(1, pos=(x, y)))
    world._collide(1 / 60)
    brute = {frozenset(p) for p in combinations(world.bodies, 2) if collide(*p) is not None}
    assert {frozenset(p) for p in world.arbiters} == brute
    assert len(brute) > 10


def floor_world(**kwargs) -> World:
    world = World(gravity=(0, -10), **kwargs)
    world.add(Body.box(40, 2, pos=(0, -1), static=True))
    return world


def test_stack_rests_and_sleeps():
    world = floor_world()
    boxes = [world.add(Body.box(2, 2, pos=(0, 1 + 2 * k))) for k in range(5)]
    run(world, 3)
    for k, box in enumerate(boxes):
        assert abs(box.pos.y - (1 + 2 * k)) < 0.05 and abs(box.pos.x) < 0.01
    assert not any(box.awake for box in boxes)


def test_sleeping_body_wakes_when_hit_or_pushed():
    world = floor_world()
    box = world.add(Body.box(2, 2, pos=(0, 1)))
    run(world, 1)
    assert not box.awake
    box.apply_impulse((5, 0))
    assert box.awake and box.v.x == 5
    run(world, 2)
    assert not box.awake
    ball = world.add(Body.circle(0.5, pos=(box.pos.x, 6)))
    woke = False
    for _ in range(120):
        world.step(1 / 60)
        woke |= box.awake
    assert woke
    assert abs(ball.pos.y - 2.5) < 0.05  # landed on the box instead of passing through


def test_sleep_can_be_disabled():
    world = floor_world()
    world.time_to_sleep = math.inf
    box = world.add(Body.box(2, 2, pos=(0, 1)))
    run(world, 2)
    assert box.awake


def test_revolute_pendulum_keeps_its_pin():
    world = World(gravity=(0, -10))
    ground = world.add(Body.circle(0.1, static=True))
    bob = world.add(Body.box(0.5, 0.5, pos=(3, 0)))
    joint = world.add(RevoluteJoint(ground, bob, (2, 0)))
    for _ in range(300):
        world.step(1 / 60)
        pin_a, pin_b = joint.anchors()
        assert pin_a.dist_square(pin_b) < 0.02 ** 2
    assert bob.pos.dist_square(Vec2(3, 0)) > 0.1  # it swung


def test_revolute_motor_reaches_speed():
    world = World()
    hub = world.add(Body.circle(0.1, static=True))
    blade = world.add(Body.box(6, 0.5))
    world.add(RevoluteJoint(hub, blade, (0, 0), motor_speed=2, max_motor_torque=100))
    run(world, 1)
    assert math.isclose(blade.w, 2, rel_tol=1e-6)
    assert blade.pos.dist_square(Vec2(0, 0)) < 1e-6


def test_weld_holds_shape_on_impact():
    world = floor_world()
    a = world.add(Body.box(4, 1, pos=(0, 5)))
    b = world.add(Body.box(1, 3, pos=(-1.5, 7)))
    world.add(WeldJoint(a, b, (-1.5, 5)))
    offset, turn = a.to_local(b.pos), b.angle - a.angle
    run(world, 2)
    assert a.to_local(b.pos).dist_square(offset) < 0.05 ** 2
    assert abs(b.angle - a.angle - turn) < 0.02


def test_distance_rod_and_spring():
    world = World(gravity=(0, -10))
    ground = world.add(Body.circle(0.1, static=True))
    rod_end = world.add(Body.circle(0.3, pos=(4, 0)))
    rod = world.add(DistanceJoint(ground, rod_end, (0, 0), (4, 0)))
    run(world, 2)
    assert abs((rod_end.pos - ground.pos).length() - rod.length) < 0.02

    # undamped 1 Hz spring stretched by 1: half a period later it is compressed by about 1
    world = World()
    ground = world.add(Body.circle(0.1, static=True))
    bob = world.add(Body.circle(0.3, pos=(5, 0)))
    world.add(DistanceJoint(ground, bob, (0, 0), (5, 0), length=4, hertz=1))
    run(world, 0.5, dt=1 / 240)
    assert -1.05 < bob.pos.x - 4 < -0.85


def test_prismatic_stays_on_rail():
    world = World(gravity=(0, -10))
    ground = world.add(Body.circle(0.1, static=True))
    slider = world.add(Body.box(1, 1, pos=(0, 0)))
    world.add(PrismaticJoint(ground, slider, (0, 0), (1, -1)))
    run(world, 1)
    assert abs(slider.pos.x + slider.pos.y) < 0.01  # on the line y = -x
    assert slider.pos.x > 1  # slid down it
    assert abs(slider.angle) < 0.01


def test_mouse_joint_drags_body_to_target():
    world = World(gravity=(0, -10))
    body = world.add(Body.box(1, 1))
    drag = world.add(MouseJoint(body, (0.5, 0)))
    drag.target = (5, 3)
    run(world, 2)
    assert body.to_world(drag.local_b).dist_square(Vec2(5, 3)) < 0.1 ** 2  # soft: sags a little


def test_scene_stays_inside_walls():
    world, _, _ = make_scene()
    run(world, 10)
    assert all(abs(b.pos.x) < 45 and abs(b.pos.y) < 32 for b in world.bodies if not b.is_static)


@pytest.mark.parametrize("bullet", [Body.circle(0.2, restitution=1), Body.box(0.4, 0.4, restitution=1)])
def test_fast_body_does_not_tunnel(bullet):
    # 300 units/s moves 5 per step, 25x the wall's thickness
    world = World()
    world.add(Body.box(0.2, 10, static=True))
    bullet.pos, bullet.v = Vec2(-3, 0), Vec2(300, 0)
    world.add(bullet)
    run(world, 0.5)
    assert bullet.pos.x < 0
    assert bullet.v.x < -290  # bounced back at about the speed it came in


def test_tunnels_without_reach():
    """The same shot with speculative reach switched off passes through: the check above is not vacuous."""
    world = World()
    world.add(Body.box(0.2, 10, static=True))
    bullet = world.add(Body.circle(0.2, pos=(-3, 0), v=(300, 0)))
    step_collide = world._collide
    world._collide = lambda dt: step_collide(0.0)  # zero dt: zero reach
    run(world, 0.5)
    assert bullet.pos.x > 0


def test_revolute_limits_stop_a_swing():
    world = World(gravity=(0, -10))
    ground = world.add(Body.circle(0.1, static=True))
    arm = world.add(Body.box(4, 0.4, pos=(2, 0)))
    hinge = world.add(RevoluteJoint(ground, arm, (0, 0), lower=-math.pi / 4, upper=math.pi / 4))
    lowest = 0.0
    for _ in range(180):
        world.step(1 / 60)
        lowest = min(lowest, hinge.angle)
    assert -math.pi / 4 - 0.02 < lowest
    assert abs(hinge.angle + math.pi / 4) < 0.02  # resting on the lower stop


def test_prismatic_limits_and_motor():
    world = World(gravity=(0, -10))
    ground = world.add(Body.circle(0.1, static=True))
    slider = world.add(Body.box(1, 1))
    rail = world.add(PrismaticJoint(ground, slider, (0, 0), (0, 1), lower=-2, upper=3))
    run(world, 2)
    assert abs(rail.translation + 2) < 0.02  # fell onto the lower stop

    rail.limits.motor_speed, rail.limits.max_motor = 4, 100
    slider.wake()
    run(world, 0.25)
    assert math.isclose(slider.v.y, 4, rel_tol=1e-6)  # motor speed despite gravity
    run(world, 2)
    assert abs(rail.translation - 3) < 0.02  # driven up to the upper stop
