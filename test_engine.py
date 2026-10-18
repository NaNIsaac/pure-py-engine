import math
import os

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

from itertools import combinations

import pygame
import pytest

from collide import (
    Contact,
    circle_polygon_collide,
    circles_collide,
    point_in_convex,
    point_in_triangle,
    polygons_collide,
)
from gmath import Vec2, polygon_centroid, polygon_signed_area, rectangle
from main import HEIGHT, WIDTH, Camera, Entity, RigidBody, contact, make_scene, resolve

# equilateral, side 2, centroid at origin, counter-clockwise
TRIANGLE = [Vec2(-1, -math.sqrt(3) / 3), Vec2(1, -math.sqrt(3) / 3), Vec2(0, math.sqrt(3) * 2 / 3)]


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
    assert math.isclose(1 / RigidBody(1, 1, vertices=TRIANGLE).inv_inertia, 4 / 12)


def test_vertices_follow_angle_back_to_zero():
    body = RigidBody(1, 1, vertices=TRIANGLE)
    body.set_angle(90)
    body.update(1, 1)
    body.set_angle(0)
    body.update(1, 1)
    assert math.isclose(body.vertices[0].x, TRIANGLE[0].x)


def test_entity_hit_test_in_screen_space():
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    camera = Camera(0, 0, WIDTH, HEIGHT)
    center = Vec2(WIDTH / 2, HEIGHT / 2)
    for body in (RigidBody(2, 1), RigidBody(1, 1, vertices=TRIANGLE)):
        entity = Entity(body, screen)
        entity.update_viewport_shape(camera)
        assert entity.is_inside(center)
        assert not entity.is_inside(Vec2(0, 0))


def test_polygon_normalized_ccw_and_centered():
    shifted_cw = [Vec2(v.x + 7, v.y - 3) for v in reversed(TRIANGLE)]
    body = RigidBody(1, 1, vertices=shifted_cw)
    assert polygon_signed_area(body.vertices) > 0
    c = polygon_centroid(body.vertices)
    assert math.isclose(c.x, 0, abs_tol=1e-12) and math.isclose(c.y, 0, abs_tol=1e-12)
    assert math.isclose(body.inv_inertia, 3.0)
    with pytest.raises(ValueError):
        RigidBody(1, 1, vertices=[Vec2(0, 0), Vec2(2, 0), Vec2(1, 0.2), Vec2(1, 2)])
    with pytest.raises(ValueError):
        RigidBody(1, 1, vertices=[Vec2(0, 0), Vec2(1, 0), Vec2(2, 0)])


def test_initial_angle_applied():
    body = RigidBody(1, 1, angle=90, vertices=rectangle(4, 2))
    assert math.isclose(body.get_aabb().max_y, 2)


def square(x, y):
    return [Vec2(x - 1, y - 1), Vec2(x + 1, y - 1), Vec2(x + 1, y + 1), Vec2(x - 1, y + 1)]


def test_sat():
    assert polygons_collide(square(0, 0), square(1.5, 0)) == Contact(Vec2(1, 0), 0.5)
    assert polygons_collide(square(0, 0), square(2.5, 0)) is None
    assert polygons_collide(square(1.5, 0.2), square(0, 0)).normal == Vec2(-1, 0)
    assert circle_polygon_collide(Vec2(0, 1.5), 1, square(0, 0)) == Contact(Vec2(0, -1), 0.5)
    assert circle_polygon_collide(Vec2(2, 2), 1, square(0, 0)) is None  # corner gap: sqrt(2) - 1
    hit = circle_polygon_collide(Vec2(1.6, 1.6), 1, square(0, 0))
    assert math.isclose(hit.depth, 1 - 0.6 * math.sqrt(2))
    assert circles_collide(Vec2(0, 0), 1, Vec2(1.5, 0), 1) == Contact(Vec2(1, 0), 0.5)


def moving(body, x, vx):
    body.set_position(x, 0)
    body.set_velocity(vx, 0)
    return body


def test_resolve_elastic_head_on():
    a, b = moving(RigidBody(1, 1), -0.9, 3), moving(RigidBody(1, 1), 0.9, -1)
    assert resolve(a, b, restitution=1)
    assert math.isclose(a.v.x, -1) and math.isclose(b.v.x, 3)
    assert contact(a, b) is None


def test_resolve_conserves_momentum():
    a = moving(RigidBody(1, 2, vertices=rectangle(2, 2)), 0, 2)
    b = moving(RigidBody(1, 1), 1.8, -4)
    resolve(a, b)
    assert math.isclose(2 * a.v.x + b.v.x, 2 * 2 + -4, abs_tol=1e-12)


def test_static_wall_does_not_move():
    wall = RigidBody(0, math.inf, vertices=rectangle(2, 10))
    ball = moving(RigidBody(1, 1), 1.5, -5)
    resolve(wall, ball, restitution=1)
    assert wall.pos == Vec2(0, 0) and wall.v == Vec2(0, 0)
    assert math.isclose(ball.v.x, 5)


def test_scene_stays_inside_walls():
    entities = make_scene(pygame.Surface((1, 1)))
    bodies = [e.body for e in entities if e.body.inv_mass > 0]
    for _ in range(600):
        for e in entities:
            e.body.update(1 / 60, 3)
        for a, b in combinations(entities, 2):
            resolve(a.body, b.body)
    assert all(abs(b.pos.x) < 45 and abs(b.pos.y) < 32 for b in bodies)
