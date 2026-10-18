import math
import os

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

import pygame

from collide import point_in_convex, point_in_triangle
from gmath import Vec2
from main import HEIGHT, WIDTH, Camera, Entity, RigidBody

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
