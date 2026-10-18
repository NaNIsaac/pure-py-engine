import math
from dataclasses import dataclass
from itertools import combinations

import pygame
from pygame.typing import ColorLike
from seltools import circular_pairwise

from collide import (
    Contact,
    circle_polygon_collide,
    circles_collide,
    point_in_convex,
    polygons_collide,
)
from gmath import AABB, Vec2, centered_convex, rectangle, regular_polygon

SCREEN_SHAPE = 1024, 768
WIDTH, HEIGHT = SCREEN_SHAPE
BACKGROUND_COLOR = "#1B1B1B"

class RigidBody:
    """A circle of radius `size`, or a convex polygon when `vertices` is given.

    Polygon vertices may be in either winding and anywhere around the origin;
    they are stored counter-clockwise around their centroid. `mass=math.inf`
    makes the body static.
    """

    def __init__(
            self,
            size: float,
            mass: float,
            angle: float = 0,
            vertices: list[Vec2] | None = None
        ):

        if vertices is not None:
            vertices = centered_convex(vertices)
        self.size: float = size
        self.default_vertices: list[Vec2] | None = vertices
        self.vertices: list[Vec2] | None = vertices
        self.angle: float = angle
        self.pos: Vec2 = Vec2.zero()
        self.v: Vec2 = Vec2.zero()
        self.w: float = 0

        self.inv_mass: float = 1 / mass
        self.inv_inertia: float = 1 / self._calc_inertia(mass, vertices)

        self.impulse: Vec2 = Vec2.zero()
        self.angular_impulse: float = 0
        self._update_vertices()

    def _calc_inertia(self, mass: float, vertices: list[Vec2] | None) -> float:
        if vertices is None:
            return 0.5 * mass * self.size * self.size

        numerator: float = 0
        denominator: float = 0
        for a, b in circular_pairwise(vertices):
            q: float = a.square() + a.dot(b) + b.square()
            cross_ab = abs(a.cross(b))
            numerator += q * cross_ab
            denominator += cross_ab

        return mass * numerator / denominator / 6

    def world_vertices(self) -> list[Vec2]:
        return [self.pos + vertex for vertex in self.vertices]

    def get_aabb(self) -> AABB:
        if self.vertices is None:
            return AABB.from_circle(self.pos, self.size)
        return AABB.from_vertices(self.world_vertices())

    def set_angle(self, angle: float):
        self.angle: float = angle

    def set_angular_velocity(self, angular_velocity: float):
        self.w: float = angular_velocity

    def set_position(self, x: float, y: float):
        self.pos = Vec2(x, y)

    def set_velocity(self, vx: float, vy: float):
        self.v = Vec2(vx, vy)

    def apply_impulse(self, impulse: Vec2):
        self.impulse += impulse

    def apply_angular_impulse(self, angular_impulse: float):
        self.angular_impulse += angular_impulse

    def _update_vertices(self):
        if self.default_vertices is None:
            return
        self.vertices = [vertex.rotate(self.angle) for vertex in self.default_vertices]

    def update(self, dt: float, times: int):

        gravity = Vec2(0, 0)
        sub_dt: float = dt / times

        force: Vec2 = self.impulse / sub_dt + gravity
        torque: float = self.angular_impulse / sub_dt

        for _ in range(times):
            accel: Vec2 = force * self.inv_mass
            self.v: Vec2 = self.v + accel * sub_dt
            self.pos: Vec2 = self.pos + self.v * sub_dt

            ang_accel: float = torque * self.inv_inertia
            self.w: float = self.w + ang_accel * sub_dt
            self.angle: float = (self.angle + self.w * sub_dt) % 360
        self.impulse: Vec2 = gravity
        self.angular_impulse: float = 0
        self._update_vertices()


def contact(a: RigidBody, b: RigidBody) -> Contact | None:
    """Overlap of two bodies; the normal points from `a` to `b`."""
    if not a.get_aabb().is_overlap(b.get_aabb()):
        return None
    match a.vertices, b.vertices:
        case None, None:
            return circles_collide(a.pos, a.size, b.pos, b.size)
        case None, _:
            return circle_polygon_collide(a.pos, a.size, b.world_vertices())
        case _, None:
            hit = circle_polygon_collide(b.pos, b.size, a.world_vertices())
            return None if hit is None else Contact(-hit.normal, hit.depth)
        case _:
            return polygons_collide(a.world_vertices(), b.world_vertices())


def resolve(a: RigidBody, b: RigidBody, restitution: float = 0.5) -> bool:
    """Push overlapping bodies apart and bounce them. True if they touched."""
    # ponytail: linear response only, no friction or spin from contacts; add contact points + angular impulse when rotation must react
    hit = contact(a, b)
    total = a.inv_mass + b.inv_mass
    if hit is None or total == 0:
        return False
    normal, depth = hit
    a.pos -= normal * (depth * a.inv_mass / total)
    b.pos += normal * (depth * b.inv_mass / total)
    closing = (b.v - a.v).dot(normal)
    if closing < 0:
        j = -(1 + restitution) * closing / total
        a.v -= normal * (j * a.inv_mass)
        b.v += normal * (j * b.inv_mass)
    return True


@dataclass
class Camera:
    x: float
    y: float
    width: float
    height: float
    zoom: float = 10

    @property
    def aabb(self) -> AABB:
        real_half_width: float = 0.5 * self.width / self.zoom
        real_half_height: float = 0.5 * self.height / self.zoom
        return AABB(
            self.x - real_half_width,
            self.y - real_half_height,
            self.x + real_half_width,
            self.y + real_half_height
        )

    def to_screen(self, point: Vec2) -> Vec2:
        """World to screen pixels; screen y grows downward."""
        return Vec2(
            self.zoom * (point.x - self.x) + self.width * 0.5,
            self.zoom * -(point.y - self.y) + self.height * 0.5
        )

    def to_world(self, point: Vec2) -> Vec2:
        return Vec2(
            (point.x - self.width * 0.5) / self.zoom + self.x,
            -(point.y - self.height * 0.5) / self.zoom + self.y
        )


class Entity:
    def __init__(self, body: RigidBody, screen: pygame.surface.Surface, color: ColorLike = "blue"):
        self.body: RigidBody = body
        self.screen: pygame.surface.Surface = screen
        self.color: ColorLike = color

        self.screen_pos: Vec2 = Vec2.zero()
        self.screen_radius: float = 0
        self.screen_vertices: list[Vec2] | None = None

    def is_inside(self, point: Vec2) -> bool:
        """Hit test in screen space, against the last update_viewport_shape."""
        if self.body.vertices is None:
            return self.screen_pos.dist_square(point) <= self.screen_radius ** 2
        # the y-flip to screen space turns the counter-clockwise vertices clockwise
        return point_in_convex(point, self.screen_vertices[::-1])

    def is_camera_collide(self, camera: Camera):
        return camera.aabb.is_overlap(self.body.get_aabb())

    def update_viewport_shape(self, camera: Camera) -> None:
        self.screen_pos = camera.to_screen(self.body.pos)
        self.screen_radius = camera.zoom * self.body.size
        if self.body.vertices is not None:
            self.screen_vertices = [camera.to_screen(vertex) for vertex in self.body.world_vertices()]

    def draw(self, color: ColorLike | None = None, width: int = 0) -> None:
        """Filled by default; width > 0 draws only the outline."""
        color = self.color if color is None else color
        if self.body.vertices is None:
            pygame.draw.circle(self.screen, color, self.screen_pos.to_tuple(), self.screen_radius, width)
        else:
            pygame.draw.polygon(self.screen, color, [v.to_tuple() for v in self.screen_vertices], width)


def make_scene(screen: pygame.surface.Surface) -> list[Entity]:
    """Four static walls around a 90 x 64 box and a few moving shapes."""
    walls = [
        ((0, -33), rectangle(92, 2)),
        ((0, 33), rectangle(92, 2)),
        ((-46, 0), rectangle(2, 68)),
        ((46, 0), rectangle(2, 68)),
    ]
    entities = []
    for (x, y), vertices in walls:
        body = RigidBody(0, math.inf, vertices=vertices)
        body.set_position(x, y)
        entities.append(Entity(body, screen, "#555555"))

    shapes = [
        (RigidBody(4, 1, vertices=regular_polygon(3, 5)), (-25, 10), (20, -5), "#4C72B0"),
        (RigidBody(4, 2, vertices=rectangle(8, 6)), (20, 5), (-15, 10), "#DD8452"),
        (RigidBody(4, 1), (0, -15), (5, 25), "#55A868"),
        (RigidBody(4, 1.5, vertices=regular_polygon(5, 5)), (-10, 20), (-10, -20), "#C44E52"),
        (RigidBody(3, 0.5), (30, -20), (-25, 15), "#8172B3"),
    ]
    for body, (x, y), (vx, vy), color in shapes:
        body.set_position(x, y)
        body.set_velocity(vx, vy)
        entities.append(Entity(body, screen, color))
    return entities


def main():
    pygame.init()
    clock = pygame.time.Clock()
    screen = pygame.display.set_mode(SCREEN_SHAPE)

    entities = make_scene(screen)
    camera = Camera(0, 0, WIDTH, HEIGHT, zoom=10)
    grabbed: Entity | None = None
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            if event.type == pygame.MOUSEWHEEL:
                if event.y > 0:
                    camera.zoom *= 1.2
                elif event.y < 0:
                    camera.zoom *= 0.8

            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                point = Vec2(*event.pos)
                grabbed = next((e for e in entities if e.body.inv_mass > 0 and e.is_inside(point)), None)
            if event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                grabbed = None

        keys: pygame.key.ScancodeWrapper = pygame.key.get_pressed()
        if keys[pygame.K_LEFT]:
            camera.x -= 5
        if keys[pygame.K_RIGHT]:
            camera.x += 5
        if keys[pygame.K_DOWN]:
            camera.y -= 5
        if keys[pygame.K_UP]:
            camera.y += 5
        if keys[pygame.K_SPACE]:
            for entity in entities:
                entity.body.apply_angular_impulse(1)

        if grabbed is not None:
            # chase the cursor through velocity, so collisions still push back
            target = camera.to_world(Vec2(*pygame.mouse.get_pos()))
            grabbed.body.v = (target - grabbed.body.pos) * 10

        dt = clock.tick(60) / 1000.0
        split_dt_factor = 3
        for entity in entities:
            entity.body.update(dt, split_dt_factor)
        for a, b in combinations(entities, 2):
            resolve(a.body, b.body)

        screen.fill(BACKGROUND_COLOR)
        for entity in entities:
            if entity.is_camera_collide(camera):
                entity.update_viewport_shape(camera)
                entity.draw()
        if grabbed is not None:
            grabbed.draw((255, 255, 255), width=1)
        pygame.display.flip()


if __name__ == "__main__":
    main()
