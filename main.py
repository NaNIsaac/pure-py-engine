from re import X
import math
from dataclasses import dataclass

import pygame
from pygame.typing import ColorLike

from gmath import AABB, Vec2
from collide import point_in_convex

SCREEN_SHAPE = 1024, 768
WIDTH, HEIGHT = SCREEN_SHAPE
BACKGROUND_COLOR = "#1B1B1B"

class RigidBody:
    def __init__(
            self, 
            size: float,
            mass: float,
            angle: float = 0,
            vertices: list[Vec2] | None = None
        ):

        self.size: float = size
        self.default_vertices: list[Vec2] | None = vertices
        self.vertices: list[Vec2] | None = vertices
        self.angle: float = angle
        self.pos: Vec2 = Vec2.zero()
        self.v: Vec2 = Vec2.zero()
        self.w: float = 0

        self.inv_mass: float = 1 / mass
        self.inv_inertia: float = 1 / self._calc_inertia(1 / self.inv_mass, vertices)

        self.impulse: Vec2 = Vec2.zero()
        self.angular_impulse: float = 0

    def _calc_inertia(self, mass: float, vertices: list[Vec2] | None) -> float:
        if vertices is None:
            return 0.5 * mass * self.size * self.size

        numerator: float = 0
        denominator: float = 0
        for i, _ in enumerate(vertices):
            a: Vec2 = vertices[i]
            b: Vec2 = vertices[(i + 1) % len(vertices)]
            q: float = a.square() + a.dot(b) + b.square()
            cross_ab = abs(a.cross(b))
            numerator += q * cross_ab
            denominator += cross_ab

        return mass * numerator / denominator / 6

    def get_aabb(self) -> AABB:
        if self.vertices is None:
            return AABB.from_circle(self.pos, self.size)
        return AABB.from_vertices(self.vertices)

    def set_angle(self, angle: float):
        self.angle: float = angle

    def set_angular_velocity(self, angular_velocity: float):
        self.w: float = angular_velocity

    def set_position(self, x: float, y: float):
        self.pos = Vec2(x, y)
    
    def set_valocity(self, vx: float, vy: float):
        self.v = Vec2(vx, vy)

    def apply_impulse(self, impulse: Vec2):
        self.impulse += impulse

    def apply_angular_impulse(self, angular_impulse: float):
        self.angular_impulse += angular_impulse

    def _update_vertices(self):
        if self.default_vertices is None:
            return
        if self.angle == 0:
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


class Entity:
    def __init__(self, body: RigidBody, screen: pygame.surface.Surface):
        self.body: RigidBody = body
        self.screen: pygame.surface.Surface = screen
        self.mouse_on = False

        self.screen_width: float = WIDTH
        self.screen_height: float = HEIGHT

        self.screen_pos: Vec2 = Vec2.zero()
        self.screen_radius: float = 0
        self.screen_vertices: list[Vec2] | None = None

    def is_inside(self, point: Vec2):
        if not self.body.get_aabb().is_point_inside(point):
            return False

        if self.body.vertices is None:
            return self.screen_pos.dist_squre(point) <= self.screen_radius

    def draw(self, camera: Camera, color: ColorLike = 0) -> None:

        if self.body.vertices is None:

            x = camera.zoom * (self.body.pos.x - camera.x) + WIDTH * 0.5
            y = camera.zoom * -(self.body.pos.y - camera.y) + HEIGHT * 0.5
            x, y = int(x), int(y)
            radius = camera.zoom * self.body.size
            pygame.draw.circle(self.screen, color, (x, y), radius)

            return
        def vertex_function(vertex: Vec2) -> Vec2:
            vertex = Vec2(
                camera.zoom * (self.body.pos.x + vertex.x - camera.x) + WIDTH * 0.5,
                camera.zoom * -(self.body.pos.y + vertex.y - camera.y) + HEIGHT * 0.5
            )
            return vertex
        screen_vertices: map[Vec2] = map(vertex_function, self.body.vertices)
        # Draw
        points: list[tuple[float, float]] = [vertex.to_tuple() for vertex in screen_vertices]
        pygame.draw.polygon(self.screen, color, points)

    def is_camera_collide(self, camera: Camera):
        return camera.aabb.is_overlap(self.body.get_aabb())

    def react(self) -> None:
        if pygame.mouse.get_pressed()[0]:
            mouse_pos = Vec2(*pygame.mouse.get_pos())
            print(point_in_convex(mouse_pos, self.screen_vertices))
            if point_in_convex(mouse_pos, self.screen_vertices):
                points: list[tuple[float, float]] = [vertex.to_tuple() for vertex in screen_vertices]
                pygame.draw.polygon(self.screen, (255, 255, 255), points, width=1)
    
    def update_viewport_shape(self, camera):
        if self.body.vertices is None:
            x = camera.zoom * (self.body.pos.x - camera.x) + WIDTH * 0.5
            y = camera.zoom * -(self.body.pos.y - camera.y) + HEIGHT * 0.5
            x, y = int(x), int(y)

            self.screen_pos = Vec2(x, y)
            self.radius = camera.zoom * self.body.size
            return

        self.screen_vertices: list[Vec2] = [
            Vec2(
                camera.zoom * (self.body.pos.x + vertex.x - camera.x) + WIDTH * 0.5,
                camera.zoom * -(self.body.pos.y + vertex.y - camera.y) + HEIGHT * 0.5
            ) for vertex in self.body.vertices
        ]

    def update(self, dt: float, dt_times: int, camera) -> None:
        if self.is_camera_collide(camera):
            self.update_viewport_shape(camera)
            self.react()
        self.body.update(dt, dt_times)


def main():
    pygame.init()
    clock = pygame.time.Clock()
    screen = pygame.display.set_mode(SCREEN_SHAPE)

    body = RigidBody(10, 1, 30,
        [
            Vec2(-1, -math.sqrt(3) / 3),
            Vec2(1, -math.sqrt(3) / 3),
            Vec2(0, math.sqrt(3) * 2 / 3)
        ]
    )
    entity = Entity(body, screen)
    camera = Camera(0, 0, WIDTH, HEIGHT, zoom=10)
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
            
            # if event.type == pygame.KEYDOWN:
            #     if event.key == pygame.K_SPACE:
                    
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
            entity.body.apply_angular_impulse(1)

        dt = clock.tick(60) / 1000.0
        split_dt_factor = 3
        entity.update(dt, split_dt_factor, camera)
        # print(f"{entity.body.angle}\r", end="")
        screen.fill(BACKGROUND_COLOR)
        # Draw objects
        entity.draw(camera, "blue")
        pygame.display.flip()


if __name__ == "__main__":
    main()
