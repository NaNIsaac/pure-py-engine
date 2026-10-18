from dataclasses import dataclass

import pygame

from engine import (
    AABB,
    Body,
    DistanceJoint,
    MouseJoint,
    PrismaticJoint,
    RevoluteJoint,
    Vec2,
    WeldJoint,
    World,
)

SCREEN_SHAPE = 1024, 768
WIDTH, HEIGHT = SCREEN_SHAPE
BACKGROUND_COLOR = "#1B1B1B"
JOINT_COLOR = "#E0E0E0"
DT = 1 / 60


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

    def to_screen(self, point: Vec2) -> tuple[float, float]:
        """World to screen pixels; screen y grows downward."""
        return (
            self.zoom * (point.x - self.x) + self.width * 0.5,
            self.zoom * -(point.y - self.y) + self.height * 0.5
        )

    def to_world(self, point) -> Vec2:
        x, y = point
        return Vec2(
            (x - self.width * 0.5) / self.zoom + self.x,
            -(y - self.height * 0.5) / self.zoom + self.y
        )


def make_scene() -> tuple[World, dict[Body, str]]:
    """A walled 90 x 64 box with loose shapes, a stack, and one of each joint."""
    world = World(gravity=(0, -30))
    colors: dict[Body, str] = {}

    def add(body: Body, color: str) -> Body:
        colors[world.add(body)] = color
        return body

    walls = [((0, -33), (92, 2)), ((0, 33), (92, 2)), ((-46, 0), (2, 68)), ((46, 0), (2, 68))]
    floor, ceiling, *_ = [add(Body.box(w, h, pos=pos, static=True), "#555555") for pos, (w, h) in walls]

    # loose shapes
    add(Body.regular(3, 5, mass=1, pos=(-25, 10), v=(20, -5)), "#4C72B0")
    add(Body.circle(4, mass=1, pos=(0, -15), v=(5, 25)), "#55A868")
    add(Body.regular(5, 5, mass=1.5, pos=(-10, 20), v=(-10, -20)), "#C44E52")
    add(Body.circle(3, mass=0.5, pos=(25, -10), v=(-25, 15)), "#8172B3")

    # a stack that settles and falls asleep
    for k in range(4):
        add(Body.box(6, 4, mass=2, pos=(36, -29.95 + 4 * k)), "#DD8452")

    # revolute: a chain hanging from the ceiling
    link_top = 32.0
    parent = ceiling
    for k in range(4):
        link = add(Body.box(1, 4, mass=0.5, pos=(-32, link_top - 2), v=(8, 0)), "#64B5CD")
        world.add(RevoluteJoint(parent, link, (-32, link_top)))
        parent, link_top = link, link_top - 4

    # revolute with a motor: a windmill on a fixed hub
    hub = add(Body.circle(0.6, pos=(10, 2), static=True), "#555555")
    blade = add(Body.box(18, 1, mass=3, pos=(10, 2)), "#CCB974")
    world.add(RevoluteJoint(hub, blade, (10, 2), motor_speed=1.5, max_motor_torque=5000))

    # weld: an L-shape made of two boxes
    arm = add(Body.box(8, 1.5, mass=1, pos=(-20, -20)), "#8C8C8C")
    leg = add(Body.box(1.5, 6, mass=1, pos=(-23.25, -16.25)), "#8C8C8C")
    world.add(WeldJoint(arm, leg, (-23.25, -20)))

    # distance with a spring: a bob hanging from the ceiling
    bob = add(Body.regular(6, 2.5, mass=1, pos=(22, 14)), "#DA8BC3")
    world.add(DistanceJoint(ceiling, bob, (22, 32), (22, 14), length=14, hertz=1.5, damping_ratio=0.1,
                              collide_connected=True))

    # prismatic: a slider on a diagonal rail anchored to the floor
    slider = add(Body.box(4, 2, mass=1, pos=(-38, -12)), "#937860")
    world.add(PrismaticJoint(floor, slider, (-38, -12), (1, -1), lower=0, upper=14))
    return world, colors


def draw_body(screen: pygame.Surface, camera: Camera, body: Body, color: str) -> None:
    if not body.awake and not body.is_static:
        color = pygame.Color(color).lerp(BACKGROUND_COLOR, 0.5)
    if body.vertices is None:
        center = camera.to_screen(body.pos)
        pygame.draw.circle(screen, color, center, camera.zoom * body.radius)
        # a spoke, so spin is visible
        pygame.draw.line(screen, BACKGROUND_COLOR, center, camera.to_screen(body.to_world((body.radius, 0))), 2)
    else:
        pygame.draw.polygon(screen, color, [camera.to_screen(v) for v in body.vertices])


def main():
    pygame.init()
    clock = pygame.time.Clock()
    screen = pygame.display.set_mode(SCREEN_SHAPE)

    world, colors = make_scene()
    camera = Camera(0, 0, WIDTH, HEIGHT, zoom=10)
    drag: MouseJoint | None = None
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
                point = camera.to_world(event.pos)
                body = world.body_at(point)
                if body is not None and not body.is_static:
                    drag = world.add(MouseJoint(body, point))
            if event.type == pygame.MOUSEBUTTONUP and event.button == 1 and drag is not None:
                world.remove(drag)
                drag = None
            if event.type == pygame.KEYDOWN and event.key == pygame.K_b:
                # a bullet: speculative contacts stop it at the first surface instead of tunnelling
                bullet = world.add(Body.circle(0.5, mass=0.2, pos=(-42, 25), v=(400, -60), restitution=0.6))
                colors[bullet] = "#FFFFFF"

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
            for body in world.bodies:
                if not body.is_static:
                    body.apply_impulse((0, 2 / body.inv_mass))

        if drag is not None:
            drag.target = camera.to_world(pygame.mouse.get_pos())

        clock.tick(60)
        world.step(DT)

        screen.fill(BACKGROUND_COLOR)
        view = camera.aabb
        for body in world.bodies:
            if view.is_overlap(body.aabb):
                draw_body(screen, camera, body, colors[body])
        for joint in world.joints:
            start, end = (camera.to_screen(p) for p in joint.anchors())
            pygame.draw.line(screen, JOINT_COLOR, start, end, 1)
            pygame.draw.circle(screen, JOINT_COLOR, end, 3)
        pygame.display.flip()


if __name__ == "__main__":
    main()
