import math
import sys
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

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

# a world, a color per body, and a one-line readout of what the scene should show
type Scene = tuple[World, dict[Body, str], Callable[[], str]]


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
            self.y + real_half_height,
        )

    def to_screen(self, point: Vec2) -> tuple[float, float]:
        """World to screen pixels; screen y grows downward."""
        return (
            self.zoom * (point.x - self.x) + self.width * 0.5,
            self.zoom * -(point.y - self.y) + self.height * 0.5,
        )

    def to_world(self, point) -> Vec2:
        x, y = point
        return Vec2(
            (x - self.width * 0.5) / self.zoom + self.x,
            -(y - self.height * 0.5) / self.zoom + self.y,
        )


def make_scene() -> Scene:
    """A walled 90 x 64 box with loose shapes, a stack, and one of each joint."""
    world = World(gravity=(0, -30))
    colors: dict[Body, str] = {}

    def add(body: Body, color: str) -> Body:
        colors[world.add(body)] = color
        return body

    walls = [((0, -33), (92, 2)), ((0, 33), (92, 2)), ((-46, 0), (2, 68)), ((46, 0), (2, 68))]
    floor, ceiling, *_ = [
        add(Body.box(w, h, pos=pos, static=True), "#555555") for pos, (w, h) in walls
    ]

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
    world.add(
        DistanceJoint(
            ceiling,
            bob,
            (22, 32),
            (22, 14),
            length=14,
            hertz=1.5,
            damping_ratio=0.1,
            collide_connected=True,
        )
    )

    # prismatic: a slider on a diagonal rail anchored to the floor
    slider = add(Body.box(4, 2, mass=1, pos=(-38, -12)), "#937860")
    rail = world.add(PrismaticJoint(floor, slider, (-38, -12), (1, -1), lower=0, upper=14))
    return world, colors, lambda: f"slider at {rail.translation:.2f}, stops at 0 and 14"


def pyramid() -> Scene:
    """Ten rows of 2 x 2 boxes: a stable solver keeps the top box in place and lets the pile sleep."""
    world = World(gravity=(0, -30))
    colors = {world.add(Body.box(92, 2, pos=(0, -33), static=True)): "#555555"}
    rows = 10
    for row in range(rows):
        for k in range(rows - row):
            colors[world.add(Body.box(2, 2, pos=(2 * k - (rows - row - 1), -31 + 2 * row)))] = (
                "#DD8452"
            )
    top = world.bodies[-1]
    start = top.pos
    return (
        world,
        colors,
        lambda: (
            f"top box drift {(top.pos - start).length():.3f}, "
            f"asleep {sum(not b.awake for b in world.bodies[1:])}/{len(world.bodies) - 1}"
        ),
    )


def cradle() -> Scene:
    """Newton's cradle: elastic, frictionless balls on rods; the last ball should rise as high as the first fell."""
    world = World(gravity=(0, -30))
    bar = world.add(Body.box(30, 1, pos=(0, 20.5), static=True))
    colors = {bar: "#555555"}
    rod, radius = 25, 2
    rest_y = 20 - rod
    balls = []
    for k in range(5):
        x = (k - 2) * (2 * radius + 0.01)
        pos = Vec2(x, rest_y)
        if k == 0:  # pulled back 60 degrees
            pos = Vec2(x - rod * math.sin(math.pi / 3), 20 - rod * math.cos(math.pi / 3))
        ball = world.add(Body.circle(radius, pos=pos, friction=0, restitution=1))
        world.add(DistanceJoint(bar, ball, (x, 20), pos))
        colors[ball] = "#64B5CD"
        balls.append(ball)
    drop = balls[0].pos.y - rest_y
    start_energy = energy(world)
    swing = world.gravity.length() * drop / balls[0].inv_mass  # energy the pulled ball brings in
    peak = 0.0

    def check() -> str:
        nonlocal peak
        peak = max(peak, balls[-1].pos.y - rest_y)
        return f"last ball peak {peak:.2f} of {drop:.2f}, energy change {(energy(world) - start_energy) / swing:+.2%}"

    return world, colors, check


def ramp() -> Scene:
    """Boxes on a 30 degree slope: each should hold exactly when its friction coefficient is at least tan 30."""
    world = World(gravity=(0, -30))
    angle = -math.pi / 6
    slope = world.add(Body.box(60, 2, angle=angle, friction=1, static=True))
    floor = world.add(Body.box(92, 2, pos=(0, -33), static=True))
    colors = {slope: "#555555", floor: "#555555"}
    along, up = Vec2(math.cos(angle), math.sin(angle)), Vec2(-math.sin(angle), math.cos(angle))
    limit = math.tan(-angle)
    boxes = []
    # holders uphill, so the sliders move away from them
    for s, friction in zip((-21, -9, 3, 15), (0.8, 0.4, 0.3, 0.1)):
        box = world.add(Body.box(3, 3, pos=along * s + up * 2.5, angle=angle, friction=friction))
        # the contact uses sqrt(box friction * slope friction), and the slope's is 1
        mu = math.sqrt(friction)
        colors[box] = "#55A868" if mu >= limit else "#C44E52"
        boxes.append((box, box.pos, mu))

    def check() -> str:
        return ", ".join(
            f"mu {mu:.2f} {'holds' if mu >= limit else 'slides'}: moved {(b.pos - start).length():.2f}"
            for b, start, mu in boxes
        )

    return world, colors, check


def bullets() -> Scene:
    """Bullets at 100 to 800 units/s in a box of 0.2 thick walls: none should get out, even after hitting each other."""
    world = World()
    walls = [
        ((-40, 0), (0.2, 40)),
        ((40, 0), (0.2, 40)),
        ((0, -20), (80, 0.2)),
        ((0, 20), (80, 0.2)),
    ]
    colors = {world.add(Body.box(w, h, pos=pos, static=True)): "#555555" for pos, (w, h) in walls}
    shots = []
    for k in range(8):
        pos, v = (-30, -14 + 4 * k), (100 * (k + 1), 0)
        shot = (
            Body.circle(0.3, pos=pos, v=v, restitution=0.8)
            if k % 2
            else Body.box(0.6, 0.6, pos=pos, v=v, restitution=0.8)
        )
        colors[world.add(shot)] = "#FFFFFF"
        shots.append(shot)
    out = lambda: sum(abs(b.pos.x) > 40 or abs(b.pos.y) > 20 for b in shots)
    return world, colors, lambda: f"got out: {out()} of {len(shots)}"


SCENES = {
    "demo": make_scene,
    "pyramid": pyramid,
    "cradle": cradle,
    "ramp": ramp,
    "bullets": bullets,
}


def energy(world: World) -> float:
    """Kinetic plus gravitational energy of the dynamic bodies."""
    total = 0.0
    for b in world.bodies:
        if not b.is_static:
            mass, inertia = 1 / b.inv_mass, 1 / b.inv_inertia
            total += 0.5 * (mass * b.v.square() + inertia * b.w * b.w) - mass * world.gravity.dot(
                b.pos
            )
    return total


def draw_body(screen: pygame.Surface, camera: Camera, body: Body, color: str) -> None:
    if not body.awake and not body.is_static:
        color = pygame.Color(color).lerp(BACKGROUND_COLOR, 0.5)
    if body.vertices is None:
        center = camera.to_screen(body.pos)
        pygame.draw.circle(screen, color, center, camera.zoom * body.radius)
        # a spoke, so spin is visible
        pygame.draw.line(
            screen, BACKGROUND_COLOR, center, camera.to_screen(body.to_world((body.radius, 0))), 2
        )
    else:
        pygame.draw.polygon(screen, color, [camera.to_screen(v) for v in body.vertices])


def draw(
    screen: pygame.Surface, camera: Camera, world: World, colors: dict[Body, str], hud: list[str]
) -> None:
    screen.fill(BACKGROUND_COLOR)
    view = camera.aabb
    for body in world.bodies:
        if view.is_overlap(body.aabb):
            draw_body(screen, camera, body, colors[body])
    for joint in world.joints:
        start, end = (camera.to_screen(p) for p in joint.anchors())
        pygame.draw.line(screen, JOINT_COLOR, start, end, 1)
        pygame.draw.circle(screen, JOINT_COLOR, end, 3)
    font = pygame.font.Font(None, 22)
    for k, line in enumerate(hud):
        screen.blit(font.render(line, True, JOINT_COLOR), (8, 8 + 20 * k))


def status(name: str, world: World, check: Callable[[], str], step_ms: float) -> list[str]:
    dynamic = [b for b in world.bodies if not b.is_static]
    return [
        f"{name}: {check()}",
        f"step {step_ms:.1f} ms, awake {sum(b.awake for b in dynamic)}/{len(dynamic)}, contacts {len(world.arbiters)}",
        "  ".join(f"{k} {n}" for k, n in enumerate(SCENES, 1)) + "  |  R restart, P pause",
    ]


def save_screenshots(folder: Path, seconds: float = 3) -> None:
    """Run each scene without a window for `seconds` and save the last frame as <scene>.png."""
    pygame.init()
    screen = pygame.Surface(SCREEN_SHAPE)
    camera = Camera(0, 0, WIDTH, HEIGHT, zoom=10)
    folder.mkdir(parents=True, exist_ok=True)
    for name, make in SCENES.items():
        world, colors, check = make()
        step_ms = 0.0
        for _ in range(round(seconds / DT)):
            t = time.perf_counter()
            world.step(DT)
            step_ms = 1000 * (time.perf_counter() - t)
            check()  # the cradle's readout tracks a running peak
        hud = status(name, world, check, step_ms)
        draw(screen, camera, world, colors, hud)
        pygame.image.save(screen, folder / f"{name}.png")
        print(*hud[:2], sep="\n")


def main():
    if sys.argv[1:2] == ["--save"]:
        save_screenshots(Path(sys.argv[2]))
        return
    pygame.init()
    clock = pygame.time.Clock()
    screen = pygame.display.set_mode(SCREEN_SHAPE)

    name = next(iter(SCENES))
    world, colors, check = SCENES[name]()
    camera = Camera(0, 0, WIDTH, HEIGHT, zoom=10)
    drag: MouseJoint | None = None
    paused = False
    step_ms = 0.0
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            if event.type == pygame.KEYDOWN:
                number = event.key - pygame.K_1
                if 0 <= number < len(SCENES) or event.key == pygame.K_r:
                    if event.key != pygame.K_r:
                        name = list(SCENES)[number]
                    world, colors, check = SCENES[name]()
                    drag = None
                if event.key == pygame.K_p:
                    paused = not paused

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
                bullet = world.add(
                    Body.circle(0.5, mass=0.2, pos=(-42, 25), v=(400, -60), restitution=0.6)
                )
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
        if not paused:
            t = time.perf_counter()
            world.step(DT)
            step_ms = 1000 * (time.perf_counter() - t)

        draw(screen, camera, world, colors, status(name, world, check, step_ms))
        pygame.display.flip()


if __name__ == "__main__":
    main()
