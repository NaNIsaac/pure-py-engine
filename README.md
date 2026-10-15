# pure-py-engine

Beginnings of a 2D rigid-body physics engine in plain Python, drawn with pygame-ce: vector/AABB math, polygon moment of inertia, sub-stepped impulse integration and a point-in-convex-polygon test.

Status: early experiment. The demo shows one triangle. Holding the left mouse button prints the `point_in_convex` hit test every frame, but it never reports a hit: the screen-space vertices are clockwise after the y-flip, and the last line of `point_in_convex` uses `p` where it should use `point`.

## Requirements

Python 3.14+, pygame-ce.

## Usage

```bash
uv run main.py
```

Arrow keys pan the camera, mouse wheel zooms, holding Space applies angular impulse to spin the triangle.

## Structure

```
main.py      RigidBody, Camera, Entity, game loop (1024x768, 60 fps)
gmath.py     Vec2, AABB
collide.py   point_in_triangle, point_in_convex (counter-clockwise vertices only)
```
