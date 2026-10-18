# pure-py-engine

Beginnings of a 2D rigid-body physics engine in plain Python, drawn with pygame-ce: vector/AABB math, polygon moment of inertia, sub-stepped impulse integration and a point-in-convex-polygon test.

Status: early experiment. The demo shows one triangle. Holding the left mouse button prints the `point_in_convex` hit test against the triangle every frame (`True` while the cursor is inside it); the screen-space vertices are reversed before the test because the y-flip makes them clockwise.

## Requirements

Python 3.14+, pygame-ce, `../seltools` (path dependency).

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
