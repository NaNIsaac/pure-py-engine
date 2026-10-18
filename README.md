# pure-py-engine

Beginnings of a 2D rigid-body physics engine in plain Python, drawn with pygame-ce: vector/AABB math, polygon moment of inertia, sub-stepped impulse integration and a point-in-convex-polygon test.

Status: early experiment. The demo shows one triangle. While the left mouse button is held over it, a white outline is drawn (`Entity.is_inside`, a screen-space hit test for circles and convex polygons).

## Requirements

Python 3.14+, pygame-ce, `../seltools` (path dependency).

## Usage

```bash
uv run main.py
```

Arrow keys pan the camera, mouse wheel zooms, holding Space applies angular impulse to spin the triangle.

## Test

```bash
uv run --with pytest pytest
```

## Structure

```
main.py      RigidBody, Camera, Entity, game loop (1024x768, 60 fps)
gmath.py     Vec2, AABB
collide.py   point_in_triangle, point_in_convex (counter-clockwise vertices only)
test_engine.py  hit tests, inertia, rotation, entity hit test
```
