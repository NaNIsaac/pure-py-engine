# pure-py-engine

A 2D rigid-body physics engine in plain Python, drawn with pygame-ce: vector/AABB math, convex polygon geometry, separating-axis collision for polygons and circles, impulse response with spin and Coulomb friction, and sub-stepped integration. Angles are in radians.

Status: early experiment. Contacts push bodies apart, then apply one normal + friction impulse at the mean contact point (Box2D-style edge clipping), so off-center hits spin bodies. No resting-contact solver or stacking yet.

## Requirements

Python 3.14+, pygame-ce, `../seltools` (path dependency).

## Usage

```bash
uv run main.py
```

Five shapes bounce inside four static walls. Drag a shape with the left mouse button; it follows the cursor through its velocity, so it still collides. Arrow keys pan the camera, mouse wheel zooms, holding Space spins every body.

Polygon vertices can be given in either winding and anywhere around the origin; `RigidBody` stores them counter-clockwise around their centroid and rejects non-convex or zero-area input with `ValueError`. `mass=math.inf` makes a body static.

## Test

```bash
uv run --with pytest pytest
```

## Structure

```
main.py         RigidBody, contact, resolve, Camera, Entity, make_scene, game loop (1024x768, 60 fps)
gmath.py        Vec2, AABB, polygon area / centroid / convexity, regular_polygon, rectangle
collide.py      point_in_triangle, point_in_convex, SAT: polygons_collide, circle_polygon_collide, circles_collide
test_engine.py  geometry, SAT, collision response, a 10 s scene that must stay inside the walls
```
