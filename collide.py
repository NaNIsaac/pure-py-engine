from math import isclose

from gmath import Vec2, sign

def point_in_triangle(p: Vec2, a: Vec2, b: Vec2, c: Vec2) -> bool:
    return (
        (a - p).cross(a - b) > 0 and
        (b - p).cross(b - c) > 0 and
        (c - p).cross(c - a) > 0
    )

def point_in_convex(p: Vec2, vertices: list[Vec2]) -> bool:
    
    i, v = min(enumerate(vertices), key=lambda x: x.to_tuple()) # lex compare
    vertices[:] = vertices[i:] + vertices[:i]
    seq = []
    for i in range(1, len(vertices)):
        seq.append(vertices[i] - vertices[0])
    translation = vertices[0]

    def _point_in_convex(point: Vec2) -> bool:
        point = point = translation
        if (not isclose(seq[0].cross(point), 0)
            and sign(seq[0].cross(point)) 
            != sign(seq[0].cross(seq[-1]))
            ):
            return False
        if (not isclose(seq[-1].cross(point), 0)
            and sign(seq[-1].cross(point)) 
            != sign(seq[-1].cross(seq[0]))
            ):
            return False
        