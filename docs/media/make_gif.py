"""Render docs/media/demo.gif: the demo and cradle scenes of main.py, without a window.

uv run --with pillow docs/media/make_gif.py
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import pygame
from PIL import Image

from main import DT, SCENES, Camera, draw

SIZE = 640, 480
EVERY = 3  # 60 Hz physics, 20 fps GIF
CLIPS = [("demo", 5.0), ("cradle", 3.0)]
COLORS = 64

pygame.init()
screen = pygame.Surface(SIZE)
# the frame main.py shows at 1024 x 768 and zoom 10, scaled down
camera = Camera(0, 0, *SIZE, zoom=10 * SIZE[0] / 1024)

frames = []
for name, seconds in CLIPS:
    world, colors, check = SCENES[name]()
    for step in range(round(seconds / DT)):
        world.step(DT)
        readout = check()  # the cradle's readout tracks a running peak, so it runs every step
        if step % EVERY == 0:
            draw(screen, camera, world, colors, [f"{name}: {readout}"])
            frames.append(Image.frombytes("RGB", SIZE, pygame.image.tobytes(screen, "RGB")))

# one palette for every frame, so the GIF can store only what changed between frames
sample = Image.new("RGB", (SIZE[0], SIZE[1] * len(frames[::10])))
for k, frame in enumerate(frames[::10]):
    sample.paste(frame, (0, SIZE[1] * k))
palette = sample.quantize(COLORS, method=Image.Quantize.MEDIANCUT)
indexed = [f.quantize(palette=palette, dither=Image.Dither.NONE) for f in frames]
indexed[0].save(
    ROOT / "docs/media/demo.gif",
    save_all=True,
    append_images=indexed[1:],
    duration=round(1000 * DT * EVERY),
    loop=0,
    optimize=False,
)
print(f"{len(indexed)} frames")
