# README media

`demo.gif` is rendered by `make_gif.py` from this commit's `main.py` and `engine`, without a window:

```sh
uv run --with pillow docs/media/make_gif.py
```

It steps the demo scene for 5 s and the cradle scene for 3 s at 60 Hz, draws every third step
(20 fps) at 640 x 480 with the same camera framing as the 1024 x 768 window, and writes 160 frames
with one 64-colour palette. The top line is the scene's readout; the step time is left out so that
the file does not depend on the machine. Running it again gives the same file byte for byte
(checked on Windows, pygame-ce 2.5.7, Pillow 13.0.0).
