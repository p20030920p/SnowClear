#!/usr/bin/env python3
"""XWD -> PNG (TrueColor, 24 or 32 bits per pixel).

xwd is the only X capture tool needed; Pillow cannot read XWD, so this decodes the
header (25 big-endian u32s + a colour map) and rebuilds the rows.
"""
import pathlib, struct, sys
import numpy as np
from PIL import Image
for xwd in sorted(pathlib.Path(sys.argv[1]).glob("*.xwd")):
    raw = xwd.read_bytes()
    if len(raw) < 100:
        print(f"{xwd.name}: empty"); continue
    h = struct.unpack(">25I", raw[:100])
    hsize, w, hh, bline, ncolors = h[0], h[4], h[5], h[12], h[19]
    body = raw[hsize + ncolors * 12: hsize + ncolors * 12 + bline * hh]
    if bline == w * 4:
        px = np.frombuffer(body, dtype=np.uint8).reshape(hh, bline)[:, :w*4].reshape(hh, w, 4)
    else:
        px = np.frombuffer(body, dtype=np.uint8).reshape(hh, bline)[:, :w*3].reshape(hh, w, 3)
    rgb = px[:, :, [2, 1, 0]]
    out = xwd.with_suffix(".png")
    Image.fromarray(rgb).save(out)
    vp = rgb[80:int(hh*0.95), int(w*0.42):]
    print(f"{out.name}: {w}x{hh} mean={rgb.mean():.1f} viewport_bright={int((vp.astype(int).sum(axis=2) > 150).sum())}")
