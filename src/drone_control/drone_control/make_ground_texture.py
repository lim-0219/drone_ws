#!/usr/bin/env python3
"""
Generate a feature-rich, non-repeating ground texture for the mosaic mission.

The stock Gazebo `default` world has a uniform grey floor: SIFT finds zero
features on it and the stitcher (correctly) refuses to build a map.  This
tool draws a `size_m` x `size_m` metre ground image that

* is full of random, non-repeating shapes and speckle (feature points),
* carries a 1 m grid with the *world coordinates* written in every cell
  (``E+3`` / ``N-2``), a North arrow, an East arrow, an origin marker and a
  ring at the mission target - so a finished mosaic can be checked by eye
  for scale, orientation and alignment,
* maps one metre to ``pixels / size_m`` pixels (default 4096 px / 40 m =
  102 px/m).

Pixel <-> world: the image centre is the world origin, image right is East
(+x in the Gazebo ENU world), image top is North (+y).  That is how Gazebo
maps a plane's texture: u along +x, v=0 at the +y edge.  If the mosaic shows
the labels mirrored or rotated, that assumption (or the camera one) is off -
the labels make it obvious.

Output is a JPEG (a 4096 px PNG of noise is ~30 MB; JPEG q90 is ~5 MB).

    ros2 run drone_control make_ground_texture --output ground.jpg --preview preview.png
"""

from __future__ import annotations

import argparse
import math
import os
import random
import sys
from typing import Tuple

from PIL import Image, ImageDraw, ImageFont

FONT_BOLD = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'


def _font(size: int) -> ImageFont.ImageFont:
    try:
        return ImageFont.truetype(FONT_BOLD, size)
    except OSError:
        return ImageFont.load_default()


def generate(size_m: float = 40.0, pixels: int = 4096, seed: int = 7,
             target_ne: Tuple[float, float] = (5.0, 0.0)) -> Image.Image:
    """Return the ground texture as a PIL RGB image."""
    ppm = pixels / size_m
    half = size_m / 2.0
    rng = random.Random(seed)

    def px(east: float, north: float) -> Tuple[float, float]:
        """World metres (E, N) -> image pixel (u, v); centre = origin, top = North."""
        return pixels / 2.0 + east * ppm, pixels / 2.0 - north * ppm

    img = Image.new('RGB', (pixels, pixels), (158, 150, 132))
    draw = ImageDraw.Draw(img)

    # 1) Random patches: ellipses, rectangles, polygons in varied earth/pastel
    #    colours.  Random sizes and positions make every square metre unique.
    for _ in range(int(9000 * (size_m / 40.0) ** 2)):
        cx, cy = rng.uniform(0, pixels), rng.uniform(0, pixels)
        r = rng.uniform(0.08, 0.9) * ppm
        colour = tuple(rng.randint(35, 235) for _ in range(3))
        kind = rng.random()
        if kind < 0.45:
            ry = r * rng.uniform(0.4, 1.0)
            draw.ellipse([cx - r, cy - ry, cx + r, cy + ry], fill=colour)
        elif kind < 0.8:
            w, h = r, r * rng.uniform(0.3, 1.0)
            draw.rectangle([cx - w, cy - h, cx + w, cy + h], fill=colour)
        else:
            n = rng.randint(3, 6)
            points = []
            for k in range(n):
                angle = 2.0 * math.pi * k / n + rng.uniform(-0.4, 0.4)
                rad = r * rng.uniform(0.5, 1.0)
                points.append((cx + rad * math.cos(angle), cy + rad * math.sin(angle)))
            draw.polygon(points, fill=colour)

    # 2) Fine speckle so even the inside of a patch has texture.
    noise = Image.effect_noise((pixels, pixels), 48).convert('RGB')
    img = Image.blend(img, noise, 0.18)
    draw = ImageDraw.Draw(img)

    # 3) 1 m grid (thin, dark) and 5 m grid (thicker, light).
    cells = int(math.floor(half))
    for m in range(-cells, cells + 1):
        u, v = px(m, m)
        thick = (m % 5 == 0)
        width = 5 if thick else 2
        colour = (245, 245, 245) if thick else (25, 25, 25)
        draw.line([(u, 0), (u, pixels)], fill=colour, width=width)
        draw.line([(0, v), (pixels, v)], fill=colour, width=width)

    # 4) Every 1 m cell is labelled with the world coordinates of its
    #    south-west corner: "E+3" over "N-2".
    cell_font = _font(max(10, int(0.24 * ppm)))
    for east in range(-cells, cells):
        for north in range(-cells, cells):
            u, v = px(east + 0.5, north + 0.5)
            draw.text((u, v - 0.13 * ppm), f'E{east:+d}', font=cell_font, fill=(0, 0, 0),
                      anchor='mm', stroke_width=2, stroke_fill=(255, 255, 255))
            draw.text((u, v + 0.13 * ppm), f'N{north:+d}', font=cell_font, fill=(0, 0, 0),
                      anchor='mm', stroke_width=2, stroke_fill=(255, 255, 255))

    # 5) Origin marker, North and East arrows.
    big_font = _font(int(0.6 * ppm))
    u0, v0 = px(0.0, 0.0)
    r0 = 0.45 * ppm
    draw.ellipse([u0 - r0, v0 - r0, u0 + r0, v0 + r0], fill=(255, 255, 255),
                 outline=(0, 0, 0), width=6)
    draw.text((u0, v0), 'ORIGIN', font=_font(int(0.18 * ppm)), fill=(0, 0, 0), anchor='mm')
    arrows = (('N', (0.0, 3.0), (200, 0, 0)), ('E', (3.0, 0.0), (0, 0, 200)))
    for label, (de, dn), colour in arrows:
        tip = px(de * 1.0, dn * 1.0)
        base = px(de * 0.2, dn * 0.2)
        draw.line([base, tip], fill=colour, width=int(0.12 * ppm))
        # arrow head
        angle = math.atan2(tip[1] - base[1], tip[0] - base[0])
        head = 0.5 * ppm
        left = (tip[0] - head * math.cos(angle - 0.5), tip[1] - head * math.sin(angle - 0.5))
        right = (tip[0] - head * math.cos(angle + 0.5), tip[1] - head * math.sin(angle + 0.5))
        draw.polygon([tip, left, right], fill=colour)
        draw.text(px(de * 1.3, dn * 1.3), label, font=big_font, fill=colour, anchor='mm',
                  stroke_width=4, stroke_fill=(255, 255, 255))

    # 6) Ring at the mission target (grid cell 5).
    north_t, east_t = target_ne
    ut, vt = px(east_t, north_t)
    rt = 0.6 * ppm
    draw.ellipse([ut - rt, vt - rt, ut + rt, vt + rt], outline=(230, 0, 0), width=int(0.08 * ppm))
    draw.text((ut, vt - 0.95 * ppm), f'TARGET N{north_t:+.0f} E{east_t:+.0f}',
              font=_font(int(0.28 * ppm)), fill=(230, 0, 0), anchor='mm',
              stroke_width=3, stroke_fill=(255, 255, 255))
    return img


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument('--output', required=True, help='texture file to write (.jpg or .png)')
    parser.add_argument('--size-m', type=float, default=40.0, help='ground side length, metres')
    parser.add_argument('--pixels', type=int, default=4096, help='texture side length, pixels')
    parser.add_argument('--seed', type=int, default=7)
    parser.add_argument('--target', type=float, nargs=2, default=(5.0, 0.0), metavar=('N', 'E'),
                        help='mission target (grid cell 5) to mark, NED metres')
    parser.add_argument('--preview', help='also write a 1024 px preview here')
    args = parser.parse_args(argv)

    image = generate(args.size_m, args.pixels, args.seed, (args.target[0], args.target[1]))
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    if args.output.lower().endswith(('.jpg', '.jpeg')):
        image.save(args.output, quality=90)
    else:
        image.save(args.output)
    print(f'{args.output}: {image.width}x{image.height} px = {args.size_m} m '
          f'({args.pixels / args.size_m:.1f} px/m), {os.path.getsize(args.output) / 1e6:.1f} MB')
    if args.preview:
        image.resize((1024, 1024), Image.LANCZOS).save(args.preview)
        print(f'preview: {args.preview}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
