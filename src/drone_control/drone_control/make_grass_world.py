#!/usr/bin/env python3
"""
Build a realistic test world: green grass with Korean standard garbage bags on it.

Writes three things into the PX4 Gazebo resource tree (``--px4-gz``):

* ``models/grass_ground``  - a visual-only plane with a generated grass
  texture: large-scale lush/dry patches, mowing stripes, bare soil, and the
  high-frequency detail a camera really sees at 5 m - grass blades, clover
  and dandelion dots, fallen leaves.  The texture is generated at a higher
  pixel density than the survey camera samples the ground with (default
  8192 px / 30 m = 273 px/m vs ~187 px/m for the 1280 px camera at 5 m), so
  the detail survives rendering instead of being magnified into a blur,
* ``models/trash_bags``    - N static ellipsoid "bags" with random size class,
  colour and heading.  Each bag carries a texture with wrinkles, fold lines,
  a tie knot and printed text (real 종량제 봉투 are printed), because a
  smooth single-colour blob gives a feature matcher almost nothing.
  ``bags.json`` records the ground positions so a stitched map can be
  checked against where the bags really are,
* ``worlds/grass_bags.sdf`` - the stock PX4 ``default`` world plus the two.

Bag sizes follow the 종량제 봉투 classes (5/10/20/30/50/75/100 L). The
numbers in ``BAG_CLASSES`` are footprints of a *filled bag lying on the
ground* derived from the usual flat bag dimensions (e.g. 20 L ~ 50 x 70 cm
flat), not official specifications.

    ros2 run drone_control make_grass_world --px4-gz ~/PX4-Autopilot/Tools/simulation/gz
    PX4_GZ_WORLD=grass_bags make px4_sitl gz_x500_survey
"""

from __future__ import annotations

import argparse
import json
import math
import os
import random
import sys
from typing import Dict, List, Tuple

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont

# litres -> (length m, width m, height m) of a filled bag lying on the ground,
# and the relative frequency of that class.
BAG_CLASSES = {
    5: ((0.28, 0.20, 0.14), 1.0),
    10: ((0.38, 0.28, 0.20), 2.0),
    20: ((0.48, 0.36, 0.26), 3.0),
    30: ((0.55, 0.42, 0.30), 2.0),
    50: ((0.65, 0.50, 0.36), 2.0),
    75: ((0.75, 0.58, 0.42), 1.0),
    100: ((0.85, 0.65, 0.48), 0.7),
}
# name -> RGB 0..255
BAG_COLOURS = {
    'white': (235, 235, 230),
    'translucent_white': (215, 220, 205),
    'yellow': (242, 204, 40),
    'blue': (65, 115, 215),
    'green': (115, 180, 90),
    'pink': (242, 140, 180),
    'orange': (242, 140, 40),
    'black': (28, 28, 30),
    'grey': (140, 140, 145),
}
FONT_CANDIDATES = (
    '/usr/share/fonts/truetype/nanum/NanumGothicBold.ttf',
    '/usr/share/fonts/truetype/nanum/NanumGothic.ttf',
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',
    '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',
)


def _font(size: int) -> ImageFont.ImageFont:
    for path in FONT_CANDIDATES:
        if os.path.isfile(path):
            try:
                return ImageFont.truetype(path, size)
            except OSError:
                continue
    return ImageFont.load_default()


def _has_hangul_font() -> bool:
    return any(os.path.isfile(p) and 'DejaVu' not in p for p in FONT_CANDIDATES)


# ----------------------------------------------------------------------
# grass texture
# ----------------------------------------------------------------------
def _octave(rng: np.random.Generator, size: int, cells: int) -> np.ndarray:
    """Smooth noise in [-0.5, 0.5] with `cells` random values across the image."""
    small = rng.random((cells + 1, cells + 1), dtype=np.float32)
    img = Image.fromarray((small * 255).astype(np.uint8)).resize((size, size), Image.BICUBIC)
    return np.asarray(img, dtype=np.float32) / 255.0 - 0.5


def grass_texture(pixels: int, size_m: float, seed: int) -> Image.Image:
    """Return an RGB grass image, `pixels` wide, covering `size_m` metres."""
    rng = np.random.default_rng(seed)
    prng = random.Random(seed)
    ppm = pixels / size_m

    # 1) Large-scale structure at a quarter resolution, then upsampled:
    #    lush/dry patches, clumps, mowing stripes.
    base_px = max(1024, pixels // 4)
    base_ppm = base_px / size_m
    height = 0.22 * _octave(rng, base_px, 6)
    height += 0.16 * _octave(rng, base_px, 24)
    height += 0.14 * _octave(rng, base_px, 96)
    height += 0.12 * _octave(rng, base_px, 384)
    y, x = np.mgrid[0:base_px, 0:base_px].astype(np.float32)
    height += 0.04 * np.sign(np.sin((x * 0.26 + y * 0.97) / (1.5 * base_ppm) * math.pi))
    value = np.clip(0.55 + height, 0.05, 1.0)
    dryness = np.clip(0.5 + 0.9 * _octave(rng, base_px, 10), 0.0, 1.0)
    rgb = np.stack([45 + 110 * value + 45 * dryness,
                    95 + 120 * value + 10 * dryness,
                    25 + 55 * value - 10 * dryness], axis=-1)
    img = Image.fromarray(np.clip(rgb, 0, 255).astype(np.uint8), 'RGB')
    img = img.resize((pixels, pixels), Image.BICUBIC)
    del height, value, dryness, rgb, x, y

    # 2) Fine grain at full resolution (what makes it look like turf, not paint).
    grain = Image.effect_noise((pixels, pixels), 55).convert('RGB')
    img = Image.blend(img, grain, 0.22)
    del grain

    # 3) Blades: many short strokes, darker and lighter than the ground.
    draw = ImageDraw.Draw(img)
    blades = int(pixels * pixels / 60)
    for _ in range(blades):
        x0, y0 = prng.uniform(0, pixels), prng.uniform(0, pixels)
        length = prng.uniform(0.01, 0.035) * ppm
        angle = prng.gauss(1.1, 0.7)
        shade = prng.randint(-38, 30)
        colour = (max(0, 70 + shade), max(0, 135 + shade), max(0, 40 + shade // 2))
        draw.line([(x0, y0), (x0 + length * math.cos(angle), y0 + length * math.sin(angle))],
                  fill=colour, width=1)
    # 4) Clover / dandelions / small flowers and fallen leaves.
    for _ in range(int(pixels * pixels / 12000)):
        x0, y0 = prng.uniform(0, pixels), prng.uniform(0, pixels)
        r = prng.uniform(0.006, 0.02) * ppm
        colour = prng.choice([(240, 230, 90), (235, 235, 235), (55, 110, 45), (90, 150, 60)])
        draw.ellipse([x0 - r, y0 - r, x0 + r, y0 + r], fill=colour)
    for _ in range(int(pixels * pixels / 150000)):
        x0, y0 = prng.uniform(0, pixels), prng.uniform(0, pixels)
        rx, ry = prng.uniform(0.02, 0.06) * ppm, prng.uniform(0.01, 0.03) * ppm
        colour = prng.choice([(150, 110, 60), (120, 80, 45), (170, 140, 80)])
        draw.ellipse([x0 - rx, y0 - ry, x0 + rx, y0 + ry], fill=colour)

    # 5) A few bare soil patches with soft edges.
    soil = Image.new('RGB', (pixels, pixels), (120, 92, 60))
    mask = Image.new('L', (pixels, pixels), 0)
    mdraw = ImageDraw.Draw(mask)
    for _ in range(14):
        cx, cy = prng.uniform(0, pixels), prng.uniform(0, pixels)
        rx, ry = prng.uniform(0.3, 1.2) * ppm, prng.uniform(0.3, 1.2) * ppm
        mdraw.ellipse([cx - rx, cy - ry, cx + rx, cy + ry], fill=prng.randint(110, 200))
    mask = mask.filter(ImageFilter.GaussianBlur(0.12 * ppm))
    img = Image.composite(soil, img, mask)
    del soil, mask
    return img


# ----------------------------------------------------------------------
# bag textures
# ----------------------------------------------------------------------
def bag_texture(colour_name: str, seed: int, pixels: int = 1024) -> Image.Image:
    """Plastic-bag albedo: wrinkles, fold lines, a knot, printed text."""
    rng = np.random.default_rng(seed)
    prng = random.Random(seed)
    base = np.array(BAG_COLOURS[colour_name], dtype=np.float32)
    wrinkle = (0.5 * _octave(rng, pixels, 8) + 0.3 * _octave(rng, pixels, 32)
               + 0.2 * _octave(rng, pixels, 128))
    shade = 1.0 + 0.45 * wrinkle                      # +-22 % brightness
    rgb = np.clip(base[None, None, :] * shade[:, :, None], 0, 255).astype(np.uint8)
    img = Image.fromarray(rgb, 'RGB')
    draw = ImageDraw.Draw(img)
    dark = tuple(int(v * 0.55) for v in base)
    light = tuple(min(255, int(v * 1.25) + 25) for v in base)
    # fold lines and creases
    for _ in range(40):
        x0, y0 = prng.uniform(0, pixels), prng.uniform(0, pixels)
        angle = prng.uniform(0, math.pi)
        length = prng.uniform(0.1, 0.6) * pixels
        pts = [(x0, y0)]
        for k in range(1, 6):
            angle += prng.gauss(0, 0.35)
            pts.append((pts[-1][0] + length / 5 * math.cos(angle),
                        pts[-1][1] + length / 5 * math.sin(angle)))
        draw.line(pts, fill=dark if prng.random() < 0.6 else light, width=prng.randint(2, 6))
    # tie knot
    kx, ky = pixels * 0.5, pixels * 0.12
    draw.ellipse([kx - 0.08 * pixels, ky - 0.05 * pixels, kx + 0.08 * pixels, ky + 0.05 * pixels],
                 fill=dark)
    # printed text (real bags carry the district's print); repeat it around the bag
    ink = (20, 20, 20) if colour_name not in ('black', 'blue') else (230, 230, 230)
    label = '종량제봉투' if _has_hangul_font() else 'STANDARD BAG'
    font = _font(int(0.09 * pixels))
    small = _font(int(0.045 * pixels))
    for row, ty in enumerate((0.38, 0.62)):
        draw.text((pixels * 0.5, pixels * ty), label, font=font, fill=ink, anchor='mm')
        draw.text((pixels * 0.5, pixels * (ty + 0.09)),
                  'GENERAL WASTE 20L  No. %04d' % (seed % 10000),
                  font=small, fill=ink, anchor='mm')
    return img


# ----------------------------------------------------------------------
# bags
# ----------------------------------------------------------------------
def make_bags(count: int, centre_ne: Tuple[float, float], area_m: float, seed: int,
              keep_clear_ne: Tuple[float, float] = (0.0, 0.0),
              clear_radius: float = 1.5) -> List[dict]:
    """Random bags (ground positions in NED N/E metres) around the mission centre."""
    prng = random.Random(seed + 1)
    litres = list(BAG_CLASSES)
    weights = [BAG_CLASSES[v][1] for v in litres]
    colours = list(BAG_COLOURS)
    bags: List[dict] = []
    attempts = 0
    while len(bags) < count and attempts < count * 50:
        attempts += 1
        north = centre_ne[0] + prng.uniform(-area_m / 2, area_m / 2)
        east = centre_ne[1] + prng.uniform(-area_m / 2, area_m / 2)
        if math.hypot(north - keep_clear_ne[0], east - keep_clear_ne[1]) < clear_radius:
            continue   # never on the takeoff pad
        size = prng.choices(litres, weights)[0]
        length, width, height = (v * prng.uniform(0.85, 1.15) for v in BAG_CLASSES[size][0])
        if any(math.hypot(north - b['north_m'], east - b['east_m'])
               < 0.5 * (length + b['length_m']) for b in bags):
            continue   # no overlapping bags
        bags.append({
            'id': len(bags), 'litres': size, 'colour': prng.choice(colours),
            'north_m': round(north, 3), 'east_m': round(east, 3),
            'yaw_rad': round(prng.uniform(-math.pi, math.pi), 3),
            'length_m': round(length, 3), 'width_m': round(width, 3),
            'height_m': round(height, 3),
        })
    return bags


def bags_model_sdf(bags: List[dict]) -> str:
    links = []
    for bag in bags:
        # Gazebo world frame is ENU: x = East, y = North. Ellipsoid resting on the ground.
        pose = (f"{bag['east_m']} {bag['north_m']} {bag['height_m'] / 2:.3f} "
                f"0 0 {-bag['yaw_rad']:.3f}")
        radii = f"{bag['length_m'] / 2:.3f} {bag['width_m'] / 2:.3f} {bag['height_m'] / 2:.3f}"
        texture = f"model://trash_bags/materials/textures/bag_{bag['colour']}.jpg"
        links.append(f"""    <link name="bag_{bag['id']:02d}">
      <pose>{pose}</pose>
      <visual name="visual">
        <geometry><ellipsoid><radii>{radii}</radii></ellipsoid></geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <specular>0.4 0.4 0.4 1</specular>
          <pbr><metal>
            <albedo_map>{texture}</albedo_map>
            <roughness>0.45</roughness><metalness>0.0</metalness>
          </metal></pbr>
        </material>
      </visual>
      <collision name="collision">
        <geometry><ellipsoid><radii>{radii}</radii></ellipsoid></geometry>
      </collision>
    </link>""")
    body = '\n'.join(links)
    return f"""<?xml version="1.0" encoding="UTF-8"?>
<!-- Generated by drone_control/make_grass_world.py: {len(bags)} garbage bags.
     Positions (NED metres) are in bags.json next to this file. -->
<sdf version="1.9">
  <model name="trash_bags">
    <static>true</static>
{body}
  </model>
</sdf>
"""


def model_config(name: str, description: str) -> str:
    return f"""<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author><name>drone_control</name><email>joan0219@yonsei.ac.kr</email></author>
  <description>{description}</description>
</model>
"""


def ground_model_sdf(size_m: float) -> str:
    return f"""<?xml version="1.0" encoding="UTF-8"?>
<!-- Generated by drone_control/make_grass_world.py:
     {size_m:.0f} x {size_m:.0f} m grass (visual only). -->
<sdf version="1.9">
  <model name="grass_ground">
    <static>true</static>
    <link name="link">
      <visual name="grass">
        <cast_shadows>false</cast_shadows>
        <geometry><plane><normal>0 0 1</normal><size>{size_m} {size_m}</size></plane></geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <specular>0.02 0.02 0.02 1</specular>
          <pbr><metal>
            <albedo_map>model://grass_ground/materials/textures/grass.jpg</albedo_map>
            <roughness>1.0</roughness><metalness>0.0</metalness>
          </metal></pbr>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def overview_camera_sdf(look_at_ne: Tuple[float, float], distance: float = 13.0,
                        height: float = 8.0) -> str:
    """Return the SDF of a static camera south-west of the field, aimed at its centre."""
    north, east = look_at_ne
    # Gazebo ENU: x = East, y = North.  Place the camera west of the centre,
    # looking +x, pitched down onto the centre point.
    cam_x, cam_y, cam_z = east - distance, north, height
    pitch = math.atan2(height, distance)
    return f"""<?xml version="1.0" encoding="UTF-8"?>
<!-- Generated by drone_control/make_grass_world.py: third-person camera for mission videos.
     Topic /overview_camera/image, 1280x720 @ 10 Hz. -->
<sdf version="1.9">
  <model name="overview_camera">
    <static>true</static>
    <pose>{cam_x:.2f} {cam_y:.2f} {cam_z:.2f} 0 {pitch:.3f} 0</pose>
    <link name="link">
      <sensor name="overview_camera" type="camera">
        <topic>/overview_camera/image</topic>
        <camera>
          <horizontal_fov>1.25</horizontal_fov>
          <image><width>1280</width><height>720</height><format>R8G8B8</format></image>
          <clip><near>0.1</near><far>200</far></clip>
        </camera>
        <always_on>1</always_on>
        <update_rate>10</update_rate>
        <visualize>false</visualize>
      </sensor>
    </link>
  </model>
</sdf>
"""


def world_sdf(default_world_path: str) -> str:
    source = open(default_world_path, encoding='utf-8').read()
    assert source.count('<world name="default">') == 1, 'unexpected default.sdf'
    source = source.replace('<world name="default">', '<world name="grass_bags">')
    anchor = '    <model name="ground_plane">'
    assert source.count(anchor) == 1
    include = """    <!-- drone_control: grass field with garbage bags (make_grass_world.py).
         Visual only; the ground_plane below still provides the collision. -->
    <include>
      <uri>model://grass_ground</uri>
      <name>grass_ground</name>
      <pose>0 0 0.001 0 0 0</pose>
    </include>
    <include>
      <uri>model://trash_bags</uri>
      <name>trash_bags</name>
    </include>
    <include>
      <uri>model://overview_camera</uri>
      <name>overview_camera</name>
    </include>
"""
    return source.replace(anchor, include + anchor)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument('--px4-gz',
                        default=os.path.expanduser('~/PX4-Autopilot/Tools/simulation/gz'),
                        help='PX4 Gazebo resource dir containing models/ and worlds/')
    parser.add_argument('--size-m', type=float, default=30.0, help='grass plane side, metres')
    parser.add_argument('--pixels', type=int, default=8192, help='grass texture side, pixels')
    parser.add_argument('--bags', type=int, default=35)
    parser.add_argument('--area-m', type=float, default=16.0,
                        help='square (side) around the centre in which bags are scattered')
    parser.add_argument('--centre', type=float, nargs=2, default=(5.0, 0.0), metavar=('N', 'E'),
                        help='mission target = centre of the bag area (NED metres)')
    parser.add_argument('--seed', type=int, default=3)
    parser.add_argument('--preview', help='also write a 1024 px preview of the grass texture')
    args = parser.parse_args(argv)

    root = os.path.expanduser(args.px4_gz)
    grass_dir = os.path.join(root, 'models', 'grass_ground')
    bags_dir = os.path.join(root, 'models', 'trash_bags')
    os.makedirs(os.path.join(grass_dir, 'materials', 'textures'), exist_ok=True)
    os.makedirs(os.path.join(bags_dir, 'materials', 'textures'), exist_ok=True)

    texture = grass_texture(args.pixels, args.size_m, args.seed)
    texture_path = os.path.join(grass_dir, 'materials', 'textures', 'grass.jpg')
    texture.save(texture_path, quality=88)
    if args.preview:
        texture.resize((1024, 1024), Image.LANCZOS).save(args.preview)
        crop = int(2.0 * args.pixels / args.size_m)   # a 2 x 2 m patch at full resolution
        texture.crop((0, 0, crop, crop)).save(args.preview.replace('.png', '_2m.png'))
    del texture
    with open(os.path.join(grass_dir, 'model.sdf'), 'w') as handle:
        handle.write(ground_model_sdf(args.size_m))
    with open(os.path.join(grass_dir, 'model.config'), 'w') as handle:
        handle.write(model_config('grass_ground', 'Generated grass field (visual only).'))

    bags = make_bags(args.bags, tuple(args.centre), args.area_m, args.seed)
    used: Dict[str, int] = {}
    for index, colour in enumerate(sorted({b['colour'] for b in bags})):
        bag_texture(colour, args.seed * 100 + index).save(
            os.path.join(bags_dir, 'materials', 'textures', f'bag_{colour}.jpg'), quality=90)
        used[colour] = sum(1 for b in bags if b['colour'] == colour)
    with open(os.path.join(bags_dir, 'model.sdf'), 'w') as handle:
        handle.write(bags_model_sdf(bags))
    with open(os.path.join(bags_dir, 'model.config'), 'w') as handle:
        handle.write(model_config('trash_bags', f'{len(bags)} generated garbage bags.'))
    with open(os.path.join(bags_dir, 'bags.json'), 'w') as handle:
        json.dump({'frame': 'NED metres, origin = takeoff pad', 'centre_ne': list(args.centre),
                   'area_m': args.area_m, 'seed': args.seed, 'bags': bags}, handle, indent=2)

    cam_dir = os.path.join(root, 'models', 'overview_camera')
    os.makedirs(cam_dir, exist_ok=True)
    with open(os.path.join(cam_dir, 'model.sdf'), 'w') as handle:
        handle.write(overview_camera_sdf(tuple(args.centre)))
    with open(os.path.join(cam_dir, 'model.config'), 'w') as handle:
        handle.write(model_config('overview_camera', 'Third-person camera for mission videos.'))

    world_path = os.path.join(root, 'worlds', 'grass_bags.sdf')
    with open(world_path, 'w') as handle:
        handle.write(world_sdf(os.path.join(root, 'worlds', 'default.sdf')))

    sizes: Dict[int, int] = {}
    for bag in bags:
        sizes[bag['litres']] = sizes.get(bag['litres'], 0) + 1
    print(f'{texture_path}: {args.pixels}x{args.pixels} px over {args.size_m} m '
          f'({args.pixels / args.size_m:.0f} px/m), {os.path.getsize(texture_path) / 1e6:.1f} MB')
    print(f'{bags_dir}/model.sdf: {len(bags)} bags, litres {dict(sorted(sizes.items()))}, '
          f'colours {used}, label font hangul={_has_hangul_font()}')
    print(f'{world_path}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
