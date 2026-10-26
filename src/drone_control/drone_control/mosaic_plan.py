#!/usr/bin/env python3
"""
Mosaic geometry: camera footprint, flight altitude and 3x3 grid step.

Pure Python (no ROS imports) so the plan can be checked on any machine
before a flight::

    python3 -m drone_control.mosaic_plan                    # B0353, 20 x 10 m
    python3 -m drone_control.mosaic_plan --altitude 5.0     # what a fixed altitude gives
    python3 -m drone_control.mosaic_plan --hfov-deg 90      # if calibration says 90 deg
    ros2 run drone_control mosaic_plan --extent 20 10 --overlap 0.5

:func:`plan_mosaic` is what ``waypoint_navigator`` calls, so the printed
numbers are exactly the ones flown.

Geometry
--------
A nadir pinhole camera at height ``h`` sees ``footprint_u = 2 h tan(hfov/2)``
along the image width and ``footprint_v = footprint_u * H / W`` along the
image height.  The nine cells of the mosaic are the 3x3 grid the stitcher
expects (cell 5 in the middle, X = image right, Y = image top), so the
stitched area is::

    extent_x = footprint_u + 2 * spacing_x
    extent_y = footprint_v + 2 * spacing_y

and the overlap between neighbouring cells is ``1 - spacing / footprint``.
Given the wanted extent, the altitude fixes the footprint and therefore the
spacing; the overlap must stay above ``min_overlap`` on both axes or the
feature matcher has nothing to work with.

Arducam B0353 (the real camera)
-------------------------------
AR0234 colour global-shutter sensor, 1920 x 1200 active pixels of 3.0 um,
behind the stock 3.6 mm M12 lens (Arducam datasheet).  Treating it as a
pinhole::

    fx = 3.6 mm / 3.0 um = 1200 px
    hfov = 2 atan(1920 / (2 * 1200)) = 77.3 deg = 1.3495 rad
    vfov = 2 atan(1200 / (2 * 1200)) = 53.1 deg

Arducam's brochure says 90 deg (H) - a wide-angle lens with barrel
distortion covers more at the edges than the paraxial focal length
predicts.  The pinhole figure is the conservative one (a smaller footprint
means *more* overlap on the ground), so it is the default until a
checkerboard calibration replaces it: ``hfov = 2 atan(W / (2 fx))`` with
``fx`` from the calibration YAML.

With that lens the footprint is ``1.6 h`` x ``1.0 h``; a 20 x 10 m mosaic at
50 % overlap along X needs ``footprint_u = 20 / (3 - 2 * 0.5) = 10 m``, i.e.
**6.25 m AGL**, giving a 10.0 x 6.25 m footprint, a 5.0 x 1.875 m grid step,
50 % / 70 % overlap and 5.2 mm ground sampling distance.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from typing import Optional, Sequence, Tuple


@dataclass(frozen=True)
class CameraModel:
    """Pinhole description of a camera from its datasheet."""

    name: str
    width: int
    height: int
    pixel_um: float
    focal_mm: float

    @property
    def fx_px(self) -> float:
        return self.focal_mm / (self.pixel_um * 1e-3)

    @property
    def hfov_rad(self) -> float:
        return 2.0 * math.atan(self.width / (2.0 * self.fx_px))

    @property
    def vfov_rad(self) -> float:
        return 2.0 * math.atan(self.height / (2.0 * self.fx_px))


# Arducam B0353: AR0234 (1/2.6", 1920x1200, 3.0 um) + 3.6 mm F/3.0 M12 lens.
ARDUCAM_B0353 = CameraModel('Arducam B0353 (AR0234 + 3.6 mm M12)', 1920, 1200, 3.0, 3.6)

# Gazebo x500_survey camera in the SITL world (horizontal_fov 1.2 rad, 1280x960).
GAZEBO_SURVEY_HFOV_RAD = 1.2

# Mission requirement: stitched area along image right (X) and image top (Y).
DEFAULT_EXTENT_M = (20.0, 10.0)
DEFAULT_OVERLAP = 0.5
DEFAULT_MIN_OVERLAP = 0.3


def camera_footprint(agl: float, hfov: float, width: int, height: int) -> Tuple[float, float]:
    """
    Ground area seen by a nadir pinhole camera (along image u, along image v).

    u = 2 * agl * tan(hfov / 2); square pixels give v = u * height / width.
    """
    along_u = 2.0 * agl * math.tan(hfov / 2.0)
    return along_u, along_u * height / width


def altitude_for_footprint(along_u: float, hfov: float) -> float:
    """Height at which the footprint along the image width equals ``along_u``."""
    return along_u / (2.0 * math.tan(hfov / 2.0))


@dataclass(frozen=True)
class MosaicPlan:
    """Everything the navigator needs to lay out the nine cells."""

    altitude: float                    # m above the takeoff point (= AGL over flat ground)
    footprint: Tuple[float, float]     # m along image right / image top
    spacing: Tuple[float, float]       # grid step along X / Y, m
    overlap: Tuple[float, float]       # 1 - spacing / footprint per axis
    extent: Tuple[float, float]        # resulting mosaic size, m
    gsd_m: float                       # ground sampling distance, m per pixel
    altitude_min: float                # below this the overlap drops under min_overlap
    altitude_max: float                # at/above this one footprint already exceeds the extent
    altitude_source: str               # 'parameter' or 'derived from extent and overlap'

    def describe(self) -> str:
        return (
            f'altitude {self.altitude:.2f} m ({self.altitude_source}; valid '
            f'{self.altitude_min:.2f}-{self.altitude_max:.2f} m), footprint '
            f'{self.footprint[0]:.2f} x {self.footprint[1]:.2f} m, grid step '
            f'{self.spacing[0]:.2f} x {self.spacing[1]:.2f} m, overlap '
            f'{100 * self.overlap[0]:.0f} % / {100 * self.overlap[1]:.0f} %, mosaic '
            f'{self.extent[0]:.1f} x {self.extent[1]:.1f} m, GSD {1000 * self.gsd_m:.1f} mm/px'
        )


def plan_mosaic(hfov: float, width: int, height: int,
                extent: Sequence[float] = DEFAULT_EXTENT_M,
                altitude: Optional[float] = None,
                overlap: float = DEFAULT_OVERLAP,
                min_overlap: float = DEFAULT_MIN_OVERLAP) -> MosaicPlan:
    """
    Lay a 3x3 mosaic of ``extent`` (X, Y) metres over the ground.

    ``altitude`` given: it is flown as is and the grid step follows from the
    footprint at that height.  ``altitude`` None/<= 0: it is derived so that
    the X axis has exactly ``overlap``.  Either way both axes must end up
    with at least ``min_overlap`` and a positive step, otherwise a
    ``ValueError`` names the altitude range that would work.
    """
    if not 0.0 < hfov < math.pi:
        raise ValueError('hfov must be in (0, pi) radians')
    if width <= 0 or height <= 0:
        raise ValueError('image size must be positive')
    extent_x, extent_y = float(extent[0]), float(extent[1])
    if extent_x <= 0.0 or extent_y <= 0.0:
        raise ValueError('mosaic extent must be positive on both axes')
    if not 0.0 <= overlap < 1.0 or not 0.0 <= min_overlap < 1.0:
        raise ValueError('overlap values must be in [0, 1)')

    aspect = height / width
    tan_half = math.tan(hfov / 2.0)
    # footprint_u * (3 - 2 * overlap) == extent_x  <=>  overlap on X exactly `overlap`
    altitude_min = max(extent_x / ((3.0 - 2.0 * min_overlap) * 2.0 * tan_half),
                       extent_y / ((3.0 - 2.0 * min_overlap) * 2.0 * tan_half * aspect))
    altitude_max = min(extent_x / (2.0 * tan_half), extent_y / (2.0 * tan_half * aspect))

    if altitude is None or altitude <= 0.0:
        altitude = altitude_for_footprint(extent_x / (3.0 - 2.0 * overlap), hfov)
        source = f'derived from extent and {100 * overlap:.0f} % overlap along X'
    else:
        source = 'parameter'

    footprint = camera_footprint(altitude, hfov, width, height)
    spacing = ((extent_x - footprint[0]) / 2.0, (extent_y - footprint[1]) / 2.0)
    if spacing[0] <= 0.0 or spacing[1] <= 0.0:
        raise ValueError(
            f'altitude {altitude:.2f} m is too high for a {extent_x:g} x {extent_y:g} m mosaic: '
            f'one footprint is already {footprint[0]:.2f} x {footprint[1]:.2f} m. '
            f'Fly between {altitude_min:.2f} and {altitude_max:.2f} m')
    overlaps = (1.0 - spacing[0] / footprint[0], 1.0 - spacing[1] / footprint[1])
    if min(overlaps) < min_overlap:
        raise ValueError(
            f'altitude {altitude:.2f} m is too low for a {extent_x:g} x {extent_y:g} m mosaic: '
            f'footprint {footprint[0]:.2f} x {footprint[1]:.2f} m gives '
            f'{100 * overlaps[0]:.0f} % / {100 * overlaps[1]:.0f} % overlap, below the '
            f'{100 * min_overlap:.0f} % minimum. Fly between {altitude_min:.2f} and '
            f'{altitude_max:.2f} m')
    return MosaicPlan(
        altitude=altitude,
        footprint=footprint,
        spacing=spacing,
        overlap=overlaps,
        extent=(footprint[0] + 2.0 * spacing[0], footprint[1] + 2.0 * spacing[1]),
        gsd_m=footprint[0] / width,
        altitude_min=altitude_min,
        altitude_max=altitude_max,
        altitude_source=source,
    )


def _format_table(camera_name: str, hfov: float, width: int, height: int,
                  extent: Tuple[float, float], overlap: float, min_overlap: float,
                  altitudes: Sequence[float]) -> str:
    lines = [
        f'camera: {camera_name}',
        f'  {width} x {height} px, hfov {math.degrees(hfov):.1f} deg '
        f'({hfov:.4f} rad), vfov '
        f'{math.degrees(2.0 * math.atan(math.tan(hfov / 2.0) * height / width)):.1f} deg',
        f'mosaic: {extent[0]:g} x {extent[1]:g} m (3x3 cells, X = image right, Y = image top), '
        f'minimum overlap {100 * min_overlap:.0f} %',
        '',
        f'{"alt m":>7} {"footprint m":>15} {"step m":>15} {"overlap X/Y":>13} '
        f'{"mosaic m":>13} {"GSD mm":>7}  note',
    ]
    for altitude in altitudes:
        try:
            plan = plan_mosaic(hfov, width, height, extent, altitude, overlap, min_overlap)
        except ValueError as error:
            reason = 'too high' if 'too high' in str(error) else 'too low'
            footprint = camera_footprint(altitude, hfov, width, height)
            lines.append(f'{altitude:7.2f} {footprint[0]:7.2f} x {footprint[1]:5.2f} '
                         f'{"-":>15} {"-":>13} {"-":>13} {"-":>7}  {reason}')
            continue
        lines.append(
            f'{plan.altitude:7.2f} {plan.footprint[0]:7.2f} x {plan.footprint[1]:5.2f} '
            f'{plan.spacing[0]:7.2f} x {plan.spacing[1]:5.2f} '
            f'{100 * plan.overlap[0]:5.0f}%/{100 * plan.overlap[1]:3.0f}% '
            f'{plan.extent[0]:6.1f} x {plan.extent[1]:4.1f} {1000 * plan.gsd_m:7.1f}'
            + ('  <- recommended' if plan.altitude_source != 'parameter' else ''))
    return '\n'.join(lines)


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description='Altitude / grid step for the 3x3 mosaic (default: Arducam B0353, 20 x 10 m).')
    parser.add_argument('--extent', nargs=2, type=float, metavar=('X', 'Y'),
                        default=list(DEFAULT_EXTENT_M), help='mosaic size in metres')
    parser.add_argument('--overlap', type=float, default=DEFAULT_OVERLAP,
                        help='overlap along X used to derive the altitude (default 0.5)')
    parser.add_argument('--min-overlap', type=float, default=DEFAULT_MIN_OVERLAP)
    parser.add_argument('--altitude', type=float, default=None,
                        help='check this altitude instead of deriving one')
    parser.add_argument('--width', type=int, default=ARDUCAM_B0353.width)
    parser.add_argument('--height', type=int, default=ARDUCAM_B0353.height)
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--hfov-rad', type=float, default=None)
    group.add_argument('--hfov-deg', type=float, default=None)
    group.add_argument('--fx', type=float, default=None,
                       help='fx in pixels from a calibration YAML (camera_matrix[0])')
    group.add_argument('--gazebo', action='store_true',
                       help='the SITL survey camera (hfov 1.2 rad, 1280x960)')
    args = parser.parse_args(argv)

    width, height = args.width, args.height
    if args.gazebo:
        hfov, name = GAZEBO_SURVEY_HFOV_RAD, 'Gazebo x500_survey camera'
        if (width, height) == (ARDUCAM_B0353.width, ARDUCAM_B0353.height):
            width, height = 1280, 960
    elif args.hfov_rad is not None:
        hfov, name = args.hfov_rad, 'custom hfov'
    elif args.hfov_deg is not None:
        hfov, name = math.radians(args.hfov_deg), 'custom hfov'
    elif args.fx is not None:
        hfov, name = 2.0 * math.atan(width / (2.0 * args.fx)), f'calibrated fx={args.fx:.1f} px'
    else:
        hfov, name = ARDUCAM_B0353.hfov_rad, ARDUCAM_B0353.name

    extent = (args.extent[0], args.extent[1])
    try:
        recommended = plan_mosaic(hfov, width, height, extent, args.altitude, args.overlap,
                                  args.min_overlap)
    except ValueError as error:
        print(f'error: {error}')
        return 2
    print(_format_table(name, hfov, width, height, extent, args.overlap, args.min_overlap,
                        [a / 2.0 for a in range(6, 31)] if args.altitude is None
                        else [args.altitude]))
    print()
    print(recommended.describe())
    print(f'launch: target_altitude:={recommended.altitude:.2f} camera_hfov_rad:={hfov:.4f} '
          f'camera_width:={width} camera_height:={height} '
          f'mosaic_extent_x:={extent[0]:g} mosaic_extent_y:={extent[1]:g}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
