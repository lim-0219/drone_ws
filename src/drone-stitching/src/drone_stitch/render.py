"""Bounded canvas, explicit coverage masks, ROI warping and feather blending."""
from __future__ import annotations

import cv2
import numpy as np

from .geometry import project, validate_footprint


def canvas_geometry(frames, homographies, config):
    polygons = [validate_footprint(homographies[f.id], f.width, f.height) for f in frames]
    points = np.concatenate(polygons)
    ppm = config.pixels_per_meter
    raster = points * [ppm, -ppm]
    low = np.floor(raster.min(axis=0)) - 2
    high = np.ceil(raster.max(axis=0)) + 2
    size = high - low + 1
    if (not np.isfinite(size).all() or np.any(size <= 0) or size.max() > config.max_canvas_dimension
            or size.prod() > config.max_canvas_megapixels * 1e6):
        raise ValueError(f"Canvas {size.tolist()} exceeds configured memory/dimension limit; "
                         "check registration or lower pixels_per_meter")
    transform = np.array([[ppm, 0, -low[0]], [0, -ppm, -low[1]], [0, 0, 1.]])
    return tuple(size.astype(int)), transform


def blend(frames, homographies, config):
    (width, height), ground_to_map = canvas_geometry(frames, homographies, config)
    accumulator = np.zeros((height, width, 3), dtype=np.float32)
    weight_sum = np.zeros((height, width), dtype=np.float32)
    coverage = np.zeros((height, width), dtype=np.uint8)
    image_to_map = {}
    for f in frames:
        h = ground_to_map @ homographies[f.id]
        image_to_map[f.id] = h
        polygon = validate_footprint(h, f.width, f.height)
        low = np.maximum(0, np.floor(polygon.min(axis=0)).astype(int) - 1)
        high = np.minimum([width, height], np.ceil(polygon.max(axis=0)).astype(int) + 2)
        x0, y0 = low
        x1, y1 = high
        roi_size = (int(x1 - x0), int(y1 - y0))
        shift = np.array([[1., 0, -x0], [0, 1, -y0], [0, 0, 1]])
        roi_h = shift @ h
        # Masks distinguish real black image content from absence of coverage.
        mask = (f.mask > 0).astype(np.float32)
        distance = cv2.distanceTransform(np.pad(f.mask, 1), cv2.DIST_L2, 3)[1:-1, 1:-1]
        weights = np.minimum(distance / config.feather_radius_px, 1.) * mask
        warped_weight = cv2.warpPerspective(weights, roi_h, roi_size, flags=cv2.INTER_LINEAR)
        # Warp premultiplied colours so border interpolation never adds dark fringes.
        warped = cv2.warpPerspective(f.image.astype(np.float32) * weights[:, :, None], roi_h,
                                    roi_size, flags=cv2.INTER_LINEAR)
        accumulator[y0:y1, x0:x1] += warped
        weight_sum[y0:y1, x0:x1] += warped_weight
        coverage[y0:y1, x0:x1] += (warped_weight > 1e-6).astype(np.uint8)
    valid = weight_sum > 1e-6
    # Row chunks bound the temporary arrays for a full-resolution map.
    result = np.zeros((height, width, 3), dtype=np.uint8)
    for y in range(0, height, 256):
        colour = accumulator[y:y + 256] / np.maximum(weight_sum[y:y + 256, :, None], 1e-6)
        result[y:y + 256] = np.clip(np.rint(colour), 0, 255).astype(np.uint8)
    return result, valid.astype(np.uint8) * 255, coverage, ground_to_map, image_to_map


def preview_overlay(image, frames, homographies, ground_to_map):
    overlay = image.copy()
    for f in frames:
        h = ground_to_map @ homographies[f.id]
        polygon = np.rint(validate_footprint(h, f.width, f.height)).astype(np.int32)
        cv2.polylines(overlay, [polygon], True, (70, 220, 255), 2, cv2.LINE_AA)
        p = project(h, [[(f.width - 1) / 2, (f.height - 1) / 2]])[0]
        cv2.putText(overlay, str(f.id), tuple(np.rint(p).astype(int)), cv2.FONT_HERSHEY_SIMPLEX,
                    .9, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(overlay, str(f.id), tuple(np.rint(p).astype(int)), cv2.FONT_HERSHEY_SIMPLEX,
                    .9, (70, 220, 255), 2, cv2.LINE_AA)
    origin = tuple(np.rint(project(ground_to_map, [[0, 0]])[0]).astype(int))
    cv2.drawMarker(overlay, origin, (30, 30, 255), cv2.MARKER_CROSS, 32, 3)
    return overlay
