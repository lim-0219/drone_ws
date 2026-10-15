"""Coordinate conventions: image u right/v down; local X right/Y up, metres."""
from __future__ import annotations

import numpy as np

CAPTURE_ORDER = (5, 4, 1, 2, 3, 6, 9, 8, 7)
GRID = {i: ((i - 1) % 3 - 1, (i - 1) // 3 - 1) for i in range(1, 10)}


def project(matrix, points):
    """Apply a 3x3 homography to an Nx2 array; reject points at infinity."""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    q = np.c_[points, np.ones(len(points))] @ np.asarray(matrix, dtype=float).T
    if not np.isfinite(q).all() or np.any(np.abs(q[:, 2]) < 1e-10):
        raise ValueError("Non-finite projection or point on homography horizon")
    return q[:, :2] / q[:, 2:3]


def normalize_h(matrix):
    matrix = np.asarray(matrix, dtype=float)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError("Homography must be a finite 3x3 matrix")
    if abs(matrix[2, 2]) < 1e-12 or abs(np.linalg.det(matrix)) < 1e-14:
        raise ValueError("Singular homography")
    return matrix / matrix[2, 2]


def corners(width, height):
    # Pixel centres are integer coordinates; footprint boundaries are half-integers.
    return np.array([[-.5, -.5], [width - .5, -.5],
                     [width - .5, height - .5], [-.5, height - .5]])


def center(width, height):
    return np.array([[(width - 1) / 2, (height - 1) / 2]])


def pixel_normalizer(width, height):
    return np.array([[1 / width, 0, -(width - 1) / (2 * width)],
                     [0, 1 / height, -(height - 1) / (2 * height)], [0, 0, 1.]])


def image_to_ground(width, height, footprint, xy=(0, 0), angle_rad=0, anchor_pixel=None):
    """angle: image top clockwise from local +Y. No roll/pitch model."""
    c, s = np.cos(angle_rad), np.sin(angle_rad)
    sx, sy = np.asarray(footprint) / [width, height]
    a = np.array([[c * sx, -s * sy], [-s * sx, -c * sy]])
    h = np.eye(3)
    h[:2, :2] = a
    anchor = center(width, height)[0] if anchor_pixel is None else np.asarray(anchor_pixel)
    h[:2, 2] = np.asarray(xy) - a @ anchor
    return h


def ned_to_local_matrix(origin_ned, heading_rad):
    """[absolute north, east, 1] -> [local X, Y, 1]. +Y heading clockwise N."""
    c, s = np.cos(heading_rad), np.sin(heading_rad)
    a = np.array([[-s, c], [c, s]])
    h = np.eye(3)
    h[:2, :2] = a
    h[:2, 2] = -a @ np.asarray(origin_ned)[:2]
    return h


def validate_footprint(h, width, height):
    """Reject horizons through an image and folded/degenerate quadrilaterals."""
    h = normalize_h(h)
    p = corners(width, height)
    z = np.c_[p, np.ones(4)] @ h[2]
    if z.min() <= 0 <= z.max() or np.min(np.abs(z)) < 1e-6:
        raise ValueError("Homography horizon intersects image")
    q = project(h, p)
    e = np.roll(q, -1, axis=0) - q
    e_next = np.roll(e, -1, axis=0)
    cross = e[:, 0] * e_next[:, 1] - e[:, 1] * e_next[:, 0]
    if not (np.all(cross > 1e-10) or np.all(cross < -1e-10)):
        raise ValueError("Folded or degenerate image footprint")
    return q


def signed_area(points):
    x, y = np.asarray(points).T
    return float(.5 * np.sum(x * np.roll(y, -1) - y * np.roll(x, -1)))
