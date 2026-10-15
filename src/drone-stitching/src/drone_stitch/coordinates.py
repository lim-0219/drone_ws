"""Load exported transforms to place a drone or a detected image point on the map."""
from __future__ import annotations

import json
from pathlib import Path
import cv2
import numpy as np

from .geometry import project


class MapCoordinates:
    def __init__(self, path):
        self.path = Path(path)
        self.data = json.loads(self.path.read_text(encoding="utf-8"))
        if self.data.get("schema_version") != 1:
            raise ValueError("Unsupported transforms schema")

    def local_to_pixel(self, xy_m):
        return project(self.data["map"]["local_to_map"], xy_m)

    def pixel_to_local(self, uv):
        return project(self.data["map"]["map_to_local"], uv)

    def ned_to_pixel(self, north_east_m, frame_id=None):
        origin = self.data["origin"]
        if origin["ned_ne_to_local_matrix"] is None:
            raise ValueError("NED alignment unavailable; need reference NED and grid heading or reference yaw")
        if origin.get("ned_frame_id") and frame_id != origin["ned_frame_id"]:
            raise ValueError("Supply the recorded ned_frame_id; an EKF reset/session change needs re-anchoring")
        xy = project(origin["ned_ne_to_local_matrix"], north_east_m)
        return self.local_to_pixel(xy)

    def image_to_pixel(self, image_id, uv, raw_pixels=False):
        points = np.asarray(uv, dtype=float).reshape(-1, 2)
        calibration = self.data["camera"].get("calibration")
        if raw_pixels and calibration:
            k = np.asarray(calibration["K"], dtype=float)
            d = np.asarray(calibration["distortion"], dtype=float)
            points = cv2.undistortPoints(points.reshape(-1, 1, 2), k, d, P=k).reshape(-1, 2)
        return project(self.data["images"][str(image_id)]["rectified_image_to_map"], points)

    def coverage_at(self, uv):
        points = np.asarray(uv, dtype=float).reshape(-1, 2)
        if not np.isfinite(points).all():
            raise ValueError("Coordinates must be finite")
        mask = cv2.imread(str(self.path.parent / self.data["map"]["valid_mask"]), cv2.IMREAD_GRAYSCALE)
        if mask is None:
            raise ValueError("Missing valid_mask.png")
        inside = ((points[:, 0] >= -.5) & (points[:, 0] < mask.shape[1] - .5)
                  & (points[:, 1] >= -.5) & (points[:, 1] < mask.shape[0] - .5))
        covered = np.zeros(len(points), dtype=bool)
        p = np.floor(points[inside] + .5).astype(int)
        covered[inside] = mask[p[:, 1], p[:, 0]] > 0
        return {"inside_canvas": inside.tolist(), "covered": covered.tolist()}

