from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import cv2
import numpy as np

from .geometry import CAPTURE_ORDER, GRID, image_to_ground, ned_to_local_matrix, project
from .metadata import read_telemetry, sample_telemetry, validate_metadata


@dataclass
class Frame:
    id: int
    path: Path
    image: np.ndarray
    mask: np.ndarray
    metadata: dict
    provenance: dict
    timestamp_s: float | None
    anchor_pixel: tuple | None = None

    @property
    def width(self):
        return self.image.shape[1]

    @property
    def height(self):
        return self.image.shape[0]


def _positive_pair(value, name):
    a = np.asarray(value, dtype=float)
    if a.shape != (2,) or not np.isfinite(a).all() or np.any(a <= 0):
        raise ValueError(f"{name} requires two positive finite numbers")
    return a


def read_manifest(path, config):
    path = Path(path).resolve()
    doc = json.loads(path.read_text(encoding="utf-8"))
    if doc.get("version") != 1:
        raise ValueError("Manifest version must be 1")
    entries = doc.get("images", [])
    if len(entries) != 9:
        raise ValueError("Exactly nine images are required")
    ids = [entry.get("id", CAPTURE_ORDER[i]) for i, entry in enumerate(entries)]
    if ids != list(CAPTURE_ORDER):
        raise ValueError(f"images must be in capture order {list(CAPTURE_ORDER)}")
    if doc.get("dataset_kind", "unspecified") not in ("synthetic", "real", "unspecified"):
        raise ValueError("dataset_kind must be synthetic, real, or unspecified")
    camera = doc.get("camera", {})
    _positive_pair(camera.get("footprint_m"), "camera.footprint_m [image width, image height]")
    ref_agl = float(camera.get("reference_agl_m", 5))
    offset = float(camera.get("image_top_yaw_offset_rad", 0))
    if not np.isfinite([ref_agl, offset]).all() or ref_agl <= 0:
        raise ValueError("Invalid camera altitude or mount yaw")
    grid = doc.get("grid", {})
    _positive_pair(grid.get("spacing_m", [2.5, 5]), "grid.spacing_m")
    if grid.get("heading_rad") is not None and not np.isfinite(float(grid["heading_rad"])):
        raise ValueError("Invalid grid.heading_rad")
    rows = read_telemetry(path.parent / doc["telemetry_csv"]) if doc.get("telemetry_csv") else []
    frames, warnings, paths = [], [], set()
    for frame_id, entry in zip(ids, entries):
        image_path = (path.parent / entry["file"]).resolve()
        if image_path in paths:
            raise ValueError("Duplicate image file in manifest")
        paths.add(image_path)
        # Ignore EXIF auto-rotation: calibration and matrices refer to stored pixels.
        im = cv2.imread(str(image_path), cv2.IMREAD_COLOR | cv2.IMREAD_IGNORE_ORIENTATION)
        if im is None:
            raise ValueError(f"Cannot read image {image_path}")
        if min(im.shape[:2]) < 64:
            raise ValueError(f"Image {frame_id} is too small (<64 px)")
        mask = np.full(im.shape[:2], 255, np.uint8)
        calibration = camera.get("calibration")
        anchor = ((im.shape[1] - 1) / 2, (im.shape[0] - 1) / 2)
        if calibration:
            if calibration.get("image_size") != [im.shape[1], im.shape[0]]:
                raise ValueError("Calibration image_size must equal stored image resolution")
            k = np.asarray(calibration["K"], dtype=float)
            dist = np.asarray(calibration["distortion"], dtype=float)
            if (k.shape != (3, 3) or dist.ndim != 1 or dist.size not in (4, 5, 8, 12, 14)
                    or not np.isfinite(k).all() or not np.isfinite(dist).all()
                    or k[0, 0] <= 0 or k[1, 1] <= 0
                    or not np.allclose(k[2], [0, 0, 1]) or abs(k[0, 1]) > 1e-9 or abs(k[1, 0]) > 1e-9
                    or not 0 <= k[0, 2] < im.shape[1] or not 0 <= k[1, 2] < im.shape[0]):
                raise ValueError("Invalid OpenCV calibration K/distortion")
            mx, my = cv2.initUndistortRectifyMap(k, dist, None, k, (im.shape[1], im.shape[0]), cv2.CV_32FC1)
            im = cv2.remap(im, mx, my, cv2.INTER_LINEAR)
            mask = cv2.remap(mask, mx, my, cv2.INTER_LINEAR)
            mask = (mask == 255).astype(np.uint8) * 255
            anchor = (float(k[0, 2]), float(k[1, 2]))
        timestamp = entry.get("timestamp_s")
        if timestamp is not None:
            timestamp = float(timestamp)
            if not np.isfinite(timestamp):
                raise ValueError("Image timestamp must be finite")
        metadata, provenance = {}, {}
        if rows and timestamp is not None:
            metadata, provenance = sample_telemetry(rows, timestamp, config.telemetry_max_gap_s,
                                                    config.telemetry_nearest_tolerance_s)
            if not metadata:
                warnings.append(f"Image {frame_id}: no telemetry close enough to exposure timestamp")
        direct = validate_metadata(entry)
        metadata.update(direct)
        provenance.update({k: {"method": "manifest"} for k in direct})
        frames.append(Frame(frame_id, image_path, im, mask, metadata, provenance, timestamp, anchor))
    times = [f.timestamp_s for f in frames if f.timestamp_s is not None]
    if any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError("Capture timestamps must increase in capture order")
    if not camera.get("calibration"):
        warnings.append("Lens distortion is uncorrected unless inputs were already rectified")
    return doc, frames, warnings


def initial_placement(doc, frames):
    camera, grid = doc["camera"], doc.get("grid", {})
    reference = frames[0]
    offset = float(camera.get("image_top_yaw_offset_rad", 0))
    ref_yaw = reference.metadata.get("yaw_rad")
    heading = grid.get("heading_rad")
    heading_source = "configured_grid_heading"
    if heading is None and ref_yaw is not None:
        heading = ref_yaw + offset
        heading_source = "reference_yaw_plus_camera_mount"
    elif heading is None:
        heading_source = "unknown_assumed_alignment"
    origin_ned = reference.metadata.get("ned_m")
    ned_h = ned_to_local_matrix(origin_ned, heading) if origin_ned is not None and heading is not None else None
    warnings = []
    if ned_h is None and any("ned_m" in f.metadata for f in frames):
        warnings.append("NED positions ignored: image 5 needs NED plus grid heading or reference yaw")
    if heading is None and any("yaw_rad" in f.metadata for f in frames):
        warnings.append("Yaw ignored: grid heading or image 5 yaw is required")
    spacing = np.asarray(grid.get("spacing_m", [2.5, 5]), dtype=float)
    initial, info = {}, {}
    for f in frames:
        xy = np.array(GRID[f.id]) * spacing
        position_source = "grid"
        if ned_h is not None and "ned_m" in f.metadata:
            xy = project(ned_h, [f.metadata["ned_m"][:2]])[0]
            position_source = "relative_ned"
        if f.id == 5:
            xy = np.zeros(2)
        angle = 0.
        yaw_source = "assumed_grid_alignment"
        if heading is not None and "yaw_rad" in f.metadata:
            angle = f.metadata["yaw_rad"] + offset - heading
            yaw_source = "metadata_yaw"
        agl = f.metadata.get("agl_m", camera.get("reference_agl_m", 5.))
        footprint = np.asarray(camera["footprint_m"]) * agl / camera.get("reference_agl_m", 5.)
        initial[f.id] = image_to_ground(f.width, f.height, footprint, xy, angle, f.anchor_pixel)
        info[f.id] = {"center_local_m": xy.tolist(), "image_top_angle_rad": angle,
                      "footprint_m": footprint.tolist(), "position_source": position_source,
                      "yaw_source": yaw_source, "agl_used_m": agl}
    origin = {"reference_image_id": 5, "reference_local_xy_m": [0., 0.],
              "reference_image_pixel": list(reference.anchor_pixel),
              "reference_timestamp_s": reference.timestamp_s, "reference_ned_m": origin_ned,
              "reference_yaw_rad": ref_yaw, "grid_heading_rad": heading,
              "heading_source": heading_source, "image_top_yaw_offset_rad": offset,
              "ned_ne_to_local_matrix": None if ned_h is None else ned_h.tolist(),
              "local_to_ned_ne_matrix": None if ned_h is None else np.linalg.inv(ned_h).tolist()}
    return initial, info, origin, warnings
