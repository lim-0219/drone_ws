from __future__ import annotations

from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
import logging
from pathlib import Path
import platform
import tempfile
import time
import cv2
import numpy as np
import scipy

from . import __version__
from .config import Config
from .features import match_graph
from .geometry import CAPTURE_ORDER, center, project, validate_footprint
from .inputs import initial_placement, read_manifest
from .optimize import optimize, validate_solution
from .render import blend, preview_overlay

LOG = logging.getLogger(__name__)


def write_json(path, data):
    Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def write_image(path, data):
    if not cv2.imwrite(str(path), data):
        raise OSError(f"Cannot write {path}")


def run_pipeline(manifest_path, output_dir, config=None):
    """Create a new run directory; never silently overwrite a previous result."""
    config = (config or Config()).validate()
    output_dir = Path(output_dir).resolve()
    if output_dir.exists() and any(output_dir.iterdir()):
        raise ValueError(f"Output directory must be new or empty: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    report = {"schema_version": 1, "status": "running", "warnings": [], "config": asdict(config),
              "created_utc": datetime.now(timezone.utc).isoformat(),
              "environment": {"python": platform.python_version(), "platform": platform.platform(),
                              "opencv": cv2.__version__, "numpy": np.__version__, "scipy": scipy.__version__,
                              "package": __version__, "execution": "CPU"}}
    try:
        cv2.setNumThreads(config.opencv_threads)
        cv2.setRNGSeed(config.seed)
        cv2.ocl.setUseOpenCL(False)
        LOG.info("Loading nine images and resolving metadata")
        doc, frames, warnings = read_manifest(manifest_path, config)
        report["warnings"].extend(warnings)
        report["dataset_kind"] = doc.get("dataset_kind", "unspecified")
        initial, placement, origin, warnings = initial_placement(doc, frames)
        report["warnings"].extend(warnings)
        report["warnings"].append("Metric coordinates are approximate: anchored to image 5 and configured footprint; not surveyed ground truth")
        report["initial_placement"] = placement
        LOG.info("Extracting %s features and matching neighboring images", config.detector.upper())
        edges = match_graph(frames, config, report)
        LOG.info("Accepted %d / %d overlap constraints; optimizing all images", len(edges), len(report["pairs"]))
        refined, optimization_report = optimize(frames, initial, placement, edges, config)
        report["optimization"] = optimization_report
        validate_solution(frames, initial, refined, optimization_report, config)
        if any(sum(e.a == f.id or e.b == f.id for e in edges) < 2 for f in frames):
            report["warnings"].append("Some images have only one overlap link; loop verification is unavailable for those images")
        LOG.info("Warping and blending")
        mosaic, valid_mask, coverage, local_to_map, image_to_map = blend(frames, refined, config)
        origin["reference_map_pixel"] = project(local_to_map, [[0, 0]])[0].tolist()
        origin["ned_frame_id"] = doc.get("ned_frame_id")
        if origin["ned_ne_to_local_matrix"] is not None and not origin["ned_frame_id"]:
            report["warnings"].append("No ned_frame_id recorded: verify the same Pixhawk EKF origin before displaying live positions")
        images = {}
        for f in frames:
            images[str(f.id)] = {
                "file": str(f.path), "sha256": hashlib.sha256(f.path.read_bytes()).hexdigest(),
                "image_size": [f.width, f.height], "timestamp_s": f.timestamp_s,
                "metadata": f.metadata, "metadata_provenance": f.provenance,
                "initial_rectified_image_to_local": initial[f.id].tolist(),
                "rectified_image_to_local": refined[f.id].tolist(),
                "rectified_image_to_map": image_to_map[f.id].tolist(),
                "map_to_rectified_image": np.linalg.inv(image_to_map[f.id]).tolist(),
                "center_local_m": project(refined[f.id], center(f.width, f.height))[0].tolist(),
                "nadir_anchor_pixel": list(f.anchor_pixel),
                "estimated_capture_local_m": project(refined[f.id], [f.anchor_pixel])[0].tolist(),
                "footprint_local_m": validate_footprint(refined[f.id], f.width, f.height).tolist()}
        transforms = {
            "schema_version": 1, "dataset_kind": report["dataset_kind"], "capture_order": list(CAPTURE_ORDER),
            "coordinate_convention": {"local": "X grid-right, Y grid-up, meters; image 5 nadir anchor is (0,0)",
                                      "pixels": "u right, v down; integer pixel centers; column vectors [u,v,1]",
                                      "matrix_application": "q = H @ p; divide q[:2] by q[2]",
                                      "ned": "north/east/down in meters; absolute within a single EKF origin",
                                      "yaw": "radians, clockwise from north; image-top mounting offset added"},
            "metric_reference": {"status": "approximate_configured_footprint",
                                 "method": "image_5_nadir_footprint_anchor_with_weak_pose_priors",
                                 "survey_validated": False, "note": "Image registration is not metric/geodetic calibration"},
            "map": {"file": "map.png", "valid_mask": "valid_mask.png", "coverage_count": "coverage.png",
                    "size_pixels": [mosaic.shape[1], mosaic.shape[0]], "pixels_per_meter": config.pixels_per_meter,
                    "local_to_map": local_to_map.tolist(), "map_to_local": np.linalg.inv(local_to_map).tolist()},
            "origin": origin, "camera": doc["camera"], "images": images,
            "input_manifest": doc, "warnings": report["warnings"]}
        report["map_size_pixels"] = transforms["map"]["size_pixels"]
        report["valid_pixel_fraction"] = float(np.mean(valid_mask > 0))
        report["elapsed_seconds"] = time.perf_counter() - started
        report["status"] = "success"
        with tempfile.TemporaryDirectory(prefix=".export-", dir=output_dir) as temp:
            staging = Path(temp)
            write_image(staging / "map.png", mosaic)
            write_image(staging / "valid_mask.png", valid_mask)
            write_image(staging / "coverage.png", coverage)
            write_image(staging / "overview.png", preview_overlay(mosaic, frames, refined, local_to_map))
            write_json(staging / "transforms.json", transforms)
            write_json(staging / "diagnostics.json", report)
            for item in staging.iterdir():
                item.replace(output_dir / item.name)
        LOG.info("Saved %s (%dx%d)", output_dir / "map.png", mosaic.shape[1], mosaic.shape[0])
        return transforms, report
    except Exception as e:
        report.update(status="failed", error=f"{type(e).__name__}: {e}", elapsed_seconds=time.perf_counter() - started)
        write_json(output_dir / "diagnostics.json", report)
        raise
