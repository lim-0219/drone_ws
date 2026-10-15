"""Deterministic artificial planar scenes, independent ground-truth transforms."""
from __future__ import annotations

import csv
import json
from pathlib import Path
import cv2
import numpy as np

from .geometry import CAPTURE_ORDER, GRID, center, image_to_ground, project
from .pipeline import write_image, write_json


def generate(output_dir, seed=7):
    out = Path(output_dir)
    if out.exists() and any(out.iterdir()):
        raise ValueError("Synthetic output directory must be new or empty")
    out.mkdir(parents=True, exist_ok=True)
    (out / "images").mkdir(exist_ok=True)
    rng = np.random.default_rng(seed)
    # 20 x 30 m textured plane sampled at 100 px/m. No generated/real-world photos.
    h, w, ppm = 3000, 2000, 100.
    noise = rng.integers(30, 215, (h, w), dtype=np.uint8)
    gray = cv2.GaussianBlur(noise, (5, 5), 1.)
    ground = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    for _ in range(3600):
        x, y = int(rng.integers(w)), int(rng.integers(h))
        radius = int(rng.integers(3, 17))
        colour = tuple(int(c) for c in rng.integers(5, 250, 3))
        if rng.random() < .5:
            cv2.circle(ground, (x, y), radius, colour, -1, cv2.LINE_AA)
        else:
            cv2.rectangle(ground, (x, y), (x + radius * 2, y + radius), colour, -1)
    for i in range(60):
        x, y = int(rng.integers(100, w - 180)), int(rng.integers(100, h - 100))
        cv2.putText(ground, f"S{i:02d}", (x, y), cv2.FONT_HERSHEY_SIMPLEX, .8, (20, 20, 20), 2, cv2.LINE_AA)
    cv2.rectangle(ground, (750, 1350), (850, 1450), (0, 0, 0), -1)
    world_to_texture = np.array([[ppm, 0, w / 2], [0, -ppm, h / 2], [0, 0, 1.]])
    footprint = [5., 10.]
    iw, ih = 400, 800
    heading = .4
    ned_origin = np.array([100., 200., -5.])
    # Inverse NED-to-local linear map: columns are local X and Y basis in NE.
    local_to_ne = np.array([[-np.sin(heading), np.cos(heading)], [np.cos(heading), np.sin(heading)]])
    entries, truth, telemetry = [], {}, []
    for seq, image_id in enumerate(CAPTURE_ORDER):
        xy = np.array(GRID[image_id], dtype=float) * [2.5, 5.]
        angle, altitude = 0., 5.
        if image_id != 5:
            xy += rng.normal(0, .12, 2)
            angle = float(rng.uniform(-.055, .055))
            altitude += float(rng.uniform(-.12, .12))
        matrix = image_to_ground(iw, ih, np.array(footprint) * altitude / 5, xy, angle)
        if image_id != 5:
            # Small projective perturbation about image centre emulates mild tilt.
            c = center(iw, ih)[0]
            t = np.array([[1., 0, -c[0]], [0, 1., -c[1]], [0, 0, 1.]])
            perturb = np.eye(3)
            perturb[2, :2] = rng.uniform(-1e-5, 1e-5, 2)
            matrix = matrix @ np.linalg.inv(t) @ perturb @ t
        image = cv2.warpPerspective(ground, np.linalg.inv(world_to_texture @ matrix), (iw, ih), flags=cv2.INTER_LINEAR)
        if image_id != 5:
            image = np.clip(image.astype(float) * rng.uniform(.9, 1.1) + rng.uniform(-3, 3), 0, 255).astype(np.uint8)
        filename = f"images/capture_{seq + 1:02d}_id{image_id}.png"
        write_image(out / filename, image)
        time = 1000. + seq * 2
        entry = {"id": image_id, "file": filename, "timestamp_s": time}
        entries.append(entry)
        ne = ned_origin[:2] + local_to_ne @ xy
        # Telemetry has noise; image 5 anchors the same NE origin exactly.
        measured_ne = ne + (rng.normal(0, .06, 2) if image_id != 5 else 0)
        measured_yaw = heading + angle + (float(rng.normal(0, .005)) if image_id != 5 else 0)
        for dt in (-.1, .1):
            telemetry.append([time + dt, measured_ne[0], measured_ne[1], -altitude, measured_yaw, altitude])
        truth[str(image_id)] = {"image_to_local": matrix.tolist(), "center_local_m": xy.tolist(),
                                "ned_m": [float(ne[0]), float(ne[1]), -altitude], "yaw_rad": heading + angle}
    manifest = {"version": 1, "dataset_kind": "synthetic", "ned_frame_id": "synthetic-session-1",
                "camera": {"footprint_m": footprint, "reference_agl_m": 5., "image_top_yaw_offset_rad": 0.},
                "grid": {"spacing_m": [2.5, 5.], "heading_rad": heading},
                "telemetry_csv": "telemetry.csv", "images": entries}
    write_json(out / "manifest.json", manifest)
    no_metadata = json.loads(json.dumps(manifest))
    no_metadata.pop("telemetry_csv")
    no_metadata.pop("ned_frame_id")
    no_metadata["grid"].pop("heading_rad")
    write_json(out / "manifest_no_metadata.json", no_metadata)
    with (out / "telemetry.csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp_s", "north_m", "east_m", "down_m", "yaw_rad", "agl_m"])
        writer.writerows(telemetry)
    write_image(out / "ground_truth.png", ground)
    write_json(out / "ground_truth.json", {"dataset_kind": "synthetic", "seed": seed,
                                          "image_size": [iw, ih], "world_to_texture": world_to_texture.tolist(), "images": truth})
    return out / "manifest.json"


def evaluate(transforms_path, truth_path):
    result = json.loads(Path(transforms_path).read_text())
    truth = json.loads(Path(truth_path).read_text())
    if result.get("dataset_kind") != "synthetic" or truth.get("dataset_kind") != "synthetic":
        raise ValueError("This evaluation requires synthetic data and synthetic ground truth")
    errors = []
    per_image = {}
    for image_id, image in result["images"].items():
        w, h = image["image_size"]
        x, y = np.meshgrid(np.linspace(0, w - 1, 7), np.linspace(0, h - 1, 7))
        points = np.c_[x.ravel(), y.ravel()]
        actual = project(image["rectified_image_to_local"], points)
        expected = project(truth["images"][image_id]["image_to_local"], points)
        error = np.linalg.norm(actual - expected, axis=1)
        errors.extend(error.tolist())
        per_image[image_id] = {"rmse_m": float(np.sqrt(np.mean(error**2))), "max_m": float(error.max())}
    return {"validation_type": "synthetic_only", "point_count": len(errors),
            "rmse_m": float(np.sqrt(np.mean(np.square(errors)))), "max_m": float(max(errors)),
            "per_image": per_image, "real_data_validated": False}

