"""Optional timestamped telemetry; interpolate yaw along the shortest arc."""
from __future__ import annotations

import csv
import numpy as np


def validate_metadata(data):
    result = {}
    for name, size in (("ned_m", 3), ("yaw_rad", 1), ("agl_m", 1)):
        if data.get(name) is None:
            continue
        a = np.asarray(data[name], dtype=float)
        if (size == 3 and a.shape != (3,)) or (size == 1 and a.ndim != 0):
            raise ValueError(f"Invalid {name} shape")
        if not np.isfinite(a).all():
            raise ValueError(f"Non-finite {name}")
        if name == "agl_m" and float(a) <= 0:
            raise ValueError("agl_m must be positive; NED z is not ground height")
        result[name] = a.tolist()
    return result


def read_telemetry(path):
    rows = []
    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        if "timestamp_s" not in (reader.fieldnames or []):
            raise ValueError("Telemetry CSV requires timestamp_s")
        for row in reader:
            t = float(row["timestamp_s"])
            if not np.isfinite(t):
                raise ValueError("Non-finite telemetry timestamp")
            item = {"timestamp_s": t}
            ned = [row.get(k, "") for k in ("north_m", "east_m", "down_m")]
            if any(ned) and not all(ned):
                raise ValueError("Provide all north_m/east_m/down_m, or leave all empty")
            if all(ned):
                item["ned_m"] = list(map(float, ned))
            for key in ("yaw_rad", "agl_m"):
                if row.get(key):
                    item[key] = float(row[key])
            rows.append({"timestamp_s": t, **validate_metadata(item)})
    if any(b["timestamp_s"] <= a["timestamp_s"] for a, b in zip(rows, rows[1:])):
        raise ValueError("Telemetry timestamps must strictly increase (no clock resets/duplicates)")
    return rows


def sample_telemetry(rows, timestamp, max_gap_s=.5, nearest_tolerance_s=.05):
    """Each signal sampled independently. Never bridge large gaps or extrapolate."""
    values, provenance = {}, {}
    for key in ("ned_m", "yaw_rad", "agl_m"):
        samples = [r for r in rows if key in r]
        if not samples:
            continue
        ts = np.array([r["timestamp_s"] for r in samples])
        i = int(np.searchsorted(ts, timestamp))
        nearest = int(np.argmin(abs(ts - timestamp)))
        if abs(ts[nearest] - timestamp) < 1e-9:
            value = samples[nearest][key]
            source = {"method": "exact", "sample_times_s": [float(ts[nearest])]}
        elif 0 < i < len(ts) and ts[i] - ts[i - 1] <= max_gap_s:
            fraction = (timestamp - ts[i - 1]) / (ts[i] - ts[i - 1])
            a, b = np.asarray(samples[i - 1][key]), np.asarray(samples[i][key])
            delta = b - a
            if key == "yaw_rad":
                delta = np.arctan2(np.sin(delta), np.cos(delta))
            value = (a + fraction * delta).tolist()
            source = {"method": "interpolated", "sample_times_s": ts[i - 1:i + 1].tolist()}
        elif abs(ts[nearest] - timestamp) <= nearest_tolerance_s:
            value = samples[nearest][key]
            source = {"method": "nearest", "sample_times_s": [float(ts[nearest])]}
        else:
            continue
        values[key], provenance[key] = value, source
    return values, provenance

