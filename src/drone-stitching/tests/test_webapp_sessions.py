from __future__ import annotations

import json
from pathlib import Path
import time

import cv2
from fastapi.testclient import TestClient
import numpy as np
import pytest

import drone_stitch.webapp as webapp


CAPTURE_ORDER = [5, 4, 1, 2, 3, 6, 9, 8, 7]


def _png_bytes(seed: int) -> bytes:
    image = np.full((72, 96, 3), 40 + seed * 20, dtype=np.uint8)
    image[:, : 48 + seed] = (35, 120, 220)
    ok, encoded = cv2.imencode(".png", image)
    assert ok
    return encoded.tobytes()


def _metadata(sequence: int, **extra) -> str:
    data = {"sequence": sequence, "grid_id": CAPTURE_ORDER[sequence - 1], "total": 9,
            "timestamp_s": 100.0 + sequence, "ned_m": [float(sequence), 0.0, -5.0],
            "yaw_rad": 0.1, "agl_m": 5.0}
    data.update(extra)
    return json.dumps(data)


def _post(client, session_id, sequence, *, metadata=None, payload=None, name=None):
    return client.post(
        f"/api/sessions/{session_id}/frames",
        files={"image": (name or f"capture_{sequence:02d}.png", payload or _png_bytes(sequence), "image/png")},
        data={"metadata": metadata if metadata is not None else _metadata(sequence)},
    )


def _wait(client, session_id, timeout=3.0):
    deadline = time.monotonic() + timeout
    latest = None
    while time.monotonic() < deadline:
        latest = client.get(f"/api/sessions/{session_id}").json()
        if latest["status"] in {"completed", "failed"}:
            return latest
        time.sleep(0.01)
    pytest.fail(f"session did not finish: {latest}")


@pytest.fixture
def app_client(tmp_path):
    static_dir = tmp_path / "static"
    static_dir.mkdir()
    app = webapp.create_app(runs_root=tmp_path / "runs", static_dir=static_dir)
    try:
        with TestClient(app) as client:
            yield app, client, tmp_path / "runs"
    finally:
        app.state.executor.shutdown(wait=True, cancel_futures=True)


def test_frames_fill_a_session_and_the_ninth_starts_the_stitch(app_client, monkeypatch):
    _, client, runs_root = app_client
    observed = {}

    def fake_pipeline(manifest_path: Path, output_dir: Path):
        observed["manifest"] = json.loads(Path(manifest_path).read_text(encoding="utf-8"))
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True)
        (output_dir / "map.png").write_bytes(_png_bytes(0))
        return {}, {"elapsed_seconds": 0.5}

    monkeypatch.setattr(webapp, "run_pipeline", fake_pipeline)
    camera = {"footprint_m": [6.8, 5.1], "reference_agl_m": 5.0, "image_top_yaw_offset_rad": 0.0}
    grid = {"spacing_m": [3.4, 2.5]}

    assert client.get("/api/sessions/latest").status_code == 404
    for sequence in range(1, 9):
        response = _post(client, "flight-1", sequence,
                         metadata=_metadata(sequence, camera=camera, grid=grid, ned_frame_id="ekf-0"))
        assert response.status_code == 202, response.text
        body = response.json()
        assert body["received"] == sequence and body["status"] == "collecting"
        assert body["missing"] == list(range(sequence + 1, 10))
    latest = client.get("/api/sessions/latest").json()
    assert latest["id"] == "flight-1" and latest["received"] == 8

    response = _post(client, "flight-1", 9)
    assert response.status_code == 202, response.text
    assert response.json()["status"] in {"queued", "scanning", "completed"}

    done = _wait(client, "flight-1")
    assert done["status"] == "completed"
    assert done["map_url"] == f"/api/jobs/{done['job_id']}/map"
    assert Path(done["output_dir"]) == runs_root / "sessions" / "flight-1" / "output"
    assert client.get(done["map_url"]).status_code == 200
    assert client.get(f"/api/jobs/{done['job_id']}").json()["status"] == "completed"

    manifest = observed["manifest"]
    assert [item["id"] for item in manifest["images"]] == CAPTURE_ORDER
    assert [item["file"] for item in manifest["images"]] == [
        f"capture_{n:02d}_id{CAPTURE_ORDER[n - 1]}.png" for n in range(1, 10)]
    assert manifest["camera"] == camera and manifest["grid"] == grid
    assert manifest["ned_frame_id"] == "ekf-0"
    assert manifest["images"][0]["ned_m"] == [1.0, 0.0, -5.0]
    assert manifest["images"][8]["timestamp_s"] == 109.0
    assert all(item["agl_m"] == 5.0 for item in manifest["images"])

    # The session is sealed once it stitches.
    assert _post(client, "flight-1", 3).status_code == 409


def test_session_rejects_wrong_order_and_bad_metadata(app_client):
    _, client, _ = app_client
    wrong_cell = _metadata(2, grid_id=1)      # sequence 2 must be cell 4
    assert _post(client, "s", 2, metadata=wrong_cell).status_code == 422
    assert _post(client, "s", 1, metadata="not json").status_code == 422
    assert _post(client, "s", 1, metadata=json.dumps({"sequence": 1})).status_code == 422
    assert _post(client, "s", 1, metadata=_metadata(1, total=4)).status_code == 422
    assert _post(client, "bad id!", 1).status_code == 422
    assert _post(client, "s", 1, name="capture.txt").status_code == 415
    assert client.get("/api/sessions/s").status_code == 404


def test_duplicate_frame_replaces_only_when_different(app_client):
    _, client, runs_root = app_client
    first = _post(client, "dup", 1)
    assert first.status_code == 202 and first.json()["received"] == 1
    again = _post(client, "dup", 1)              # identical bytes: ignored
    assert again.status_code == 202 and again.json()["received"] == 1
    replaced = _post(client, "dup", 1, payload=_png_bytes(7))
    assert replaced.status_code == 202 and replaced.json()["received"] == 1
    stored = runs_root / "sessions" / "dup" / "input" / "capture_01_id5.png"
    assert stored.read_bytes() == _png_bytes(7)
    assert not list((runs_root / "sessions" / "dup" / "input").glob(".*.part"))
