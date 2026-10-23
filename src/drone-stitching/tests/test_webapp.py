from __future__ import annotations

import json
from pathlib import Path
import time

import cv2
from fastapi.testclient import TestClient
import numpy as np
import pytest

import drone_stitch.webapp as webapp


EXPECTED_CAPTURE_ORDER = [5, 4, 1, 2, 3, 6, 9, 8, 7]


def _png_bytes() -> bytes:
    image = np.zeros((72, 96, 3), dtype=np.uint8)
    image[:, :48] = (35, 120, 220)
    image[:, 48:] = (80, 200, 65)
    ok, encoded = cv2.imencode(".png", image)
    assert ok
    return encoded.tobytes()


VALID_PNG = _png_bytes()


def _uploads(
    count: int = 9,
    *,
    payload: bytes = VALID_PNG,
    extension: str = ".png",
) -> list[tuple[str, tuple[str, bytes, str]]]:
    return [
        ("images", (f"frame-{index}{extension}", payload, "image/png"))
        for index in range(1, count + 1)
    ]


def _wait_for_terminal_status(
    client: TestClient,
    job_id: str,
    *,
    timeout: float = 3.0,
) -> dict:
    deadline = time.monotonic() + timeout
    latest: dict | None = None
    while time.monotonic() < deadline:
        response = client.get(f"/api/jobs/{job_id}")
        assert response.status_code == 200, response.text
        latest = response.json()
        if latest["status"] in {"completed", "failed"}:
            return latest
        time.sleep(0.01)
    pytest.fail(f"job did not finish before timeout; latest response: {latest}")


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


def test_health_reports_an_empty_isolated_worker(app_client):
    _, client, _ = app_client

    response = client.get("/api/health")

    assert response.status_code == 200
    assert response.json() == {"status": "ok", "jobs": {}, "worker_limit": 1}


def test_nine_images_complete_automatically_and_serve_map(app_client, monkeypatch):
    _, client, runs_root = app_client
    observed: dict = {}

    def fake_pipeline(manifest_path: Path, output_dir: Path):
        observed["manifest_path"] = Path(manifest_path)
        observed["manifest"] = json.loads(Path(manifest_path).read_text(encoding="utf-8"))
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True)
        (output_dir / "map.png").write_bytes(VALID_PNG)
        return {}, {"elapsed_seconds": 0.012}

    monkeypatch.setattr(webapp, "run_pipeline", fake_pipeline)
    # Client filenames are deliberately untrusted. The server must generate safe names.
    files = _uploads()
    files[0] = ("images", ("../../outside.png", VALID_PNG, "image/png"))

    response = client.post("/api/jobs", files=files)

    assert response.status_code == 202, response.text
    accepted = response.json()
    assert accepted["status"] in {"queued", "scanning", "completed"}
    assert accepted["image_count"] == 9
    assert accepted["capture_order"] == EXPECTED_CAPTURE_ORDER

    completed = _wait_for_terminal_status(client, accepted["id"])
    assert completed["status"] == "completed"
    assert completed["map_url"] == f"/api/jobs/{accepted['id']}/map"
    assert completed["elapsed_seconds"] == pytest.approx(0.012)

    manifest = observed["manifest"]
    assert [item["id"] for item in manifest["images"]] == EXPECTED_CAPTURE_ORDER
    assert [item["file"] for item in manifest["images"]] == [
        f"capture_{index:02d}.png" for index in range(1, 10)
    ]
    input_dir = observed["manifest_path"].parent
    assert len(list(input_dir.glob("capture_*.png"))) == 9
    assert not (runs_root / "outside.png").exists()

    map_response = client.get(completed["map_url"])
    assert map_response.status_code == 200
    assert map_response.headers["content-type"] == "image/png"
    assert map_response.content == VALID_PNG

    health = client.get("/api/health").json()
    assert health["jobs"] == {"completed": 1}


def test_pipeline_failure_is_reported_and_map_remains_unavailable(app_client, monkeypatch):
    _, client, _ = app_client

    def failing_pipeline(manifest_path: Path, output_dir: Path):
        raise RuntimeError("deliberate stitching failure")

    monkeypatch.setattr(webapp, "run_pipeline", failing_pipeline)

    response = client.post("/api/jobs", files=_uploads())

    assert response.status_code == 202, response.text
    failed = _wait_for_terminal_status(client, response.json()["id"])
    assert failed["status"] == "failed"
    assert "RuntimeError: deliberate stitching failure" in failed["error"]
    assert "map_url" not in failed

    map_response = client.get(f"/api/jobs/{failed['id']}/map")
    assert map_response.status_code == 409
    assert map_response.json()["detail"] == "Map is not ready"


def test_eight_images_are_rejected_without_creating_a_job(app_client):
    app, client, runs_root = app_client

    response = client.post("/api/jobs", files=_uploads(8))

    assert response.status_code == 422
    assert response.json()["detail"] == "Exactly 9 images are required"
    assert app.state.jobs == {}
    assert not runs_root.exists()


@pytest.mark.parametrize(
    ("files", "expected_status", "detail_fragment"),
    [
        (_uploads(extension=".txt"), 415, "unsupported extension"),
        (
            [
                ("images", ("broken.png", b"this is not an image", "image/png")),
                *_uploads(8),
            ],
            422,
            "cannot be decoded",
        ),
    ],
)
def test_unsafe_image_inputs_are_rejected_and_cleaned_up(
    app_client,
    files,
    expected_status,
    detail_fragment,
):
    app, client, runs_root = app_client

    response = client.post("/api/jobs", files=files)

    assert response.status_code == expected_status
    assert detail_fragment in response.json()["detail"]
    assert app.state.jobs == {}
    assert not runs_root.exists() or list(runs_root.iterdir()) == []


def test_oversized_image_is_rejected_and_cleaned_up(app_client, monkeypatch):
    app, client, runs_root = app_client
    monkeypatch.setattr(webapp, "MAX_IMAGE_BYTES", len(VALID_PNG) - 1)

    response = client.post("/api/jobs", files=_uploads())

    assert response.status_code == 422
    assert "exceeds" in response.json()["detail"]
    assert app.state.jobs == {}
    assert list(runs_root.iterdir()) == []


def test_unknown_job_and_map_return_not_found(app_client):
    _, client, _ = app_client

    status_response = client.get("/api/jobs/does-not-exist")
    map_response = client.get("/api/jobs/does-not-exist/map")

    assert status_response.status_code == 404
    assert map_response.status_code == 404
