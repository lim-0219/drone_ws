from __future__ import annotations

import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import hashlib
import json
import logging
import os
from pathlib import Path
import re
import shutil
from threading import Lock
from typing import Any
from uuid import uuid4

import cv2
from fastapi import FastAPI, File, Form, HTTPException, UploadFile, status
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from .geometry import CAPTURE_ORDER
from .pipeline import run_pipeline


LOG = logging.getLogger(__name__)

PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_RUNS_ROOT = PROJECT_ROOT / "runs" / "web"
DEFAULT_STATIC_DIR = PROJECT_ROOT / "web" / "static"

EXPECTED_IMAGE_COUNT = 9
ALLOWED_IMAGE_EXTENSIONS = frozenset({".jpg", ".jpeg", ".png", ".webp", ".bmp", ".tif", ".tiff"})
MAX_IMAGE_BYTES = 25 * 1024 * 1024
MAX_IMAGE_PIXELS = 50_000_000
UPLOAD_CHUNK_BYTES = 1024 * 1024
SESSION_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$")
SESSION_METADATA_FIELDS = ("timestamp_s", "ned_m", "yaw_rad", "agl_m")
SESSION_SETUP_FIELDS = ("camera", "grid", "ned_frame_id")


class UploadValidationError(ValueError):
    """An uploaded part is not a safe, usable image."""


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


DEFAULT_CAMERA = {"footprint_m": [5.0, 10.0], "reference_agl_m": 5.0, "image_top_yaw_offset_rad": 0.0}
DEFAULT_GRID = {"spacing_m": [2.5, 5.0]}


def _manifest(
    image_names: list[str],
    camera: dict[str, Any] | None = None,
    grid: dict[str, Any] | None = None,
    ned_frame_id: str | None = None,
    image_metadata: list[dict[str, Any]] | None = None,
) -> dict[str, Any]:
    """Stitcher input. Without a camera/grid/metadata the test assumptions apply."""
    manifest: dict[str, Any] = {
        "version": 1,
        "dataset_kind": "real",
        "camera": dict(camera or DEFAULT_CAMERA),
        "grid": dict(grid or DEFAULT_GRID),
        "images": [],
    }
    if ned_frame_id:
        manifest["ned_frame_id"] = ned_frame_id
    for index, (frame_id, image_name) in enumerate(zip(CAPTURE_ORDER, image_names)):
        entry: dict[str, Any] = {"id": frame_id, "file": image_name}
        if image_metadata and image_metadata[index]:
            entry.update({k: v for k, v in image_metadata[index].items()
                          if k in SESSION_METADATA_FIELDS and v is not None})
        manifest["images"].append(entry)
    return manifest


def _parse_session_metadata(raw: str) -> dict[str, Any]:
    """Validate the per-frame metadata JSON of the session API (shape only)."""
    try:
        data = json.loads(raw)
    except ValueError as exc:
        raise UploadValidationError(f"metadata is not valid JSON: {exc}") from exc
    if not isinstance(data, dict):
        raise UploadValidationError("metadata must be a JSON object")
    try:
        sequence = int(data["sequence"])
        grid_id = int(data["grid_id"])
    except (KeyError, TypeError, ValueError) as exc:
        raise UploadValidationError("metadata needs integer sequence and grid_id") from exc
    total = int(data.get("total", EXPECTED_IMAGE_COUNT))
    if total != EXPECTED_IMAGE_COUNT:
        raise UploadValidationError(f"a session holds exactly {EXPECTED_IMAGE_COUNT} frames")
    if not 1 <= sequence <= EXPECTED_IMAGE_COUNT:
        raise UploadValidationError(f"sequence must be 1..{EXPECTED_IMAGE_COUNT}")
    if CAPTURE_ORDER[sequence - 1] != grid_id:
        raise UploadValidationError(
            f"sequence {sequence} must carry grid_id {CAPTURE_ORDER[sequence - 1]} "
            f"(capture order {list(CAPTURE_ORDER)}), got {grid_id}"
        )
    for key in SESSION_SETUP_FIELDS:
        value = data.get(key)
        if value is not None and not isinstance(value, (dict if key != "ned_frame_id" else str)):
            raise UploadValidationError(f"{key} has the wrong type")
    return {"sequence": sequence, "grid_id": grid_id, "total": total,
            **{k: data.get(k) for k in SESSION_METADATA_FIELDS},
            **{k: data.get(k) for k in SESSION_SETUP_FIELDS}}


def _public_session(session: dict[str, Any], job: dict[str, Any] | None) -> dict[str, Any]:
    received = sorted(session["frames"])
    result: dict[str, Any] = {
        "id": session["id"],
        "received": len(received),
        "total": EXPECTED_IMAGE_COUNT,
        "missing": [n for n in range(1, EXPECTED_IMAGE_COUNT + 1) if n not in session["frames"]],
        "frames": [
            {"sequence": n, "grid_id": session["frames"][n]["grid_id"],
             "received_at": session["frames"][n]["received_at"]}
            for n in received
        ],
        "capture_order": list(CAPTURE_ORDER),
        "created_at": session["created_at"],
        "updated_at": session["updated_at"],
        "status": "collecting",
        "job_id": session.get("job_id"),
    }
    if job is not None:
        result["status"] = job["status"]
        result["updated_at"] = max(result["updated_at"], job["updated_at"])
        if job["status"] == "completed":
            result["map_url"] = f"/api/jobs/{job['id']}/map"
            result["output_dir"] = str(Path(job["job_dir"]) / "output")
            if "elapsed_seconds" in job:
                result["elapsed_seconds"] = job["elapsed_seconds"]
        elif job["status"] == "failed":
            result["error"] = job.get("error", "Map generation failed")
    return result


def _public_job(job: dict[str, Any]) -> dict[str, Any]:
    result = {
        "id": job["id"],
        "status": job["status"],
        "created_at": job["created_at"],
        "updated_at": job["updated_at"],
        "image_count": EXPECTED_IMAGE_COUNT,
        "capture_order": list(CAPTURE_ORDER),
    }
    if job["status"] == "completed":
        result["map_url"] = f"/api/jobs/{job['id']}/map"
        if "elapsed_seconds" in job:
            result["elapsed_seconds"] = job["elapsed_seconds"]
    elif job["status"] == "failed":
        result["error"] = job.get("error", "Map generation failed")
    return result


def _set_job_fields(app: FastAPI, job_id: str, **fields: Any) -> None:
    with app.state.jobs_lock:
        job = app.state.jobs[job_id]
        job.update(fields)
        job["updated_at"] = _now_iso()


def _run_job(app: FastAPI, job_id: str, manifest_path: Path, output_dir: Path) -> None:
    _set_job_fields(app, job_id, status="scanning")
    try:
        _, report = run_pipeline(manifest_path, output_dir)
        map_path = output_dir / "map.png"
        if not map_path.is_file():
            raise RuntimeError("Stitching finished without producing map.png")
        _set_job_fields(
            app,
            job_id,
            status="completed",
            elapsed_seconds=report.get("elapsed_seconds"),
        )
    except Exception as exc:  # The failure is surfaced through the job status API.
        LOG.exception("Web stitching job %s failed", job_id)
        _set_job_fields(app, job_id, status="failed", error=f"{type(exc).__name__}: {exc}")


async def _save_upload(upload: UploadFile, destination: Path, capture_number: int) -> None:
    declared_size = getattr(upload, "size", None)
    if declared_size is not None and declared_size > MAX_IMAGE_BYTES:
        raise UploadValidationError(
            f"Image {capture_number} exceeds the {MAX_IMAGE_BYTES // (1024 * 1024)} MiB limit"
        )

    written = 0
    with destination.open("xb") as handle:
        while chunk := await upload.read(UPLOAD_CHUNK_BYTES):
            written += len(chunk)
            if written > MAX_IMAGE_BYTES:
                raise UploadValidationError(
                    f"Image {capture_number} exceeds the {MAX_IMAGE_BYTES // (1024 * 1024)} MiB limit"
                )
            handle.write(chunk)
    if written == 0:
        raise UploadValidationError(f"Image {capture_number} is empty")

    decoded = cv2.imread(str(destination), cv2.IMREAD_COLOR | cv2.IMREAD_IGNORE_ORIENTATION)
    if decoded is None:
        raise UploadValidationError(f"Image {capture_number} cannot be decoded")
    height, width = decoded.shape[:2]
    if min(width, height) < 64:
        raise UploadValidationError(f"Image {capture_number} is too small (minimum side is 64 px)")
    if width * height > MAX_IMAGE_PIXELS:
        raise UploadValidationError(f"Image {capture_number} exceeds the 50 megapixel limit")


def create_app(
    runs_root: str | Path | None = None,
    static_dir: str | Path | None = None,
) -> FastAPI:
    """Build an isolated local web app; useful both for uvicorn and API tests."""
    resolved_runs_root = Path(runs_root or os.environ.get("DRONE_STITCH_RUNS_DIR", DEFAULT_RUNS_ROOT)).resolve()
    resolved_static_dir = Path(static_dir or DEFAULT_STATIC_DIR).resolve()

    web_app = FastAPI(title="Drone Stitching", version="0.1.0")
    web_app.state.runs_root = resolved_runs_root
    web_app.state.jobs: dict[str, dict[str, Any]] = {}
    web_app.state.jobs_lock = Lock()
    web_app.state.sessions: dict[str, dict[str, Any]] = {}
    # run_pipeline changes process-wide OpenCV settings, so never run two copies concurrently.
    web_app.state.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="drone-stitch")

    @web_app.get("/api/health")
    def health() -> dict[str, Any]:
        with web_app.state.jobs_lock:
            counts = Counter(job["status"] for job in web_app.state.jobs.values())
        return {"status": "ok", "jobs": dict(counts), "worker_limit": 1}

    @web_app.post("/api/jobs", status_code=status.HTTP_202_ACCEPTED)
    async def create_job(images: list[UploadFile] = File(...)) -> dict[str, Any]:
        if len(images) != EXPECTED_IMAGE_COUNT:
            for upload in images:
                await upload.close()
            raise HTTPException(
                status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
                detail=f"Exactly {EXPECTED_IMAGE_COUNT} images are required",
            )

        suffixes: list[str] = []
        for index, upload in enumerate(images, start=1):
            suffix = Path(upload.filename or "").suffix.lower()
            if suffix not in ALLOWED_IMAGE_EXTENSIONS:
                for item in images:
                    await item.close()
                allowed = ", ".join(sorted(ALLOWED_IMAGE_EXTENSIONS))
                raise HTTPException(
                    status_code=status.HTTP_415_UNSUPPORTED_MEDIA_TYPE,
                    detail=f"Image {index} has an unsupported extension; allowed: {allowed}",
                )
            suffixes.append(suffix)

        resolved_runs_root.mkdir(parents=True, exist_ok=True)
        while True:
            job_id = uuid4().hex
            job_dir = resolved_runs_root / job_id
            try:
                job_dir.mkdir()
                break
            except FileExistsError:  # Practically impossible, but never reuse another job directory.
                continue

        input_dir = job_dir / "input"
        input_dir.mkdir()
        image_names = [f"capture_{index:02d}{suffix}" for index, suffix in enumerate(suffixes, start=1)]
        try:
            for index, (upload, image_name) in enumerate(zip(images, image_names), start=1):
                await _save_upload(upload, input_dir / image_name, index)
            manifest_path = input_dir / "manifest.json"
            manifest_path.write_text(
                json.dumps(_manifest(image_names), ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
        except (OSError, UploadValidationError, cv2.error) as exc:
            shutil.rmtree(job_dir, ignore_errors=True)
            raise HTTPException(status_code=status.HTTP_422_UNPROCESSABLE_ENTITY, detail=str(exc)) from exc
        finally:
            for upload in images:
                await upload.close()

        timestamp = _now_iso()
        job = {
            "id": job_id,
            "status": "queued",
            "created_at": timestamp,
            "updated_at": timestamp,
            "job_dir": job_dir,
        }
        with web_app.state.jobs_lock:
            web_app.state.jobs[job_id] = job
        queued_response = _public_job(job)
        try:
            web_app.state.executor.submit(_run_job, web_app, job_id, manifest_path, job_dir / "output")
        except RuntimeError as exc:
            _set_job_fields(web_app, job_id, status="failed", error="Stitching worker is unavailable")
            raise HTTPException(
                status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                detail="Stitching worker is unavailable",
            ) from exc
        return queued_response

    def _session_view(session_id: str) -> dict[str, Any]:
        with web_app.state.jobs_lock:
            session = web_app.state.sessions.get(session_id)
            if session is None:
                raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Session not found")
            job = web_app.state.jobs.get(session.get("job_id") or "")
            return _public_session(session, job)

    @web_app.post("/api/sessions/{session_id}/frames", status_code=status.HTTP_202_ACCEPTED)
    async def add_session_frame(
        session_id: str,
        image: UploadFile = File(...),
        metadata: str = Form(...),
    ) -> dict[str, Any]:
        """One drone photo at a time; the ninth one starts the stitch automatically.

        `metadata` is JSON: sequence (1..9), grid_id (must follow the capture
        order 5,4,1,2,3,6,9,8,7), total (9), optional timestamp_s, ned_m,
        yaw_rad, agl_m, and optional camera / grid / ned_frame_id blocks that
        replace the test-assumption defaults (last one sent wins).
        """
        if not SESSION_ID_PATTERN.match(session_id):
            await image.close()
            raise HTTPException(status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
                                detail="session_id may use letters, digits, '.', '_' and '-'")
        suffix = Path(image.filename or "").suffix.lower()
        if suffix not in ALLOWED_IMAGE_EXTENSIONS:
            await image.close()
            allowed = ", ".join(sorted(ALLOWED_IMAGE_EXTENSIONS))
            raise HTTPException(status_code=status.HTTP_415_UNSUPPORTED_MEDIA_TYPE,
                                detail=f"unsupported extension; allowed: {allowed}")
        try:
            meta = _parse_session_metadata(metadata)
        except UploadValidationError as exc:
            await image.close()
            raise HTTPException(status_code=status.HTTP_422_UNPROCESSABLE_ENTITY, detail=str(exc)) from exc

        session_dir = resolved_runs_root / "sessions" / session_id
        input_dir = session_dir / "input"
        with web_app.state.jobs_lock:
            session = web_app.state.sessions.get(session_id)
            if session is None:
                input_dir.mkdir(parents=True, exist_ok=True)
                session = {"id": session_id, "dir": session_dir, "frames": {}, "camera": None,
                           "grid": None, "ned_frame_id": None, "job_id": None,
                           "created_at": _now_iso(), "updated_at": _now_iso()}
                web_app.state.sessions[session_id] = session
            elif session.get("job_id"):
                await image.close()
                raise HTTPException(status_code=status.HTTP_409_CONFLICT,
                                    detail="session already stitching; use a new session_id")

        image_name = f"capture_{meta['sequence']:02d}_id{meta['grid_id']}{suffix}"
        destination = input_dir / image_name
        staging = input_dir / f".{image_name}.part"
        try:
            if staging.exists():
                staging.unlink()
            await _save_upload(image, staging, meta["sequence"])
        except (OSError, UploadValidationError, cv2.error) as exc:
            staging.unlink(missing_ok=True)
            raise HTTPException(status_code=status.HTTP_422_UNPROCESSABLE_ENTITY, detail=str(exc)) from exc
        finally:
            await image.close()
        digest = hashlib.sha256(staging.read_bytes()).hexdigest()

        with web_app.state.jobs_lock:
            previous = session["frames"].get(meta["sequence"])
            if previous is not None and previous["sha256"] == digest:
                staging.unlink(missing_ok=True)   # exact re-send: keep the first copy
            else:
                staging.replace(destination)
                session["frames"][meta["sequence"]] = {
                    "grid_id": meta["grid_id"], "file": image_name, "sha256": digest,
                    "received_at": _now_iso(),
                    "metadata": {k: meta[k] for k in SESSION_METADATA_FIELDS},
                }
            for key in SESSION_SETUP_FIELDS:
                if meta.get(key) is not None:
                    session[key] = meta[key]
            session["updated_at"] = _now_iso()
            ready = len(session["frames"]) == EXPECTED_IMAGE_COUNT and session["job_id"] is None
            if ready:
                ordered = [session["frames"][n] for n in range(1, EXPECTED_IMAGE_COUNT + 1)]
                manifest_path = input_dir / "manifest.json"
                manifest_path.write_text(
                    json.dumps(_manifest([f["file"] for f in ordered], session["camera"], session["grid"],
                                         session["ned_frame_id"], [f["metadata"] for f in ordered]),
                               ensure_ascii=False, indent=2) + "\n",
                    encoding="utf-8",
                )
                job_id = uuid4().hex
                timestamp = _now_iso()
                web_app.state.jobs[job_id] = {"id": job_id, "status": "queued", "created_at": timestamp,
                                              "updated_at": timestamp, "job_dir": session_dir,
                                              "session_id": session_id}
                session["job_id"] = job_id
        if ready:
            output_dir = session_dir / "output"
            if output_dir.exists():   # a previous run of this session id on disk
                shutil.rmtree(output_dir, ignore_errors=True)
            try:
                web_app.state.executor.submit(_run_job, web_app, job_id, manifest_path, output_dir)
            except RuntimeError:
                _set_job_fields(web_app, job_id, status="failed", error="Stitching worker is unavailable")
        return _session_view(session_id)

    @web_app.get("/api/sessions/latest")
    def latest_session() -> dict[str, Any]:
        with web_app.state.jobs_lock:
            if not web_app.state.sessions:
                raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="No sessions yet")
            latest = max(web_app.state.sessions.values(), key=lambda item: item["updated_at"])
            session_id = latest["id"]
        return _session_view(session_id)

    @web_app.get("/api/sessions/{session_id}")
    def get_session(session_id: str) -> dict[str, Any]:
        return _session_view(session_id)

    @web_app.get("/api/jobs/{job_id}")
    def get_job(job_id: str) -> dict[str, Any]:
        with web_app.state.jobs_lock:
            job = web_app.state.jobs.get(job_id)
            if job is None:
                raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Job not found")
            return _public_job(job)

    @web_app.get("/api/jobs/{job_id}/map")
    def get_map(job_id: str) -> FileResponse:
        with web_app.state.jobs_lock:
            job = web_app.state.jobs.get(job_id)
            if job is None:
                raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Job not found")
            if job["status"] != "completed":
                raise HTTPException(status_code=status.HTTP_409_CONFLICT, detail="Map is not ready")
            map_path = Path(job["job_dir"]) / "output" / "map.png"
        if not map_path.is_file():
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Map file not found")
        return FileResponse(map_path, media_type="image/png")

    # API routes must be registered before the catch-all static mounts.
    web_app.mount("/static", StaticFiles(directory=resolved_static_dir, check_dir=False), name="static")
    web_app.mount("/", StaticFiles(directory=resolved_static_dir, html=True, check_dir=False), name="web")
    return web_app


app = create_app()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run the local drone-stitching web app")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--reload", action="store_true", help="Reload after source changes (development only)")
    args = parser.parse_args(argv)

    import uvicorn

    uvicorn.run("drone_stitch.webapp:app", host=args.host, port=args.port, reload=args.reload, workers=1)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
