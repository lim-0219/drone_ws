"""Build the curated portable ZIP and refresh its integrity manifest."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import zipfile


ROOT = Path(__file__).resolve().parents[1]
ARCHIVE_ROOT = Path("drone-stitching")
DEFAULT_NAME = "drone-stitching-web-portable-2026-10-16"

ROOT_FILES = (
    ".gitignore",
    "HANDOFF_WEB_APP_2026-10-16.md",
    "PORTABILITY_CHECK.md",
    "README.md",
    "START_HERE.md",
    "THIRD_PARTY_NOTICES.md",
    "VALIDATION.md",
    "WEB_APP.md",
    "pyproject.toml",
    "requirements-dev.txt",
    "requirements.txt",
)

SOURCE_DIRECTORIES = (
    "datasets",
    "examples",
    "scripts",
    "src/drone_stitch",
    "tests",
    "web",
)

# These are the curated reference inputs/results from the original portable
# bundle. New local jobs and failed/repeated demos under runs/ stay local.
REFERENCE_RUN_DIRECTORIES = (
    "runs/sheffield_selected9_sift",
    "runs/sheffield_selected9_sift_west",
    "runs/verified_input",
    "runs/verified_orb",
    "runs/verified_sift",
)

EXCLUDED_DIRECTORY_NAMES = {
    ".git",
    ".mypy_cache",
    ".pytest_cache",
    ".ruff_cache",
    ".venv",
    "__pycache__",
}
EXCLUDED_SUFFIXES = {".pyc", ".pyo"}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def allowed(path: Path) -> bool:
    relative = path.relative_to(ROOT)
    return (
        path.is_file()
        and not any(part in EXCLUDED_DIRECTORY_NAMES for part in relative.parts)
        and path.suffix.lower() not in EXCLUDED_SUFFIXES
    )


def collect_files() -> list[Path]:
    files: set[Path] = set()
    missing: list[str] = []

    for relative in ROOT_FILES:
        path = ROOT / relative
        if not path.is_file():
            missing.append(relative)
        else:
            files.add(path)

    for relative in (*SOURCE_DIRECTORIES, *REFERENCE_RUN_DIRECTORIES):
        directory = ROOT / relative
        if not directory.is_dir():
            missing.append(relative)
            continue
        files.update(path for path in directory.rglob("*") if allowed(path))

    if missing:
        raise SystemExit("Missing required bundle content: " + ", ".join(missing))
    return sorted(files, key=lambda path: path.relative_to(ROOT).as_posix())


def write_manifest(files: list[Path], bundle_name: str) -> Path:
    inventory = {
        "schema_version": 1,
        "bundle_name": bundle_name,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "files": [
            {
                "path": path.relative_to(ROOT).as_posix(),
                "bytes": path.stat().st_size,
                "sha256": sha256(path),
            }
            for path in files
        ],
    }
    manifest = ROOT / "BUNDLE_MANIFEST.json"
    manifest.write_text(
        json.dumps(inventory, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return manifest


def build(output: Path) -> tuple[int, int, str, Path]:
    output = output.resolve()
    if output.parent == ROOT or ROOT in output.parents:
        raise SystemExit("Write the portable archive outside the project directory")
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        raise SystemExit(f"Refusing to overwrite existing archive: {output}")

    files = collect_files()
    manifest = write_manifest(files, output.stem)
    packaged_files = [*files, manifest]
    with zipfile.ZipFile(
        output,
        mode="x",
        compression=zipfile.ZIP_DEFLATED,
        compresslevel=9,
        allowZip64=True,
    ) as archive:
        for path in packaged_files:
            relative = path.relative_to(ROOT)
            archive.write(path, (ARCHIVE_ROOT / relative).as_posix())

    archive_hash = sha256(output)
    checksum_path = output.with_suffix(output.suffix + ".sha256")
    checksum_path.write_text(f"{archive_hash}  {output.name}\n", encoding="utf-8")
    return len(packaged_files), sum(path.stat().st_size for path in packaged_files), archive_hash, checksum_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT.parent / f"{DEFAULT_NAME}.zip",
        help="Destination ZIP (must be outside the project directory)",
    )
    args = parser.parse_args()
    count, raw_bytes, archive_hash, checksum_path = build(args.output)
    print(f"Created: {args.output.resolve()}")
    print(f"Files: {count}; uncompressed bytes: {raw_bytes}")
    print(f"SHA-256: {archive_hash}")
    print(f"Checksum: {checksum_path}")


if __name__ == "__main__":
    main()
