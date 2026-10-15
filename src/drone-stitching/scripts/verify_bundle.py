"""Verify the packaged files without third-party dependencies."""
import hashlib
import json
from pathlib import Path


def main():
    root = Path(__file__).resolve().parents[1]
    inventory = root / "BUNDLE_MANIFEST.json"
    if not inventory.exists():
        raise SystemExit("BUNDLE_MANIFEST.json is available in the distribution archive.")
    document = json.loads(inventory.read_text(encoding="utf-8"))
    errors = []
    for item in document["files"]:
        path = root / item["path"]
        if not path.is_file():
            errors.append(f"Missing: {item['path']}")
        elif hashlib.sha256(path.read_bytes()).hexdigest() != item["sha256"]:
            errors.append(f"Changed: {item['path']}")
    if errors:
        raise SystemExit("\n".join(errors))
    print(f"OK: {len(document['files'])} packaged files verified. Extra generated files are ignored.")


if __name__ == "__main__":
    main()
