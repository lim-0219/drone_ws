"""Start the bundled web app with the project's virtual environment."""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import subprocess
import sys
from threading import Thread
import time
from urllib.error import URLError
from urllib.request import urlopen
import webbrowser


def _virtualenv_python(root: Path) -> Path:
    candidates = (
        root / ".venv" / "Scripts" / "python.exe",
        root / ".venv" / "bin" / "python",
    )
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    raise SystemExit(
        "The project environment is missing. First run: "
        f"{sys.executable} {root / 'scripts' / 'setup_env.py'}"
    )


def _open_when_ready(url: str) -> None:
    deadline = time.monotonic() + 30.0
    health_url = f"{url}/api/health"
    while time.monotonic() < deadline:
        try:
            with urlopen(health_url, timeout=1.0) as response:
                if response.status == 200:
                    webbrowser.open(url)
                    return
        except (OSError, URLError):
            time.sleep(0.25)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Start the local drone-stitching web app")
    parser.add_argument("--host", default="127.0.0.1", help="Listen address (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=8000, help="Listen port (default: 8000)")
    parser.add_argument("--no-browser", action="store_true", help="Do not open a browser automatically")
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")

    root = Path(__file__).resolve().parents[1]
    python = _virtualenv_python(root)
    browser_host = "127.0.0.1" if args.host in {"0.0.0.0", "::"} else args.host
    url = f"http://{browser_host}:{args.port}"

    environment = os.environ.copy()
    environment.pop("PYTHONHOME", None)
    environment.pop("PYTHONPATH", None)
    environment["PYTHONNOUSERSITE"] = "1"

    print(f"Drone stitching web app: {url}", flush=True)
    print("Stop it with Ctrl+C.", flush=True)
    if not args.no_browser:
        Thread(target=_open_when_ready, args=(url,), daemon=True).start()

    command = [
        str(python),
        "-I",
        "-m",
        "drone_stitch.webapp",
        "--host",
        args.host,
        "--port",
        str(args.port),
    ]
    try:
        return subprocess.call(command, cwd=root, env=environment)
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
