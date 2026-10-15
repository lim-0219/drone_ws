"""Create an isolated environment next to this project; no activation required."""
from pathlib import Path
import subprocess
import sys
import venv


def main():
    root = Path(__file__).resolve().parents[1]
    if not (3, 10) <= sys.version_info[:2] <= (3, 12):
        raise SystemExit("Use Python 3.10, 3.11 or 3.12 for the pinned dependencies.")
    environment = root / ".venv"
    if environment.exists():
        raise SystemExit(".venv already exists. Use its Python, or rename it before a fresh setup.")
    print(f"Creating environment: {environment}", flush=True)
    venv.EnvBuilder(with_pip=True).create(environment)
    python = environment / ("Scripts/python.exe" if sys.platform == "win32" else "bin/python")
    subprocess.run([str(python), "-m", "pip", "install", "-r", str(root / "requirements-dev.txt")],
                   cwd=root, check=True)
    subprocess.run([str(python), "-m", "pip", "install", "--no-deps", "-e", str(root)],
                   cwd=root, check=True)
    subprocess.run([str(python), "-m", "pip", "check"], cwd=root, check=True)
    print(f"Ready. Run: {python} scripts/run_demo.py sheffield", flush=True)


if __name__ == "__main__":
    try:
        main()
    except subprocess.CalledProcessError as error:
        print("Setup failed. See START_HERE.md for retry instructions.", file=sys.stderr)
        raise SystemExit(error.returncode)
