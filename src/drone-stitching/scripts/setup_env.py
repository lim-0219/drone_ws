"""Create an isolated environment next to this project; no activation required."""
import os
from pathlib import Path
import shutil
import subprocess
import sys
import venv


def _isolated_environment() -> dict[str, str]:
    """Keep sourced ROS/Python environments out of the project virtualenv."""
    environment = os.environ.copy()
    environment.pop("PYTHONHOME", None)
    environment.pop("PYTHONPATH", None)
    environment["PYTHONNOUSERSITE"] = "1"
    return environment


def main():
    root = Path(__file__).resolve().parents[1]
    if not (3, 10) <= sys.version_info[:2] <= (3, 12):
        raise SystemExit("Use Python 3.10, 3.11 or 3.12 for the pinned dependencies.")
    environment = root / ".venv"
    if environment.exists():
        raise SystemExit(".venv already exists. Use its Python, or rename it before a fresh setup.")
    print(f"Creating environment: {environment}", flush=True)
    try:
        venv.EnvBuilder(with_pip=True).create(environment)
    except KeyboardInterrupt:
        shutil.rmtree(environment, ignore_errors=True)
        raise
    except BaseException as error:
        shutil.rmtree(environment, ignore_errors=True)
        raise SystemExit(
            "Could not create the virtual environment. On Ubuntu, install "
            "python3-venv and run this setup command again."
        ) from error
    python = environment / ("Scripts/python.exe" if sys.platform == "win32" else "bin/python")
    clean_environment = _isolated_environment()
    pip = [str(python), "-I", "-m", "pip", "--isolated"]
    subprocess.run(
        [*pip, "install", "-r", str(root / "requirements-dev.txt")],
        cwd=root,
        env=clean_environment,
        check=True,
    )
    subprocess.run(
        [*pip, "install", "--no-deps", "-e", str(root)],
        cwd=root,
        env=clean_environment,
        check=True,
    )
    subprocess.run([*pip, "check"], cwd=root, env=clean_environment, check=True)
    launcher = root / "scripts" / "start_webapp.py"
    print(f"Ready. Start the web app with: {sys.executable} {launcher}", flush=True)


if __name__ == "__main__":
    try:
        main()
    except subprocess.CalledProcessError as error:
        print("Setup failed. See START_HERE.md for retry instructions.", file=sys.stderr)
        raise SystemExit(error.returncode)
