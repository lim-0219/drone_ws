"""Run bundled examples using the installed package and a fresh output folder."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description="Run the synthetic or Sheffield nine-image demo")
    parser.add_argument("demo", choices=["sheffield", "synthetic"])
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    destination = root / "runs" / (args.demo + "-" + datetime.now().strftime("%Y%m%d-%H%M%S-%f"))
    destination.mkdir(parents=True, exist_ok=False)

    def run(*arguments):
        subprocess.run(
            [sys.executable, "-I", "-m", "drone_stitch", *map(str, arguments)],
            cwd=root,
            check=True,
        )

    if args.demo == "sheffield":
        source = root / "datasets/sheffield_park_3/selected9"
        result = destination / "sift"
        run("stitch", source / "manifest.json", "--config", source / "config.json", "--output", result)
        run("point", result / "transforms.json", "--local", "0", "0")
    else:
        source = destination / "input"
        run("synthetic", "--output", source)
        for detector, manifest in [("sift", "manifest.json"), ("orb", "manifest_no_metadata.json")]:
            result = destination / detector
            run("stitch", source / manifest, "--detector", detector, "--output", result)
            run("evaluate", result / "transforms.json", source / "ground_truth.json",
                "--output", result / "evaluation.json")
    summary = {"demo": args.demo, "output_directory": str(destination), "status": "success"}
    (destination / "demo_summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(f"\nResults: {destination}")


if __name__ == "__main__":
    try:
        main()
    except subprocess.CalledProcessError as error:
        raise SystemExit(error.returncode)
