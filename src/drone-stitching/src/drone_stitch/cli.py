from __future__ import annotations

import argparse
import json
import logging
from pathlib import Path
import sys
import cv2

from .config import Config
from .coordinates import MapCoordinates
from .pipeline import run_pipeline, write_json
from .synthetic import evaluate, generate


def main(argv=None):
    parser = argparse.ArgumentParser(description="Nine-frame drone ground stitching (CPU OpenCV)")
    sub = parser.add_subparsers(dest="command", required=True)
    stitch = sub.add_parser("stitch", help="Register, jointly optimize and blend 9 frames")
    stitch.add_argument("manifest", type=Path)
    stitch.add_argument("--output", type=Path, required=True)
    stitch.add_argument("--config", type=Path)
    stitch.add_argument("--detector", choices=["orb", "sift"])
    synthetic = sub.add_parser("synthetic", help="Generate deterministic synthetic test data")
    synthetic.add_argument("--output", type=Path, required=True)
    synthetic.add_argument("--seed", type=int, default=7)
    ev = sub.add_parser("evaluate", help="Compare with synthetic ground truth; never a real-flight score")
    ev.add_argument("transforms", type=Path)
    ev.add_argument("truth", type=Path)
    ev.add_argument("--output", type=Path)
    point = sub.add_parser("point", help="Convert local/NED coordinates to map pixels or pixels to local")
    point.add_argument("transforms", type=Path)
    group = point.add_mutually_exclusive_group(required=True)
    group.add_argument("--local", nargs=2, type=float, metavar=("X", "Y"))
    group.add_argument("--ned", nargs=2, type=float, metavar=("NORTH", "EAST"))
    group.add_argument("--pixel", nargs=2, type=float, metavar=("U", "V"))
    point.add_argument("--frame-id", help="Must match saved ned_frame_id for NED input")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    try:
        if args.command == "stitch":
            config = Config.load(args.config)
            if args.detector:
                config.detector = args.detector
            _, report = run_pipeline(args.manifest, args.output, config)
            print(json.dumps({"status": report["status"], "output": str(args.output.resolve()),
                              "dataset_kind": report["dataset_kind"], "elapsed_seconds": report["elapsed_seconds"]}, indent=2))
        elif args.command == "synthetic":
            print(generate(args.output, args.seed).resolve())
        elif args.command == "evaluate":
            result = evaluate(args.transforms, args.truth)
            if args.output:
                write_json(args.output, result)
            print(json.dumps(result, indent=2))
        elif args.command == "point":
            coords = MapCoordinates(args.transforms)
            if args.local is not None:
                pixel = coords.local_to_pixel([args.local])
            elif args.ned is not None:
                pixel = coords.ned_to_pixel([args.ned], args.frame_id)
            else:
                pixel = [args.pixel]
            result = {"pixel_uv": coords.local_to_pixel(coords.pixel_to_local(pixel))[0].tolist(),
                      "local_xy_m": coords.pixel_to_local(pixel)[0].tolist(), **coords.coverage_at(pixel)}
            print(json.dumps(result, indent=2))
        return 0
    except (ValueError, OSError, KeyError, TypeError, cv2.error) as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

