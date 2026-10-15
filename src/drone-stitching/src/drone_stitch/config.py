from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from pathlib import Path
import numpy as np


@dataclass
class Config:
    detector: str = "sift"
    nfeatures: int = 6000
    feature_max_dimension: int = 1600
    ratio_test: float = .75
    ransac_threshold_px: float = 3.
    min_inliers: int = 18
    min_inlier_ratio: float = .25
    min_inlier_coverage: float = .015
    max_matches_per_edge: int = 300
    diagonals: bool = True
    match_all: bool = False
    grid_prior_sigma_m: float = 5.
    metadata_prior_sigma_m: float = 1.5
    robust_loss_scale_px: float = 2.
    max_nfev: int = 250
    max_edge_median_error_px: float = 4.
    pixels_per_meter: float = 100.
    feather_radius_px: float = 50.
    max_canvas_megapixels: float = 24.
    max_canvas_dimension: int = 16000
    telemetry_max_gap_s: float = .5
    telemetry_nearest_tolerance_s: float = .05
    seed: int = 42
    opencv_threads: int = 4

    def validate(self):
        if self.detector not in ("sift", "orb"):
            raise ValueError("detector must be sift or orb")
        integer_fields = ("nfeatures", "feature_max_dimension", "min_inliers",
                          "max_matches_per_edge", "max_nfev", "max_canvas_dimension", "opencv_threads")
        for key in integer_fields:
            v = getattr(self, key)
            if type(v) is not int or v <= 0:
                raise ValueError(f"{key} must be a positive integer")
        if type(self.seed) is not int or not 0 <= self.seed <= 2**31 - 1:
            raise ValueError("seed must be an integer in [0, 2**31-1]")
        for key in ("diagonals", "match_all"):
            if type(getattr(self, key)) is not bool:
                raise ValueError(f"{key} must be a boolean")
        excluded = set(integer_fields) | {"detector", "seed", "diagonals", "match_all"}
        for key, v in asdict(self).items():
            if key not in excluded and (isinstance(v, bool) or not isinstance(v, (int, float))
                                        or not np.isfinite(v) or v <= 0):
                raise ValueError(f"{key} must be positive and finite")
        for key in ("ratio_test", "min_inlier_ratio", "min_inlier_coverage"):
            if getattr(self, key) >= 1:
                raise ValueError(f"{key} must be below 1")
        if self.min_inliers < 4 or self.max_matches_per_edge < self.min_inliers:
            raise ValueError("Require 4 <= min_inliers <= max_matches_per_edge")
        return self

    @classmethod
    def load(cls, path=None):
        data = json.loads(Path(path).read_text()) if path else {}
        try:
            return cls(**data).validate()
        except TypeError as e:
            raise ValueError(f"Unknown or invalid configuration field: {e}") from e

