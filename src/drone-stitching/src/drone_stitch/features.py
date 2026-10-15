from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations
import cv2
import numpy as np

from .geometry import GRID, normalize_h, project, signed_area, validate_footprint


@dataclass
class Features:
    points: np.ndarray
    descriptors: np.ndarray | None


@dataclass
class Edge:
    a: int
    b: int
    h_a_to_b: np.ndarray
    points_a: np.ndarray
    points_b: np.ndarray
    stats: dict


def extract(frame, config):
    gray = cv2.cvtColor(frame.image, cv2.COLOR_BGR2GRAY)
    factor = min(1., config.feature_max_dimension / max(frame.width, frame.height))
    width, height = round(frame.width * factor), round(frame.height * factor)
    small = cv2.resize(gray, (width, height), interpolation=cv2.INTER_AREA)
    mask = cv2.resize(frame.mask, (width, height), interpolation=cv2.INTER_NEAREST)
    if config.detector == "sift":
        detector = cv2.SIFT_create(nfeatures=config.nfeatures)
    else:
        detector = cv2.ORB_create(nfeatures=config.nfeatures, fastThreshold=10)
    keypoints, descriptors = detector.detectAndCompute(small, mask)
    points = np.array([k.pt for k in keypoints], dtype=float).reshape(-1, 2)
    # Inverse of resize's pixel-centre mapping, not just division by a nominal factor.
    points = (points + .5) / [width / frame.width, height / frame.height] - .5
    return Features(points, descriptors)


def candidates(frames, config):
    pairs = []
    for a, b in combinations(sorted(f.id for f in frames), 2):
        dx, dy = np.abs(np.array(GRID[a]) - GRID[b])
        if config.match_all or dx + dy == 1 or (config.diagonals and dx == dy == 1):
            pairs.append((a, b))
    return pairs


def _ratio_matches(matcher, a, b, ratio):
    return {m[0].queryIdx: m[0].trainIdx for m in matcher.knnMatch(a, b, k=2)
            if len(m) == 2 and m[0].distance < ratio * m[1].distance}


def match_pair(a, b, fa, fb, config):
    stats = {"a": a.id, "b": b.id, "accepted": False}

    def reject(reason):
        stats["reason"] = reason
        return None, stats

    if fa.descriptors is None or fb.descriptors is None or min(len(fa.points), len(fb.points)) < 4:
        return reject("insufficient_features")
    norm = cv2.NORM_HAMMING if config.detector == "orb" else cv2.NORM_L2
    matcher = cv2.BFMatcher(norm)
    ab = _ratio_matches(matcher, fa.descriptors, fb.descriptors, config.ratio_test)
    ba = _ratio_matches(matcher, fb.descriptors, fa.descriptors, config.ratio_test)
    pairs = [(i, j) for i, j in ab.items() if ba.get(j) == i]
    stats["mutual_ratio_matches"] = len(pairs)
    if len(pairs) < config.min_inliers:
        return reject("insufficient_mutual_matches")
    pa = np.array([fa.points[i] for i, _ in pairs])
    pb = np.array([fb.points[j] for _, j in pairs])
    h, mask = cv2.findHomography(pa, pb, cv2.RANSAC, config.ransac_threshold_px,
                                maxIters=5000, confidence=.999)
    if h is None or mask is None:
        return reject("ransac_failed")
    try:
        h = normalize_h(h)
        footprint = validate_footprint(h, a.width, a.height)
        area = signed_area(footprint) / (b.width * b.height)
        if not .15 < area < 6.:
            return reject("implausible_scale_or_reflection")
        forward = np.linalg.norm(project(h, pa) - pb, axis=1)
        backward = np.linalg.norm(project(np.linalg.inv(h), pb) - pa, axis=1)
    except (ValueError, np.linalg.LinAlgError):
        return reject("invalid_homography")
    keep = mask.ravel().astype(bool) & (forward < config.ransac_threshold_px * 1.5) & (backward < config.ransac_threshold_px * 1.5)
    n = int(keep.sum())
    stats.update(inliers=n, inlier_ratio=n / len(pairs))
    if n < config.min_inliers or n / len(pairs) < config.min_inlier_ratio:
        return reject("insufficient_ransac_consensus")
    pa, pb = pa[keep], pb[keep]
    coverages = [cv2.contourArea(cv2.convexHull(p.astype(np.float32))) / (f.width * f.height)
                 for p, f in ((pa, a), (pb, b))]
    stats["coverage"] = coverages
    if min(coverages) < config.min_inlier_coverage:
        return reject("inliers_too_local_or_collinear")
    stats["median_symmetric_error_px"] = float(np.median((forward[keep] + backward[keep]) / 2))
    if n > config.max_matches_per_edge:
        rng = np.random.default_rng(config.seed + a.id * 100 + b.id)
        indices = rng.choice(n, config.max_matches_per_edge, replace=False)
        pa, pb = pa[indices], pb[indices]
    stats.update(accepted=True, optimization_matches=len(pa), h_a_to_b=h.tolist())
    return Edge(a.id, b.id, h, pa, pb, stats), stats


def match_graph(frames, config, report):
    by_id = {f.id: f for f in frames}
    feats = {f.id: extract(f, config) for f in frames}
    report["feature_counts"] = {str(k): len(v.points) for k, v in feats.items()}
    report["pairs"] = []
    edges = []
    for a, b in candidates(frames, config):
        edge, stats = match_pair(by_id[a], by_id[b], feats[a], feats[b], config)
        report["pairs"].append(stats)
        if edge is not None:
            edges.append(edge)
    return edges

