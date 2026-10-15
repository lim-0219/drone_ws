"""Joint 8-DOF homography refinement over all accepted overlap constraints."""
from __future__ import annotations

import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix

from .geometry import center, corners, normalize_h, pixel_normalizer, project, signed_area, validate_footprint


def graph_seed(initial, edges):
    """Maximum-confidence spanning tree only initializes; final solve uses ALL edges."""
    result = {5: initial[5].copy()}
    while len(result) < len(initial):
        crossing = [e for e in edges if (e.a in result) != (e.b in result)]
        if not crossing:
            missing = sorted(set(initial) - set(result))
            raise ValueError(f"Disconnected match graph; images unreachable from 5: {missing}. "
                             "Inspect overlap/texture and diagnostics.json; no mosaic exported.")
        e = max(crossing, key=lambda x: x.stats["inliers"] / (1 + x.stats["median_symmetric_error_px"]))
        if e.a in result:
            result[e.b] = normalize_h(result[e.a] @ np.linalg.inv(e.h_a_to_b))
        else:
            result[e.a] = normalize_h(result[e.b] @ e.h_a_to_b)
    return result


def edge_errors(homographies, edges, reference_h):
    inverse_ref = np.linalg.inv(reference_h)
    result = []
    for e in edges:
        a = project(inverse_ref @ homographies[e.a], e.points_a)
        b = project(inverse_ref @ homographies[e.b], e.points_b)
        error = np.linalg.norm(a - b, axis=1)
        result.append({"a": e.a, "b": e.b, "median_reference_px": float(np.median(error)),
                       "p95_reference_px": float(np.percentile(error, 95)), "rmse_reference_px": float(np.sqrt(np.mean(error**2)))})
    return result


def optimize(frames, initial, placement_info, edges, config):
    seed = graph_seed(initial, edges)
    ids = sorted(set(initial) - {5})
    by_id = {f.id: f for f in frames}
    # Both source and destination coordinates are normalized for numerical stability.
    characteristic_m = max(placement_info[5]["footprint_m"])
    world_scale = np.diag([1 / characteristic_m, 1 / characteristic_m, 1.])
    normalizers = {f.id: pixel_normalizer(f.width, f.height) for f in frames}

    def encode(hs):
        return np.concatenate([normalize_h(world_scale @ hs[i] @ np.linalg.inv(normalizers[i])).ravel()[:8] for i in ids])

    fixed = normalize_h(world_scale @ initial[5] @ np.linalg.inv(normalizers[5]))

    def decode_normalized(x):
        return {5: fixed, **{i: np.r_[x[j * 8:j * 8 + 8], 1.].reshape(3, 3) for j, i in enumerate(ids)}}

    pairs = [(e, project(normalizers[e.a], e.points_a), project(normalizers[e.b], e.points_b)) for e in edges]
    reference_linear = np.linalg.inv(initial[5])[:2, :2] * characteristic_m
    normalized_corners = np.array([[-.5, -.5], [.5, -.5], [.5, .5], [-.5, .5]])
    targets = {i: project(world_scale @ initial[i] @ np.linalg.inv(normalizers[i]), normalized_corners) for i in ids}
    sigmas = {i: config.metadata_prior_sigma_m if (placement_info[i]["position_source"] == "relative_ned"
               or placement_info[i]["yaw_source"] == "metadata_yaw") else config.grid_prior_sigma_m for i in ids}

    def residual(x):
        hs = decode_normalized(x)
        values = [((project(hs[e.a], pa) - project(hs[e.b], pb)) @ reference_linear.T).ravel()
                  for e, pa, pb in pairs]
        # Weak corner priors carry initial position, yaw, altitude and footprint assumptions.
        values.extend(((project(hs[i], normalized_corners) - targets[i]) * characteristic_m / sigmas[i]).ravel() for i in ids)
        return np.concatenate(values)

    rows = sum(2 * len(e.points_a) for e in edges) + len(ids) * 8
    sparsity = lil_matrix((rows, len(ids) * 8), dtype=int)
    row = 0
    index = {i: j for j, i in enumerate(ids)}
    for e in edges:
        count = 2 * len(e.points_a)
        for i in (e.a, e.b):
            if i != 5:
                j = index[i] * 8
                sparsity[row:row + count, j:j + 8] = 1
        row += count
    for i in ids:
        j = index[i] * 8
        sparsity[row:row + 8, j:j + 8] = 1
        row += 8
    lower, upper = np.full(len(ids) * 8, -np.inf), np.full(len(ids) * 8, np.inf)
    for j in range(len(ids)):
        # With normalized corners +/-0.5, denominator is always >=0.2.
        lower[j * 8 + 6:j * 8 + 8] = -.8
        upper[j * 8 + 6:j * 8 + 8] = .8
    x_seed, x_initial = encode(seed), encode(initial)
    def robust_cost(x):
        r = residual(x) / config.robust_loss_scale_px
        return np.sum(np.sqrt(1 + r * r) - 1)
    seed_valid = np.all(x_seed > lower) and np.all(x_seed < upper)
    use_visual = seed_valid and robust_cost(x_seed) < robust_cost(x_initial)
    x0 = x_seed if use_visual else x_initial
    result = least_squares(residual, x0, jac_sparsity=sparsity.tocsr(), bounds=(lower, upper),
                           loss="soft_l1", f_scale=config.robust_loss_scale_px, x_scale="jac",
                           max_nfev=config.max_nfev, ftol=1e-7, xtol=1e-7, gtol=1e-7)
    refined = {i: normalize_h(np.linalg.inv(world_scale) @ h @ normalizers[i])
               for i, h in decode_normalized(result.x).items()}
    report = {"success": bool(result.success), "message": result.message, "nfev": result.nfev,
              "initialization": "visual_spanning_tree" if use_visual else "grid_and_metadata",
              "reference_image_fixed": 5, "variables": len(result.x), "accepted_edges": len(edges),
              "independent_graph_cycles": len(edges) - len(frames) + 1,
              "objective_before": float(robust_cost(x0)), "objective_after": float(robust_cost(result.x)),
              "initial_edge_errors": edge_errors(initial, edges, initial[5]),
              "final_edge_errors": edge_errors(refined, edges, initial[5])}
    return refined, report


def validate_solution(frames, initial, refined, report, config):
    if not report["success"]:
        raise ValueError(f"Global optimization did not converge: {report['message']}")
    if report["objective_after"] > report["objective_before"] + 1e-6:
        raise ValueError("Global objective increased")
    for f in frames:
        q = validate_footprint(refined[f.id], f.width, f.height)
        p = validate_footprint(initial[f.id], f.width, f.height)
        area_ratio = signed_area(q) / signed_area(p)
        if not .2 < area_ratio < 5:
            raise ValueError(f"Image {f.id}: implausible optimized footprint scale/reflection")
    bad = [e for e in report["final_edge_errors"] if e["median_reference_px"] > config.max_edge_median_error_px]
    if bad:
        raise ValueError(f"Inconsistent overlap constraints after optimization: {[(e['a'], e['b']) for e in bad]}")
    ref = next(f for f in frames if f.id == 5)
    anchor = [ref.anchor_pixel] if ref.anchor_pixel is not None else center(ref.width, ref.height)
    if not np.allclose(project(refined[5], anchor), 0, atol=1e-8):
        raise ValueError("Reference origin moved during optimization")
