import json
from pathlib import Path
import cv2
import numpy as np
import pytest

from drone_stitch.config import Config
from drone_stitch.coordinates import MapCoordinates
from drone_stitch.geometry import CAPTURE_ORDER, project
from drone_stitch.inputs import initial_placement, read_manifest
from drone_stitch.pipeline import run_pipeline
from drone_stitch.synthetic import evaluate, generate


@pytest.fixture(scope="module")
def synthetic(tmp_path_factory):
    folder = tmp_path_factory.mktemp("synthetic")
    generate(folder, seed=7)
    return folder


@pytest.mark.parametrize("detector,metadata,feature_size", [("sift", True, 480), ("orb", False, 1600)])
def test_full_pipeline_against_independent_synthetic_truth(synthetic, tmp_path, detector, metadata, feature_size):
    manifest = synthetic / ("manifest.json" if metadata else "manifest_no_metadata.json")
    config = Config(detector=detector, feature_max_dimension=feature_size)
    output = tmp_path / "map"
    transforms, diagnostics = run_pipeline(manifest, output, config)
    score = evaluate(output / "transforms.json", synthetic / "ground_truth.json")
    assert score["rmse_m"] < .04, score
    assert score["max_m"] < .12, score
    assert score["real_data_validated"] is False
    assert set(transforms["images"]) == set(map(str, CAPTURE_ORDER))
    assert diagnostics["status"] == "success"
    assert diagnostics["optimization"]["independent_graph_cycles"] >= 4
    assert diagnostics["optimization"]["objective_after"] <= diagnostics["optimization"]["objective_before"]
    coords = MapCoordinates(output / "transforms.json")
    xy = np.array([[0., 0.], [2., -3.], [-4., 6.]])
    np.testing.assert_allclose(coords.pixel_to_local(coords.local_to_pixel(xy)), xy, atol=1e-10)
    assert coords.coverage_at(coords.local_to_pixel([[0, 0]]))["covered"] == [True]
    np.testing.assert_allclose(transforms["images"]["5"]["estimated_capture_local_m"], [0, 0], atol=1e-12)
    np.testing.assert_allclose(coords.local_to_pixel([[0, 0]])[0], transforms["origin"]["reference_map_pixel"])
    if metadata:
        ned_pixel = coords.ned_to_pixel([[100, 200]], "synthetic-session-1")
        np.testing.assert_allclose(ned_pixel, coords.local_to_pixel([[0, 0]]), atol=1e-9)
        with pytest.raises(ValueError, match="ned_frame_id"):
            coords.ned_to_pixel([[100, 200]], "wrong-session")
        assert transforms["images"]["6"]["metadata_provenance"]["yaw_rad"]["method"] == "interpolated"
    else:
        with pytest.raises(ValueError, match="NED alignment unavailable"):
            coords.ned_to_pixel([[100, 200]])
    for name in ("map.png", "valid_mask.png", "coverage.png", "overview.png"):
        im = cv2.imread(str(output / name), cv2.IMREAD_UNCHANGED)
        assert list(im.shape[1::-1]) == transforms["map"]["size_pixels"]
    assert coords.coverage_at([[-100, -100]])["covered"] == [False]
    with pytest.raises(ValueError, match="new or empty"):
        run_pipeline(manifest, output, config)


def _absolute_manifest(synthetic):
    doc = json.loads((synthetic / "manifest_no_metadata.json").read_text())
    for entry in doc["images"]:
        entry["file"] = str(synthetic / entry["file"])
    return doc


def test_disconnected_image_fails_without_partial_map(synthetic, tmp_path):
    doc = _absolute_manifest(synthetic)
    blank = tmp_path / "blank.png"
    cv2.imwrite(str(blank), np.zeros((800, 400, 3), dtype=np.uint8))
    doc["images"][-1]["file"] = str(blank)
    manifest = tmp_path / "manifest.json"
    manifest.write_text(json.dumps(doc))
    output = tmp_path / "failed"
    with pytest.raises(ValueError, match="Disconnected match graph"):
        run_pipeline(manifest, output, Config(nfeatures=2000))
    assert not (output / "map.png").exists()
    diag = json.loads((output / "diagnostics.json").read_text())
    assert diag["status"] == "failed"
    assert diag["feature_counts"]["7"] == 0


@pytest.mark.parametrize("problem", ["order", "missing", "duplicate", "count"])
def test_input_validation(synthetic, tmp_path, problem):
    doc = _absolute_manifest(synthetic)
    if problem == "order":
        doc["images"][0], doc["images"][1] = doc["images"][1], doc["images"][0]
    elif problem == "missing":
        doc["images"][0]["file"] = str(tmp_path / "missing.png")
    elif problem == "duplicate":
        doc["images"][1]["file"] = doc["images"][0]["file"]
    else:
        doc["images"].pop()
    p = tmp_path / "manifest.json"
    p.write_text(json.dumps(doc))
    with pytest.raises(ValueError):
        read_manifest(p, Config())


def test_calibrated_principal_point_remains_origin(synthetic, tmp_path):
    doc = _absolute_manifest(synthetic)
    doc["camera"]["calibration"] = {"image_size": [400, 800],
                                    "K": [[400, 0, 185], [0, 400, 375], [0, 0, 1]],
                                    "distortion": [0, 0, 0, 0, 0]}
    p = tmp_path / "manifest.json"
    p.write_text(json.dumps(doc))
    doc, frames, _ = read_manifest(p, Config())
    initial, _, origin, _ = initial_placement(doc, frames)
    assert origin["reference_image_pixel"] == [185., 375.]
    np.testing.assert_allclose(project(initial[5], [[185, 375]]), [[0, 0]], atol=1e-10)

