import json
from pathlib import Path
import cv2
import numpy as np
import pytest

from drone_stitch.config import Config
from drone_stitch.coordinates import MapCoordinates
from drone_stitch.geometry import image_to_ground, ned_to_local_matrix, project, validate_footprint
from drone_stitch.inputs import Frame, initial_placement
from drone_stitch.metadata import read_telemetry, sample_telemetry, validate_metadata
from drone_stitch.render import blend, canvas_geometry


def frame(i=5, colour=0):
    return Frame(i, Path("test.png"), np.full((80, 40, 3), colour, np.uint8),
                 np.full((80, 40), 255, np.uint8), {}, {}, None, (19.5, 39.5))


def test_ned_axes_and_heading():
    h = ned_to_local_matrix([100, 200, -5], 0)
    np.testing.assert_allclose(project(h, [[100, 200], [101, 200], [100, 201]]),
                               [[0, 0], [0, 1], [1, 0]])
    eastward = ned_to_local_matrix([100, 200, -5], np.pi / 2)
    np.testing.assert_allclose(project(eastward, [[101, 200], [100, 201]]), [[-1, 0], [0, 1]], atol=1e-12)


def test_pixel_axes_yaw_and_noncentral_principal_point():
    h = image_to_ground(400, 800, [5, 10], angle_rad=np.pi / 2, anchor_pixel=(180, 380))
    np.testing.assert_allclose(project(h, [[180, 380], [180, 300], [260, 380]]),
                               [[0, 0], [1, 0], [0, -1]], atol=1e-12)


def test_telemetry_yaw_wrap_and_partial_signals():
    rows = [{"timestamp_s": 0., "yaw_rad": np.deg2rad(179), "ned_m": [0, 0, -5]},
            {"timestamp_s": .1, "agl_m": 5.},
            {"timestamp_s": .2, "yaw_rad": np.deg2rad(-179), "ned_m": [2, 4, -7]}]
    result, source = sample_telemetry(rows, .1)
    assert abs(abs(result["yaw_rad"]) - np.pi) < 1e-12
    assert result["ned_m"] == [1, 2, -6]
    assert result["agl_m"] == 5
    assert source["yaw_rad"]["method"] == "interpolated"


def test_stale_metadata_never_bridges_large_gap():
    rows = [{"timestamp_s": 0., "ned_m": [0, 0, 0]}, {"timestamp_s": 10., "ned_m": [10, 0, 0]}]
    assert sample_telemetry(rows, 5)[0] == {}
    assert sample_telemetry(rows, 10.2)[0] == {}
    assert sample_telemetry(rows, 10.01)[1]["ned_m"]["method"] == "nearest"


def test_telemetry_rejects_clock_reset(tmp_path):
    p = tmp_path / "telemetry.csv"
    p.write_text("timestamp_s,yaw_rad\n2,0\n1,0\n")
    with pytest.raises(ValueError, match="strictly increase"):
        read_telemetry(p)


@pytest.mark.parametrize("metadata", [{"ned_m": [1, 2]}, {"yaw_rad": float("nan")}, {"agl_m": -5}])
def test_invalid_metadata(metadata):
    with pytest.raises(ValueError):
        validate_metadata(metadata)


def test_missing_heading_does_not_silently_swap_ned_axes():
    ref, other = frame(), frame(6)
    ref.metadata = {"ned_m": [100, 200, -100]}
    other.metadata = {"ned_m": [100, 210, -200]}
    doc = {"camera": {"footprint_m": [5, 10], "reference_agl_m": 5.}}
    _, info, origin, warnings = initial_placement(doc, [ref, other])
    assert info[6]["center_local_m"] == [2.5, 0.]
    assert info[6]["agl_used_m"] == 5.  # NED down is deliberately not AGL.
    assert origin["ned_ne_to_local_matrix"] is None
    assert warnings


def test_yaw_mount_offset_and_ned_origin():
    ref, other = frame(), frame(6)
    ref.metadata = {"ned_m": [100, 200, -5], "yaw_rad": np.pi / 2}
    other.metadata = {"ned_m": [100, 202.5, -5], "yaw_rad": np.pi / 2}
    doc = {"camera": {"footprint_m": [5, 10], "image_top_yaw_offset_rad": -np.pi / 2}}
    matrices, info, origin, _ = initial_placement(doc, [ref, other])
    assert origin["grid_heading_rad"] == 0
    assert info[6]["position_source"] == "relative_ned"
    np.testing.assert_allclose(project(matrices[6], [other.anchor_pixel]), [[2.5, 0]])


def test_black_pixels_are_valid_during_blending():
    a, b = frame(5, 0), frame(6, 100)
    h = image_to_ground(40, 80, [5, 10])
    image, mask, coverage, matrix, _ = blend([a, b], {5: h, 6: h}, Config(pixels_per_meter=8))
    u, v = np.rint(project(matrix, [[0, 0]])[0]).astype(int)
    assert mask[v, u] == 255
    assert coverage[v, u] == 2
    np.testing.assert_array_equal(image[v, u], [50, 50, 50])


def test_canvas_limit_before_allocation():
    f = frame()
    with pytest.raises(ValueError, match="Canvas"):
        canvas_geometry([f], {5: image_to_ground(40, 80, [500, 1000])}, Config())


def test_reject_homography_horizon():
    h = np.eye(3)
    h[2, 0] = -.02
    with pytest.raises(ValueError, match="horizon"):
        validate_footprint(h, 100, 100)


def test_raw_pixel_undistortion_mapping(tmp_path):
    k = np.array([[300., 0, 200], [0, 300., 400], [0, 0, 1.]])
    distortion = np.array([-.1, .01, 0., 0., 0.])
    rectified = np.array([[130., 280.], [220., 430.]])
    rays = np.c_[(rectified - [200, 400]) / 300, np.ones(2)]
    raw, _ = cv2.projectPoints(rays, np.zeros(3), np.zeros(3), k, distortion)
    path = tmp_path / "transforms.json"
    path.write_text(json.dumps({"schema_version": 1, "camera": {"calibration": {"K": k.tolist(), "distortion": distortion.tolist()}},
                               "images": {"5": {"rectified_image_to_map": np.eye(3).tolist()}}}))
    coords = MapCoordinates(path)
    np.testing.assert_allclose(coords.image_to_pixel(5, raw, raw_pixels=True), rectified, atol=1e-4)


@pytest.mark.parametrize("config", [Config(pixels_per_meter=0), Config(min_inliers=3),
                                    Config(seed=-1), Config(opencv_threads=0), Config(ratio_test=1.2)])
def test_bad_config(config):
    with pytest.raises(ValueError):
        config.validate()

