from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from rpent_traditional_grasp.stereo import StereoCalibration
from rpent_traditional_grasp.thor import (
    ImagePairStereoCamera,
    ThorStereoCamera,
    load_transform,
)


class _Frame:
    def __init__(self, bgr: np.ndarray | None) -> None:
        self.bgr = bgr


class _SequenceSubscriber:
    def __init__(self, frames: list[_Frame]) -> None:
        self.frames = frames
        self.calls = 0

    def subscribe(
        self,
        host: str,
        port: int,
        *,
        request_bgr: bool,
    ) -> _Frame:
        assert host == "camera.test"
        assert port == 55555
        assert request_bgr is True
        index = min(self.calls, len(self.frames) - 1)
        self.calls += 1
        return self.frames[index]


def test_checked_in_thor_legacy_calibration_is_parseable() -> None:
    root = Path(__file__).parents[2]
    calibration = StereoCalibration.from_json(
        root / "config/thor_stereo_legacy.json"
    )

    assert calibration.image_size == (640, 480)
    assert calibration.translation_m.shape == (3, 1)
    assert 0.05 < calibration.baseline_m < 0.08


def test_checked_in_thor_legacy_transform_is_rigid() -> None:
    root = Path(__file__).parents[2]
    transform = load_transform(root / "config/thor_camera_to_body_legacy.json")

    assert transform.shape == (4, 4)
    np.testing.assert_allclose(
        np.linalg.det(transform[:3, :3]), 1.0, atol=1e-6
    )


def test_thor_camera_waits_for_asynchronous_first_frame() -> None:
    stereo = np.zeros((4, 16, 3), dtype=np.uint8)
    subscriber = _SequenceSubscriber([_Frame(None), _Frame(stereo)])
    camera = ThorStereoCamera(
        host="camera.test",
        capture_timeout_s=0.1,
        poll_interval_s=0.001,
    )
    camera._subscriber = subscriber

    left, right, timestamp_s = camera.capture_stereo()

    assert subscriber.calls == 2
    assert left.shape == right.shape == (4, 8, 3)
    assert timestamp_s > 0.0


def test_thor_camera_fails_after_bounded_timeout() -> None:
    camera = ThorStereoCamera(
        host="camera.test",
        capture_timeout_s=0.002,
        poll_interval_s=0.001,
    )
    camera._subscriber = _SequenceSubscriber([_Frame(None)])

    with pytest.raises(RuntimeError, match="未返回图像"):
        camera.capture_stereo()


def test_image_pair_camera_loads_without_online_subscription(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    left_path = tmp_path / "left.jpg"
    right_path = tmp_path / "right.jpg"
    left_path.touch()
    right_path.touch()
    images = {
        str(left_path): np.ones((4, 8, 3), dtype=np.uint8),
        str(right_path): np.full((4, 8, 3), 2, dtype=np.uint8),
    }
    fake_cv2 = SimpleNamespace(
        IMREAD_COLOR=1,
        imread=lambda path, _mode: images[path],
    )
    monkeypatch.setitem(sys.modules, "cv2", fake_cv2)

    camera = ImagePairStereoCamera(left_path, right_path)
    left, right, timestamp_s = camera.capture_stereo()

    np.testing.assert_array_equal(left, images[str(left_path)])
    np.testing.assert_array_equal(right, images[str(right_path)])
    assert timestamp_s > 0.0


def test_image_pair_camera_rejects_mismatched_sizes(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    left_path = tmp_path / "left.jpg"
    right_path = tmp_path / "right.jpg"
    left_path.touch()
    right_path.touch()
    images = {
        str(left_path): np.zeros((4, 8, 3), dtype=np.uint8),
        str(right_path): np.zeros((5, 8, 3), dtype=np.uint8),
    }
    fake_cv2 = SimpleNamespace(
        IMREAD_COLOR=1,
        imread=lambda path, _mode: images[path],
    )
    monkeypatch.setitem(sys.modules, "cv2", fake_cv2)

    with pytest.raises(ValueError, match="尺寸不一致"):
        ImagePairStereoCamera(left_path, right_path).capture_stereo()
