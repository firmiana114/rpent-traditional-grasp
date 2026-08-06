"""TensorRT CREStereo backend: fallback behaviour, preprocessing and config."""

from __future__ import annotations

import numpy as np
import pytest

from rpent_traditional_grasp.config import TraditionalGraspConfig
from rpent_traditional_grasp.stereo import TensorRTCREStereoBackend

H, W = 480, 640


class _RecordingFallback:
    """Stands in for the vendor onnxruntime backend."""

    def __init__(self) -> None:
        self.calls = 0

    def predict_disparity(self, left, right):
        self.calls += 1
        return np.full((H, W), 7.0, dtype=np.float32)


def _frame() -> np.ndarray:
    return np.zeros((H, W, 3), dtype=np.uint8)


def test_missing_engine_falls_back_instead_of_raising(tmp_path):
    """A machine that never built an engine must still be able to grasp."""
    fallback = _RecordingFallback()
    backend = TensorRTCREStereoBackend(tmp_path / "absent.engine", fallback=fallback)
    disparity = backend.predict_disparity(_frame(), _frame())
    assert fallback.calls == 1
    assert disparity.shape == (H, W)


def test_unreadable_engine_falls_back(tmp_path):
    """A truncated or version-mismatched engine degrades, it does not crash."""
    engine = tmp_path / "broken.engine"
    engine.write_bytes(b"not a serialized engine")
    fallback = _RecordingFallback()
    backend = TensorRTCREStereoBackend(engine, fallback=fallback)
    assert backend.predict_disparity(_frame(), _frame()).shape == (H, W)
    assert fallback.calls == 1


def test_load_failure_is_not_retried_every_frame(tmp_path):
    """Deserialization is expensive; a known-bad engine must be tried once."""
    engine = tmp_path / "broken.engine"
    engine.write_bytes(b"still not an engine")
    fallback = _RecordingFallback()
    backend = TensorRTCREStereoBackend(engine, fallback=fallback)
    for _ in range(3):
        backend.predict_disparity(_frame(), _frame())
    assert fallback.calls == 3
    assert backend._load_failed is True


def test_no_fallback_surfaces_the_failure(tmp_path):
    """Without a fallback the caller must hear about it rather than get zeros."""
    backend = TensorRTCREStereoBackend(tmp_path / "absent.engine", fallback=None)
    with pytest.raises(RuntimeError):
        backend.predict_disparity(_frame(), _frame())


def test_preprocessing_matches_the_vendor_layout():
    """BGR->RGB, NCHW, float32 and raw 0-255: CREStereo takes unnormalized input."""
    backend = TensorRTCREStereoBackend("unused.engine")
    image = np.zeros((H, W, 3), dtype=np.uint8)
    image[:, :, 0] = 10  # blue
    image[:, :, 1] = 20  # green
    image[:, :, 2] = 30  # red
    tensor = backend._prepare_input(image)
    assert tensor.shape == (1, 3, H, W)
    assert tensor.dtype == np.float32
    # Channel order must be RGB after the conversion, and values unscaled.
    assert (tensor[0, 0] == 30).all()
    assert (tensor[0, 1] == 20).all()
    assert (tensor[0, 2] == 10).all()
    assert tensor.flags["C_CONTIGUOUS"]


def test_preprocessing_resizes_to_the_engine_input():
    backend = TensorRTCREStereoBackend("unused.engine", input_size=(320, 240))
    tensor = backend._prepare_input(np.zeros((H, W, 3), dtype=np.uint8))
    assert tensor.shape == (1, 3, 240, 320)


def test_close_is_safe_before_any_load():
    TensorRTCREStereoBackend("unused.engine").close()


def test_tensorrt_is_the_default_depth_backend():
    assert TraditionalGraspConfig().perception.depth_backend == "tensorrt"


def test_onnx_depth_backend_remains_selectable():
    config = TraditionalGraspConfig()
    config.perception.depth_backend = "onnx"
    config.validate()


def test_unknown_depth_backend_is_refused():
    config = TraditionalGraspConfig()
    config.perception.depth_backend = "bogus"
    with pytest.raises(ValueError):
        config.validate()


def test_tensorrt_backend_requires_an_engine_path():
    config = TraditionalGraspConfig()
    config.resources.crestereo_engine = ""
    with pytest.raises(ValueError):
        config.validate()
