from __future__ import annotations

import importlib.util
import logging
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from rpent_traditional_grasp.image_trace import (
    pixel_sha256,
    save_segmentation_pngs,
)
from rpent_traditional_grasp.models import Detection
from rpent_traditional_grasp.perception import (
    Sam2BoxSegmenter,
    YoloWorldDetector,
)


class _FakeYoloWorld:
    def __init__(self, model_path: str) -> None:
        self.model_path = model_path


def test_yolo_world_falls_back_to_pt_without_tensorrt_bindings(
    tmp_path: Path,
    monkeypatch,
) -> None:
    engine_path = tmp_path / "model.engine"
    pt_path = tmp_path / "model.pt"
    engine_path.touch()
    pt_path.touch()
    fake_ultralytics = SimpleNamespace(
        YOLO=lambda _path, task: (_path, task),
        YOLOWorld=_FakeYoloWorld,
    )
    monkeypatch.setitem(sys.modules, "ultralytics", fake_ultralytics)
    original_find_spec = importlib.util.find_spec
    monkeypatch.setattr(
        importlib.util,
        "find_spec",
        lambda name: None if name == "tensorrt" else original_find_spec(name),
    )
    detector = YoloWorldDetector(engine_path, pt_path)

    model = detector._load(["bottle"])

    assert isinstance(model, _FakeYoloWorld)
    assert model.model_path == str(pt_path)
    assert detector._dynamic_classes is True


class _FakeSamPredictor:
    def __init__(self, masks: np.ndarray, scores: np.ndarray) -> None:
        self.masks = masks
        self.scores = scores
        self.image: np.ndarray | None = None
        self.box: np.ndarray | None = None

    def set_image(self, image: np.ndarray) -> None:
        self.image = image

    def predict(self, **kwargs: object) -> tuple[np.ndarray, np.ndarray, None]:
        self.box = np.asarray(kwargs["box"])
        return self.masks, self.scores, None


def test_sam2_log_correlates_frame_box_scores_and_mask(
    tmp_path: Path,
    caplog,
) -> None:
    image = np.arange(6 * 8 * 3, dtype=np.uint8).reshape(6, 8, 3)
    masks = np.zeros((3, 6, 8), dtype=bool)
    masks[1, 2:5, 3:7] = True
    predictor = _FakeSamPredictor(
        masks,
        np.asarray([0.1, 0.9, 0.2], dtype=np.float32),
    )
    segmenter = Sam2BoxSegmenter(
        tmp_path,
        tmp_path / "checkpoint.pt",
        "config.yaml",
        device="cpu",
    )
    segmenter._predictor = predictor
    detection = Detection("water bottle", 1.0, (2, 1, 7, 5))

    with caplog.at_level(logging.INFO, logger="rpent_traditional_grasp"):
        mask = segmenter.segment(image, detection)

    np.testing.assert_array_equal(mask, masks[1])
    np.testing.assert_array_equal(predictor.image, image)
    np.testing.assert_array_equal(
        predictor.box,
        np.asarray(detection.bbox_xyxy, dtype=np.float32),
    )
    rendered = "\n".join(caplog.messages)
    assert f"image_sha256={pixel_sha256(image)}" in rendered
    assert "bbox=(2, 1, 7, 5)" in rendered
    assert "scores=[0.1, 0.9, 0.2]" in rendered
    assert "mask_bbox=(3, 2, 7, 5)" in rendered


def test_sam2_publishes_and_clears_the_artifact_paths(tmp_path: Path) -> None:
    """``search_object`` can only report the diagnostic images it is handed.

    The paths are recomputed per frame, so a stale value would point the upper
    layer at the previous run's mask; the reset therefore matters as much as
    the assignment.
    """
    pytest.importorskip("cv2")
    image = np.zeros((6, 8, 3), dtype=np.uint8)
    masks = np.zeros((2, 6, 8), dtype=bool)
    masks[0, 1:4, 2:6] = True
    predictor = _FakeSamPredictor(
        masks,
        np.asarray([0.9, 0.1], dtype=np.float32),
    )
    detection = Detection("water bottle", 1.0, (2, 1, 7, 5))
    segmenter = Sam2BoxSegmenter(
        tmp_path,
        tmp_path / "checkpoint.pt",
        "config.yaml",
        device="cpu",
        artifact_dir=tmp_path / "artifacts",
    )
    segmenter._predictor = predictor

    segmenter.segment(image, detection)

    assert set(segmenter.last_artifacts) == {
        "bbox_image_path",
        "mask_image_path",
        "overlay_image_path",
        "result_image_path",
    }
    assert all(
        Path(path).is_file() for path in segmenter.last_artifacts.values()
    )
    assert (
        segmenter.last_artifacts["result_image_path"]
        == segmenter.last_artifacts["overlay_image_path"]
    )

    segmenter.artifact_dir = None
    segmenter.segment(image, detection)

    assert segmenter.last_artifacts == {}


def test_segmentation_artifacts_include_box_mask_and_overlay(
    tmp_path: Path,
) -> None:
    cv2 = pytest.importorskip("cv2")
    image = np.zeros((20, 30, 3), dtype=np.uint8)
    mask = np.zeros((20, 30), dtype=bool)
    mask[6:15, 10:19] = True

    bbox_path, mask_path, overlay_path = save_segmentation_pngs(
        tmp_path,
        image=image,
        image_sha256=pixel_sha256(image),
        bbox_xyxy=(8, 4, 21, 17),
        mask=mask,
    )

    assert bbox_path.is_file()
    assert mask_path.is_file()
    assert overlay_path.is_file()
    saved_mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
    assert saved_mask is not None
    np.testing.assert_array_equal(saved_mask > 0, mask)
