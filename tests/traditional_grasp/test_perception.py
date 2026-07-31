from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace

from rpent_traditional_grasp.perception import YoloWorldDetector


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
