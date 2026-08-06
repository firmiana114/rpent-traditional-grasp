"""VLM grounding backend: reply parsing, backend selection and failure modes."""

from __future__ import annotations

import numpy as np
import pytest

from rpent_traditional_grasp.config import TraditionalGraspConfig
from rpent_traditional_grasp.perception import VlmDetector

W, H = 640, 480
COLA = (202, 177, 297, 207)


def _detector() -> VlmDetector:
    return VlmDetector("http://localhost:8000/v1", "test-model")


def test_normalized_grid_is_converted_to_pixels():
    """Qwen reports boxes on a 0-1000 grid, not in pixels."""
    assert _detector()._parse('{"bbox":[316,369,463,431]}', W, H) == [
        (202, 177, 296, 207)
    ]


def test_pixel_box_is_left_alone():
    """A box already inside the frame must not be rescaled."""
    assert _detector()._parse('{"bbox":[202,177,297,207]}', W, H) == [COLA]


def test_alternate_key_names_are_accepted():
    """The same model answered with bbox and bbox_2d on consecutive calls."""
    for key in ("bbox", "bbox_2d", "box"):
        assert _detector()._parse('{"%s":[202,177,297,207]}' % key, W, H) == [COLA]


def test_json_wrapped_in_markdown_fence_is_recovered():
    reply = '```json\n{"bbox": [202, 177, 297, 207]}\n```'
    assert _detector()._parse(reply, W, H) == [COLA]


def test_nested_list_takes_the_first_box():
    assert _detector()._parse('{"boxes":[[202,177,297,207]]}', W, H) == [COLA]


@pytest.mark.parametrize(
    "reply",
    [
        '{"bbox":null}',            # 模型明确说没有
        "no box here",              # 完全不是 JSON
        '{"bbox":[1,2,3]}',         # 长度不对
        '{"bbox":["a","b","c","d"]}',   # 不是数值
        '{"bbox":[10,10,11,11]}',   # 退化成一个点
        "{not json}",               # JSON 解析失败
    ],
)
def test_unusable_replies_yield_no_detection(reply):
    assert _detector()._parse(reply, W, H) == []


def test_box_is_clamped_to_the_frame():
    (x1, y1, x2, y2), = _detector()._parse('{"bbox":[-30,-20,900,700]}', W, H)
    assert (x1, y1) == (0, 0) and (x2, y2) == (W, H)


def test_inverted_corners_are_reordered():
    assert _detector()._parse('{"bbox":[297,207,202,177]}', W, H) == [COLA]


def test_detect_reports_the_target_and_survives_a_dead_endpoint():
    """An unreachable model must degrade to "not found", never raise."""
    d = VlmDetector("http://127.0.0.1:9/v1", "test-model", timeout_s=0.2)
    assert d.detect(np.zeros((H, W, 3), dtype=np.uint8), ["the cola bottle"]) == []


def test_detect_uses_only_the_first_prompt():
    """Generic fallbacks exist for YOLO's vocabulary and would mislead a VLM."""
    seen = {}

    class _Stub(VlmDetector):
        def _request(self, image, target, width, height):
            seen["target"] = target
            return '{"bbox":[202,177,297,207]}'

    got = _Stub("http://x/v1", "m").detect(
        np.zeros((H, W, 3), dtype=np.uint8),
        ["the black cola bottle", "bottle", "water bottle"],
    )
    assert seen["target"] == "the black cola bottle"
    assert [d.bbox_xyxy for d in got] == [COLA]
    assert got[0].class_name == "the black cola bottle"


def test_empty_prompts_are_rejected_without_a_request():
    d = VlmDetector("http://127.0.0.1:9/v1", "m", timeout_s=0.2)
    image = np.zeros((H, W, 3), dtype=np.uint8)
    assert d.detect(image, []) == []
    assert d.detect(image, ["   "]) == []


def test_vlm_is_the_default_backend():
    assert TraditionalGraspConfig().perception.detector_backend == "vlm"


def test_yolo_world_remains_selectable():
    c = TraditionalGraspConfig()
    c.perception.detector_backend = "yolo_world"
    c.validate()


@pytest.mark.parametrize(
    "field,value",
    [
        ("detector_backend", "bogus"),
        ("vlm_timeout_s", 0.0),
        ("vlm_max_tokens", 0),
    ],
)
def test_invalid_detector_settings_are_refused(field, value):
    c = TraditionalGraspConfig()
    setattr(c.perception, field, value)
    with pytest.raises(ValueError):
        c.validate()


def test_vlm_backend_requires_an_endpoint():
    c = TraditionalGraspConfig()
    c.resources.vlm_endpoint = ""
    with pytest.raises(ValueError):
        c.validate()
