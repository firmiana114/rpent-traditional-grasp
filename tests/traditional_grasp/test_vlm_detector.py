"""VLM grounding backend: reply parsing, backend selection and failure modes."""

from __future__ import annotations

import numpy as np
import pytest

from rpent_traditional_grasp.config import TraditionalGraspConfig
from rpent_traditional_grasp.perception import VlmDetector

W, H = 640, 480
COLA = (202, 177, 297, 207)


def _detector(space: str = "normalized_1000") -> VlmDetector:
    return VlmDetector("http://localhost:8000/v1", "test-model", coordinate_space=space)


def test_normalized_grid_is_converted_to_pixels():
    """Qwen reports boxes on a 0-1000 grid, not in pixels."""
    assert _detector()._parse('{"bbox":[316,369,463,431]}', W, H) == [
        (202, 177, 296, 207)
    ]


def test_the_space_cannot_be_guessed_from_the_numbers():
    """The measured reply is a valid box in BOTH spaces; only config resolves it.

    This is why the space is configured rather than sniffed: [316,369,463,431]
    fits inside 640x480 as pixels, yet only the scaled reading lands on the
    bottle.
    """
    reply = '{"bbox":[316,369,463,431]}'
    assert _detector("normalized_1000")._parse(reply, W, H) == [(202, 177, 296, 207)]
    assert _detector("pixel")._parse(reply, W, H) == [(316, 369, 463, 431)]


def test_pixel_space_leaves_the_box_alone():
    assert _detector("pixel")._parse('{"bbox":[202,177,297,207]}', W, H) == [COLA]


def test_auto_only_rules_out_values_overflowing_the_grid():
    """auto assumes the 0-1000 grid unless a value cannot fit on it."""
    assert _detector("auto")._parse('{"bbox":[316,369,463,431]}', W, H) == [
        (202, 177, 296, 207)
    ]
    # >1000 cannot be a grid coordinate, so it is read as pixels and clamped.
    assert _detector("auto")._parse('{"bbox":[100,100,1400,1450]}', W, H) == [
        (100, 100, W, H)
    ]


def test_unknown_coordinate_space_is_refused():
    with pytest.raises(ValueError):
        VlmDetector("http://x/v1", "m", coordinate_space="bogus")


def test_alternate_key_names_are_accepted():
    """The same model answered with bbox and bbox_2d on consecutive calls."""
    for key in ("bbox", "bbox_2d", "box"):
        assert _detector("pixel")._parse(
            '{"%s":[202,177,297,207]}' % key, W, H
        ) == [COLA]


def test_json_wrapped_in_markdown_fence_is_recovered():
    reply = '```json\n{"bbox": [202, 177, 297, 207]}\n```'
    assert _detector("pixel")._parse(reply, W, H) == [COLA]


def test_nested_list_takes_the_first_box():
    assert _detector("pixel")._parse('{"boxes":[[202,177,297,207]]}', W, H) == [COLA]


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
    assert _detector("pixel")._parse(reply, W, H) == []


def test_box_is_clamped_to_the_frame():
    """Clamping happens after conversion, so it holds in either space."""
    (x1, y1, x2, y2), = _detector("pixel")._parse('{"bbox":[-30,-20,900,700]}', W, H)
    assert (x1, y1) == (0, 0) and (x2, y2) == (W, H)
    (x1, y1, x2, y2), = _detector()._parse('{"bbox":[-50,-50,1200,1200]}', W, H)
    assert (x1, y1) == (0, 0) and (x2, y2) == (W, H)


def test_inverted_corners_are_reordered():
    assert _detector("pixel")._parse('{"bbox":[297,207,202,177]}', W, H) == [COLA]


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

    got = _Stub("http://x/v1", "m", coordinate_space="pixel").detect(
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
        ("vlm_coordinate_space", "bogus"),
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
