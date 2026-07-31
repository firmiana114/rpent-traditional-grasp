from __future__ import annotations

import pytest

from rpent_traditional_grasp.config import TraditionalGraspConfig


def test_live_mode_requires_all_motion_gates() -> None:
    with pytest.raises(ValueError, match="live 模式安全门未满足"):
        TraditionalGraspConfig.from_mapping(
            {"safety": {"mode": "live", "allow_motion": True}}
        )


def test_live_mode_requires_gripper_tcp_calibration() -> None:
    with pytest.raises(ValueError, match="gripper_tcp_calibration_validated"):
        TraditionalGraspConfig.from_mapping(
            {
                "safety": {
                    "mode": "live",
                    "allow_motion": True,
                    "stereo_calibration_validated": True,
                    "camera_to_body_validated": True,
                }
            }
        )


def test_unknown_config_field_is_rejected() -> None:
    with pytest.raises(ValueError, match="未知字段"):
        TraditionalGraspConfig.from_mapping(
            {"planner": {"automatic_home": True}}
        )
