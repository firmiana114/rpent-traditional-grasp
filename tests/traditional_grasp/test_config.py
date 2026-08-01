from __future__ import annotations

from pathlib import Path

import pytest

from rpent_traditional_grasp.config import PlannerConfig, TraditionalGraspConfig
from rpent_traditional_grasp.gripper import load_gripper_specification


def test_default_tcp_matches_shared_dex1_1_specification() -> None:
    root = Path(__file__).resolve().parents[2]
    specification = load_gripper_specification(
        root / "config/g1d_dex1_1_nominal.json"
    )

    assert specification.model == "Unitree Dex1-1"
    assert specification.status == "manufacturer_nominal_unvalidated"
    assert specification.is_robot_validated is False
    assert PlannerConfig().tip_offset_m == specification.wrist_to_tcp_xyz_m[0]
    assert specification.jaw_minimum_m == 0.005876
    assert specification.jaw_maximum_m == 0.094876


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


def test_side_grasp_angle_outside_limit_is_rejected() -> None:
    with pytest.raises(ValueError, match="侧抓姿态候选角度超过"):
        TraditionalGraspConfig.from_mapping(
            {"planner": {"side_grasp_pitch_degrees": [50.0]}}
        )
