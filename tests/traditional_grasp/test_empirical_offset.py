"""Tests for the empirical end-to-end grasp offset.

The offset comes from teleop grasps, not from a validated gripper calibration,
so the tests pin down that it stays optional, stays out of ``tip_offset_m`` and
is applied in the same contact frame the source data used.
"""

from __future__ import annotations

import numpy as np
import pytest

from rpent_traditional_grasp.config import PlannerConfig, TraditionalGraspConfig
from rpent_traditional_grasp.models import BottleEstimate
from rpent_traditional_grasp.planning import (
    LEFT_SHOULDER_BODY,
    compute_gripper_tcp_target,
)


def _estimate(center: tuple[float, float, float]) -> BottleEstimate:
    return BottleEstimate(
        class_name="bottle",
        confidence=1.0,
        bbox_xyxy=(0, 0, 10, 10),
        center_uv=(5.0, 5.0),
        front_center_camera_m=np.zeros(3),
        center_camera_m=np.zeros(3),
        center_body_m=np.asarray(center, dtype=np.float64),
        axis_camera=np.array([0.0, 1.0, 0.0]),
        diameter_m=0.06,
        front_depth_m=0.5,
        depth_mad_m=0.001,
        valid_depth_pixels=500,
    )


CENTER = (0.4414, 0.0740, 0.0524)


def test_offset_defaults_to_disabled() -> None:
    config = PlannerConfig()
    assert config.left_empirical_grasp_offset_m == (0.0, 0.0, 0.0)
    target = compute_gripper_tcp_target(
        _estimate(CENTER), requested_arm="left", config=config
    )
    assert np.allclose(target.tcp_body_xyz_m, CENTER)


def test_lateral_offset_moves_target_perpendicular_to_the_approach() -> None:
    config = PlannerConfig(left_empirical_grasp_offset_m=(0.0, 0.0874, 0.0))
    target = compute_gripper_tcp_target(
        _estimate(CENTER), requested_arm="left", config=config
    )
    delta = np.asarray(target.tcp_body_xyz_m) - np.asarray(CENTER)
    approach = np.asarray(CENTER) - LEFT_SHOULDER_BODY
    approach[2] = 0.0
    approach /= np.linalg.norm(approach)
    # A pure y correction must not change how far along the approach we stop.
    assert abs(float(delta @ approach)) < 1e-9
    assert np.isclose(np.linalg.norm(delta), 0.0874)
    # +y in the contact frame is the left of the approach, so body y grows.
    assert delta[1] > 0.0


def test_axial_offset_moves_along_the_approach() -> None:
    config = PlannerConfig(left_empirical_grasp_offset_m=(0.0146, 0.0, 0.0))
    target = compute_gripper_tcp_target(
        _estimate(CENTER), requested_arm="left", config=config
    )
    delta = np.asarray(target.tcp_body_xyz_m) - np.asarray(CENTER)
    approach = np.asarray(CENTER) - LEFT_SHOULDER_BODY
    approach[2] = 0.0
    approach /= np.linalg.norm(approach)
    assert np.isclose(float(delta @ approach), 0.0146)


def test_arms_use_their_own_offset() -> None:
    config = PlannerConfig(
        left_empirical_grasp_offset_m=(0.0, 0.0874, 0.0),
        right_empirical_grasp_offset_m=(0.0, 0.0388, 0.0),
    )
    left = compute_gripper_tcp_target(
        _estimate(CENTER), requested_arm="left", config=config
    )
    right = compute_gripper_tcp_target(
        _estimate((0.4414, -0.0740, 0.0524)), requested_arm="right", config=config
    )
    left_delta = np.linalg.norm(np.asarray(left.tcp_body_xyz_m) - np.asarray(CENTER))
    right_delta = np.linalg.norm(
        np.asarray(right.tcp_body_xyz_m) - np.asarray((0.4414, -0.0740, 0.0524))
    )
    assert np.isclose(left_delta, 0.0874)
    assert np.isclose(right_delta, 0.0388)


def test_absurd_offset_is_rejected_by_validation() -> None:
    config = TraditionalGraspConfig()
    config.planner.left_empirical_grasp_offset_m = (0.0, 0.4, 0.0)
    with pytest.raises(ValueError, match=r"0\.15"):
        config.validate()


def test_offset_does_not_touch_the_kinematic_tip_offset() -> None:
    # The chain already carries wrist→TCP; folding the empirical correction in
    # there would corrupt the kinematic model and the collision geometry.
    config = PlannerConfig(left_empirical_grasp_offset_m=(0.0146, 0.0874, 0.0013))
    assert config.tip_offset_m == 0.150215608966


def test_configured_thor_offsets_carry_the_centre_convention_correction() -> None:
    """The shipped numbers must be the corrected ones, not the raw calibration.

    tcp_calibration.json measures against the median of the whole masked cloud.
    Our centre is 17.9 mm further along the approach and 16.6 mm lower, so the
    raw values would apply a centre-convention difference as a gripper error.
    """
    config = TraditionalGraspConfig.from_json("thor.example.json")
    left = np.asarray(config.planner.left_empirical_grasp_offset_m)
    right = np.asarray(config.planner.right_empirical_grasp_offset_m)
    raw_left = np.array([0.0178, 0.0874, 0.0013])
    raw_right = np.array([-0.0015, 0.0388, 0.0301])
    # depth 0.1034 vs our equivalent 0.100216, then the measured centre gap.
    axial = 0.1034 - 0.100216
    centre_gap = np.array([0.0179, 0.0025, -0.0166])
    assert np.allclose(left, raw_left - centre_gap - [axial, 0, 0], atol=5e-5)
    assert np.allclose(right, raw_right - centre_gap - [axial, 0, 0], atol=5e-5)
    # The lateral term is what the two chains agree on; it must survive.
    assert left[1] > 0.08
