from __future__ import annotations

import numpy as np
import pytest

from rpent_traditional_grasp.config import PlannerConfig
from rpent_traditional_grasp.models import BottleEstimate
from rpent_traditional_grasp.xyz import (
    build_gripper_xyz_report,
    build_xyz_report,
)


def _estimate() -> BottleEstimate:
    return BottleEstimate(
        class_name="bottle",
        confidence=0.9,
        bbox_xyxy=(10, 20, 30, 80),
        center_uv=(20.0, 50.0),
        front_center_camera_m=np.array([0.01, -0.02, 0.55]),
        center_camera_m=np.array([0.01, -0.02, 0.58]),
        center_body_m=np.array([0.52, 0.08, 0.06]),
        axis_camera=np.array([0.0, 1.0, 0.0]),
        diameter_m=0.06,
        front_depth_m=0.55,
        depth_mad_m=0.004,
        valid_depth_pixels=500,
    )


def test_xyz_report_without_truth_is_not_accuracy_acceptance() -> None:
    report = build_xyz_report(
        estimate=_estimate(),
        left_image="left.jpg",
        right_image="right.jpg",
        target="bottle",
        stereo_calibration_validated=False,
        camera_to_body_validated=False,
    )

    assert report["success"] is True
    assert report["coordinates"]["object_center_body_xyz_m"] == [
        0.52,
        0.08,
        0.06,
    ]
    assert report["acceptance"]["evaluated"] is False
    assert report["acceptance"]["passed"] is None
    assert report["calibration"]["metric_xyz_approved"] is False


def test_xyz_report_passes_measured_truth_within_tolerance() -> None:
    report = build_xyz_report(
        estimate=_estimate(),
        left_image="left.jpg",
        right_image="right.jpg",
        target="bottle",
        stereo_calibration_validated=True,
        camera_to_body_validated=True,
        expected_body_xyz_m=[0.51, 0.08, 0.06],
        tolerance_m=0.02,
    )

    assert report["success"] is True
    assert report["acceptance"]["passed"] is True
    assert report["acceptance"]["euclidean_error_m"] == pytest.approx(0.01)
    assert report["calibration"]["metric_xyz_approved"] is True


def test_xyz_report_fails_truth_outside_tolerance() -> None:
    report = build_xyz_report(
        estimate=_estimate(),
        left_image="left.jpg",
        right_image="right.jpg",
        target="bottle",
        stereo_calibration_validated=True,
        camera_to_body_validated=True,
        expected_body_xyz_m=[0.40, 0.08, 0.06],
        tolerance_m=0.03,
    )

    assert report["success"] is False
    assert report["acceptance"]["passed"] is False
    assert report["acceptance"]["euclidean_error_m"] == pytest.approx(0.12)


def test_xyz_report_rejects_invalid_truth_shape() -> None:
    with pytest.raises(ValueError, match="必须包含 3 个元素"):
        build_xyz_report(
            estimate=_estimate(),
            left_image="left.jpg",
            right_image="right.jpg",
            target="bottle",
            stereo_calibration_validated=True,
            camera_to_body_validated=True,
            expected_body_xyz_m=[0.5, 0.1],
        )


def test_gripper_xyz_report_targets_bottle_center_and_preserves_orientation() -> None:
    report = build_gripper_xyz_report(
        estimate=_estimate(),
        left_image="left.jpg",
        right_image="right.jpg",
        target="bottle",
        requested_arm="auto",
        planner_config=PlannerConfig(),
        stereo_calibration_validated=False,
        camera_to_body_validated=False,
        gripper_tcp_calibration_validated=False,
    )

    target = report["gripper_target"]
    assert report["stage"] == "stereo_image_to_gripper_xyz"
    assert report["entrypoint"] == "pick_object"
    assert target["selected_arm"] == "left"
    assert target["final_tcp_body_xyz_m"] == [0.52, 0.08, 0.06]
    assert target["orientation_policy"] == "preserve_initial"
    assert target["orientation_commanded"] is False
    assert report["calibration"]["metric_gripper_xyz_approved"] is False
    assert (
        target["tcp_offset_application"]
        == "encoded_in_kinematic_chain_not_added_again"
    )


def test_gripper_xyz_report_applies_truth_gate() -> None:
    report = build_gripper_xyz_report(
        estimate=_estimate(),
        left_image="left.jpg",
        right_image="right.jpg",
        target="bottle",
        requested_arm="right",
        planner_config=PlannerConfig(),
        stereo_calibration_validated=True,
        camera_to_body_validated=True,
        gripper_tcp_calibration_validated=True,
        expected_gripper_xyz_m=[0.51, 0.08, 0.06],
        tolerance_m=0.02,
    )

    assert report["success"] is True
    assert report["gripper_target"]["selected_arm"] == "right"
    assert report["acceptance"]["passed"] is True
    assert report["calibration"]["metric_gripper_xyz_approved"] is True
