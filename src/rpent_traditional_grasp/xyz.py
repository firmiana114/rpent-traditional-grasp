"""Structured XYZ reports for the image-only stage-one acceptance test."""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path
from typing import Any

import numpy as np

from rpent_traditional_grasp.config import PlannerConfig
from rpent_traditional_grasp.logging import get_logger
from rpent_traditional_grasp.models import BottleEstimate
from rpent_traditional_grasp.planning import compute_gripper_tcp_target

logger = get_logger("xyz")


def build_xyz_report(
    *,
    estimate: BottleEstimate,
    left_image: str | Path,
    right_image: str | Path,
    target: str,
    stereo_calibration_validated: bool,
    camera_to_body_validated: bool,
    expected_body_xyz_m: Sequence[float] | None = None,
    tolerance_m: float = 0.03,
) -> dict[str, Any]:
    """Build a machine-readable image-to-XYZ result and optional truth check.

    A result without ``expected_body_xyz_m`` proves that the pipeline produced
    a finite XYZ estimate. It does not prove metric accuracy. When measured
    ground truth is supplied, the report also applies a Euclidean-error gate.
    """
    body_xyz = _xyz(estimate.center_body_m, "center_body_m")
    camera_xyz = _xyz(estimate.center_camera_m, "center_camera_m")
    front_camera_xyz = _xyz(
        estimate.front_center_camera_m,
        "front_center_camera_m",
    )
    if tolerance_m <= 0.0 or not np.isfinite(tolerance_m):
        raise ValueError("tolerance_m 必须是有限正数")

    calibration_approved = bool(
        stereo_calibration_validated and camera_to_body_validated
    )
    acceptance: dict[str, Any]
    if expected_body_xyz_m is None:
        acceptance = {
            "evaluated": False,
            "passed": None,
            "reason": "ground_truth_not_provided",
            "tolerance_m": float(tolerance_m),
        }
        success = True
    else:
        expected = _xyz(expected_body_xyz_m, "expected_body_xyz_m")
        error = body_xyz - expected
        error_norm = float(np.linalg.norm(error))
        passed = error_norm <= tolerance_m
        acceptance = {
            "evaluated": True,
            "passed": passed,
            "expected_body_xyz_m": expected.tolist(),
            "error_body_xyz_m": error.tolist(),
            "euclidean_error_m": error_norm,
            "tolerance_m": float(tolerance_m),
        }
        success = passed
        logger.info(
            "图片到 XYZ 真值验收完成: passed=%s error=%.4fm tolerance=%.4fm",
            passed,
            error_norm,
            tolerance_m,
        )

    logger.info(
        "图片到 XYZ 输出完成: target=%s body_xyz=[%.4f,%.4f,%.4f] "
        "camera_xyz=[%.4f,%.4f,%.4f] calibration_approved=%s",
        target,
        *body_xyz,
        *camera_xyz,
        calibration_approved,
    )
    return {
        "schema_version": 1,
        "stage": "stereo_image_to_xyz",
        "success": success,
        "inputs": {
            "left_image": str(left_image),
            "right_image": str(right_image),
            "target": target,
        },
        "coordinates": {
            "object_center_body_xyz_m": body_xyz.tolist(),
            "object_center_camera_xyz_m": camera_xyz.tolist(),
            "front_surface_camera_xyz_m": front_camera_xyz.tolist(),
        },
        "frames": {
            "body": "x_forward_y_left_z_up",
            "camera": "x_right_y_down_z_forward",
        },
        "detection": {
            "class_name": estimate.class_name,
            "confidence": float(estimate.confidence),
            "bbox_xyxy": list(estimate.bbox_xyxy),
            "center_uv": list(estimate.center_uv),
        },
        "quality": {
            "front_depth_m": float(estimate.front_depth_m),
            "diameter_m": float(estimate.diameter_m),
            "depth_mad_m": float(estimate.depth_mad_m),
            "valid_depth_pixels": int(estimate.valid_depth_pixels),
        },
        "calibration": {
            "stereo_validated": bool(stereo_calibration_validated),
            "camera_to_body_validated": bool(camera_to_body_validated),
            "metric_xyz_approved": calibration_approved,
        },
        "acceptance": acceptance,
    }


def build_gripper_xyz_report(
    *,
    estimate: BottleEstimate,
    left_image: str | Path,
    right_image: str | Path,
    target: str,
    requested_arm: str,
    planner_config: PlannerConfig,
    stereo_calibration_validated: bool,
    camera_to_body_validated: bool,
    gripper_tcp_calibration_validated: bool,
    expected_gripper_xyz_m: Sequence[float] | None = None,
    tolerance_m: float = 0.03,
) -> dict[str, Any]:
    """Build the stage-two final gripper TCP XYZ report."""
    report = build_xyz_report(
        estimate=estimate,
        left_image=left_image,
        right_image=right_image,
        target=target,
        stereo_calibration_validated=stereo_calibration_validated,
        camera_to_body_validated=camera_to_body_validated,
        tolerance_m=tolerance_m,
    )
    gripper = compute_gripper_tcp_target(
        estimate,
        requested_arm=requested_arm,
        config=planner_config,
    )
    actual = _xyz(gripper.tcp_body_xyz_m, "gripper_tcp_body_xyz_m")
    if expected_gripper_xyz_m is None:
        acceptance = {
            "evaluated": False,
            "passed": None,
            "reason": "ground_truth_not_provided",
            "tolerance_m": float(tolerance_m),
        }
        success = True
    else:
        expected = _xyz(
            expected_gripper_xyz_m,
            "expected_gripper_xyz_m",
        )
        error = actual - expected
        error_norm = float(np.linalg.norm(error))
        passed = error_norm <= tolerance_m
        acceptance = {
            "evaluated": True,
            "passed": passed,
            "expected_gripper_xyz_m": expected.tolist(),
            "error_gripper_xyz_m": error.tolist(),
            "euclidean_error_m": error_norm,
            "tolerance_m": float(tolerance_m),
        }
        success = passed
        logger.info(
            "最终夹爪 XYZ 真值验收完成: passed=%s error=%.4fm "
            "tolerance=%.4fm",
            passed,
            error_norm,
            tolerance_m,
        )

    report["stage"] = "stereo_image_to_gripper_xyz"
    report["entrypoint"] = "pick_object"
    report["success"] = success
    report["calibration"]["gripper_tcp_validated"] = bool(
        gripper_tcp_calibration_validated
    )
    report["calibration"]["metric_gripper_xyz_approved"] = bool(
        report["calibration"]["metric_xyz_approved"]
        and gripper_tcp_calibration_validated
    )
    report["gripper_target"] = {
        "selected_arm": gripper.arm,
        "requested_arm": requested_arm,
        "selection_policy": gripper.selection_policy,
        "final_tcp_body_xyz_m": actual.tolist(),
        "tcp_reference": "modeled_grasp_center_between_fingers",
        "orientation_policy": "preserve_initial",
        "orientation_commanded": False,
        "distance_from_body_origin_m": gripper.distance_m,
        "max_reach_gate_m": float(planner_config.max_reach_m),
        "wrist_to_tcp_offset_m": float(planner_config.tip_offset_m),
        "tcp_offset_application": "encoded_in_kinematic_chain_not_added_again",
    }
    report["acceptance"] = acceptance
    logger.info(
        "图片到最终夹爪 XYZ 输出完成: arm=%s xyz=[%.4f,%.4f,%.4f]",
        gripper.arm,
        *actual,
    )
    return report


def _xyz(values: Sequence[float] | np.ndarray, name: str) -> np.ndarray:
    xyz = np.asarray(values, dtype=np.float64)
    if xyz.shape != (3,):
        raise ValueError(f"{name} 必须包含 3 个元素")
    if not np.all(np.isfinite(xyz)):
        raise ValueError(f"{name} 包含非有限值")
    return xyz
