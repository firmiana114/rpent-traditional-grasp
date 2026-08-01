from __future__ import annotations

import numpy as np
import pytest

from rpent_traditional_grasp.config import PlannerConfig
from rpent_traditional_grasp.models import BottleEstimate
from rpent_traditional_grasp.planning import (
    compute_gripper_tcp_target,
    interpolate_joint_bridge,
    plan_fixed_side_grasp,
    side_grasp_rotation_candidates,
)


def _estimate(xyz: tuple[float, float, float]) -> BottleEstimate:
    return BottleEstimate(
        class_name="bottle",
        confidence=0.9,
        bbox_xyxy=(10, 20, 30, 80),
        center_uv=(20.0, 50.0),
        front_center_camera_m=np.array([0.0, 0.0, 0.5]),
        center_camera_m=np.array([0.0, 0.0, 0.53]),
        center_body_m=np.asarray(xyz, dtype=np.float64),
        axis_camera=np.array([0.0, 1.0, 0.0]),
        diameter_m=0.06,
        front_depth_m=0.5,
        depth_mad_m=0.004,
        valid_depth_pixels=500,
    )


def test_auto_arm_uses_body_lateral_side() -> None:
    config = PlannerConfig()

    left = compute_gripper_tcp_target(
        _estimate((0.5, 0.08, 0.1)),
        requested_arm="auto",
        config=config,
    )
    right = compute_gripper_tcp_target(
        _estimate((0.5, -0.08, 0.1)),
        requested_arm="auto",
        config=config,
    )

    assert left.arm == "left"
    assert right.arm == "right"
    np.testing.assert_allclose(left.tcp_body_xyz_m, [0.5, 0.08, 0.1])


def test_final_grasp_path_preserves_initial_tcp_rotation() -> None:
    rotation = np.array(
        [
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )

    waypoints = plan_fixed_side_grasp(
        _estimate((0.5, 0.08, 0.1)),
        "left",
        PlannerConfig(),
        tcp_rotation=rotation,
    )

    for waypoint in waypoints:
        np.testing.assert_allclose(waypoint.pose.rotation, rotation)
    np.testing.assert_allclose(waypoints[1].pose.position_m, [0.5, 0.08, 0.1])


def test_gripper_target_rejects_distance_outside_gate() -> None:
    with pytest.raises(ValueError, match="超出最大距离"):
        compute_gripper_tcp_target(
            _estimate((0.9, 0.0, 0.0)),
            requested_arm="auto",
            config=PlannerConfig(max_reach_m=0.78),
        )


def test_side_grasp_candidates_are_bounded_and_include_pitch_twenty() -> None:
    config = PlannerConfig()

    candidates = side_grasp_rotation_candidates(np.eye(3), config)

    assert candidates[0].name == "initial"
    names = {candidate.name for candidate in candidates}
    assert "pitch_+20deg" in names
    assert "pitch_+20deg_yaw_-10deg" in names
    assert max(
        np.degrees(candidate.angular_offset_rad) for candidate in candidates
    ) <= config.max_side_grasp_tilt_degrees


def test_pitched_side_grasp_keeps_final_approach_horizontal() -> None:
    config = PlannerConfig()
    rotation = next(
        candidate.rotation
        for candidate in side_grasp_rotation_candidates(np.eye(3), config)
        if candidate.name == "pitch_+20deg"
    )

    waypoints = plan_fixed_side_grasp(
        _estimate((0.48, 0.08, 0.05)),
        "left",
        config,
        tcp_rotation=rotation,
    )

    pregrasp_delta = waypoints[1].pose.position_m - waypoints[0].pose.position_m
    assert pregrasp_delta[2] == pytest.approx(0.0)
    assert np.linalg.norm(pregrasp_delta) == pytest.approx(
        config.pregrasp_offset_m
    )


def test_joint_bridge_respects_maximum_joint_step() -> None:
    bridge = interpolate_joint_bridge(
        np.zeros(7),
        np.array([0.2, -0.1, 0.0, 0.16, 0.0, 0.0, 0.0]),
        max_joint_step_rad=0.08,
    )

    previous = np.zeros(7)
    for position in bridge:
        assert np.max(np.abs(position - previous)) <= 0.08 + 1e-12
        previous = position
    np.testing.assert_allclose(bridge[-1], [0.2, -0.1, 0.0, 0.16, 0, 0, 0])
