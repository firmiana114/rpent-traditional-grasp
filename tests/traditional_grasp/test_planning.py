from __future__ import annotations

import numpy as np
import pytest

from rpent_traditional_grasp.config import PlannerConfig
from rpent_traditional_grasp.models import BottleEstimate
from rpent_traditional_grasp.planning import (
    compute_gripper_tcp_target,
    plan_fixed_side_grasp,
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
