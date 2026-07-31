from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from rpent_traditional_grasp.config import PlannerConfig
from rpent_traditional_grasp.ik import (
    MockIKSolver,
    TracIKProcess,
    solve_continuous_path,
)
from rpent_traditional_grasp.models import (
    BottleEstimate,
    CartesianWaypoint,
    Pose,
)
from rpent_traditional_grasp.planning import (
    interpolate_waypoints,
    plan_fixed_side_grasp,
)


@pytest.mark.parametrize("arm", ["left", "right"])
def test_standalone_trac_ik_round_trip(arm: str) -> None:
    root = Path(__file__).parents[2]
    binary = root / "native/build/g1_trac_ik"
    chain = root / f"robot/chains/g1_{arm}_arm.chain"
    if not binary.exists():
        pytest.skip("standalone TRAC-IK binary is only present in Linux build")
    with TracIKProcess(binary, chain, arm, timeout_s=0.1) as solver:
        seed = np.zeros(7)
        target = solver.forward(
            np.array([0.2, -0.15, 0.1, 0.45, -0.1, 0.2, 0.05])
        )
        solution = solver.solve(seed, target)
        actual = solver.forward(solution)

    assert np.linalg.norm(actual.position_m - target.position_m) < 1e-4


@pytest.mark.parametrize(
    ("arm", "target"),
    [
        ("left", np.array([0.28, 0.20, 0.05])),
        ("right", np.array([0.28, -0.20, 0.05])),
    ],
)
def test_g1_fixed_side_path_is_continuously_reachable(
    arm: str, target: np.ndarray
) -> None:
    root = Path(__file__).parents[2]
    binary = root / "native/build/g1_trac_ik"
    chain = root / f"robot/chains/g1_{arm}_arm.chain"
    if not binary.exists():
        pytest.skip("standalone TRAC-IK binary is only present in Linux build")
    config = PlannerConfig(
        ik_timeout_s=0.1,
        fk_position_tolerance_m=1e-4,
        fk_rotation_tolerance_rad=1e-3,
    )
    estimate = BottleEstimate(
        class_name="bottle",
        confidence=1.0,
        bbox_xyxy=(0, 0, 1, 1),
        center_uv=(0.0, 0.0),
        front_center_camera_m=np.zeros(3),
        center_camera_m=np.zeros(3),
        center_body_m=target,
        axis_camera=np.array([0.0, 1.0, 0.0]),
        diameter_m=0.06,
        front_depth_m=0.6,
        depth_mad_m=0.0,
        valid_depth_pixels=100,
    )
    with TracIKProcess(binary, chain, arm, timeout_s=0.1) as solver:
        seed = np.zeros(7)
        dense = interpolate_waypoints(
            solver.forward(seed),
            plan_fixed_side_grasp(estimate, arm, config),
            config.cartesian_step_m,
            config.rotation_step_rad,
        )
        path = solve_continuous_path(arm, solver, seed, dense, config)

    assert np.isfinite(path.max_joint_step_rad)
    assert path.waypoint_names.count("grasp") == 1


def test_continuous_path_records_large_joint_step_without_rejecting() -> None:
    solver = MockIKSolver("left")
    target = Pose(
        position_m=np.array([0.4, 0.0, 0.0]),
        rotation=np.eye(3),
    )

    path = solve_continuous_path(
        "left",
        solver,
        np.zeros(7),
        [CartesianWaypoint("large-step", target)],
        PlannerConfig(),
    )

    assert path.max_joint_step_rad == pytest.approx(0.4)
    assert path.waypoint_names == ["large-step"]


@pytest.mark.parametrize("arm", ["left", "right"])
def test_standalone_trac_ik_position_only_diagnostic(arm: str) -> None:
    root = Path(__file__).parents[2]
    binary = root / "native/build/g1_trac_ik"
    chain = root / f"robot/chains/g1_{arm}_arm.chain"
    if not binary.exists():
        pytest.skip("standalone TRAC-IK binary is only present in Linux build")
    expected_joints = np.array([0.2, -0.15, 0.1, 0.45, -0.1, 0.2, 0.05])
    with TracIKProcess(binary, chain, arm, timeout_s=0.1) as solver:
        reachable = solver.forward(expected_joints)
        target = Pose(reachable.position_m, np.eye(3))
        solution = solver.solve_position_only(np.zeros(7), target)
        actual = solver.forward(solution)

    assert np.linalg.norm(actual.position_m - target.position_m) < 1e-4
