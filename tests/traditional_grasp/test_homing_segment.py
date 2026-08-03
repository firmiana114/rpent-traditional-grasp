"""The post-grasp homing segment rides the grasp plan, and never costs a grasp.

Putting the retract into the planned path is what earns it the collision check,
the loaded settle threshold and the single continuous trajectory the lift
segment already gets. The price would be losing an otherwise fine grasp when the
homing target is awkward, so homing degrades to absent instead of to no plan.
"""

from __future__ import annotations

import numpy as np
import pytest

from rpent_traditional_grasp.planning import plan_homing_pose
from tests.traditional_grasp.test_api import make_api


def test_homing_target_is_mirrored_per_arm_and_uses_the_ready_orientation() -> None:
    left = plan_homing_pose((0.30, 0.12, 0.14865), "left")
    right = plan_homing_pose((0.30, 0.12, 0.14865), "right")

    np.testing.assert_allclose(left.position_m, [0.30, 0.14865, 0.12])
    np.testing.assert_allclose(right.position_m, [0.30, -0.14865, 0.12])
    # 位置与朝向是一个整体。只取 ready 的位置而留着抓取姿态。
    # 结果是肩肘收回而腕部拧到约 60 度去维持机身系里的固定朝向。
    # 2026-08-03 实测的畸形姿态即此。
    np.testing.assert_allclose(left.rotation, np.eye(3))
    np.testing.assert_allclose(right.rotation, np.eye(3))


def test_negative_abs_y_is_still_mirrored_outward() -> None:
    left = plan_homing_pose((0.30, 0.12, -0.14865), "left")
    assert left.position_m[1] > 0.0


@pytest.mark.parametrize(
    "home,arm",
    [
        ((float("nan"), 0.12, 0.14), "left"),
        ((0.30, float("inf"), 0.14), "left"),
        ((0.30, 0.12, 0.14), "both"),
    ],
)
def test_invalid_homing_inputs_are_rejected(home, arm) -> None:
    with pytest.raises(ValueError):
        plan_homing_pose(home, arm)


def test_plan_without_homing_is_unchanged() -> None:
    api = make_api()

    plain = api.plan_pick_object(object_prompt="bottle")

    assert plain["success"] is True
    assert "home" not in plain["plan"]["waypoint_names"]


def test_homing_extends_the_path_and_ends_at_the_home_pose() -> None:
    api = make_api()

    homed = api.plan_pick_object(
        object_prompt="bottle", home_xz_and_abs_y_m=(0.30, 0.12, 0.14865)
    )

    assert homed["success"] is True
    names = homed["plan"]["waypoint_names"]
    assert names[-1] == "home", names[-3:]
    # 归位段必须整体排在抓取点之后 - 否则父项目会用空载阈值驱动它
    assert names.index("grasp") < names.index("home")
    plain = make_api().plan_pick_object(object_prompt="bottle")
    assert len(names) > len(plain["plan"]["waypoint_names"])


def test_unsolvable_home_costs_the_segment_not_the_grasp(monkeypatch) -> None:
    """An awkward retract must never discard a candidate that grasps fine."""
    import rpent_traditional_grasp.api as api_module

    def _boom(*args, **kwargs):
        raise RuntimeError("归位目标不可达")

    monkeypatch.setattr(api_module, "plan_homing_pose", _boom)
    api = make_api()

    homed = api.plan_pick_object(
        object_prompt="bottle", home_xz_and_abs_y_m=(0.30, 0.12, 0.14865)
    )

    assert homed["success"] is True, "归位不可达不得否掉抓取"
    assert "home" not in homed["plan"]["waypoint_names"]


def test_service_drops_a_malformed_home_pose_instead_of_failing() -> None:
    """A typo in the request must not cost the caller its grasp."""
    from rpent_traditional_grasp.service import _parse_home_pose

    assert _parse_home_pose(None) is None
    assert _parse_home_pose("0.3,0.12,0.15") is None
    assert _parse_home_pose([0.30, 0.12]) is None
    assert _parse_home_pose([float("nan"), 0.12, 0.15]) is None
    assert _parse_home_pose([0.30, 0.12, 0.14865]) == (0.30, 0.12, 0.14865)


def test_homing_does_not_change_candidate_ranking() -> None:
    """Homing is the same fixed retract for every candidate."""
    plain = make_api().plan_pick_object(object_prompt="bottle")
    homed = make_api().plan_pick_object(
        object_prompt="bottle", home_xz_and_abs_y_m=(0.30, 0.12, 0.14865)
    )

    assert homed["plan"]["orientation_candidate"] == plain["plan"]["orientation_candidate"]
    assert homed["plan"]["score"] == pytest.approx(plain["plan"]["score"])


def test_homing_lands_on_the_neutral_seed_solution_not_the_retreat_branch() -> None:
    """The seed picks the redundancy branch, and retreat's branch is the folded one.

    A 7-axis arm has a null space, so the same ready pose has many joint
    solutions. Seeding the homing solve from retreat kept the arm in retreat's
    branch: shoulder and elbow folded in while the wrist cranked to ~60 deg to
    hold the orientation. Solving from the neutral seed is what reproduces the
    posture ``ready_arm`` reaches.
    """
    api = make_api()
    solver = api.ik_solvers["left"]
    home_pose = plan_homing_pose((0.30, 0.12, 0.14865), "left")
    expected = solver.solve(np.zeros(7), home_pose)

    homed = api.plan_pick_object(
        object_prompt="bottle",
        arm_side="left",
        home_xz_and_abs_y_m=(0.30, 0.12, 0.14865),
    )

    final = np.asarray(homed["plan"]["positions_rad"][-1], dtype=np.float64)
    np.testing.assert_allclose(final, expected, atol=1e-9)


def test_homing_is_a_joint_bridge_with_bounded_steps() -> None:
    """Joint-space keeps every step inside the configured bridge limit."""
    api = make_api()
    step_limit = api.config.planner.joint_bridge_step_rad

    homed = api.plan_pick_object(
        object_prompt="bottle",
        arm_side="left",
        home_xz_and_abs_y_m=(0.30, 0.12, 0.14865),
    )

    names = homed["plan"]["waypoint_names"]
    positions = [np.asarray(q, dtype=np.float64) for q in homed["plan"]["positions_rad"]]
    start = names.index("retreat")
    steps = [
        float(np.max(np.abs(positions[i + 1] - positions[i])))
        for i in range(start, len(positions) - 1)
    ]
    assert steps, "归位段必须至少有一个路点"
    assert max(steps) <= step_limit + 1e-9, f"最大单步 {max(steps):.4f} 超过 {step_limit}"
