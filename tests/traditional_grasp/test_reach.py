from __future__ import annotations

import logging
from pathlib import Path

import numpy as np
import pytest

from rpent_traditional_grasp.diagnostics import (
    assess_arm_reach,
    load_arm_chain_geometry,
    required_base_advance_m,
)

PROJECT_ROOT = Path(__file__).parents[2]
# Bottle center measured on Thor from the cached desktop stereo pair.
CACHED_SCENE_BOTTLE_M = np.array([0.6037144558, -0.0319783264, -0.0564951343])


def _geometries() -> dict[str, object]:
    return {
        arm: load_arm_chain_geometry(
            PROJECT_ROOT / f"robot/chains/g1_{arm}_arm.chain", arm
        )
        for arm in ("left", "right")
    }


def test_cached_scene_bottle_is_proven_out_of_reach() -> None:
    assessments = assess_arm_reach(
        CACHED_SCENE_BOTTLE_M,
        _geometries(),
        planning_radius_m=0.54,
    )

    for arm, assessment in assessments.items():
        assert not assessment.within_serial_length_upper_bound, arm
        assert not assessment.within_planning_radius, arm
        assert assessment.margin_m < 0.0
    assert assessments["right"].shoulder_distance_m == pytest.approx(0.675980, abs=1e-5)
    assert assessments["left"].shoulder_distance_m == pytest.approx(0.685397, abs=1e-5)


def test_required_base_advance_puts_the_target_on_the_planning_radius() -> None:
    geometry = _geometries()["right"]
    radius = 0.54

    advance = required_base_advance_m(CACHED_SCENE_BOTTLE_M, geometry, radius)

    assert advance == pytest.approx(0.158894, abs=1e-5)
    moved = CACHED_SCENE_BOTTLE_M - np.array([advance, 0.0, 0.0])
    distance = float(np.linalg.norm(moved - geometry.shoulder_body_xyz_m))
    assert distance == pytest.approx(radius, abs=1e-9)


def test_target_already_inside_the_radius_needs_no_advance() -> None:
    geometry = _geometries()["right"]
    inside = geometry.shoulder_body_xyz_m + np.array([0.30, 0.0, 0.0])

    assert required_base_advance_m(inside, geometry, 0.54) == 0.0


def test_laterally_unreachable_target_cannot_be_fixed_by_driving_forward() -> None:
    geometry = _geometries()["right"]
    # Offset the target sideways by more than the radius; no forward travel
    # along body +x can ever shrink that component.
    far_side = geometry.shoulder_body_xyz_m + np.array([0.40, 0.90, 0.0])

    assert required_base_advance_m(far_side, geometry, 0.54) is None


def test_assess_arm_reach_rejects_invalid_input() -> None:
    geometries = _geometries()

    with pytest.raises(ValueError, match="三个有限数值"):
        assess_arm_reach(np.array([0.5, 0.0]), geometries, planning_radius_m=0.54)
    with pytest.raises(ValueError, match="必须为正"):
        assess_arm_reach(
            CACHED_SCENE_BOTTLE_M, geometries, planning_radius_m=0.0
        )


def _far_target_api():
    """Reuse the offline API fixture but force the cached-scene bottle center."""
    from rpent_traditional_grasp.models import (
        BottleEstimate,
        Detection,
        PipelineContext,
    )
    from tests.traditional_grasp.test_api import make_api

    api = make_api()
    api.arm_geometries = _geometries()
    detection = Detection("bottle", 0.92, (140, 50, 180, 210))
    estimate = BottleEstimate(
        class_name="bottle",
        confidence=0.92,
        bbox_xyxy=detection.bbox_xyxy,
        center_uv=(160.0, 130.0),
        front_center_camera_m=CACHED_SCENE_BOTTLE_M.copy(),
        center_camera_m=CACHED_SCENE_BOTTLE_M.copy(),
        center_body_m=CACHED_SCENE_BOTTLE_M.copy(),
        axis_camera=np.array([0.0, 1.0, 0.0]),
        diameter_m=0.0644,
        front_depth_m=0.681,
        depth_mad_m=0.0099,
        valid_depth_pixels=1161,
    )

    def _search(*_args, **_kwargs):
        api.context = PipelineContext(detection=detection, estimate=estimate)
        return {"found": True, "success": True, "bbox": detection.bbox_xyxy}

    api.search_object = _search
    return api


def test_plan_rejects_an_out_of_reach_target_before_solving_ik(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    api = _far_target_api()
    # The bound is exact but is applied to a perceived target, so the refusal
    # only stands once both calibrations placing that target are accepted.
    api.config.safety.stereo_calibration_validated = True
    api.config.safety.camera_to_body_validated = True
    solved: list[str] = []
    for arm, solver in api.ik_solvers.items():
        original = solver.solve

        def guard(seed, target, _arm=arm, _original=original):
            solved.append(_arm)
            return _original(seed, target)

        monkeypatch.setattr(solver, "solve", guard)

    plan = api.plan_pick_object(object_prompt="bottle")

    assert plan["success"] is False
    assert plan["status"] == "unreachable"
    assert "advance the base by 0.159 m" in plan["error"]
    assert plan["reach"]["required_base_advance_m"] == pytest.approx(0.158894, abs=1e-5)
    assert plan["reach"]["any_arm_within_serial_length_upper_bound"] is False
    # The verdict is geometric, so no IK solve is spent on a hopeless target.
    assert solved == []


def test_approach_reports_the_advance_that_the_body_origin_gate_hides() -> None:
    api = _far_target_api()
    # The body-origin distance is 0.606 m, inside the coarse 0.78 m gate, so the
    # previous check reported this target as reachable.
    assert float(np.linalg.norm(CACHED_SCENE_BOTTLE_M)) < api.config.planner.max_reach_m

    approach = api.approach_object("bottle")

    assert approach["approached"] is False
    assert approach["reason"] == "base_motion_not_authorized"
    assert approach["required_base_advance_m"] == pytest.approx(0.158894, abs=1e-5)


def test_reach_precheck_is_skipped_without_chain_geometry() -> None:
    api = _far_target_api()
    api.arm_geometries = {}

    assert api.assess_target_reach(CACHED_SCENE_BOTTLE_M) is None
    approach = api.approach_object("bottle")
    # Falls back to the coarse body-origin gate, preserving the old behaviour.
    assert approach["approached"] is True
    assert approach["reach"] is None


def test_unvalidated_calibration_keeps_solving_a_target_beyond_the_bound(
    caplog: pytest.LogCaptureFixture,
) -> None:
    api = _far_target_api()
    assert api.config.safety.stereo_calibration_validated is False
    assert api.config.safety.camera_to_body_validated is False

    with caplog.at_level(logging.WARNING, logger="rpent_traditional_grasp.api"):
        plan = api.plan_pick_object(object_prompt="bottle")

    # Teleoperation has grasped a bottle this pipeline called out of reach, so an
    # unvalidated calibration must not veto the solve before it is attempted.
    assert plan["status"] != "unreachable"
    assert "标定尚未验收，继续尝试求解" in caplog.text
