from __future__ import annotations

import inspect
from pathlib import Path

import numpy as np
import pytest

from rpent_traditional_grasp.api import TraditionalGraspAPI
from rpent_traditional_grasp.config import (
    PlannerConfig,
    SafetyConfig,
    TraditionalGraspConfig,
)
from rpent_traditional_grasp.execution import (
    AlwaysSafeCollisionChecker,
    MockArmExecutor,
)
from rpent_traditional_grasp.ik import MockIKSolver, TracIKProcess
from rpent_traditional_grasp.models import Detection, StereoObservation
from rpent_traditional_grasp.perception import StaticDetector, StaticSegmenter
from rpent_traditional_grasp.stereo import StaticStereoSource


def make_api() -> TraditionalGraspAPI:
    height, width = 240, 320
    image = np.zeros((height, width, 3), dtype=np.uint8)
    mask = np.zeros((height, width), dtype=bool)
    mask[50:210, 140:180] = True
    depth = np.full((height, width), np.nan, dtype=np.float32)
    depth[mask] = 0.6
    projection = np.array(
        [[600.0, 0, 160.0, 0], [0, 600.0, 120.0, 0], [0, 0, 1, 0]]
    )
    observation = StereoObservation(
        left=image,
        right=image.copy(),
        depth_m=depth,
        projection_left=projection,
    )
    identity_rotation = tuple(np.eye(3).reshape(-1).tolist())
    config = TraditionalGraspConfig(
        planner=PlannerConfig(
            left_tcp_rotation=identity_rotation,
            right_tcp_rotation=identity_rotation,
        ),
        safety=SafetyConfig(
            mode="offline",
            collision_check_required=True,
        ),
    )
    return TraditionalGraspAPI(
        config=config,
        stereo_source=StaticStereoSource(observation),
        detector=StaticDetector(
            [Detection("bottle", 0.92, (140, 50, 180, 210))]
        ),
        segmenter=StaticSegmenter(mask),
        ik_solvers={
            "left": MockIKSolver("left"),
            "right": MockIKSolver("right"),
        },
        executor=MockArmExecutor(contact_detected=True),
        camera_to_body=np.eye(4),
        collision_checker=AlwaysSafeCollisionChecker(),
    )


def test_original_api_names_complete_simulated_pick() -> None:
    api = make_api()

    search = api.search_object("bottle")
    approach = api.approach_object("bottle")
    pick = api.pick_object(object_prompt="bottle")
    verify = api.verify_grasp()

    assert search["found"] is True
    assert approach["approached"] is True
    assert pick["success"] is True
    assert pick["status"] == "executed"
    assert pick["verification"] == "gripper_contact"
    assert pick["execution"]["simulated"] is True
    assert pick["execution"]["grasp_verified"] is True
    assert pick["execution"]["lift_completed"] is True
    assert pick["execution"]["max_joint_step_rad"] >= 0.0
    assert {
        "success",
        "action",
        "object_prompt",
        "requested_arm_side",
        "selected_arm_side",
        "status",
        "verification",
        "execution",
        "bbox",
        "bbox_image_path",
        "depth_image_path",
        "raw_depth_image_path",
        "result_image_path",
        "backend",
    } <= pick.keys()
    assert verify["verified"] is True


def test_search_object_forwards_the_segmentation_artifact_paths() -> None:
    """Only the segmenter knows where it wrote the mask images.

    The bridge copies these keys verbatim into the tool result, so a segmenter
    without them must still produce a valid search result rather than raise.
    """
    api = make_api()
    assert not hasattr(api.segmenter, "last_artifacts")

    without_artifacts = api.search_object("bottle")

    assert without_artifacts["found"] is True
    assert "overlay_image_path" not in without_artifacts

    api.segmenter.last_artifacts = {  # type: ignore[attr-defined]
        "bbox_image_path": "/run/bbox.png",
        "mask_image_path": "/run/mask.png",
        "overlay_image_path": "/run/overlay.png",
        "result_image_path": "/run/overlay.png",
    }

    with_artifacts = api.search_object("bottle")

    assert with_artifacts["overlay_image_path"] == "/run/overlay.png"
    assert with_artifacts["result_image_path"] == "/run/overlay.png"
    assert with_artifacts["position_body_m"] == without_artifacts["position_body_m"]

    api.segmenter.last_artifacts = "not a dict"  # type: ignore[attr-defined]

    assert api.search_object("bottle")["found"] is True


def test_nominal_gripper_spec_cannot_be_claimed_as_robot_validated() -> None:
    api = make_api()
    api.close()
    config = api.config
    config.safety.gripper_tcp_calibration_validated = True

    with pytest.raises(ValueError, match="夹爪规格状态冲突"):
        TraditionalGraspAPI(
            config=config,
            stereo_source=api.stereo_source,
            detector=api.detector,
            segmenter=api.segmenter,
            ik_solvers={
                "left": MockIKSolver("left"),
                "right": MockIKSolver("right"),
            },
            executor=MockArmExecutor(contact_detected=True),
            camera_to_body=np.eye(4),
            collision_checker=AlwaysSafeCollisionChecker(),
        )


def test_no_detection_fails_without_execution() -> None:
    api = make_api()
    api.detector = StaticDetector([])

    result = api.pick_object(object_prompt="missing bottle")

    assert result["success"] is False
    assert result["status"] == "detect_failed"
    assert result["detector_result"]["reason"] == "not_detected"
    assert api.verify_grasp()["verified"] is False


def test_pick_object_xyz_stage_stops_before_ik_and_motion() -> None:
    api = make_api()

    result = api.preview_pick_object_xyz(
        object_prompt="bottle",
        arm_side="auto",
    )

    assert result["success"] is True
    assert result["action"] == "pick_object"
    assert result["phase"] == "gripper_xyz"
    assert result["status"] == "xyz_ready"
    assert result["selected_arm_side"] == "left"
    assert len(result["final_tcp_body_xyz_m"]) == 3
    assert result["orientation_policy"] == "bounded_side_grasp_candidates"
    assert api.context.ik_path is None
    assert api.context.execution is None


def test_plan_pick_object_serializes_path_without_execution() -> None:
    api = make_api()

    result = api.plan_pick_object(
        object_prompt="bottle",
        arm_side="left",
    )

    assert result["success"] is True
    assert result["status"] == "planned"
    assert result["motion_commanded"] is False
    assert result["selected_arm_side"] == "left"
    assert len(result["plan"]["positions_rad"]) > 0
    assert result["plan"]["waypoint_names"].count("grasp") == 1
    assert api.context.execution is None
    assert api.executor.executed_paths == []


def test_latest_field_target_selects_bounded_side_grasp_candidate() -> None:
    root = Path(__file__).parents[2]
    binary = root / "native/build/g1_trac_ik"
    if not binary.exists():
        pytest.skip("standalone TRAC-IK binary is not built")
    api = make_api()
    api.config.planner.ik_timeout_s = 0.05
    estimate = api.context.estimate
    if estimate is None:
        api.search_object("bottle")
        estimate = api.context.estimate
    assert estimate is not None
    estimate.center_body_m = np.array(
        [0.4818, 0.07389839906243502, 0.04838482502786132]
    )

    with TracIKProcess(
        binary,
        root / "robot/chains/g1_left_arm.chain",
        "left",
        timeout_s=0.05,
    ) as solver:
        api.ik_solvers["left"] = solver
        candidates = api.plan_contact_grasp_candidates(
            estimate,
            "left",
            current_joints=np.zeros(7),
        )

    names = [metadata["orientation_candidate"] for _, metadata in candidates]
    assert "initial" not in names
    assert "pitch_+20deg" in names
    assert "pitch_+20deg_yaw_-10deg" in names
    selected, metadata = candidates[0]
    assert metadata["orientation_policy"] == "bounded_side_grasp_candidates"
    assert selected.waypoint_names.count("pregrasp") == 1
    assert selected.waypoint_names.count("grasp") == 1


def test_pick_object_public_signature_matches_rpent() -> None:
    signature = inspect.signature(TraditionalGraspAPI.pick_object)

    assert list(signature.parameters) == [
        "self",
        "object_prompt",
        "arm_side",
        "bbox",
        "bbox_format",
    ]
    assert signature.parameters["object_prompt"].kind is inspect.Parameter.KEYWORD_ONLY


def test_pick_object_converts_internal_error_to_rpent_failure_result() -> None:
    result = make_api().pick_object(
        object_prompt="bottle",
        arm_side="invalid",
    )

    assert result["success"] is False
    assert result["action"] == "pick_object"
    assert result["status"] == "execution_failed"
    assert result["requested_arm_side"] == "invalid"
    assert "ValueError" in result["error"]


def test_thor_compatible_keywords_and_bbox() -> None:
    api = make_api()

    search = api.search_object(
        object_prompt="bottle",
        bbox=[140, 50, 180, 210],
        bbox_format="pixel",
    )
    pick = api.pick_object(
        object_prompt="bottle",
        arm_side="left",
        bbox=[140 / 320, 50 / 240, 180 / 320, 210 / 240],
        bbox_format="norm01",
    )
    verify = api.verify_grasp(target_prompt="bottle", arm_side="left")

    assert search["success"] is True
    assert search["visible"] is True
    assert search["bbox"] == (140, 50, 180, 210)
    assert pick["success"] is True
    assert pick["selected_arm_side"] == "left"
    assert verify["success"] is True
    assert verify["target_prompt"] == "bottle"
