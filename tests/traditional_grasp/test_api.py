from __future__ import annotations

import numpy as np

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
from rpent_traditional_grasp.ik import MockIKSolver
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
            max_joint_step_rad=0.05,
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
    pick = api.pick_object("bottle")
    verify = api.verify_grasp()

    assert search["found"] is True
    assert approach["approached"] is True
    assert pick["picked"] is True
    assert pick["simulated"] is True
    assert pick["max_joint_step_rad"] <= 0.05
    assert verify["verified"] is True


def test_no_detection_fails_without_execution() -> None:
    api = make_api()
    api.detector = StaticDetector([])

    result = api.pick_object("missing bottle")

    assert result["picked"] is False
    assert result["reason"] == "not_detected"
    assert api.verify_grasp()["verified"] is False


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
