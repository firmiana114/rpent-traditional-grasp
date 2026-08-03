from __future__ import annotations

import logging

import numpy as np
import pytest

from rpent_traditional_grasp.execution import PlanningArmExecutor
from rpent_traditional_grasp.service import TraditionalGraspPlanningService


class FakePlanningAPI:
    def __init__(self) -> None:
        self.closed = False
        self.last_kwargs: dict[str, object] | None = None
        self.last_search_kwargs: dict[str, object] | None = None

    def search_object(self, **kwargs: object) -> dict[str, object]:
        self.last_search_kwargs = kwargs
        return {
            "success": True,
            "action": "search_object",
            "found": True,
            "visible": True,
            "bbox": [10, 20, 30, 40],
            "position_body_m": [0.5, 0.08, 0.05],
            "bbox_image_path": "/run/bbox.png",
            "overlay_image_path": "/run/overlay.png",
        }

    def plan_pick_object(self, **kwargs: object) -> dict[str, object]:
        self.last_kwargs = kwargs
        return {
            "success": True,
            "status": "planned",
            "selected_arm_side": "left",
            "capture_timestamp_s": 12.0,
            "collision_checked": False,
            "collision_check_detail": "checker missing",
            "plan": {
                "arm_side": "left",
                "joint_names": [f"joint_{index}" for index in range(7)],
                "seed_q": [0.0] * 7,
                "positions_rad": [[0.1] * 7],
                "waypoint_names": ["grasp"],
                "max_joint_step_rad": 0.1,
                "score": 0.07,
            },
        }

    def close(self) -> None:
        self.closed = True


def test_planning_service_attaches_trace_metadata_without_motion(caplog) -> None:
    api = FakePlanningAPI()
    executor = PlanningArmExecutor()
    service = TraditionalGraspPlanningService(
        api,
        executor,
        code_revision="abc123",
        config_sha256="config123",
        plan_ttl_s=10.0,
        clock=lambda: 100.0,
    )

    with caplog.at_level(logging.INFO, logger="rpent_traditional_grasp"):
        result = service.handle(
            {
                "operation": "plan_pick",
                "payload": {
                    "object_prompt": "water bottle",
                    "arm_side": "auto",
                    "bbox": [10, 20, 30, 40],
                    "bbox_format": "pixel",
                    "current_q": [0.0] * 14,
                    "state_timestamp_s": 90.0,
                },
            }
        )

    assert result["success"] is True
    assert result["motion_commanded"] is False
    assert len(result["plan_id"]) == 64
    assert result["plan"]["plan_id"] == result["plan_id"]
    assert result["plan"]["expires_at_s"] == 110.0
    assert result["plan"]["collision_checked"] is False
    assert result["plan"]["object_prompt"] == "water bottle"
    assert result["plan"]["requested_arm_side"] == "auto"
    assert api.last_kwargs is not None
    assert api.last_kwargs["bbox"] == [10, 20, 30, 40]
    assert api.last_kwargs["bbox_format"] == "pixel"
    assert "bbox=[10, 20, 30, 40] bbox_format=pixel" in "\n".join(caplog.messages)
    np.testing.assert_allclose(executor.current_joints("left"), np.zeros(7))


def test_planning_service_signs_every_ranked_side_grasp_candidate() -> None:
    api = FakePlanningAPI()
    original = api.plan_pick_object

    def plan_with_candidates(**kwargs: object) -> dict[str, object]:
        result = original(**kwargs)
        primary = dict(result["plan"])
        primary["orientation_candidate"] = "pitch_+20deg"
        secondary = dict(primary)
        secondary["orientation_candidate"] = "pitch_+30deg"
        secondary["score"] = 0.08
        result["plan"] = primary
        result["candidate_plans"] = [primary, secondary]
        return result

    api.plan_pick_object = plan_with_candidates  # type: ignore[method-assign]
    service = TraditionalGraspPlanningService(
        api,
        PlanningArmExecutor(),
        code_revision="abc123",
        config_sha256="config123",
        plan_ttl_s=10.0,
        clock=lambda: 100.0,
    )

    result = service.handle(
        {
            "operation": "plan_pick",
            "payload": {
                "object_prompt": "water bottle",
                "arm_side": "left",
                "current_q": [0.0] * 14,
                "state_timestamp_s": 90.0,
            },
        }
    )

    plans = result["candidate_plans"]
    assert len(plans) == 2
    assert plans[0]["plan_id"] != plans[1]["plan_id"]
    assert all(len(plan["plan_id"]) == 64 for plan in plans)
    assert result["plan"] == plans[0]


def test_planning_service_dispatches_search_object_without_motion(caplog) -> None:
    """The parent bridge sends this operation; dropping it breaks localization.

    ``TraditionalGraspBackend.search_object`` posts ``operation=search_object``
    over the same transport as ``plan_pick``. An unhandled operation raises
    ``不支持的规划服务操作`` and the LLM loses every bbox it feeds back into
    ``pick_object``, so the dispatch is a contract between the two repositories.
    """
    api = FakePlanningAPI()
    service = TraditionalGraspPlanningService(
        api,
        PlanningArmExecutor(),
        code_revision="abc123",
        config_sha256="config123",
        plan_ttl_s=10.0,
        clock=lambda: 100.0,
    )

    with caplog.at_level(logging.INFO, logger="rpent_traditional_grasp"):
        result = service.handle(
            {
                "operation": "search_object",
                "payload": {
                    "object_prompt": "water bottle",
                    "bbox": [10, 20, 30, 40],
                    "bbox_format": "pixel",
                },
            }
        )

    assert result["success"] is True
    assert result["motion_commanded"] is False
    assert result["schema_version"] == 1
    assert result["code_revision"] == "abc123"
    assert result["config_sha256"] == "config123"
    # The upper layer reads these paths straight out of the search result.
    assert result["bbox_image_path"] == "/run/bbox.png"
    assert result["overlay_image_path"] == "/run/overlay.png"
    assert api.last_search_kwargs == {
        "object_prompt": "water bottle",
        "bbox": [10, 20, 30, 40],
        "bbox_format": "pixel",
    }
    assert "收到目标搜索请求" in "\n".join(caplog.messages)


def test_search_object_rejects_an_empty_prompt() -> None:
    api = FakePlanningAPI()
    service = TraditionalGraspPlanningService(
        api,
        PlanningArmExecutor(),
        code_revision="abc123",
        config_sha256="config123",
    )

    with pytest.raises(ValueError, match="object_prompt 不能为空"):
        service.handle(
            {"operation": "search_object", "payload": {"object_prompt": "  "}}
        )
    with pytest.raises(ValueError, match="payload 必须是对象"):
        service.handle({"operation": "search_object", "payload": None})
    assert api.last_search_kwargs is None


def test_planning_request_logs_the_seed_joint_values() -> None:
    """The seed is the only record of where the arm was when IK fails."""
    from rpent_traditional_grasp.service import _format_joint_state

    joints = np.linspace(-0.5, 0.5, 14)

    rendered = _format_joint_state(joints)

    assert rendered.startswith("[-0.500000,")
    assert rendered.count(",") == 13
    assert _format_joint_state(np.array([np.nan, 0.0])) == "invalid"
    assert _format_joint_state(np.array([])) == "invalid"
