from __future__ import annotations

import numpy as np

from rpent_traditional_grasp.execution import PlanningArmExecutor
from rpent_traditional_grasp.service import TraditionalGraspPlanningService


class FakePlanningAPI:
    def __init__(self) -> None:
        self.closed = False

    def plan_pick_object(self, **_: object) -> dict[str, object]:
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


def test_planning_service_attaches_trace_metadata_without_motion() -> None:
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

    result = service.handle(
        {
            "operation": "plan_pick",
            "payload": {
                "object_prompt": "water bottle",
                "arm_side": "auto",
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
    np.testing.assert_allclose(executor.current_joints("left"), np.zeros(7))
