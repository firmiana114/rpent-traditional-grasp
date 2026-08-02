"""Persistent no-motion planning service used by the RPent Wuxi bridge."""

from __future__ import annotations

import hashlib
import json
import math
import time
from collections.abc import Callable
from typing import Any, Protocol

import numpy as np

from rpent_traditional_grasp.execution import PlanningArmExecutor
from rpent_traditional_grasp.logging import get_logger

logger = get_logger("service")


def _bounded_repr(value: object, limit: int = 256) -> str:
    rendered = repr(value)
    return rendered if len(rendered) <= limit else rendered[: limit - 3] + "..."


def _format_joint_state(joints: np.ndarray) -> str:
    """Render the seed joint vector compactly enough to keep in every request."""
    values = np.asarray(joints, dtype=np.float64).ravel()
    if values.size == 0 or not np.all(np.isfinite(values)):
        return "invalid"
    return "[" + ",".join(f"{value:.6f}" for value in values) + "]"


class PlanningAPI(Protocol):
    """Subset of the grasp API required by the service transport."""

    def plan_pick_object(
        self,
        *,
        object_prompt: str,
        arm_side: str = "auto",
        bbox: object = None,
        bbox_format: str = "auto",
    ) -> dict[str, object]:
        """Return a serialized, no-motion pick plan."""

    def close(self) -> None:
        """Release native planning resources."""


class TraditionalGraspPlanningService:
    """Validate requests and attach replay-resistant plan metadata."""

    def __init__(
        self,
        api: PlanningAPI,
        executor: PlanningArmExecutor,
        *,
        code_revision: str,
        config_sha256: str,
        plan_ttl_s: float = 15.0,
        clock: Callable[[], float] = time.time,
    ) -> None:
        if plan_ttl_s <= 0.0:
            raise ValueError("plan_ttl_s 必须大于 0")
        self.api = api
        self.executor = executor
        self.code_revision = code_revision
        self.config_sha256 = config_sha256
        self.plan_ttl_s = float(plan_ttl_s)
        self.clock = clock

    def handle(self, request: dict[str, Any]) -> dict[str, object]:
        operation = request.get("operation")
        if operation == "ping":
            return {
                "success": True,
                "ready": True,
                "service": "traditional_grasp_planning",
                "code_revision": self.code_revision,
                "motion_commanded": False,
            }
        if operation == "close":
            self.close()
            return {"success": True, "closed": True, "motion_commanded": False}
        if operation != "plan_pick":
            raise ValueError(f"不支持的规划服务操作: {operation}")

        payload = request.get("payload")
        if not isinstance(payload, dict):
            raise ValueError("plan_pick payload 必须是对象")
        current_q = np.asarray(payload.get("current_q"), dtype=np.float64)
        self.executor.update_joint_state(current_q)
        state_timestamp_s = float(payload.get("state_timestamp_s", 0.0))
        if not math.isfinite(state_timestamp_s) or state_timestamp_s < 0.0:
            raise ValueError("state_timestamp_s 必须是非负有限数值")
        object_prompt = str(payload.get("object_prompt", "")).strip()
        if not object_prompt:
            raise ValueError("object_prompt 不能为空")
        arm_side = str(payload.get("arm_side", "auto"))
        bbox = payload.get("bbox")
        bbox_format = str(payload.get("bbox_format", "auto"))
        logger.info(
            "收到抓取规划请求: target=%s arm=%s bbox=%s bbox_format=%s "
            "state_timestamp_s=%.6f current_q_shape=%s current_q_rad=%s",
            object_prompt,
            arm_side,
            _bounded_repr(bbox),
            bbox_format,
            state_timestamp_s,
            current_q.shape,
            # The seed is the only record of where the arm actually was. Its
            # forward kinematics is an exactly known 3D point inside the camera
            # view, which is what calibration validation needs. A failed IK
            # solve produces no trajectory at all, so nothing else preserves it.
            _format_joint_state(current_q),
        )
        result = self.api.plan_pick_object(
            object_prompt=object_prompt,
            arm_side=arm_side,
            bbox=bbox,
            bbox_format=bbox_format,
        )
        result = dict(result)
        result.update(
            {
                "schema_version": 1,
                "motion_commanded": False,
                "code_revision": self.code_revision,
                "config_sha256": self.config_sha256,
                "state_timestamp_s": state_timestamp_s,
            }
        )
        if not result.get("success"):
            logger.info(
                "抓取规划请求未生成路径: target=%s arm=%s status=%s",
                object_prompt,
                arm_side,
                result.get("status"),
            )
            return result

        plan = result.get("plan")
        if not isinstance(plan, dict):
            raise RuntimeError("规划成功但缺少 plan 对象")
        created_at_s = float(self.clock())
        raw_candidates = result.get("candidate_plans")
        if not isinstance(raw_candidates, list) or not raw_candidates:
            raw_candidates = [plan]
        signed_candidates: list[dict[str, object]] = []
        for index, candidate in enumerate(raw_candidates):
            if not isinstance(candidate, dict):
                raise RuntimeError(
                    f"candidate_plans[{index}] 不是有效计划对象"
                )
            signed = dict(candidate)
            signed.update(
                {
                    "schema_version": 1,
                    "object_prompt": object_prompt,
                    "requested_arm_side": arm_side,
                    "target_bbox": result.get("bbox"),
                    "created_at_s": created_at_s,
                    "expires_at_s": created_at_s + self.plan_ttl_s,
                    "state_timestamp_s": state_timestamp_s,
                    "capture_timestamp_s": result.get(
                        "capture_timestamp_s", 0.0
                    ),
                    "code_revision": self.code_revision,
                    "config_sha256": self.config_sha256,
                    "collision_checked": bool(
                        candidate.get(
                            "collision_checked",
                            result.get("collision_checked", False),
                        )
                    ),
                    "collision_check_detail": candidate.get(
                        "collision_check_detail",
                        result.get("collision_check_detail"),
                    ),
                }
            )
            signed["plan_id"] = _plan_digest(signed)
            signed_candidates.append(signed)
        plan = signed_candidates[0]
        result["plan"] = plan
        result["candidate_plans"] = signed_candidates
        result["plan_id"] = plan["plan_id"]
        logger.info(
            "可回溯抓取计划已生成: plan_id=%s target=%s arm=%s "
            "candidate=%s ranked_candidates=%d points=%d expires_at=%.3f "
            "collision_checked=%s motion=False",
            plan["plan_id"],
            object_prompt,
            plan.get("arm_side"),
            plan.get("orientation_candidate"),
            len(signed_candidates),
            len(plan.get("positions_rad", [])),
            plan["expires_at_s"],
            plan["collision_checked"],
        )
        return result

    def close(self) -> None:
        self.api.close()


def _plan_digest(plan: dict[str, object]) -> str:
    rendered = json.dumps(
        plan,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(rendered).hexdigest()
