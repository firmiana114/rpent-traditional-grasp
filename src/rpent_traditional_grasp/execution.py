"""Execution and collision-check interfaces for hardware-safe integration."""

from __future__ import annotations

from itertools import pairwise
from typing import Protocol

import numpy as np

from rpent_traditional_grasp.logging import get_logger
from rpent_traditional_grasp.models import ExecutionEvidence, IKPath

logger = get_logger("execution")


class CollisionChecker(Protocol):
    """Collision validator implemented by the Thor deployment."""

    def check_path(self, path: IKPath) -> tuple[bool, str]:
        """Return whether the entire joint path is collision-free."""


class ArmExecutor(Protocol):
    """Seven-axis arm and gripper executor."""

    @property
    def is_hardware(self) -> bool:
        """Whether commands reach physical hardware."""

    def current_joints(self, arm: str) -> np.ndarray:
        """Return the current seven arm joints."""

    def execute_joint_path(self, path: IKPath) -> None:
        """Execute a continuous joint path without automatic homing."""

    def close_gripper(self, arm: str) -> bool:
        """Close the gripper and report contact."""

    def open_gripper(self, arm: str) -> None:
        """Open the gripper."""


class MockArmExecutor:
    """In-memory executor for local end-to-end validation."""

    def __init__(
        self,
        initial_joints: dict[str, np.ndarray] | None = None,
        contact_detected: bool = True,
    ) -> None:
        zeros = np.zeros(7, dtype=np.float64)
        self._joints = {
            "left": np.asarray(
                (initial_joints or {}).get("left", zeros), dtype=np.float64
            ).copy(),
            "right": np.asarray(
                (initial_joints or {}).get("right", zeros), dtype=np.float64
            ).copy(),
        }
        self.contact_detected = contact_detected
        self.executed_paths: list[IKPath] = []
        self.gripper_closed = {"left": False, "right": False}

    @property
    def is_hardware(self) -> bool:
        return False

    def current_joints(self, arm: str) -> np.ndarray:
        return self._joints[arm].copy()

    def execute_joint_path(self, path: IKPath) -> None:
        self.executed_paths.append(path)
        self._joints[path.arm] = path.positions[-1].copy()
        logger.info(
            "模拟执行七轴路径: arm=%s points=%d",
            path.arm,
            len(path.positions),
        )

    def close_gripper(self, arm: str) -> bool:
        self.gripper_closed[arm] = True
        return self.contact_detected

    def open_gripper(self, arm: str) -> None:
        self.gripper_closed[arm] = False


class PlanningArmExecutor:
    """Mutable joint-state source that rejects every motion command."""

    def __init__(self) -> None:
        self._joints = {
            "left": np.zeros(7, dtype=np.float64),
            "right": np.zeros(7, dtype=np.float64),
        }

    @property
    def is_hardware(self) -> bool:
        return False

    def update_joint_state(self, current_q: np.ndarray) -> None:
        values = np.asarray(current_q, dtype=np.float64)
        if values.shape != (14,) or not np.all(np.isfinite(values)):
            raise ValueError("规划关节状态必须是 14 个有限数值")
        self._joints["left"] = values[:7].copy()
        self._joints["right"] = values[7:].copy()
        logger.info("规划关节状态已更新: joints=14 motion=False")

    def current_joints(self, arm: str) -> np.ndarray:
        return self._joints[arm].copy()

    def execute_joint_path(self, path: IKPath) -> None:
        raise RuntimeError("规划执行器禁止发送关节路径")

    def close_gripper(self, arm: str) -> bool:
        raise RuntimeError(f"规划执行器禁止闭合夹爪: arm={arm}")

    def open_gripper(self, arm: str) -> None:
        raise RuntimeError(f"规划执行器禁止张开夹爪: arm={arm}")


class AlwaysSafeCollisionChecker:
    """Explicit test-only collision checker."""

    def check_path(self, path: IKPath) -> tuple[bool, str]:
        return True, f"test checker accepted {len(path.positions)} points"


def execute_path(
    path: IKPath,
    executor: ArmExecutor,
    collision_checker: CollisionChecker | None,
    collision_required: bool,
) -> ExecutionEvidence:
    """Validate and execute the path; never home the arm implicitly."""
    if collision_checker is None and collision_required:
        raise RuntimeError("缺少碰撞检查器，拒绝执行抓取")
    if collision_checker is not None:
        safe, detail = collision_checker.check_path(path)
        if not safe:
            logger.warning("碰撞检查拒绝路径: arm=%s detail=%s", path.arm, detail)
            raise RuntimeError(f"抓取路径未通过碰撞检查: {detail}")
        logger.info("碰撞检查通过: arm=%s detail=%s", path.arm, detail)
    grasp_indices = [
        index
        for index, name in enumerate(path.waypoint_names)
        if name == "grasp"
    ]
    if len(grasp_indices) != 1:
        raise RuntimeError(
            f"路径必须包含且只包含一个 grasp 点，实际 {len(grasp_indices)}"
        )
    grasp_index = grasp_indices[0]
    approach_path = _slice_path(path, 0, grasp_index + 1)
    lift_path = _slice_path(path, grasp_index + 1, len(path.positions))
    if not lift_path.positions:
        raise RuntimeError("grasp 点之后缺少抬升与后撤路径")
    try:
        executor.open_gripper(path.arm)
        executor.execute_joint_path(approach_path)
        contact = executor.close_gripper(path.arm)
        executor.execute_joint_path(lift_path)
    except Exception as exc:
        logger.exception("机械臂执行失败: arm=%s", path.arm)
        raise RuntimeError(f"{path.arm} 臂执行失败") from exc
    evidence = ExecutionEvidence(
        arm=path.arm,
        path_executed=True,
        gripper_closed=True,
        contact_detected=bool(contact),
        lift_completed=True,
        detail="path completed",
    )
    logger.info(
        "抓取路径执行完成: arm=%s contact=%s points=%d",
        path.arm,
        evidence.contact_detected,
        len(path.positions),
    )
    return evidence


def _slice_path(path: IKPath, start: int, stop: int) -> IKPath:
    positions = path.positions[start:stop]
    names = path.waypoint_names[start:stop]
    max_step = 0.0
    for first, second in pairwise(positions):
        max_step = max(max_step, float(np.max(np.abs(second - first))))
    return IKPath(
        arm=path.arm,
        joint_names=path.joint_names,
        positions=positions,
        waypoint_names=names,
        max_joint_step_rad=max_step,
        score=path.score,
    )
