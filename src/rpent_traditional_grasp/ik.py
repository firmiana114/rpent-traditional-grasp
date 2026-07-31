"""Persistent TRAC-IK subprocess adapter and continuous seven-axis solving."""

from __future__ import annotations

import selectors
import subprocess
import threading
from pathlib import Path
from typing import Protocol

import numpy as np

from rpent_traditional_grasp.config import PlannerConfig
from rpent_traditional_grasp.logging import get_logger
from rpent_traditional_grasp.models import CartesianWaypoint, IKPath, Pose
from rpent_traditional_grasp.planning import interpolate_pose

logger = get_logger("ik")

LEFT_JOINT_NAMES = (
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
)
RIGHT_JOINT_NAMES = tuple(name.replace("left_", "right_") for name in LEFT_JOINT_NAMES)


class IKSolver(Protocol):
    """Minimal solver interface used by the Cartesian path planner."""

    joint_names: tuple[str, ...]

    def solve(self, seed: np.ndarray, target: Pose) -> np.ndarray:
        """Solve one pose using the previous solution as seed."""

    def forward(self, joints: np.ndarray) -> Pose:
        """Compute the TCP pose for validation and path initialization."""


class TracIKProcess:
    """One persistent standalone TRAC-IK process for exactly one arm.

    The upstream project warns against multiple solver instances in one
    process. Keeping one subprocess per arm also isolates native solver faults.
    """

    def __init__(
        self,
        binary: str | Path,
        chain_file: str | Path,
        arm: str,
        timeout_s: float = 0.02,
        tolerance: float = 1e-5,
        response_timeout_s: float = 2.0,
    ) -> None:
        if arm not in {"left", "right"}:
            raise ValueError("arm 必须是 left 或 right")
        self.binary = Path(binary)
        self.chain_file = Path(chain_file)
        self.arm = arm
        self.timeout_s = timeout_s
        self.tolerance = tolerance
        self.response_timeout_s = response_timeout_s
        self.joint_names = LEFT_JOINT_NAMES if arm == "left" else RIGHT_JOINT_NAMES
        self._process: subprocess.Popen[str] | None = None
        self._lock = threading.Lock()

    def solve(self, seed: np.ndarray, target: Pose) -> np.ndarray:
        seed_array = _validate_joints(seed)
        rotation = np.asarray(target.rotation, dtype=np.float64).reshape(9)
        values = [
            *seed_array,
            *np.asarray(target.position_m, dtype=np.float64),
            *rotation,
        ]
        response = self._request(
            "IK "
            + " ".join(f"{value:.17g}" for value in values)
        )
        return _parse_ok_joints(response)

    def forward(self, joints: np.ndarray) -> Pose:
        values = _validate_joints(joints)
        response = self._request(
            "FK " + " ".join(f"{value:.17g}" for value in values)
        )
        tokens = response.split()
        if len(tokens) != 13 or tokens[0] != "OK":
            raise RuntimeError(f"TRAC-IK FK 响应无效: {response[:200]}")
        numbers = np.asarray([float(value) for value in tokens[1:]], dtype=np.float64)
        return Pose(numbers[:3], numbers[3:].reshape(3, 3))

    def close(self) -> None:
        process = self._process
        self._process = None
        if process is None:
            return
        try:
            if process.stdin:
                process.stdin.write("QUIT\n")
                process.stdin.flush()
            process.wait(timeout=1.0)
        except (BrokenPipeError, subprocess.TimeoutExpired):
            process.terminate()
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                process.kill()
        logger.info("TRAC-IK 子进程已关闭: arm=%s", self.arm)

    def __enter__(self) -> TracIKProcess:
        self._start()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def _start(self) -> subprocess.Popen[str]:
        if self._process is not None and self._process.poll() is None:
            return self._process
        if not self.binary.is_file():
            raise FileNotFoundError(f"TRAC-IK 可执行文件不存在: {self.binary}")
        if not self.chain_file.is_file():
            raise FileNotFoundError(f"G1 运动链文件不存在: {self.chain_file}")
        try:
            process = subprocess.Popen(
                [
                    str(self.binary),
                    str(self.chain_file),
                    f"{self.timeout_s:.17g}",
                    f"{self.tolerance:.17g}",
                ],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
            )
        except OSError as exc:
            logger.exception(
                "启动 TRAC-IK 失败: arm=%s binary=%s chain=%s",
                self.arm,
                self.binary,
                self.chain_file,
            )
            raise RuntimeError(f"无法启动 {self.arm} TRAC-IK") from exc
        self._process = process
        ready = self._readline(process)
        if ready != "READY 7":
            error = process.stderr.readline().strip() if process.stderr else ""
            self.close()
            raise RuntimeError(
                f"TRAC-IK 启动握手失败: response={ready!r} stderr={error[:300]!r}"
            )
        logger.info(
            "TRAC-IK 子进程已启动: arm=%s pid=%s chain=%s",
            self.arm,
            process.pid,
            self.chain_file,
        )
        return process

    def _request(self, command: str) -> str:
        with self._lock:
            process = self._start()
            if process.stdin is None:
                raise RuntimeError("TRAC-IK stdin 不可用")
            try:
                process.stdin.write(command + "\n")
                process.stdin.flush()
                response = self._readline(process)
            except (BrokenPipeError, OSError) as exc:
                logger.exception("TRAC-IK 通信失败: arm=%s", self.arm)
                self.close()
                raise RuntimeError(f"{self.arm} TRAC-IK 通信失败") from exc
            if response.startswith("ERR "):
                raise RuntimeError(
                    f"{self.arm} TRAC-IK 求解失败: {response[4:300]}"
                )
            return response

    def _readline(self, process: subprocess.Popen[str]) -> str:
        if process.stdout is None:
            raise RuntimeError("TRAC-IK stdout 不可用")
        selector = selectors.DefaultSelector()
        selector.register(process.stdout, selectors.EVENT_READ)
        try:
            if not selector.select(self.response_timeout_s):
                self.close()
                raise TimeoutError(
                    f"{self.arm} TRAC-IK 响应超时 "
                    f"({self.response_timeout_s:.2f}s)"
                )
            line = process.stdout.readline()
        finally:
            selector.close()
        if not line:
            error = process.stderr.readline().strip() if process.stderr else ""
            raise RuntimeError(
                f"{self.arm} TRAC-IK 意外退出: code={process.poll()} "
                f"stderr={error[:300]}"
            )
        return line.strip()


class MockIKSolver:
    """Deterministic locally invertible solver for pipeline tests."""

    def __init__(self, arm: str) -> None:
        self.joint_names = LEFT_JOINT_NAMES if arm == "left" else RIGHT_JOINT_NAMES

    def solve(self, seed: np.ndarray, target: Pose) -> np.ndarray:
        result = _validate_joints(seed).copy()
        result[:3] = np.asarray(target.position_m, dtype=np.float64)
        return result

    def forward(self, joints: np.ndarray) -> Pose:
        values = _validate_joints(joints)
        return Pose(values[:3].copy(), np.eye(3, dtype=np.float64))


def solve_continuous_path(
    arm: str,
    solver: IKSolver,
    seed: np.ndarray,
    waypoints: list[CartesianWaypoint],
    config: PlannerConfig,
) -> IKPath:
    """Solve each Cartesian sample with the previous seven-axis solution."""
    current = _validate_joints(seed)
    current_pose = solver.forward(current)
    solutions: list[np.ndarray] = []
    names: list[str] = []
    max_step = 0.0
    score = 0.0
    subdivisions = 0
    pending = [(waypoint, 0) for waypoint in waypoints]
    index = 0
    while pending:
        waypoint, depth = pending.pop(0)
        try:
            solution = _validate_joints(solver.solve(current, waypoint.pose))
            solved_pose = solver.forward(solution)
        except Exception as exc:
            if depth < config.max_adaptive_subdivisions:
                midpoint = CartesianWaypoint(
                    f"{waypoint.name}:adaptive:{depth + 1}",
                    interpolate_pose(current_pose, waypoint.pose, 0.5),
                )
                pending.insert(0, (waypoint, depth + 1))
                pending.insert(0, (midpoint, depth + 1))
                subdivisions += 1
                continue
            logger.exception(
                "连续逆解失败: arm=%s index=%d waypoint=%s",
                arm,
                index,
                waypoint.name,
            )
            raise RuntimeError(
                f"{arm} 臂在 {waypoint.name} 无连续逆解"
            ) from exc
        step = float(np.max(np.abs(solution - current)))
        position_error = float(
            np.linalg.norm(solved_pose.position_m - waypoint.pose.position_m)
        )
        rotation_error = _rotation_distance(
            solved_pose.rotation, waypoint.pose.rotation
        )
        if position_error > config.fk_position_tolerance_m:
            raise RuntimeError(
                f"{arm} 臂 FK 位置残差过大: {position_error:.5f}m"
            )
        if rotation_error > config.fk_rotation_tolerance_rad:
            raise RuntimeError(
                f"{arm} 臂 FK 姿态残差过大: {rotation_error:.5f}rad"
            )
        solutions.append(solution)
        names.append(waypoint.name)
        score += float(np.sum((solution - current) ** 2))
        max_step = max(max_step, step)
        current = solution
        current_pose = solved_pose
        index += 1
    path = IKPath(
        arm=arm,
        joint_names=solver.joint_names,
        positions=solutions,
        waypoint_names=names,
        max_joint_step_rad=max_step,
        score=score,
    )
    logger.info(
        "连续七轴逆解完成: arm=%s points=%d adaptive=%d "
        "observed_max_step=%.4frad score=%.5f",
        arm,
        len(solutions),
        subdivisions,
        max_step,
        score,
    )
    return path


def _validate_joints(joints: np.ndarray) -> np.ndarray:
    values = np.asarray(joints, dtype=np.float64)
    if values.shape != (7,):
        raise ValueError(f"七轴关节向量尺寸无效: {values.shape}")
    if not np.all(np.isfinite(values)):
        raise ValueError("七轴关节向量包含非有限值")
    return values


def _parse_ok_joints(response: str) -> np.ndarray:
    tokens = response.split()
    if len(tokens) < 8 or tokens[0] != "OK":
        raise RuntimeError(f"TRAC-IK IK 响应无效: {response[:200]}")
    return _validate_joints(np.asarray([float(value) for value in tokens[1:8]]))


def _rotation_distance(first: np.ndarray, second: np.ndarray) -> float:
    relative = (
        np.asarray(first, dtype=np.float64).T
        @ np.asarray(second, dtype=np.float64)
    )
    cosine = float(np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0))
    return float(np.arccos(cosine))
