"""No-motion IK diagnostics for the staged traditional grasp pipeline."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

import numpy as np

from rpent_traditional_grasp.config import PlannerConfig
from rpent_traditional_grasp.ik import IKSolver, solve_continuous_path
from rpent_traditional_grasp.logging import get_logger
from rpent_traditional_grasp.models import BottleEstimate, Pose
from rpent_traditional_grasp.planning import (
    interpolate_waypoints,
    plan_fixed_side_grasp,
)

logger = get_logger("diagnostics")


@dataclass(frozen=True)
class ArmChainGeometry:
    """Geometric upper bound derived from one exported arm chain."""

    root_frame: str
    shoulder_body_xyz_m: np.ndarray
    serial_length_upper_bound_m: float


@dataclass(frozen=True)
class ArmReachAssessment:
    """No-motion verdict on whether one arm can be planned to a target."""

    arm: str
    shoulder_distance_m: float
    serial_length_upper_bound_m: float
    margin_m: float
    within_serial_length_upper_bound: bool
    within_planning_radius: bool
    required_base_advance_m: float | None

    def as_dict(self) -> dict[str, object]:
        return {
            "arm": self.arm,
            "shoulder_distance_m": self.shoulder_distance_m,
            "serial_length_upper_bound_m": self.serial_length_upper_bound_m,
            "margin_m": self.margin_m,
            "within_serial_length_upper_bound": (
                self.within_serial_length_upper_bound
            ),
            "within_planning_radius": self.within_planning_radius,
            "required_base_advance_m": self.required_base_advance_m,
        }


class BoundedIKSolver(IKSolver, Protocol):
    """IK solver exposing the diagnostic-only position solve."""

    def solve_position_only(self, seed: np.ndarray, target: Pose) -> np.ndarray:
        """Solve XYZ while ignoring rotation."""


def required_base_advance_m(
    target_body_xyz_m: np.ndarray,
    geometry: ArmChainGeometry,
    radius_m: float,
) -> float | None:
    """Return the forward base travel that pulls the target inside ``radius_m``.

    The base drives along body ``+x``, so advancing by ``d`` moves the target to
    ``target - d * x_hat`` in the body frame. Returns ``0.0`` when the target is
    already inside the radius and ``None`` when no forward travel can help,
    which happens when the lateral and vertical offsets alone exceed the radius.
    """
    target = np.asarray(target_body_xyz_m, dtype=np.float64)
    offset = target - geometry.shoulder_body_xyz_m
    if float(np.linalg.norm(offset)) <= radius_m:
        return 0.0
    off_axis_squared = float(offset[1] ** 2 + offset[2] ** 2)
    remaining = radius_m**2 - off_axis_squared
    if remaining <= 0.0:
        return None
    advance = float(offset[0]) - float(np.sqrt(remaining))
    return advance if advance > 0.0 else None


def assess_arm_reach(
    target_body_xyz_m: np.ndarray,
    geometries: dict[str, ArmChainGeometry],
    *,
    planning_radius_m: float,
) -> dict[str, ArmReachAssessment]:
    """Judge every supplied arm against the rigorous bound and the planning radius.

    ``serial_length_upper_bound_m`` is the sum of the remaining link lengths, so a
    negative margin is a proof of unreachability. ``planning_radius_m`` is the
    much tighter measured radius inside which the constrained side-grasp planner
    actually returns candidates; it depends on the seed pose and on how far the
    target sits below the shoulder, so it guides advice instead of rejecting.
    """
    target = np.asarray(target_body_xyz_m, dtype=np.float64)
    if target.shape != (3,) or not np.all(np.isfinite(target)):
        raise ValueError("目标机身坐标必须是三个有限数值")
    if planning_radius_m <= 0.0:
        raise ValueError("planning_radius_m 必须为正")
    assessments: dict[str, ArmReachAssessment] = {}
    for arm, geometry in geometries.items():
        reach = _geometric_reachability(target, geometry)
        distance = float(reach["shoulder_distance_m"])
        assessments[arm] = ArmReachAssessment(
            arm=arm,
            shoulder_distance_m=distance,
            serial_length_upper_bound_m=geometry.serial_length_upper_bound_m,
            margin_m=float(reach["margin_m"]),
            within_serial_length_upper_bound=bool(
                reach["within_serial_length_upper_bound"]
            ),
            within_planning_radius=distance <= planning_radius_m,
            required_base_advance_m=required_base_advance_m(
                target, geometry, planning_radius_m
            ),
        )
    return assessments


def diagnose_ik_reachability(
    *,
    estimate: BottleEstimate,
    solvers: dict[str, BoundedIKSolver],
    seeds: dict[str, np.ndarray],
    seed_source: str,
    config: PlannerConfig,
    chain_files: dict[str, Path] | None = None,
    include_continuous: bool = True,
) -> dict[str, object]:
    """Compare exact-pose, position-only and continuous IK without motion."""
    arms: dict[str, object] = {}
    for arm in ("left", "right"):
        solver = solvers[arm]
        geometry = (
            load_arm_chain_geometry(chain_files[arm], arm)
            if chain_files is not None
            else None
        )
        seed = _joints(seeds[arm], arm)
        start = solver.forward(seed)
        waypoints = plan_fixed_side_grasp(
            estimate,
            arm,
            config,
            tcp_rotation=start.rotation,
        )
        rows: list[dict[str, object]] = []
        for waypoint in waypoints:
            exact = _attempt(
                solver,
                seed,
                waypoint.pose,
                position_only=False,
            )
            position_only = _attempt(
                solver,
                seed,
                waypoint.pose,
                position_only=True,
            )
            if exact["success"]:
                classification = "exact_pose_reachable"
            elif position_only["success"]:
                classification = "fixed_orientation_blocks"
            else:
                classification = "position_unreachable_or_chain_mismatch"
            geometric = (
                _geometric_reachability(waypoint.pose.position_m, geometry)
                if geometry is not None
                else None
            )
            if (
                not position_only["success"]
                and geometric is not None
                and not geometric["within_serial_length_upper_bound"]
            ):
                classification = "outside_chain_length_upper_bound"
            rows.append(
                {
                    "waypoint": waypoint.name,
                    "target_body_xyz_m": waypoint.pose.position_m.tolist(),
                    "classification": classification,
                    "exact_pose": exact,
                    "position_only_diagnostic": position_only,
                    "geometric_reachability": geometric,
                }
            )

        if include_continuous:
            dense = interpolate_waypoints(
                start,
                waypoints,
                config.cartesian_step_m,
                config.rotation_step_rad,
            )
            try:
                path = solve_continuous_path(
                    arm,
                    solver,
                    seed,
                    dense,
                    config,
                )
                continuous: dict[str, object] = {
                    "evaluated": True,
                    "success": True,
                    "points": len(path.positions),
                    "observed_max_joint_change_rad": path.max_joint_step_rad,
                }
            except Exception as exc:
                continuous = {
                    "evaluated": True,
                    "success": False,
                    "error": f"{type(exc).__name__}: {exc}",
                }
        else:
            continuous = {
                "evaluated": False,
                "success": None,
                "reason": "skipped_by_request",
            }
        logger.info(
            "IK 诊断完成: arm=%s exact=%d position_only=%d continuous=%s",
            arm,
            sum(bool(row["exact_pose"]["success"]) for row in rows),
            sum(
                bool(row["position_only_diagnostic"]["success"])
                for row in rows
            ),
            continuous["success"],
        )
        arms[arm] = {
            "seed_rad": seed.tolist(),
            "seed_source": seed_source,
            "start_tcp_body_xyz_m": start.position_m.tolist(),
            "start_tcp_rotation": start.rotation.tolist(),
            "chain_geometry": (
                {
                    "root_frame": geometry.root_frame,
                    "shoulder_body_xyz_m": (
                        geometry.shoulder_body_xyz_m.tolist()
                    ),
                    "serial_length_upper_bound_m": (
                        geometry.serial_length_upper_bound_m
                    ),
                }
                if geometry is not None
                else None
            ),
            "waypoints": rows,
            "continuous_exact_pose": continuous,
        }
    return {
        "schema_version": 1,
        "stage": "ik_reachability_diagnostic",
        "motion_commanded": False,
        "target_body_xyz_m": estimate.center_body_m.tolist(),
        "orientation_policy_production": "bounded_side_grasp_candidates",
        "position_only_is_diagnostic": True,
        "arms": arms,
    }


def load_arm_chain_geometry(
    chain_file: str | Path,
    expected_arm: str,
) -> ArmChainGeometry:
    """Read the shoulder and rigorous serial-length bound from a chain file."""
    path = Path(chain_file)
    try:
        lines = [
            line.split()
            for line in path.read_text(encoding="utf-8").splitlines()
            if line.strip()
        ]
        if not lines or lines[0] != ["RPENT_G1_CHAIN_V1"]:
            raise ValueError("运动链文件头无效")
        root_rows = [row for row in lines if row[0] == "BASE"]
        arm_rows = [row for row in lines if row[0] == "ARM"]
        segments = [
            row for row in lines if row[0] in {"REVOLUTE", "FIXED"}
        ]
        if len(root_rows) != 1 or len(root_rows[0]) != 2:
            raise ValueError("运动链 BASE 行无效")
        if arm_rows != [["ARM", expected_arm]]:
            raise ValueError(
                f"运动链机械臂不匹配: expected={expected_arm}"
            )
        if not segments or segments[0][0] != "REVOLUTE":
            raise ValueError("运动链缺少肩部活动关节")
        translations = [
            np.asarray([float(value) for value in row[3:6]])
            for row in segments
        ]
        shoulder = translations[0]
        upper_bound = float(
            sum(np.linalg.norm(vector) for vector in translations[1:])
        )
    except (OSError, ValueError, IndexError) as exc:
        logger.exception(
            "读取运动链几何失败: arm=%s path=%s",
            expected_arm,
            path,
        )
        raise RuntimeError(
            f"无法读取 {expected_arm} 运动链几何: {path}"
        ) from exc
    logger.info(
        "运动链几何已读取: arm=%s root=%s shoulder=[%.4f,%.4f,%.4f] "
        "serial_upper_bound=%.4fm",
        expected_arm,
        root_rows[0][1],
        *shoulder,
        upper_bound,
    )
    return ArmChainGeometry(
        root_frame=root_rows[0][1],
        shoulder_body_xyz_m=shoulder,
        serial_length_upper_bound_m=upper_bound,
    )


def _geometric_reachability(
    target_body_xyz_m: np.ndarray,
    geometry: ArmChainGeometry,
) -> dict[str, object]:
    shoulder_distance = float(
        np.linalg.norm(
            np.asarray(target_body_xyz_m, dtype=np.float64)
            - geometry.shoulder_body_xyz_m
        )
    )
    margin = geometry.serial_length_upper_bound_m - shoulder_distance
    return {
        "shoulder_distance_m": shoulder_distance,
        "serial_length_upper_bound_m": (
            geometry.serial_length_upper_bound_m
        ),
        "margin_m": margin,
        "within_serial_length_upper_bound": margin >= 0.0,
    }


def _attempt(
    solver: BoundedIKSolver,
    seed: np.ndarray,
    target: Pose,
    *,
    position_only: bool,
) -> dict[str, object]:
    try:
        solution = (
            solver.solve_position_only(seed, target)
            if position_only
            else solver.solve(seed, target)
        )
        actual = solver.forward(solution)
    except Exception as exc:
        return {
            "success": False,
            "error": f"{type(exc).__name__}: {exc}",
        }
    return {
        "success": True,
        "solution_rad": solution.tolist(),
        "observed_max_joint_change_rad": float(
            np.max(np.abs(solution - seed))
        ),
        "fk_position_error_m": float(
            np.linalg.norm(actual.position_m - target.position_m)
        ),
        "fk_rotation_error_rad": _rotation_error(
            actual.rotation,
            target.rotation,
        ),
    }


def _joints(values: np.ndarray, arm: str) -> np.ndarray:
    joints = np.asarray(values, dtype=np.float64)
    if joints.shape != (7,) or not np.all(np.isfinite(joints)):
        raise ValueError(f"{arm} seed 必须是 7 个有限关节角")
    return joints


def _rotation_error(actual: np.ndarray, expected: np.ndarray) -> float:
    relative = np.asarray(expected).T @ np.asarray(actual)
    cosine = float(np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0))
    return float(np.arccos(cosine))
