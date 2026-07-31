"""Fixed side-grasp planning and Cartesian interpolation."""

from __future__ import annotations

import math

import numpy as np

from rpent_traditional_grasp.config import PlannerConfig
from rpent_traditional_grasp.logging import get_logger
from rpent_traditional_grasp.models import (
    BottleEstimate,
    CartesianWaypoint,
    GripperTarget,
    Pose,
)

logger = get_logger("planning")


def plan_fixed_side_grasp(
    estimate: BottleEstimate,
    arm: str,
    config: PlannerConfig,
    tcp_rotation: np.ndarray | None = None,
) -> list[CartesianWaypoint]:
    """Plan a translation-only grasp while preserving the supplied TCP rotation."""
    gripper_target = compute_gripper_tcp_target(
        estimate,
        requested_arm=arm,
        config=config,
    )
    target = gripper_target.tcp_body_xyz_m
    if tcp_rotation is None:
        rotation_values = (
            config.left_tcp_rotation if arm == "left" else config.right_tcp_rotation
        )
        rotation = np.asarray(rotation_values, dtype=np.float64).reshape(3, 3)
        rotation_source = "configured_fallback"
    else:
        rotation = np.asarray(tcp_rotation, dtype=np.float64)
        rotation_source = "initial_tcp_pose"
    if rotation.shape != (3, 3):
        raise ValueError("TCP 姿态必须是 3x3 旋转矩阵")
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6):
        raise ValueError(f"{arm} TCP 固定姿态不是正交旋转矩阵")

    # G1 body frame is x-forward, y-left, z-up.
    forward = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    up = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    pregrasp = target - config.pregrasp_offset_m * forward
    lifted = target + config.lift_offset_m * up
    retreat = lifted - config.retreat_offset_m * forward
    waypoints = [
        CartesianWaypoint("pregrasp", Pose(pregrasp, rotation)),
        CartesianWaypoint("grasp", Pose(target, rotation)),
        CartesianWaypoint("lift", Pose(lifted, rotation)),
        CartesianWaypoint("retreat", Pose(retreat, rotation)),
    ]
    logger.info(
        "固定侧抓路径已生成: arm=%s target=[%.3f,%.3f,%.3f] "
        "rotation=%s pregrasp=%.3fm lift=%.3fm retreat=%.3fm",
        arm,
        *target,
        rotation_source,
        config.pregrasp_offset_m,
        config.lift_offset_m,
        config.retreat_offset_m,
    )
    return waypoints


def compute_gripper_tcp_target(
    estimate: BottleEstimate,
    *,
    requested_arm: str,
    config: PlannerConfig,
) -> GripperTarget:
    """Return the final translation of the grasp-center TCP in the body frame.

    The exported G1 chains already contain the wrist-to-TCP fixed offset.
    Consequently the TCP target is the estimated bottle grasp center itself;
    applying ``tip_offset_m`` again here would double-count that offset.
    """
    if requested_arm not in {"auto", "left", "right"}:
        raise ValueError("requested_arm 必须是 auto、left 或 right")
    target = np.asarray(estimate.center_body_m, dtype=np.float64)
    if target.shape != (3,) or not np.all(np.isfinite(target)):
        raise ValueError("瓶体机身坐标必须是三个有限数值")

    if requested_arm != "auto":
        arm = requested_arm
        selection_policy = "explicit"
    elif config.preferred_arm != "auto":
        arm = config.preferred_arm
        selection_policy = "configured_preference"
    else:
        arm = "left" if target[1] >= 0.0 else "right"
        selection_policy = "body_y_side"

    distance_m = float(np.linalg.norm(target))
    if distance_m > config.max_reach_m:
        logger.warning(
            "夹爪 TCP 目标超出距离门禁: arm=%s distance=%.3fm max=%.3fm",
            arm,
            distance_m,
            config.max_reach_m,
        )
        raise ValueError(
            f"夹爪 TCP 目标超出最大距离: {distance_m:.3f}m > "
            f"{config.max_reach_m:.3f}m"
        )
    logger.info(
        "最终夹爪 TCP XYZ 已计算: arm=%s policy=%s "
        "xyz=[%.4f,%.4f,%.4f] distance=%.4fm orientation=preserve_initial",
        arm,
        selection_policy,
        *target,
        distance_m,
    )
    return GripperTarget(
        arm=arm,
        tcp_body_xyz_m=target.copy(),
        selection_policy=selection_policy,
        distance_m=distance_m,
    )


def interpolate_waypoints(
    start: Pose,
    waypoints: list[CartesianWaypoint],
    max_translation_step_m: float,
    max_rotation_step_rad: float,
) -> list[CartesianWaypoint]:
    """Densify translations and rotations for continuous seeded IK."""
    if max_translation_step_m <= 0.0:
        raise ValueError("max_translation_step_m 必须大于 0")
    if max_rotation_step_rad <= 0.0:
        raise ValueError("max_rotation_step_rad 必须大于 0")
    output: list[CartesianWaypoint] = []
    previous = start
    for waypoint in waypoints:
        delta = waypoint.pose.position_m - previous.position_m
        rotation_angle = _rotation_angle(
            previous.rotation, waypoint.pose.rotation
        )
        segments = max(
            1,
            math.ceil(np.linalg.norm(delta) / max_translation_step_m),
            math.ceil(rotation_angle / max_rotation_step_rad),
        )
        for index in range(1, segments + 1):
            ratio = index / segments
            pose = interpolate_pose(previous, waypoint.pose, ratio)
            name = (
                waypoint.name
                if index == segments
                else f"{waypoint.name}:{index}/{segments}"
            )
            output.append(
                CartesianWaypoint(
                    name,
                    pose,
                )
            )
        previous = waypoint.pose
    return output


def interpolate_pose(first: Pose, second: Pose, ratio: float) -> Pose:
    """Interpolate one rigid pose for adaptive IK subdivision."""
    if not 0.0 <= ratio <= 1.0:
        raise ValueError("pose interpolation ratio 必须位于 [0, 1]")
    start_position = np.asarray(first.position_m, dtype=np.float64)
    target_position = np.asarray(second.position_m, dtype=np.float64)
    return Pose(
        start_position + ratio * (target_position - start_position),
        _interpolate_rotation(first.rotation, second.rotation, ratio),
    )


def _rotation_angle(first: np.ndarray, second: np.ndarray) -> float:
    relative = (
        np.asarray(first, dtype=np.float64).T
        @ np.asarray(second, dtype=np.float64)
    )
    cosine = float(np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0))
    return float(np.arccos(cosine))


def _interpolate_rotation(
    first: np.ndarray,
    second: np.ndarray,
    ratio: float,
) -> np.ndarray:
    start = np.asarray(first, dtype=np.float64)
    target = np.asarray(second, dtype=np.float64)
    relative = start.T @ target
    angle = _rotation_angle(start, target)
    if angle < 1e-10:
        return start.copy()
    sine = math.sin(angle)
    if abs(sine) < 1e-8:
        values, vectors = np.linalg.eig(relative)
        axis = np.real(vectors[:, int(np.argmin(np.abs(values - 1.0)))])
        axis /= np.linalg.norm(axis)
    else:
        axis = np.array(
            [
                relative[2, 1] - relative[1, 2],
                relative[0, 2] - relative[2, 0],
                relative[1, 0] - relative[0, 1],
            ],
            dtype=np.float64,
        )
        axis /= 2.0 * sine
    interpolated = start @ _axis_angle_rotation(axis, angle * ratio)
    # Project away numerical drift accumulated over long trajectories.
    u, _, vt = np.linalg.svd(interpolated)
    result = u @ vt
    if np.linalg.det(result) < 0:
        u[:, -1] *= -1
        result = u @ vt
    return result


def _axis_angle_rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    x, y, z = axis
    skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return (
        np.eye(3)
        + math.sin(angle) * skew
        + (1.0 - math.cos(angle)) * (skew @ skew)
    )
