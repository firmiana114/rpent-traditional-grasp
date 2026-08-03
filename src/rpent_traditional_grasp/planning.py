"""Fixed side-grasp planning and Cartesian interpolation."""

from __future__ import annotations

import math
from dataclasses import dataclass

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


@dataclass(frozen=True, slots=True)
class SideGraspRotationCandidate:
    """One bounded TCP orientation considered for a horizontal side approach."""

    name: str
    rotation: np.ndarray
    angular_offset_rad: float


def plan_fixed_side_grasp(
    estimate: BottleEstimate,
    arm: str,
    config: PlannerConfig,
    tcp_rotation: np.ndarray | None = None,
) -> list[CartesianWaypoint]:
    """Plan a horizontal side approach for one supplied TCP orientation."""
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

    # G1 body frame is x-forward, y-left, z-up. The TCP x axis is the nominal
    # tool approach axis. Projecting it onto the body XY plane keeps the final
    # insertion horizontal even when a bounded wrist pitch is selected.
    approach = rotation[:, 0].copy()
    approach[2] = 0.0
    approach_norm = float(np.linalg.norm(approach))
    if approach_norm < 1e-6:
        raise ValueError("TCP 侧抓接近轴不能垂直于桌面")
    approach /= approach_norm
    up = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    pregrasp = target - config.pregrasp_offset_m * approach
    lifted = target + config.lift_offset_m * up
    retreat = lifted - config.retreat_offset_m * approach
    waypoints = [
        CartesianWaypoint("pregrasp", Pose(pregrasp, rotation)),
        CartesianWaypoint("grasp", Pose(target, rotation)),
        CartesianWaypoint("lift", Pose(lifted, rotation)),
        CartesianWaypoint("retreat", Pose(retreat, rotation)),
    ]
    logger.info(
        "固定侧抓路径已生成: arm=%s target=[%.3f,%.3f,%.3f] "
        "rotation=%s approach_xy=[%.3f,%.3f] pregrasp=%.3fm "
        "lift=%.3fm retreat=%.3fm",
        arm,
        *target,
        rotation_source,
        *approach[:2],
        config.pregrasp_offset_m,
        config.lift_offset_m,
        config.retreat_offset_m,
    )
    return waypoints


def plan_homing_waypoint(
    home_xz_and_abs_y_m: tuple[float, float, float],
    arm: str,
    rotation: np.ndarray,
) -> CartesianWaypoint:
    """Return the post-grasp homing target, mirrored for the requested arm.

    The rotation is the one the grasp was solved with, not a canonical
    orientation. Re-orienting the wrist on the way home would rotate the held
    bottle by the candidate's own tilt (10-30 deg for the pitch/yaw candidates
    this planner emits), which risks spilling or shedding it for no benefit --
    the point of homing is to retract the arm, not to standardise its pose.
    """
    x_m, z_m, y_abs_m = (float(value) for value in home_xz_and_abs_y_m)
    if not all(math.isfinite(value) for value in (x_m, z_m, y_abs_m)):
        raise ValueError("归位位姿必须是有限数值")
    if arm not in {"left", "right"}:
        raise ValueError("归位位姿只支持 left 或 right")
    y_m = abs(y_abs_m) if arm == "left" else -abs(y_abs_m)
    position = np.array([x_m, y_m, z_m], dtype=np.float64)
    logger.info(
        "归位路点已生成: arm=%s xyz=[%.4f,%.4f,%.4f] orientation=preserve_grasp",
        arm,
        *position,
    )
    return CartesianWaypoint("home", Pose(position, np.asarray(rotation, dtype=np.float64)))


def side_grasp_rotation_candidates(
    initial_rotation: np.ndarray,
    config: PlannerConfig,
) -> list[SideGraspRotationCandidate]:
    """Generate bounded pitch/yaw alternatives around the current TCP pose."""
    initial = np.asarray(initial_rotation, dtype=np.float64)
    if initial.shape != (3, 3):
        raise ValueError("初始 TCP 姿态必须是 3x3 旋转矩阵")
    candidates = [
        SideGraspRotationCandidate("initial", initial.copy(), 0.0)
    ]
    for axis_name, axis, values in (
        (
            "pitch",
            np.array([0.0, 1.0, 0.0], dtype=np.float64),
            config.side_grasp_pitch_degrees,
        ),
        (
            "yaw",
            np.array([0.0, 0.0, 1.0], dtype=np.float64),
            config.side_grasp_yaw_degrees,
        ),
    ):
        for angle_degrees in values:
            angle = math.radians(float(angle_degrees))
            rotation = initial @ _axis_angle_rotation(axis, angle)
            candidates.append(
                SideGraspRotationCandidate(
                    f"{axis_name}_{float(angle_degrees):+g}deg",
                    rotation,
                    abs(angle),
                )
            )
    for pitch_degrees in config.side_grasp_pitch_degrees:
        pitch = math.radians(float(pitch_degrees))
        pitch_rotation = _axis_angle_rotation(
            np.array([0.0, 1.0, 0.0], dtype=np.float64),
            pitch,
        )
        for yaw_degrees in config.side_grasp_yaw_degrees:
            yaw = math.radians(float(yaw_degrees))
            rotation = (
                initial
                @ pitch_rotation
                @ _axis_angle_rotation(
                    np.array([0.0, 0.0, 1.0], dtype=np.float64),
                    yaw,
                )
            )
            angular_offset = _rotation_angle(initial, rotation)
            if angular_offset > math.radians(
                config.max_side_grasp_tilt_degrees
            ) + 1e-9:
                continue
            candidates.append(
                SideGraspRotationCandidate(
                    "pitch_"
                    f"{float(pitch_degrees):+g}deg_yaw_"
                    f"{float(yaw_degrees):+g}deg",
                    rotation,
                    angular_offset,
                )
            )
    return candidates


def interpolate_joint_bridge(
    start: np.ndarray,
    target: np.ndarray,
    max_joint_step_rad: float,
) -> list[np.ndarray]:
    """Interpolate a bounded joint-space bridge including its target."""
    start_q = np.asarray(start, dtype=np.float64)
    target_q = np.asarray(target, dtype=np.float64)
    if start_q.shape != (7,) or target_q.shape != (7,):
        raise ValueError("关节空间桥接必须使用两个 7 轴向量")
    if max_joint_step_rad <= 0.0:
        raise ValueError("max_joint_step_rad 必须大于 0")
    delta = target_q - start_q
    segments = max(
        1,
        math.ceil(float(np.max(np.abs(delta))) / max_joint_step_rad),
    )
    return [
        start_q + (index / segments) * delta
        for index in range(1, segments + 1)
    ]


LEFT_SHOULDER_BODY = np.array([0.0039563, 0.10022, 0.24778], dtype=np.float64)
RIGHT_SHOULDER_BODY = np.array([0.0039563, -0.10022, 0.24778], dtype=np.float64)


def _contact_frame_axes(target: np.ndarray, arm: str) -> np.ndarray:
    """Build the contact frame the teleop TCP calibration was expressed in.

    x points horizontally from the shoulder to the bottle, z is up, y closes
    the right-handed set. Reproduced exactly from the upstream
    ``build_horizontal_contact_pose`` so the recorded offsets keep their
    meaning; a different frame would silently rotate the correction.
    """
    shoulder = LEFT_SHOULDER_BODY if arm == "left" else RIGHT_SHOULDER_BODY
    x_axis = target - shoulder
    x_axis[2] = 0.0
    norm = float(np.linalg.norm(x_axis))
    x_axis = x_axis / norm if norm > 1e-9 else np.array([1.0, 0.0, 0.0])
    z_axis = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    y_axis = np.cross(z_axis, x_axis)
    y_norm = float(np.linalg.norm(y_axis))
    y_axis = y_axis / y_norm if y_norm > 1e-9 else np.array([0.0, 1.0, 0.0])
    z_axis = np.cross(x_axis, y_axis)
    return np.column_stack([x_axis, y_axis, z_axis])


def _apply_empirical_grasp_offset(
    target: np.ndarray,
    arm: str,
    config: PlannerConfig,
) -> tuple[np.ndarray, str]:
    """Shift the grasp target by the configured empirical correction.

    This is an end-to-end compensation measured from successful teleop grasps,
    not a validated gripper calibration; it stays separate from ``tip_offset_m``
    so it can be switched off without disturbing the kinematic model.
    """
    values = (
        config.left_empirical_grasp_offset_m
        if arm == "left"
        else config.right_empirical_grasp_offset_m
    )
    offset = np.asarray(values, dtype=np.float64)
    if not np.any(offset):
        return target, "disabled"
    rotation = _contact_frame_axes(target, arm)
    shifted = target + rotation @ offset
    detail = (
        f"contact_xyz=[{offset[0]:+.4f},{offset[1]:+.4f},{offset[2]:+.4f}]"
        f" body_delta=[{shifted[0] - target[0]:+.4f},"
        f"{shifted[1] - target[1]:+.4f},{shifted[2] - target[2]:+.4f}]"
    )
    logger.warning(
        "已施加经验抓取偏置（非验收标定，来自遥操 tcp_calibration）: arm=%s %s "
        "target_before=[%.4f,%.4f,%.4f] target_after=[%.4f,%.4f,%.4f]",
        arm,
        detail,
        *target,
        *shifted,
    )
    return shifted, detail


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

    target, offset_applied = _apply_empirical_grasp_offset(target, arm, config)

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
        "xyz=[%.4f,%.4f,%.4f] distance=%.4fm orientation=preserve_initial "
        "empirical_offset=%s",
        arm,
        selection_policy,
        *target,
        distance_m,
        offset_applied,
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
