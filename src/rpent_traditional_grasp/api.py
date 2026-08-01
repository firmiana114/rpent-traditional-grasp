"""Original AIR grasp API names backed by the traditional grasp pipeline."""

from __future__ import annotations

from pathlib import Path
from typing import Protocol

import numpy as np

from rpent_traditional_grasp.config import TraditionalGraspConfig
from rpent_traditional_grasp.execution import (
    ArmExecutor,
    CollisionChecker,
    execute_path,
)
from rpent_traditional_grasp.geometry import estimate_bottle_center
from rpent_traditional_grasp.gripper import load_gripper_specification
from rpent_traditional_grasp.ik import IKSolver, solve_continuous_path
from rpent_traditional_grasp.logging import get_logger
from rpent_traditional_grasp.models import (
    BottleEstimate,
    Detection,
    ExecutionEvidence,
    IKPath,
    PipelineContext,
    StereoObservation,
)
from rpent_traditional_grasp.perception import Detector, Segmenter
from rpent_traditional_grasp.planning import (
    compute_gripper_tcp_target,
    interpolate_waypoints,
    plan_fixed_side_grasp,
)

logger = get_logger("api")


class StereoSource(Protocol):
    """Processed stereo source used by the public API."""

    def capture(self) -> StereoObservation:
        """Return a rectified image pair and metric depth."""


class TraditionalGraspAPI:
    """Drop-in traditional implementation of the four AIR grasp calls.

    No constructor or public method homes the robot. Physical execution is
    gated by configuration, calibration validation and collision checking.
    """

    def __init__(
        self,
        config: TraditionalGraspConfig,
        stereo_source: StereoSource,
        detector: Detector,
        segmenter: Segmenter,
        ik_solvers: dict[str, IKSolver],
        executor: ArmExecutor,
        camera_to_body: np.ndarray,
        collision_checker: CollisionChecker | None = None,
    ) -> None:
        config.validate()
        gripper_spec_path = Path(config.resources.gripper_specification)
        if not gripper_spec_path.is_absolute():
            gripper_spec_path = Path(__file__).resolve().parents[2] / gripper_spec_path
        gripper_specification = load_gripper_specification(gripper_spec_path)
        if not np.isclose(
            config.planner.tip_offset_m,
            gripper_specification.wrist_to_tcp_xyz_m[0],
            atol=1e-12,
        ):
            raise ValueError(
                "planner.tip_offset_m 与夹爪规格 wrist_yaw_to_tcp 不一致: "
                f"planner={config.planner.tip_offset_m:.12f} "
                f"spec={gripper_specification.wrist_to_tcp_xyz_m[0]:.12f}"
            )
        if (
            config.safety.gripper_tcp_calibration_validated
            and not gripper_specification.is_robot_validated
        ):
            raise ValueError(
                "gripper_tcp_calibration_validated 与夹爪规格状态冲突: "
                f"status={gripper_specification.status}"
            )
        if set(ik_solvers) != {"left", "right"}:
            raise ValueError("ik_solvers 必须同时提供 left 和 right")
        transform = np.asarray(camera_to_body, dtype=np.float64)
        if transform.shape != (4, 4):
            raise ValueError("camera_to_body 必须是 4x4")
        self.config = config
        self.gripper_specification = gripper_specification
        self.stereo_source = stereo_source
        self.detector = detector
        self.segmenter = segmenter
        self.ik_solvers = ik_solvers
        self.executor = executor
        self.camera_to_body = transform
        self.collision_checker = collision_checker
        self.context = PipelineContext()
        self._last_target: str | None = None
        logger.info(
            "传统抓取 API 已初始化: mode=%s hardware=%s collision_required=%s "
            "gripper=%s gripper_spec_status=%s",
            config.safety.mode,
            executor.is_hardware,
            config.safety.collision_check_required,
            gripper_specification.model,
            gripper_specification.status,
        )

    def close(self) -> None:
        """Close native solvers without moving or homing either arm."""
        for arm, solver in self.ik_solvers.items():
            close = getattr(solver, "close", None)
            if close is not None:
                try:
                    close()
                except Exception:
                    logger.exception("关闭 IK 求解器失败: arm=%s", arm)

    def __enter__(self) -> TraditionalGraspAPI:
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def search_object(
        self,
        target: str | None = None,
        *,
        object_prompt: str | None = None,
        bbox: object = None,
        bbox_format: str = "auto",
    ) -> dict[str, object]:
        """Locate and geometrically estimate a target bottle."""
        target = object_prompt or target or "bottle"
        prompts = tuple(
            dict.fromkeys((target, *self.config.perception.target_prompts))
        )
        logger.info("开始搜索目标: target=%s prompt_count=%d", target, len(prompts))
        try:
            observation = self.compute_depth_crestereo()
            if bbox is None:
                detections = self.detector.detect(observation.left, prompts)
            else:
                detections = [
                    Detection(
                        class_name=target,
                        confidence=1.0,
                        bbox_xyxy=_coerce_bbox(
                            bbox,
                            observation.left.shape[1],
                            observation.left.shape[0],
                            bbox_format,
                        ),
                    )
                ]
            if not detections:
                self.context = PipelineContext(observation=observation)
                logger.info("未检测到目标: target=%s", target)
                return {
                    "success": False,
                    "action": "search_object",
                    "visible": False,
                    "found": False,
                    "target": target,
                    "object_prompt": target,
                    "reason": "not_detected",
                }
            detection = detections[0]
            mask = self.segment_object(observation.left, detection)
            estimate = self.mask_depth_to_pointcloud(
                detection, mask, observation
            )
            self.context = PipelineContext(
                observation=observation,
                detection=detection,
                mask=mask,
                estimate=estimate,
            )
            self._last_target = target
        except Exception as exc:
            logger.exception("目标搜索失败: target=%s", target)
            raise RuntimeError(f"搜索目标失败: {target}") from exc
        return {
            "success": True,
            "action": "search_object",
            "visible": True,
            "found": True,
            "target": target,
            "object_prompt": target,
            "class_name": estimate.class_name,
            "confidence": estimate.confidence,
            "bbox_xyxy": estimate.bbox_xyxy,
            "bbox": estimate.bbox_xyxy,
            "position_body_m": estimate.center_body_m.tolist(),
            "position": {
                "frame": "body",
                "xyz_m": estimate.center_body_m.tolist(),
                "distance_m": float(np.linalg.norm(estimate.center_body_m)),
            },
            "depth_m": estimate.front_depth_m,
            "diameter_m": estimate.diameter_m,
            "depth_mad_m": estimate.depth_mad_m,
            "valid_depth_pixels": estimate.valid_depth_pixels,
        }

    def approach_object(
        self,
        target: str | None = None,
        *,
        object_prompt: str | None = None,
        safety_distance_m: float = 1.0,
        max_step_m: float = 1.0,
        angle_threshold_deg: float = 5.0,
        max_iterations: int = 10,
        timeout_sec: float = 60.0,
        bbox: object = None,
        bbox_format: str = "auto",
        allow_base_motion: bool = False,
    ) -> dict[str, object]:
        """Check reachability without silently commanding the mobile base."""
        del max_step_m, angle_threshold_deg, max_iterations, timeout_sec
        target = object_prompt or target or "bottle"
        result = self.search_object(
            object_prompt=target,
            bbox=bbox,
            bbox_format=bbox_format,
        )
        if not result["found"]:
            return result
        assert self.context.estimate is not None
        distance = float(np.linalg.norm(self.context.estimate.center_body_m))
        within_arm_reach = distance <= self.config.planner.max_reach_m
        if within_arm_reach:
            logger.info(
                "目标位于手臂工作空间: target=%s distance=%.3fm",
                target,
                distance,
            )
            return {
                **result,
                "success": True,
                "approached": True,
                "staged": True,
                "base_moved": False,
                "distance_m": distance,
                "safety_distance_m": safety_distance_m,
            }
        if not allow_base_motion:
            logger.warning(
                "目标超出手臂工作空间且未授权底盘运动: distance=%.3fm max=%.3fm",
                distance,
                self.config.planner.max_reach_m,
            )
            return {
                **result,
                "success": False,
                "approached": False,
                "staged": False,
                "base_moved": False,
                "distance_m": distance,
                "reason": "base_motion_not_authorized",
            }
        raise RuntimeError("已授权底盘运动，但当前未注入底盘控制器")

    def pick_object(
        self,
        *,
        object_prompt: str,
        arm_side: str = "auto",
        bbox: object = None,
        bbox_format: str = "auto",
    ) -> dict[str, object]:
        """Match RPent's public pick contract while replacing its internals."""
        try:
            return self._pick_object_impl(
                object_prompt=object_prompt,
                arm_side=arm_side,
                bbox=bbox,
                bbox_format=bbox_format,
            )
        except Exception as exc:
            logger.exception(
                "pick_object 执行失败: target=%s arm=%s",
                object_prompt,
                arm_side,
            )
            return {
                "success": False,
                "action": "pick_object",
                "object_prompt": object_prompt,
                "requested_arm_side": arm_side,
                "selected_arm_side": None,
                "status": "execution_failed",
                "error": f"{type(exc).__name__}: {exc}",
                "verification": None,
                "execution": None,
                "bbox": None,
                **_empty_pick_artifacts(),
                "backend": "rpent_traditional_grasp.TraditionalGraspAPI.pick_object",
            }

    def _pick_object_impl(
        self,
        *,
        object_prompt: str,
        arm_side: str,
        bbox: object,
        bbox_format: str,
    ) -> dict[str, object]:
        """Run the replacement internals behind the stable public contract."""
        plan = self.plan_pick_object(
            object_prompt=object_prompt,
            arm_side=arm_side,
            bbox=bbox,
            bbox_format=bbox_format,
        )
        if not plan["success"]:
            return plan
        selected = self.context.ik_path
        assert selected is not None

        if self.config.safety.mode == "shadow" or (
            self.executor.is_hardware and self.config.safety.mode != "live"
        ):
            logger.info(
                "非 live 模式不执行硬件路径: mode=%s arm=%s points=%d",
                self.config.safety.mode,
                selected.arm,
                len(selected.positions),
            )
            return {
                "success": False,
                "action": "pick_object",
                "object_prompt": object_prompt,
                "requested_arm_side": arm_side,
                "selected_arm_side": selected.arm,
                "status": "motion_gated",
                "error": "non-live mode does not execute robot motion",
                "verification": None,
                "execution": {
                    "planned": True,
                    "executed": False,
                    "mode": self.config.safety.mode,
                    "path_points": len(selected.positions),
                },
                "bbox": plan.get("bbox"),
                **_empty_pick_artifacts(),
                "backend": "rpent_traditional_grasp.TraditionalGraspAPI.pick_object",
            }
        evidence = self.execute_grasp(selected)
        self.context.execution = evidence
        simulated = not self.executor.is_hardware
        picked = bool(
            evidence.path_executed
            and evidence.contact_detected
            and evidence.lift_completed
        )
        verification = "gripper_contact" if picked else "failed"
        return {
            "success": picked,
            "action": "pick_object",
            "object_prompt": object_prompt,
            "requested_arm_side": arm_side,
            "selected_arm_side": selected.arm,
            "status": "executed" if picked else "execution_failed",
            "verification": verification,
            "execution": {
                "planned": True,
                "executed": evidence.path_executed,
                "simulated": simulated,
                "grasp_verified": evidence.contact_detected,
                "lift_completed": evidence.lift_completed,
                "retreat_completed": evidence.lift_completed,
                "path_points": len(selected.positions),
                "max_joint_step_rad": selected.max_joint_step_rad,
            },
            "bbox": plan.get("bbox"),
            **_empty_pick_artifacts(),
            "backend": "rpent_traditional_grasp.TraditionalGraspAPI.pick_object",
        }

    def plan_pick_object(
        self,
        *,
        object_prompt: str,
        arm_side: str = "auto",
        bbox: object = None,
        bbox_format: str = "auto",
    ) -> dict[str, object]:
        """Plan one complete pick without commanding robot or gripper motion."""
        if arm_side not in {"auto", "left", "right"}:
            raise ValueError("arm_side 必须是 auto、left 或 right")
        search = self.search_object(
            object_prompt=object_prompt,
            bbox=bbox,
            bbox_format=bbox_format,
        )
        if not search["found"]:
            return {
                "success": False,
                "action": "pick_object",
                "object_prompt": object_prompt,
                "requested_arm_side": arm_side,
                "selected_arm_side": None,
                "status": "detect_failed",
                "error": "traditional grasp detector did not find the target",
                "verification": None,
                "execution": None,
                "bbox": search.get("bbox"),
                **_empty_pick_artifacts(),
                "backend": "rpent_traditional_grasp.TraditionalGraspAPI.pick_object",
                "detector_result": search,
            }
        assert self.context.estimate is not None
        candidates = ["left", "right"] if arm_side == "auto" else [arm_side]
        planned: list[tuple[IKPath, np.ndarray]] = []
        errors: dict[str, str] = {}
        for candidate in candidates:
            try:
                seed = self.executor.current_joints(candidate)
                path = self.plan_contact_grasp(
                    self.context.estimate,
                    candidate,
                    current_joints=seed,
                )
                planned.append((path, seed))
            except Exception as exc:
                errors[candidate] = str(exc)
                logger.warning(
                    "候选手臂规划失败: arm=%s reason=%s", candidate, exc
                )
        if not planned:
            return {
                "success": False,
                "action": "pick_object",
                "object_prompt": object_prompt,
                "requested_arm_side": arm_side,
                "selected_arm_side": None,
                "status": "planning_failed",
                "error": "no continuous IK path",
                "verification": None,
                "execution": None,
                "bbox": search.get("bbox"),
                **_empty_pick_artifacts(),
                "backend": "rpent_traditional_grasp.TraditionalGraspAPI.pick_object",
                "arm_errors": errors,
            }
        selected, seed = min(planned, key=lambda item: item[0].score)
        self.context.ik_path = selected
        collision_checked = False
        collision_detail = "collision checker not configured"
        if self.collision_checker is not None:
            collision_checked, collision_detail = self.collision_checker.check_path(
                selected
            )
            if not collision_checked:
                logger.warning(
                    "规划路径碰撞检查未通过: arm=%s detail=%s",
                    selected.arm,
                    collision_detail,
                )
        observation = self.context.observation
        logger.info(
            "无运动抓取计划完成: target=%s arm=%s points=%d "
            "collision_checked=%s motion=False",
            object_prompt,
            selected.arm,
            len(selected.positions),
            collision_checked,
        )
        return {
            "success": True,
            "action": "pick_object",
            "object_prompt": object_prompt,
            "requested_arm_side": arm_side,
            "selected_arm_side": selected.arm,
            "status": "planned",
            "verification": None,
            "execution": {
                "planned": True,
                "executed": False,
                "path_points": len(selected.positions),
                "max_joint_step_rad": selected.max_joint_step_rad,
            },
            "bbox": search.get("bbox"),
            **_empty_pick_artifacts(),
            "backend": "rpent_traditional_grasp.TraditionalGraspAPI.plan_pick_object",
            "motion_commanded": False,
            "capture_timestamp_s": (
                observation.timestamp_s if observation is not None else 0.0
            ),
            "collision_checked": collision_checked,
            "collision_check_detail": collision_detail,
            "plan": {
                "arm_side": selected.arm,
                "joint_names": list(selected.joint_names),
                "seed_q": np.asarray(seed, dtype=np.float64).tolist(),
                "positions_rad": [position.tolist() for position in selected.positions],
                "waypoint_names": list(selected.waypoint_names),
                "max_joint_step_rad": selected.max_joint_step_rad,
                "score": selected.score,
            },
        }

    def preview_pick_object_xyz(
        self,
        *,
        object_prompt: str,
        arm_side: str = "auto",
        bbox: object = None,
        bbox_format: str = "auto",
    ) -> dict[str, object]:
        """Stop the replacement pipeline at final TCP XYZ for staged testing."""
        if arm_side not in {"auto", "left", "right"}:
            raise ValueError("arm_side 必须是 auto、left 或 right")
        search = self.search_object(
            object_prompt=object_prompt,
            bbox=bbox,
            bbox_format=bbox_format,
        )
        if not search["found"]:
            return {
                **search,
                "success": False,
                "action": "pick_object",
                "object_prompt": object_prompt,
                "requested_arm_side": arm_side,
                "selected_arm_side": None,
                "status": "detect_failed",
                "error": "traditional grasp detector did not find the target",
            }
        assert self.context.estimate is not None
        gripper = compute_gripper_tcp_target(
            self.context.estimate,
            requested_arm=arm_side,
            config=self.config.planner,
        )
        final_xyz = gripper.tcp_body_xyz_m.tolist()
        logger.info(
            "pick_object 内部 XYZ 阶段完成: target=%s arm=%s "
            "xyz=[%.4f,%.4f,%.4f] ik=False motion=False",
            object_prompt,
            gripper.arm,
            *gripper.tcp_body_xyz_m,
        )
        return {
            **search,
            "success": True,
            "action": "pick_object",
            "phase": "gripper_xyz",
            "object_prompt": object_prompt,
            "requested_arm_side": arm_side,
            "selected_arm_side": gripper.arm,
            "final_tcp_body_xyz_m": final_xyz,
            "orientation_policy": "preserve_initial",
            "status": "xyz_ready",
            "reason": "staged_internal_preview",
        }

    def verify_grasp(
        self,
        *,
        target_prompt: str | None = None,
        arm_side: str = "auto",
    ) -> dict[str, object]:
        """Verify the most recent attempt from contact and completed lift."""
        if target_prompt and target_prompt != self._last_target:
            return {
                "success": False,
                "action": "verify_grasp",
                "verified": False,
                "reason": "target_mismatch",
            }
        evidence = self.context.execution
        if evidence is None:
            return {
                "success": False,
                "action": "verify_grasp",
                "verified": False,
                "reason": "no_execution_evidence",
            }
        verified = (
            evidence.path_executed
            and evidence.gripper_closed
            and evidence.contact_detected
            and evidence.lift_completed
        )
        logger.info(
            "抓取验证完成: arm=%s verified=%s contact=%s lift=%s",
            evidence.arm,
            verified,
            evidence.contact_detected,
            evidence.lift_completed,
        )
        return {
            "success": verified,
            "action": "verify_grasp",
            "verified": verified,
            "arm": evidence.arm,
            "arm_side": evidence.arm if arm_side == "auto" else arm_side,
            "target_prompt": target_prompt or self._last_target,
            "contact_detected": evidence.contact_detected,
            "lift_completed": evidence.lift_completed,
            "detail": evidence.detail,
        }

    def compute_depth_crestereo(self) -> StereoObservation:
        """Original fine-grained name: acquire rectified metric stereo depth."""
        return self.stereo_source.capture()

    def segment_object(
        self, image: np.ndarray, detection: Detection
    ) -> np.ndarray:
        """Original fine-grained name: segment a detection with SAM2."""
        mask = np.asarray(self.segmenter.segment(image, detection), dtype=bool)
        if mask.shape != image.shape[:2]:
            raise ValueError(
                f"分割掩码尺寸不匹配: {mask.shape} != {image.shape[:2]}"
            )
        return mask

    def mask_depth_to_pointcloud(
        self,
        detection: Detection,
        mask: np.ndarray,
        observation: StereoObservation,
    ) -> BottleEstimate:
        """Original fine-grained name: estimate front surface and bottle center."""
        return estimate_bottle_center(
            detection,
            mask,
            observation.depth_m,
            observation.projection_left,
            self.camera_to_body,
            self.config.perception,
        )

    def pointcloud_to_body(self, points_camera_m: np.ndarray) -> np.ndarray:
        """Original fine-grained name: transform camera points to body frame."""
        points = np.asarray(points_camera_m, dtype=np.float64)
        scalar = points.ndim == 1
        points = points.reshape(-1, 3)
        homogeneous = np.column_stack([points, np.ones(len(points))])
        body = (self.camera_to_body @ homogeneous.T).T
        body = body[:, :3] / body[:, 3:4]
        return body[0] if scalar else body

    def plan_contact_grasp(
        self,
        estimate: BottleEstimate,
        arm: str,
        current_joints: np.ndarray | None = None,
    ) -> IKPath:
        """Original fine-grained name: fixed pose plus continuous TRAC-IK."""
        solver = self.ik_solvers[arm]
        if current_joints is None:
            current_joints = self.executor.current_joints(arm)
        current_joints = np.asarray(current_joints, dtype=np.float64)
        start_pose = solver.forward(current_joints)
        sparse = plan_fixed_side_grasp(
            estimate,
            arm,
            self.config.planner,
            tcp_rotation=start_pose.rotation,
        )
        dense = interpolate_waypoints(
            start_pose,
            sparse,
            self.config.planner.cartesian_step_m,
            self.config.planner.rotation_step_rad,
        )
        return solve_continuous_path(
            arm,
            solver,
            current_joints,
            dense,
            self.config.planner,
        )

    def execute_grasp(self, path: IKPath) -> ExecutionEvidence:
        """Original fine-grained name: collision-check and execute one path."""
        return execute_path(
            path,
            self.executor,
            self.collision_checker,
            self.config.safety.collision_check_required,
        )


def _coerce_bbox(
    bbox: object,
    width: int,
    height: int,
    bbox_format: str,
) -> tuple[int, int, int, int]:
    if isinstance(bbox, dict):
        values = [
            bbox.get("x1"),
            bbox.get("y1"),
            bbox.get("x2"),
            bbox.get("y2"),
        ]
    else:
        values = list(bbox) if isinstance(bbox, (list, tuple)) else []
    if len(values) != 4 or any(value is None for value in values):
        raise ValueError("bbox 必须是四元素列表或包含 x1/y1/x2/y2 的对象")
    numbers = [float(value) for value in values]
    format_ = bbox_format.lower()
    if format_ == "auto":
        format_ = "norm01" if max(numbers) <= 1.0 else "pixel"
    if format_ in {"norm01", "normalized", "normalized01"}:
        numbers = [
            numbers[0] * width,
            numbers[1] * height,
            numbers[2] * width,
            numbers[3] * height,
        ]
    elif format_ in {"norm1000", "normalized1000"}:
        numbers = [
            numbers[0] * width / 1000.0,
            numbers[1] * height / 1000.0,
            numbers[2] * width / 1000.0,
            numbers[3] * height / 1000.0,
        ]
    elif format_ != "pixel":
        raise ValueError(f"不支持的 bbox_format: {bbox_format}")
    x1, y1, x2, y2 = (round(value) for value in numbers)
    x1, x2 = sorted((max(0, min(width - 1, x1)), max(0, min(width, x2))))
    y1, y2 = sorted((max(0, min(height - 1, y1)), max(0, min(height, y2))))
    if x2 - x1 < 4 or y2 - y1 < 4:
        raise ValueError("bbox 裁剪后过小")
    return x1, y1, x2, y2


def _empty_pick_artifacts() -> dict[str, None]:
    """Preserve RPent's pick artifact fields until this backend emits images."""
    return {
        "bbox_image_path": None,
        "depth_image_path": None,
        "raw_depth_image_path": None,
        "metric_depth_path": None,
        "point_cloud_path": None,
        "point_cloud_frame": None,
        "point_count": None,
        "point_cloud_bounds_m": None,
        "result_image_path": None,
    }
