"""Original AIR grasp API names backed by the traditional grasp pipeline."""

from __future__ import annotations

from pathlib import Path
from typing import Protocol

import numpy as np

from rpent_traditional_grasp.config import TraditionalGraspConfig
from rpent_traditional_grasp.diagnostics import (
    ArmChainGeometry,
    ArmReachAssessment,
    assess_arm_reach,
    load_arm_chain_geometry,
)
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
    interpolate_joint_bridge,
    interpolate_waypoints,
    plan_fixed_side_grasp,
    plan_homing_pose,
    side_grasp_rotation_candidates,
)

logger = get_logger("api")


def _load_arm_geometries(
    arm_chain_files: dict[str, str | Path] | None,
) -> dict[str, ArmChainGeometry]:
    """Load the optional chain geometry used by the no-motion reach precheck."""
    if not arm_chain_files:
        logger.info("未提供运动链文件，可达性预检不可用；规划失败将只报逆解错误")
        return {}
    geometries: dict[str, ArmChainGeometry] = {}
    for arm, chain_file in arm_chain_files.items():
        try:
            geometries[arm] = load_arm_chain_geometry(chain_file, arm)
        except Exception:
            logger.exception(
                "加载运动链几何失败，该臂跳过可达性预检: arm=%s path=%s",
                arm,
                chain_file,
            )
    return geometries


def _reach_summary(
    assessments: dict[str, ArmReachAssessment],
) -> dict[str, object]:
    """Fold per-arm verdicts into the fields the upper layers act on."""
    advances = [
        assessment.required_base_advance_m
        for assessment in assessments.values()
        if assessment.required_base_advance_m is not None
    ]
    return {
        "arms": {arm: item.as_dict() for arm, item in assessments.items()},
        "any_arm_within_serial_length_upper_bound": any(
            item.within_serial_length_upper_bound for item in assessments.values()
        ),
        "any_arm_within_planning_radius": any(
            item.within_planning_radius for item in assessments.values()
        ),
        "required_base_advance_m": min(advances) if advances else None,
    }


def _bounded_repr(value: object, limit: int = 256) -> str:
    rendered = repr(value)
    return rendered if len(rendered) <= limit else rendered[: limit - 3] + "..."


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
        arm_chain_files: dict[str, str | Path] | None = None,
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
        self.arm_geometries = _load_arm_geometries(arm_chain_files)
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
        close_checker = getattr(self.collision_checker, "close", None)
        if close_checker is not None:
            try:
                close_checker()
            except Exception:
                logger.exception("关闭自碰撞检查器失败")

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
        prompts = tuple(dict.fromkeys((target, *self.config.perception.target_prompts)))
        logger.info(
            "开始搜索目标: target=%s prompt_count=%d bbox_input=%s "
            "bbox_format=%s bbox_source=%s",
            target,
            len(prompts),
            _bounded_repr(bbox),
            bbox_format,
            "detector" if bbox is None else "explicit",
        )
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
            logger.info(
                "已选择目标候选: target=%s source=%s class=%s "
                "confidence=%.6f bbox=%s capture_timestamp_s=%.6f",
                target,
                "detector" if bbox is None else "explicit",
                detection.class_name,
                detection.confidence,
                detection.bbox_xyxy,
                observation.timestamp_s,
            )
            mask = self.segment_object(observation.left, detection)
            segmentation_artifacts = getattr(self.segmenter, "last_artifacts", {})
            if not isinstance(segmentation_artifacts, dict):
                segmentation_artifacts = {}
            estimate = self.mask_depth_to_pointcloud(detection, mask, observation)
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
            **dict(segmentation_artifacts),
        }

    def assess_target_reach(
        self,
        target_body_xyz_m: np.ndarray,
        *,
        arms: list[str] | None = None,
    ) -> dict[str, object] | None:
        """Judge arm reach without solving IK; ``None`` when geometry is absent."""
        if not self.arm_geometries:
            return None
        selected = {
            arm: geometry
            for arm, geometry in self.arm_geometries.items()
            if arms is None or arm in arms
        }
        if not selected:
            return None
        assessments = assess_arm_reach(
            np.asarray(target_body_xyz_m, dtype=np.float64),
            selected,
            planning_radius_m=self.config.planner.side_grasp_planning_radius_m,
        )
        summary = _reach_summary(assessments)
        logger.info(
            "可达性预检完成: arms=%s within_bound=%s within_radius=%s "
            "required_base_advance_m=%s planning_radius_m=%.3f",
            ",".join(sorted(selected)),
            summary["any_arm_within_serial_length_upper_bound"],
            summary["any_arm_within_planning_radius"],
            summary["required_base_advance_m"],
            self.config.planner.side_grasp_planning_radius_m,
        )
        return summary

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
        # The arms hang from the shoulders, not from the body origin, so the
        # body-origin distance alone reports targets as reachable that no arm
        # can serve. Prefer the shoulder-referenced verdict whenever the chain
        # geometry is available and fall back to the coarse gate otherwise.
        reach = self.assess_target_reach(self.context.estimate.center_body_m)
        if reach is None:
            within_arm_reach = distance <= self.config.planner.max_reach_m
            required_advance = None
        else:
            within_arm_reach = bool(reach["any_arm_within_planning_radius"])
            required_advance = reach["required_base_advance_m"]
        if within_arm_reach:
            logger.info(
                "目标位于手臂工作空间: target=%s body_origin_distance=%.3fm "
                "shoulder_referenced=%s",
                target,
                distance,
                reach is not None,
            )
            return {
                **result,
                "success": True,
                "approached": True,
                "staged": True,
                "base_moved": False,
                "distance_m": distance,
                "safety_distance_m": safety_distance_m,
                "reach": reach,
            }
        if not allow_base_motion:
            logger.warning(
                "目标超出手臂工作空间且未授权底盘运动: "
                "body_origin_distance=%.3fm required_base_advance_m=%s "
                "shoulder_referenced=%s",
                distance,
                required_advance,
                reach is not None,
            )
            return {
                **result,
                "success": False,
                "approached": False,
                "staged": False,
                "base_moved": False,
                "distance_m": distance,
                "required_base_advance_m": required_advance,
                "reason": "base_motion_not_authorized",
                "reach": reach,
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
        home_xz_and_abs_y_m: tuple[float, float, float] | None = None,
    ) -> dict[str, object]:
        """Plan one complete pick without commanding robot or gripper motion.

        ``home_xz_and_abs_y_m`` optionally extends every candidate with a
        retract-to-ready segment. The caller owns that pose -- it is the same
        one its ready_arm tool uses -- so it is passed per request rather than
        duplicated in this project's config, where the two could drift apart.
        """
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
        reach = self.assess_target_reach(
            self.context.estimate.center_body_m,
            arms=candidates,
        )
        if reach is not None and not reach["any_arm_within_serial_length_upper_bound"]:
            advance = reach["required_base_advance_m"]
            # The bound itself is exact, but it is applied to a perceived target.
            # Teleoperation has grasped a bottle that this pipeline reported as
            # beyond the bound, so an unvalidated calibration can push a feasible
            # target outside it. Only refuse to solve once both calibrations that
            # place the target in the body frame have been accepted.
            trusted = (
                self.config.safety.stereo_calibration_validated
                and self.config.safety.camera_to_body_validated
            )
            if trusted:
                logger.warning(
                    "目标超出手臂杆长上界，无需求解即可判定不可达: target=%s "
                    "arms=%s required_base_advance_m=%s",
                    object_prompt,
                    _bounded_repr(reach["arms"]),
                    advance,
                )
                return {
                    "success": False,
                    "action": "pick_object",
                    "object_prompt": object_prompt,
                    "requested_arm_side": arm_side,
                    "selected_arm_side": None,
                    "status": "unreachable",
                    "error": (
                        "target is beyond the arm serial-length bound; "
                        + (
                            f"advance the base by {advance:.3f} m"
                            if advance is not None
                            else "no forward base travel can bring it into reach"
                        )
                    ),
                    "verification": None,
                    "execution": None,
                    "bbox": search.get("bbox"),
                    **_empty_pick_artifacts(),
                    "backend": (
                        "rpent_traditional_grasp.TraditionalGraspAPI.pick_object"
                    ),
                    "reach": reach,
                }
            if (
                advance is not None
                and self.config.planner.skip_solve_when_base_advance_fixes_reach
            ):
                # Opt-in. Untrusted calibration forbids *refusing*, not
                # *advising*: refusing throws away a grasp that would have
                # worked, while advising costs one forward nudge and then
                # grasps anyway. That equivalence only holds while the base is
                # free to move, which this layer cannot see -- hence the flag
                # rather than a behaviour change. What it buys is the 14.6 s a
                # doomed detection-plus-thirteen-solves pass costs to reach the
                # same conclusion this assessment already holds (2026-08-04
                # 13:15).
                logger.warning(
                    "目标超出手臂杆长上界但可由底盘前进解决，跳过求解并回报所需前进量: "
                    "target=%s arms=%s required_base_advance_m=%.3f",
                    object_prompt,
                    _bounded_repr(reach["arms"]),
                    advance,
                )
                return {
                    "success": False,
                    "action": "pick_object",
                    "object_prompt": object_prompt,
                    "requested_arm_side": arm_side,
                    "selected_arm_side": None,
                    "status": "needs_base_advance",
                    "error": (
                        "target is beyond the arm serial-length bound; "
                        f"advance the base by {advance:.3f} m and retry"
                    ),
                    "required_base_advance_m": advance,
                    "verification": None,
                    "execution": None,
                    "bbox": search.get("bbox"),
                    **_empty_pick_artifacts(),
                    "backend": (
                        "rpent_traditional_grasp.TraditionalGraspAPI.pick_object"
                    ),
                    "reach": reach,
                }
            logger.warning(
                "目标超出手臂杆长上界，但标定尚未验收，继续尝试求解: "
                "target=%s arms=%s required_base_advance_m=%s "
                "stereo_validated=%s camera_to_body_validated=%s",
                object_prompt,
                _bounded_repr(reach["arms"]),
                advance,
                self.config.safety.stereo_calibration_validated,
                self.config.safety.camera_to_body_validated,
            )
        planned: list[tuple[IKPath, np.ndarray, dict[str, object]]] = []
        errors: dict[str, str] = {}
        collision_rejected: dict[str, str] = {}
        for candidate in candidates:
            try:
                seed = self.executor.current_joints(candidate)
                arm_plans = self.plan_contact_grasp_candidates(
                    self.context.estimate,
                    candidate,
                    current_joints=seed,
                    home_xz_and_abs_y_m=home_xz_and_abs_y_m,
                )
                for path, metadata in arm_plans:
                    if self.collision_checker is not None:
                        safe, detail = self.collision_checker.check_path(path)
                        metadata = {
                            **metadata,
                            "collision_checked": bool(safe),
                            "collision_check_detail": detail,
                        }
                        if not safe:
                            orientation = str(
                                metadata.get("orientation_candidate", "")
                            )
                            collision_rejected[f"{candidate}:{orientation}"] = detail
                            logger.info(
                                "侧抓候选碰撞拒绝: arm=%s candidate=%s detail=%s",
                                candidate,
                                orientation,
                                detail,
                            )
                            continue
                    planned.append((path, seed, metadata))
            except Exception as exc:
                errors[candidate] = str(exc)
                logger.warning("候选手臂规划失败: arm=%s reason=%s", candidate, exc)
        if not planned:
            advance = reach["required_base_advance_m"] if reach is not None else None
            if advance:
                logger.warning(
                    "全部侧抓候选无解，且目标在实测规划半径之外: target=%s "
                    "required_base_advance_m=%.3f arms=%s",
                    object_prompt,
                    advance,
                    _bounded_repr(reach["arms"]),
                )
            # A batch killed by self-collision is a different problem from an
            # unreachable target, and pointing at the wrong one wasted a field
            # run before; keep the two causes distinguishable in the error.
            if collision_rejected:
                logger.warning(
                    "全部侧抓候选被自碰撞预筛拒绝: target=%s rejected=%d detail=%s",
                    object_prompt,
                    len(collision_rejected),
                    _bounded_repr(collision_rejected),
                )
                reason = (
                    f"every side-grasp candidate self-collides "
                    f"({len(collision_rejected)} rejected)"
                )
            else:
                reason = "no continuous IK path"
            return {
                "success": False,
                "action": "pick_object",
                "object_prompt": object_prompt,
                "requested_arm_side": arm_side,
                "selected_arm_side": None,
                "status": "planning_failed",
                "error": (
                    reason
                    + (
                        f"; target is outside the measured planning radius, "
                        f"advance the base by {advance:.3f} m"
                        if advance
                        else ""
                    )
                ),
                "verification": None,
                "execution": None,
                "bbox": search.get("bbox"),
                **_empty_pick_artifacts(),
                "backend": "rpent_traditional_grasp.TraditionalGraspAPI.pick_object",
                "arm_errors": errors,
                "collision_rejected": collision_rejected,
                "reach": reach,
            }
        planned.sort(key=lambda item: item[0].score)
        planned = planned[: self.config.planner.max_ranked_candidates]
        selected, seed, selected_metadata = planned[0]
        self.context.ik_path = selected
        collision_checked = bool(selected_metadata.get("collision_checked"))
        collision_detail = selected_metadata.get(
            "collision_check_detail",
            "collision checker not configured",
        )
        serialized_candidates = [
            _serialize_plan(path, path_seed, metadata)
            for path, path_seed, metadata in planned
        ]
        observation = self.context.observation
        logger.info(
            "无运动侧抓计划完成: target=%s arm=%s candidate=%s "
            "ranked_candidates=%d points=%d collision_checked=%s motion=False",
            object_prompt,
            selected.arm,
            selected_metadata.get("orientation_candidate"),
            len(serialized_candidates),
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
            "orientation_candidate": selected_metadata.get(
                "orientation_candidate"
            ),
            "orientation_delta_deg": selected_metadata.get(
                "orientation_delta_deg"
            ),
            "plan": serialized_candidates[0],
            "candidate_plans": serialized_candidates,
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
            "orientation_policy": "bounded_side_grasp_candidates",
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

    def segment_object(self, image: np.ndarray, detection: Detection) -> np.ndarray:
        """Original fine-grained name: segment a detection with SAM2."""
        mask = np.asarray(self.segmenter.segment(image, detection), dtype=bool)
        if mask.shape != image.shape[:2]:
            raise ValueError(f"分割掩码尺寸不匹配: {mask.shape} != {image.shape[:2]}")
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
        """Return the lowest-cost bounded side-grasp candidate."""
        return self.plan_contact_grasp_candidates(
            estimate,
            arm,
            current_joints=current_joints,
        )[0][0]

    def _solve_homing_segment(
        self,
        *,
        arm: str,
        solver: IKSolver,
        approach: IKPath,
        home_xz_and_abs_y_m: tuple[float, float, float] | None,
    ) -> IKPath | None:
        """Solve the retract-to-ready extension, or ``None`` if unavailable.

        The target is a joint configuration, not a Cartesian path. Tracking a
        Cartesian target here would leave the 7-axis redundancy free for the
        solver to resolve, and seeded from the retreat pose it resolved it into
        a folded arm with a ~60 deg wrist -- the deformed posture measured on
        2026-08-03. Solving the ready pose once from the neutral seed reproduces
        the configuration ``ready_arm`` itself reaches, and a joint bridge to it
        cannot be contorted because the posture *is* the target rather than a
        constraint. It also mirrors how the path enters: a joint bridge in, a
        joint bridge out.

        Deliberately isolated from the grasp solve: an unreachable homing target
        must not discard a candidate that grasps perfectly well. Homing is a
        convenience at the end of a successful pick, so it degrades to "no
        homing segment" rather than to "no plan".
        """
        if home_xz_and_abs_y_m is None:
            return None
        try:
            home_pose = plan_homing_pose(home_xz_and_abs_y_m, arm)
            # Seed from the neutral pose, not from retreat: the seed is what
            # picks the redundancy branch, and retreat's branch is the folded one.
            home_q = solver.solve(np.zeros(7, dtype=np.float64), home_pose)
            last_q = np.asarray(approach.positions[-1], dtype=np.float64)
            bridge = interpolate_joint_bridge(
                last_q,
                home_q,
                self.config.planner.joint_bridge_step_rad,
            )
            positions = list(bridge)
            names = [
                f"home:{index + 1}/{len(positions)}"
                for index in range(len(positions) - 1)
            ] + ["home"]
            previous = last_q
            max_step = 0.0
            score = 0.0
            for position in positions:
                delta = np.asarray(position, dtype=np.float64) - previous
                max_step = max(max_step, float(np.max(np.abs(delta))))
                score += float(np.sum(delta**2))
                previous = position
            reached = solver.forward(home_q)
            logger.info(
                "归位段已生成: arm=%s points=%d max_step=%.4frad "
                "reached_xyz=[%.4f,%.4f,%.4f]",
                arm,
                len(positions),
                max_step,
                *reached.position_m,
            )
            return IKPath(
                arm=arm,
                joint_names=approach.joint_names,
                positions=positions,
                waypoint_names=names,
                max_joint_step_rad=max_step,
                score=score,
            )
        except Exception:
            logger.warning(
                "归位段求解失败，本候选将不带归位段: arm=%s home=%s",
                arm,
                home_xz_and_abs_y_m,
                exc_info=True,
            )
            return None

    def plan_contact_grasp_candidates(
        self,
        estimate: BottleEstimate,
        arm: str,
        current_joints: np.ndarray | None = None,
        home_xz_and_abs_y_m: tuple[float, float, float] | None = None,
    ) -> list[tuple[IKPath, dict[str, object]]]:
        """Plan bounded side-grasp orientations without commanding motion."""
        solver = self.ik_solvers[arm]
        if current_joints is None:
            current_joints = self.executor.current_joints(arm)
        current_joints = np.asarray(current_joints, dtype=np.float64)
        start_pose = solver.forward(current_joints)
        successes: list[tuple[IKPath, dict[str, object]]] = []
        failures: dict[str, str] = {}
        for orientation in side_grasp_rotation_candidates(
            start_pose.rotation,
            self.config.planner,
        ):
            try:
                sparse = plan_fixed_side_grasp(
                    estimate,
                    arm,
                    self.config.planner,
                    tcp_rotation=orientation.rotation,
                )
                # Reject an orientation before building a path if the final grasp
                # pose itself is infeasible.
                solver.solve(current_joints, sparse[1].pose)
                pregrasp_q = solver.solve(current_joints, sparse[0].pose)
                bridge = interpolate_joint_bridge(
                    current_joints,
                    pregrasp_q,
                    self.config.planner.joint_bridge_step_rad,
                )
                minimum_allowed_z = min(
                    float(start_pose.position_m[2]),
                    float(sparse[0].pose.position_m[2]),
                ) - self.config.planner.joint_bridge_max_tcp_drop_m
                bridge_tcp_z = [
                    float(solver.forward(position).position_m[2])
                    for position in bridge
                ]
                minimum_bridge_z = min(bridge_tcp_z)
                if minimum_bridge_z < minimum_allowed_z:
                    raise RuntimeError(
                        "关节空间预抓取路径 TCP 下探超限: "
                        f"minimum_z={minimum_bridge_z:.4f}m "
                        f"allowed={minimum_allowed_z:.4f}m"
                    )
                pregrasp_pose = solver.forward(pregrasp_q)
                dense_approach = interpolate_waypoints(
                    pregrasp_pose,
                    sparse[1:],
                    self.config.planner.cartesian_step_m,
                    self.config.planner.rotation_step_rad,
                )
                approach_path = solve_continuous_path(
                    arm,
                    solver,
                    pregrasp_q,
                    dense_approach,
                    self.config.planner,
                )
                homing_path = self._solve_homing_segment(
                    arm=arm,
                    solver=solver,
                    approach=approach_path,
                    home_xz_and_abs_y_m=home_xz_and_abs_y_m,
                )
                path = _combine_side_grasp_path(
                    arm=arm,
                    joint_names=approach_path.joint_names,
                    seed=current_joints,
                    bridge=bridge,
                    approach=approach_path,
                    homing=homing_path,
                    orientation_penalty=(
                        self.config.planner.side_grasp_orientation_penalty
                        * orientation.angular_offset_rad**2
                    ),
                )
                metadata: dict[str, object] = {
                    "orientation_policy": "bounded_side_grasp_candidates",
                    "orientation_candidate": orientation.name,
                    "orientation_delta_deg": float(
                        np.degrees(orientation.angular_offset_rad)
                    ),
                    "tcp_rotation": orientation.rotation.tolist(),
                    "approach_policy": (
                        "joint_space_to_pregrasp_then_cartesian_side_insertion"
                    ),
                    "joint_bridge_points": len(bridge),
                    "minimum_joint_bridge_tcp_z_m": minimum_bridge_z,
                    "collision_checked": False,
                    "collision_check_detail": "collision checker not configured",
                }
                successes.append((path, metadata))
                logger.info(
                    "侧抓姿态候选可行: arm=%s candidate=%s delta_deg=%.1f "
                    "bridge_points=%d cartesian_points=%d score=%.6f",
                    arm,
                    orientation.name,
                    metadata["orientation_delta_deg"],
                    len(bridge),
                    len(approach_path.positions),
                    path.score,
                )
            except Exception as exc:
                failures[orientation.name] = str(exc)
                logger.info(
                    "侧抓姿态候选拒绝: arm=%s candidate=%s reason=%s",
                    arm,
                    orientation.name,
                    exc,
                )
        self.context.diagnostics.setdefault("side_grasp_candidates", {})[arm] = {
            "accepted": [metadata["orientation_candidate"] for _, metadata in successes],
            "rejected": failures,
        }
        if not successes:
            raise RuntimeError(
                f"{arm} 臂无可行侧抓姿态候选: {_bounded_repr(failures)}"
            )
        return sorted(successes, key=lambda item: item[0].score)

    def execute_grasp(self, path: IKPath) -> ExecutionEvidence:
        """Original fine-grained name: collision-check and execute one path."""
        return execute_path(
            path,
            self.executor,
            self.collision_checker,
            self.config.safety.collision_check_required,
        )


def _combine_side_grasp_path(
    *,
    arm: str,
    joint_names: tuple[str, ...],
    seed: np.ndarray,
    bridge: list[np.ndarray],
    approach: IKPath,
    orientation_penalty: float,
    homing: IKPath | None = None,
) -> IKPath:
    homing_positions = list(homing.positions) if homing is not None else []
    homing_names = list(homing.waypoint_names) if homing is not None else []
    positions = [*bridge, *approach.positions, *homing_positions]
    waypoint_names = [
        *(
            [f"joint_pregrasp:{index + 1}/{len(bridge)}" for index in range(len(bridge) - 1)]
            if len(bridge) > 1
            else []
        ),
        "pregrasp",
        *approach.waypoint_names,
        *homing_names,
    ]
    if len(positions) != len(waypoint_names):
        raise RuntimeError("侧抓组合路径的位置与名称数量不一致")
    previous = np.asarray(seed, dtype=np.float64)
    max_step = 0.0
    score = float(orientation_penalty)
    # Homing is the same fixed retract for every candidate, so it must not enter
    # the score -- ranking is about which grasp orientation is cheaper to reach.
    # It does enter max_joint_step_rad, which is a safety bound on real motion.
    scored_upto = len(positions) - len(homing_positions)
    for index, position in enumerate(positions):
        delta = np.asarray(position, dtype=np.float64) - previous
        max_step = max(max_step, float(np.max(np.abs(delta))))
        if index < scored_upto:
            score += float(np.sum(delta**2))
        previous = position
    return IKPath(
        arm=arm,
        joint_names=joint_names,
        positions=positions,
        waypoint_names=waypoint_names,
        max_joint_step_rad=max_step,
        score=score,
    )


def _serialize_plan(
    path: IKPath,
    seed: np.ndarray,
    metadata: dict[str, object],
) -> dict[str, object]:
    return {
        "arm_side": path.arm,
        "joint_names": list(path.joint_names),
        "seed_q": np.asarray(seed, dtype=np.float64).tolist(),
        "positions_rad": [position.tolist() for position in path.positions],
        "waypoint_names": list(path.waypoint_names),
        "max_joint_step_rad": path.max_joint_step_rad,
        "score": path.score,
        **metadata,
    }


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
