"""Configuration for the traditional geometric grasp pipeline."""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, field, fields
from pathlib import Path
from typing import Any, TypeVar

from rpent_traditional_grasp.logging import get_logger

logger = get_logger("config")

_T = TypeVar("_T")


@dataclass(slots=True)
class ResourceConfig:
    """Paths and import names supplied by the deployment environment."""

    yolo_model: str = "weights/yolov8x-worldv2.engine"
    yolo_pt_model: str = "weights/yolov8x-worldv2.pt"
    sam2_repo: str = "sam2"
    sam2_checkpoint: str = "sam2/checkpoints/sam2.1_hiera_large.pt"
    sam2_config: str = "configs/sam2.1/sam2.1_hiera_l.yaml"
    crestereo_repo: str = "crestereo"
    crestereo_model: str = "crestereo/crestereo_eth3d.pth"
    # Thor wraps CREStereo in object_grab.py while the laptop deployment imports
    # the public ONNX-CREStereo-Depth-Estimation package, so the import name is
    # configuration rather than a constant.
    crestereo_module: str = "object_grab"
    crestereo_class: str = "CREStereo"
    stereo_calibration: str = "config/stereo_calibration.json"
    camera_to_body: str = "config/camera_to_body.json"
    gripper_specification: str = "config/g1d_dex1_1_nominal.json"
    robot_urdf: str = "robot/g1_body29_hand14.urdf"
    left_ik_binary: str = "native/build/g1_trac_ik"
    right_ik_binary: str = "native/build/g1_trac_ik"
    # Self-collision reuses the parent project's authoritative pinocchio/hpp-fcl
    # checker so ranking agrees with the gate that actually vetoes execution.
    # It lives in a different interpreter, hence a subprocess and its own paths.
    # Empty values disable the pre-filter; ranking then stays collision-blind.
    collision_checker_python: str = ""
    collision_checker_repo: str = ""
    collision_checker_module: str = "robots.air_robot.collision"
    collision_checker_class: str = "PinocchioSelfCollisionChecker"
    collision_urdf: str = ""


@dataclass(slots=True)
class PerceptionConfig:
    """Detection, segmentation and robust depth-estimation thresholds."""

    target_prompts: tuple[str, ...] = (
        "bottle",
        "water bottle",
        "bottled water",
    )
    detection_confidence: float = 0.45
    detection_iou: float = 0.45
    min_mask_pixels: int = 160
    label_band_fraction: float = 0.42
    edge_trim_fraction: float = 0.12
    min_depth_m: float = 0.15
    max_depth_m: float = 2.0
    min_valid_depth_pixels: int = 80
    max_depth_mad_m: float = 0.025
    min_bottle_diameter_m: float = 0.035
    max_bottle_diameter_m: float = 0.12


@dataclass(slots=True)
class PlannerConfig:
    """Constrained side-grasp and interpolation settings."""

    pregrasp_offset_m: float = 0.10
    lift_offset_m: float = 0.10
    retreat_offset_m: float = 0.08
    cartesian_step_m: float = 0.01
    rotation_step_rad: float = 0.12
    max_adaptive_subdivisions: int = 5
    ik_timeout_s: float = 0.02
    ik_tolerance: float = 1e-5
    fk_position_tolerance_m: float = 0.012
    fk_rotation_tolerance_rad: float = 0.08
    tip_offset_m: float = 0.150215608966
    max_reach_m: float = 0.78
    # Shoulder-to-target radius the base-advance advice aims for. The rigorous
    # serial-length bound is 0.5606 m, but where the planner actually solves is
    # strongly configuration dependent: 0.5118 m with a zero-joint seed on the
    # cached desktop scene, 0.5392 m with the simulator ready pose on the
    # replicated field scene. Closer is not better -- MuJoCo physics on the
    # field scene grasps at 0.5392 m (two finger contacts, 91.6 mm lift) yet
    # knocks the bottle over from 0.5309 m inward, so the usable window is only
    # about +-8 mm wide. 0.54 m targets that single physically validated sample.
    # Advice only; rejection uses the rigorous bound alone.
    side_grasp_planning_radius_m: float = 0.54
    preferred_arm: str = "auto"
    # Empirical end-to-end correction added to the grasp-center target, in the
    # contact frame (x = horizontal shoulder→bottle approach, y = its left,
    # z = up). Derived from teleop grasps recorded in tcp_calibration.json.
    # It is deliberately NOT folded into tip_offset_m and must never flip
    # gripper_tcp_calibration_validated: that file's object centres come from
    # the same stereo chain under question, so it absorbs TCP error and
    # perception error together and cannot separate them. All zeros disables
    # it. Per-arm because the two arms measured 87 vs 39 mm laterally, which is
    # impossible for mirror-symmetric hardware and hints the residue is
    # perception, not geometry.
    left_empirical_grasp_offset_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    right_empirical_grasp_offset_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    side_grasp_pitch_degrees: tuple[float, ...] = (
        10.0,
        -10.0,
        20.0,
        -20.0,
        30.0,
        -30.0,
    )
    side_grasp_yaw_degrees: tuple[float, ...] = (
        10.0,
        -10.0,
        20.0,
        -20.0,
    )
    max_side_grasp_tilt_degrees: float = 30.0
    joint_bridge_step_rad: float = 0.08
    joint_bridge_max_tcp_drop_m: float = 0.02
    side_grasp_orientation_penalty: float = 0.02
    max_ranked_candidates: int = 6
    # TCP axes in torso/body frame. Columns are TCP x/y/z.
    left_tcp_rotation: tuple[float, ...] = (
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
    )
    right_tcp_rotation: tuple[float, ...] = (
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
    )


@dataclass(slots=True)
class SafetyConfig:
    """Fail-closed deployment gates."""

    mode: str = "offline"
    allow_motion: bool = False
    stereo_calibration_validated: bool = False
    camera_to_body_validated: bool = False
    gripper_tcp_calibration_validated: bool = False
    collision_check_required: bool = True
    require_grasp_verification: bool = True


@dataclass(slots=True)
class TraditionalGraspConfig:
    """Top-level configuration.

    ``offline`` and ``shadow`` never authorize physical motion. ``live`` also
    requires both calibration flags and an explicitly enabled motion gate.
    """

    resources: ResourceConfig = field(default_factory=ResourceConfig)
    perception: PerceptionConfig = field(default_factory=PerceptionConfig)
    planner: PlannerConfig = field(default_factory=PlannerConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)

    @classmethod
    def from_json(cls, path: str | Path) -> TraditionalGraspConfig:
        """Load a JSON configuration and preserve useful error context."""
        config_path = Path(path)
        try:
            raw = json.loads(config_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            logger.exception("读取传统抓取配置失败: path=%s", config_path)
            raise ValueError(f"无法读取传统抓取配置: {config_path}") from exc
        config = cls.from_mapping(raw)
        logger.info(
            "已加载传统抓取配置: path=%s mode=%s",
            config_path,
            config.safety.mode,
        )
        return config

    @classmethod
    def from_mapping(cls, raw: dict[str, Any]) -> TraditionalGraspConfig:
        """Build a configuration from a nested mapping."""
        config = cls(
            resources=_dataclass_from_mapping(ResourceConfig, raw.get("resources", {})),
            perception=_dataclass_from_mapping(
                PerceptionConfig, raw.get("perception", {})
            ),
            planner=_dataclass_from_mapping(PlannerConfig, raw.get("planner", {})),
            safety=_dataclass_from_mapping(SafetyConfig, raw.get("safety", {})),
        )
        config.validate()
        return config

    @classmethod
    def from_env(cls) -> TraditionalGraspConfig:
        """Load optional JSON config and deployment path overrides."""
        config_path = os.getenv("RPENT_TRADITIONAL_GRASP_CONFIG")
        config = cls.from_json(config_path) if config_path else cls()
        overrides = {
            "yolo_model": os.getenv("RPENT_YOLO_WORLD_MODEL"),
            "sam2_repo": os.getenv("RPENT_SAM2_REPO"),
            "sam2_checkpoint": os.getenv("RPENT_SAM2_CHECKPOINT"),
            "crestereo_repo": os.getenv("RPENT_CRESTEREO_REPO"),
            "crestereo_model": os.getenv("RPENT_CRESTEREO_MODEL"),
            "stereo_calibration": os.getenv("RPENT_STEREO_CALIBRATION"),
            "camera_to_body": os.getenv("RPENT_CAMERA_TO_BODY"),
            "gripper_specification": os.getenv("RPENT_GRIPPER_SPECIFICATION"),
            "robot_urdf": os.getenv("RPENT_G1_URDF"),
            "left_ik_binary": os.getenv("RPENT_LEFT_TRAC_IK_BINARY"),
            "right_ik_binary": os.getenv("RPENT_RIGHT_TRAC_IK_BINARY"),
            "collision_checker_python": os.getenv("RPENT_COLLISION_PYTHON"),
            "collision_checker_repo": os.getenv("RPENT_COLLISION_REPO"),
            "collision_urdf": os.getenv("RPENT_COLLISION_URDF"),
        }
        applied: list[str] = []
        for name, value in overrides.items():
            if value:
                setattr(config.resources, name, value)
                applied.append(name)
        config.validate()
        if applied:
            logger.info("已应用传统抓取环境变量覆盖: fields=%s", ",".join(applied))
        return config

    def validate(self) -> None:
        """Validate values that could otherwise make motion unsafe."""
        if self.safety.mode not in {"offline", "shadow", "live"}:
            raise ValueError("safety.mode 必须是 offline、shadow 或 live")
        if self.planner.preferred_arm not in {"auto", "left", "right"}:
            raise ValueError("planner.preferred_arm 必须是 auto、left 或 right")
        if not 0.0 < self.perception.detection_confidence <= 1.0:
            raise ValueError("detection_confidence 必须位于 (0, 1]")
        if not 0.0 < self.perception.label_band_fraction <= 1.0:
            raise ValueError("label_band_fraction 必须位于 (0, 1]")
        if self.perception.min_depth_m >= self.perception.max_depth_m:
            raise ValueError("min_depth_m 必须小于 max_depth_m")
        if not 0.0 < self.planner.tip_offset_m < 0.3:
            raise ValueError("tip_offset_m 必须位于 (0, 0.3) m")
        for side in ("left", "right"):
            offset = getattr(self.planner, f"{side}_empirical_grasp_offset_m")
            if len(offset) != 3 or not all(math.isfinite(v) for v in offset):
                raise ValueError(
                    f"{side}_empirical_grasp_offset_m 必须是三个有限数值"
                )
            # A correction larger than the arm's own reach margin would be a
            # configuration mistake, not a calibration.
            if max(abs(v) for v in offset) > 0.15:
                raise ValueError(
                    f"{side}_empirical_grasp_offset_m 单轴不得超过 0.15 m"
                )
        if len(self.planner.left_tcp_rotation) != 9:
            raise ValueError("left_tcp_rotation 必须包含 9 个元素")
        if len(self.planner.right_tcp_rotation) != 9:
            raise ValueError("right_tcp_rotation 必须包含 9 个元素")
        if self.planner.max_adaptive_subdivisions < 0:
            raise ValueError("max_adaptive_subdivisions 不能为负数")
        if not 0.0 < self.planner.joint_bridge_step_rad <= 0.3:
            raise ValueError("joint_bridge_step_rad 必须位于 (0, 0.3] rad")
        if not 0.0 <= self.planner.joint_bridge_max_tcp_drop_m <= 0.1:
            raise ValueError("joint_bridge_max_tcp_drop_m 必须位于 [0, 0.1] m")
        if not 0.0 < self.planner.max_side_grasp_tilt_degrees <= 45.0:
            raise ValueError("max_side_grasp_tilt_degrees 必须位于 (0, 45] deg")
        if self.planner.side_grasp_orientation_penalty < 0.0:
            raise ValueError("side_grasp_orientation_penalty 不能为负数")
        if not 1 <= self.planner.max_ranked_candidates <= 20:
            raise ValueError("max_ranked_candidates 必须位于 [1, 20]")
        side_angles = (
            *self.planner.side_grasp_pitch_degrees,
            *self.planner.side_grasp_yaw_degrees,
        )
        if any(
            not -self.planner.max_side_grasp_tilt_degrees
            <= float(angle)
            <= self.planner.max_side_grasp_tilt_degrees
            for angle in side_angles
        ):
            raise ValueError("侧抓姿态候选角度超过 max_side_grasp_tilt_degrees")
        if self.safety.mode == "live":
            missing: list[str] = []
            if not self.safety.allow_motion:
                missing.append("allow_motion")
            if not self.safety.stereo_calibration_validated:
                missing.append("stereo_calibration_validated")
            if not self.safety.camera_to_body_validated:
                missing.append("camera_to_body_validated")
            if not self.safety.gripper_tcp_calibration_validated:
                missing.append("gripper_tcp_calibration_validated")
            if missing:
                raise ValueError("live 模式安全门未满足: " + ", ".join(missing))


def _dataclass_from_mapping(type_: type[_T], raw: Any) -> _T:
    if not isinstance(raw, dict):
        raise ValueError(f"{type_.__name__} 配置必须是对象")
    allowed = {item.name for item in fields(type_)}
    unknown = sorted(set(raw) - allowed)
    if unknown:
        raise ValueError(f"{type_.__name__} 存在未知字段: {', '.join(unknown)}")
    values = dict(raw)
    if type_ is PerceptionConfig and "target_prompts" in values:
        values["target_prompts"] = tuple(values["target_prompts"])
    if type_ is PlannerConfig:
        for name in (
            "left_tcp_rotation",
            "right_tcp_rotation",
            "side_grasp_pitch_degrees",
            "side_grasp_yaw_degrees",
        ):
            if name in values:
                values[name] = tuple(values[name])
    return type_(**values)
