"""Configuration for the traditional geometric grasp pipeline."""

from __future__ import annotations

import json
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
    stereo_calibration: str = "config/stereo_calibration.json"
    camera_to_body: str = "config/camera_to_body.json"
    robot_urdf: str = "robot/g1_body29_hand14.urdf"
    left_ik_binary: str = "native/build/g1_trac_ik"
    right_ik_binary: str = "native/build/g1_trac_ik"


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
    """Fixed side-grasp and Cartesian interpolation settings."""

    pregrasp_offset_m: float = 0.10
    lift_offset_m: float = 0.10
    retreat_offset_m: float = 0.08
    cartesian_step_m: float = 0.01
    rotation_step_rad: float = 0.12
    max_joint_step_rad: float = 0.18
    max_adaptive_subdivisions: int = 5
    ik_timeout_s: float = 0.02
    ik_tolerance: float = 1e-5
    fk_position_tolerance_m: float = 0.012
    fk_rotation_tolerance_rad: float = 0.08
    tip_offset_m: float = 0.05
    max_reach_m: float = 0.78
    preferred_arm: str = "auto"
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
            "robot_urdf": os.getenv("RPENT_G1_URDF"),
            "left_ik_binary": os.getenv("RPENT_LEFT_TRAC_IK_BINARY"),
            "right_ik_binary": os.getenv("RPENT_RIGHT_TRAC_IK_BINARY"),
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
        if len(self.planner.left_tcp_rotation) != 9:
            raise ValueError("left_tcp_rotation 必须包含 9 个元素")
        if len(self.planner.right_tcp_rotation) != 9:
            raise ValueError("right_tcp_rotation 必须包含 9 个元素")
        if self.planner.max_adaptive_subdivisions < 0:
            raise ValueError("max_adaptive_subdivisions 不能为负数")
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
        for name in ("left_tcp_rotation", "right_tcp_rotation"):
            if name in values:
                values[name] = tuple(values[name])
    return type_(**values)
