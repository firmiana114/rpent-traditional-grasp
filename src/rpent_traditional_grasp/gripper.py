"""Load and validate the shared G1-D Dex1-1 gripper specification."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from rpent_traditional_grasp.logging import get_logger

logger = get_logger("gripper")


@dataclass(frozen=True, slots=True)
class GripperSpecification:
    """Nominal gripper geometry shared by production planning and simulation."""

    model: str
    status: str
    wrist_to_tcp_xyz_m: tuple[float, float, float]
    wrist_to_tcp_rpy_rad: tuple[float, float, float]
    finger_joint_lower_m: float
    finger_joint_upper_m: float
    jaw_minimum_m: float
    jaw_maximum_m: float

    @property
    def is_robot_validated(self) -> bool:
        return self.status == "robot_calibrated_validated"


def load_gripper_specification(path: str | Path) -> GripperSpecification:
    """Read a gripper specification with diagnostic context on failure."""
    spec_path = Path(path)
    try:
        raw = json.loads(spec_path.read_text(encoding="utf-8"))
        specification = _from_mapping(raw)
    except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError) as exc:
        logger.exception("读取夹爪规格失败: path=%s", spec_path)
        raise ValueError(f"无法读取夹爪规格: {spec_path}") from exc
    logger.info(
        "已加载夹爪规格: path=%s model=%s status=%s wrist_to_tcp_x=%.6fm",
        spec_path,
        specification.model,
        specification.status,
        specification.wrist_to_tcp_xyz_m[0],
    )
    return specification


def _from_mapping(raw: dict[str, Any]) -> GripperSpecification:
    if raw.get("schema_version") != 1:
        raise ValueError("夹爪规格 schema_version 必须为 1")
    kinematics = raw["kinematics"]["wrist_yaw_to_tcp"]
    finger_joint = raw["finger_joint"]
    jaw = raw["jaw_inner_gap"]
    xyz = _triple(kinematics["xyz_m"], "wrist_yaw_to_tcp.xyz_m")
    rpy = _triple(kinematics["rpy_rad"], "wrist_yaw_to_tcp.rpy_rad")
    lower = float(finger_joint["lower_m"])
    upper = float(finger_joint["upper_m"])
    jaw_minimum = float(jaw["minimum_m"])
    jaw_maximum = float(jaw["maximum_m"])
    if not 0.0 < xyz[0] < 0.3 or not np.allclose(xyz[1:], 0.0, atol=1e-12):
        raise ValueError("当前 G1-D 运动链仅支持沿腕部 X 轴的正向 TCP")
    if not np.allclose(rpy, 0.0, atol=1e-12):
        raise ValueError("当前 G1-D 运动链仅支持零 RPY 的 TCP")
    if lower >= upper:
        raise ValueError("夹爪关节下限必须小于上限")
    if not 0.0 <= jaw_minimum < jaw_maximum:
        raise ValueError("夹爪开口范围无效")
    return GripperSpecification(
        model=str(raw["gripper_model"]),
        status=str(raw["status"]),
        wrist_to_tcp_xyz_m=xyz,
        wrist_to_tcp_rpy_rad=rpy,
        finger_joint_lower_m=lower,
        finger_joint_upper_m=upper,
        jaw_minimum_m=jaw_minimum,
        jaw_maximum_m=jaw_maximum,
    )


def _triple(values: Any, label: str) -> tuple[float, float, float]:
    vector = np.asarray(values, dtype=np.float64)
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        raise ValueError(f"{label} 必须是三个有限数值")
    return tuple(float(value) for value in vector)
