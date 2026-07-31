"""Shared data models for the traditional grasp pipeline."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np


@dataclass(slots=True)
class Detection:
    """A detector result in rectified left-image coordinates."""

    class_name: str
    confidence: float
    bbox_xyxy: tuple[int, int, int, int]


@dataclass(slots=True)
class StereoObservation:
    """Rectified stereo pair, metric depth and left projection matrix."""

    left: np.ndarray
    right: np.ndarray
    depth_m: np.ndarray
    projection_left: np.ndarray
    timestamp_s: float = 0.0


@dataclass(slots=True)
class BottleEstimate:
    """Robust bottle-center estimate in camera and robot body frames."""

    class_name: str
    confidence: float
    bbox_xyxy: tuple[int, int, int, int]
    center_uv: tuple[float, float]
    front_center_camera_m: np.ndarray
    center_camera_m: np.ndarray
    center_body_m: np.ndarray
    axis_camera: np.ndarray
    diameter_m: float
    front_depth_m: float
    depth_mad_m: float
    valid_depth_pixels: int


@dataclass(slots=True)
class GripperTarget:
    """Final gripper TCP translation selected for one arm."""

    arm: str
    tcp_body_xyz_m: np.ndarray
    selection_policy: str
    distance_m: float


@dataclass(slots=True)
class Pose:
    """Rigid pose represented by translation and a 3x3 rotation."""

    position_m: np.ndarray
    rotation: np.ndarray

    def matrix(self) -> np.ndarray:
        transform = np.eye(4, dtype=np.float64)
        transform[:3, :3] = np.asarray(self.rotation, dtype=np.float64)
        transform[:3, 3] = np.asarray(self.position_m, dtype=np.float64)
        return transform


@dataclass(slots=True)
class CartesianWaypoint:
    """A named Cartesian waypoint for one arm."""

    name: str
    pose: Pose


@dataclass(slots=True)
class IKPath:
    """Continuous seven-joint solution for a Cartesian path."""

    arm: str
    joint_names: tuple[str, ...]
    positions: list[np.ndarray]
    waypoint_names: list[str]
    max_joint_step_rad: float
    score: float


@dataclass(slots=True)
class ExecutionEvidence:
    """Evidence used by ``verify_grasp`` instead of an optimistic return."""

    arm: str
    path_executed: bool
    gripper_closed: bool
    contact_detected: bool
    lift_completed: bool
    detail: str = ""


@dataclass(slots=True)
class PipelineContext:
    """Artifacts retained between the original four public API calls."""

    observation: StereoObservation | None = None
    detection: Detection | None = None
    mask: np.ndarray | None = None
    estimate: BottleEstimate | None = None
    ik_path: IKPath | None = None
    execution: ExecutionEvidence | None = None
    diagnostics: dict[str, Any] = field(default_factory=dict)
