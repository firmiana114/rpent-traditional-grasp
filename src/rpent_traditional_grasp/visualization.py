"""Projection helpers for no-motion grasp-point visualization."""

from __future__ import annotations

import numpy as np


def body_point_to_left_camera(
    point_body_m: np.ndarray,
    camera_to_body: np.ndarray,
) -> np.ndarray:
    """Transform one body-frame point into the rectified left camera frame."""
    point = _xyz(point_body_m, "point_body_m")
    transform = np.asarray(camera_to_body, dtype=np.float64)
    if transform.shape != (4, 4):
        raise ValueError("camera_to_body 必须是 4x4")
    camera = np.linalg.inv(transform) @ np.append(point, 1.0)
    if abs(camera[3]) < 1e-12:
        raise ValueError("机身到相机变换产生无效齐次坐标")
    result = camera[:3] / camera[3]
    if result[2] <= 0.0:
        raise ValueError(f"抓取点位于相机后方: z={result[2]:.4f}m")
    return result


def project_rectified_point(
    point_left_camera_m: np.ndarray,
    projection: np.ndarray,
) -> np.ndarray:
    """Project a rectified-left-frame 3D point with a 3x4 camera matrix."""
    point = _xyz(point_left_camera_m, "point_left_camera_m")
    matrix = np.asarray(projection, dtype=np.float64)
    if matrix.shape != (3, 4):
        raise ValueError("projection 必须是 3x4")
    pixel_h = matrix @ np.append(point, 1.0)
    if not np.all(np.isfinite(pixel_h)) or abs(pixel_h[2]) < 1e-12:
        raise ValueError("抓取点投影结果无效")
    return pixel_h[:2] / pixel_h[2]


def _xyz(values: np.ndarray, name: str) -> np.ndarray:
    point = np.asarray(values, dtype=np.float64)
    if point.shape != (3,) or not np.all(np.isfinite(point)):
        raise ValueError(f"{name} 必须是 3 个有限数值")
    return point
