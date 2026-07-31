"""Bottle-specific geometry using a segmented label band and stereo depth."""

from __future__ import annotations

import numpy as np

from rpent_traditional_grasp.config import PerceptionConfig
from rpent_traditional_grasp.logging import get_logger
from rpent_traditional_grasp.models import BottleEstimate, Detection

logger = get_logger("geometry")


def estimate_bottle_center(
    detection: Detection,
    mask: np.ndarray,
    depth_m: np.ndarray,
    projection_left: np.ndarray,
    camera_to_body: np.ndarray,
    config: PerceptionConfig,
) -> BottleEstimate:
    """Estimate the geometric center of a bottle from its central label band.

    The stereo map observes the front surface. The returned center is shifted
    backward along the optical ray by the estimated bottle radius.
    """
    mask_bool = np.asarray(mask, dtype=bool)
    depth = np.asarray(depth_m, dtype=np.float64)
    projection = np.asarray(projection_left, dtype=np.float64)
    transform = np.asarray(camera_to_body, dtype=np.float64)
    if mask_bool.shape != depth.shape:
        raise ValueError("mask 与 depth_m 的尺寸不一致")
    if projection.shape not in {(3, 3), (3, 4)}:
        raise ValueError("projection_left 必须是 3x3 或 3x4")
    if transform.shape != (4, 4):
        raise ValueError("camera_to_body 必须是 4x4")

    ys, xs = np.nonzero(mask_bool)
    if len(xs) < config.min_mask_pixels:
        raise ValueError(
            f"瓶体掩码像素不足: {len(xs)} < {config.min_mask_pixels}"
        )

    points_uv = np.column_stack([xs, ys]).astype(np.float64)
    centroid_uv = np.median(points_uv, axis=0)
    centered = points_uv - centroid_uv
    covariance = centered.T @ centered / max(len(points_uv) - 1, 1)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    major_uv = eigenvectors[:, int(np.argmax(eigenvalues))]
    minor_uv = np.array([-major_uv[1], major_uv[0]], dtype=np.float64)
    if major_uv[1] < 0:
        major_uv = -major_uv
        minor_uv = -minor_uv

    major_coord = centered @ major_uv
    minor_coord = centered @ minor_uv
    major_limit = np.quantile(np.abs(major_coord), config.label_band_fraction)
    band = np.abs(major_coord) <= major_limit
    if int(np.count_nonzero(band)) < config.min_valid_depth_pixels:
        raise ValueError("瓶身中央商标带像素不足")

    band_minor = minor_coord[band]
    low = float(np.quantile(band_minor, config.edge_trim_fraction))
    high = float(np.quantile(band_minor, 1.0 - config.edge_trim_fraction))
    width_low, width_high = np.quantile(band_minor, [0.02, 0.98])
    inner = band & (minor_coord >= low) & (minor_coord <= high)
    inner_points = points_uv[inner]
    inner_depth = depth[inner_points[:, 1].astype(int), inner_points[:, 0].astype(int)]
    valid = (
        np.isfinite(inner_depth)
        & (inner_depth >= config.min_depth_m)
        & (inner_depth <= config.max_depth_m)
    )
    valid_depth = inner_depth[valid]
    if len(valid_depth) < config.min_valid_depth_pixels:
        raise ValueError(
            "商标带有效深度不足: "
            f"{len(valid_depth)} < {config.min_valid_depth_pixels}"
        )

    front_depth = float(np.median(valid_depth))
    mad = float(np.median(np.abs(valid_depth - front_depth)))
    if mad > config.max_depth_mad_m:
        raise ValueError(
            f"商标带深度离散过大: mad={mad:.4f}m > "
            f"{config.max_depth_mad_m:.4f}m"
        )

    fx = float(projection[0, 0])
    fy = float(projection[1, 1])
    cx = float(projection[0, 2])
    cy = float(projection[1, 2])
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError("相机投影矩阵焦距无效")

    band_center_uv = np.median(inner_points[valid], axis=0)
    # Use the trimmed interior only for depth statistics. Diameter keeps a
    # separate robust near-full span so edge trimming does not bias width.
    width_px = float(width_high - width_low)
    diameter_m = float(width_px * front_depth / fx)
    if not (
        config.min_bottle_diameter_m
        <= diameter_m
        <= config.max_bottle_diameter_m
    ):
        raise ValueError(
            f"瓶径越界: diameter={diameter_m:.4f}m, "
            f"range=[{config.min_bottle_diameter_m:.4f},"
            f"{config.max_bottle_diameter_m:.4f}]"
        )

    front_center = _backproject(
        band_center_uv[0], band_center_uv[1], front_depth, fx, fy, cx, cy
    )
    ray = front_center / np.linalg.norm(front_center)
    center_camera = front_center + ray * (diameter_m * 0.5)
    center_body_h = transform @ np.append(center_camera, 1.0)
    if abs(center_body_h[3]) < 1e-12:
        raise ValueError("camera_to_body 产生无效齐次坐标")
    center_body = center_body_h[:3] / center_body_h[3]

    endpoint_major = np.quantile(major_coord, [0.15, 0.85])
    endpoint_uv = np.stack(
        [
            centroid_uv + endpoint_major[0] * major_uv,
            centroid_uv + endpoint_major[1] * major_uv,
        ]
    )
    endpoint_xyz = np.stack(
        [
            _backproject(u, v, front_depth, fx, fy, cx, cy)
            for u, v in endpoint_uv
        ]
    )
    axis_camera = endpoint_xyz[1] - endpoint_xyz[0]
    axis_camera /= np.linalg.norm(axis_camera)

    estimate = BottleEstimate(
        class_name=detection.class_name,
        confidence=detection.confidence,
        bbox_xyxy=detection.bbox_xyxy,
        center_uv=(float(band_center_uv[0]), float(band_center_uv[1])),
        front_center_camera_m=front_center,
        center_camera_m=center_camera,
        center_body_m=center_body,
        axis_camera=axis_camera,
        diameter_m=diameter_m,
        front_depth_m=front_depth,
        depth_mad_m=mad,
        valid_depth_pixels=len(valid_depth),
    )
    logger.info(
        "瓶体几何估计完成: class=%s conf=%.3f depth=%.3fm "
        "diameter=%.3fm mad=%.4fm valid_pixels=%d",
        estimate.class_name,
        estimate.confidence,
        estimate.front_depth_m,
        estimate.diameter_m,
        estimate.depth_mad_m,
        estimate.valid_depth_pixels,
    )
    return estimate


def _backproject(
    u: float,
    v: float,
    depth_m: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
) -> np.ndarray:
    return np.array(
        [
            (float(u) - cx) * depth_m / fx,
            (float(v) - cy) * depth_m / fy,
            depth_m,
        ],
        dtype=np.float64,
    )
