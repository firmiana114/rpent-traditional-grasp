"""Bounded image diagnostics shared by capture and perception stages."""

from __future__ import annotations

import hashlib
import time
from datetime import datetime
from pathlib import Path

import numpy as np


def pixel_sha256(image: np.ndarray) -> str:
    """Hash image metadata and pixels without logging the image itself."""
    array = np.ascontiguousarray(image)
    digest = hashlib.sha256()
    digest.update(str(array.dtype).encode("ascii"))
    digest.update(np.asarray(array.shape, dtype=np.int64).tobytes())
    digest.update(array.tobytes())
    return digest.hexdigest()


def mask_bbox_xyxy(mask: np.ndarray) -> tuple[int, int, int, int] | None:
    """Return the half-open bounding box of a non-empty boolean mask."""
    rows, columns = np.nonzero(np.asarray(mask, dtype=bool))
    if len(columns) == 0:
        return None
    return (
        int(columns.min()),
        int(rows.min()),
        int(columns.max()) + 1,
        int(rows.max()) + 1,
    )


def save_stereo_pngs(
    artifact_dir: str | Path,
    *,
    stage: str,
    timestamp_s: float,
    left: np.ndarray,
    right: np.ndarray,
) -> tuple[Path, Path]:
    """Persist one synchronized stereo pair as lossless PNG files."""
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError("保存双目诊断图片需要安装 OpenCV") from exc

    directory = Path(artifact_dir)
    directory.mkdir(parents=True, exist_ok=True)
    capture_id = datetime.fromtimestamp(timestamp_s).strftime("%Y%m%d_%H%M%S_%f")
    left_path = directory / f"{stage}_left_{capture_id}.png"
    right_path = directory / f"{stage}_right_{capture_id}.png"
    if not cv2.imwrite(str(left_path), np.asarray(left)):
        raise OSError(f"OpenCV 写入左目图片失败: {left_path}")
    if not cv2.imwrite(str(right_path), np.asarray(right)):
        raise OSError(f"OpenCV 写入右目图片失败: {right_path}")
    return left_path, right_path


def save_segmentation_pngs(
    artifact_dir: str | Path,
    *,
    image: np.ndarray,
    image_sha256: str,
    bbox_xyxy: tuple[int, int, int, int],
    mask: np.ndarray,
) -> tuple[Path, Path, Path]:
    """Persist the prompt box, binary mask and color mask overlay."""
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError("保存分割诊断图片需要安装 OpenCV") from exc

    directory = Path(artifact_dir)
    directory.mkdir(parents=True, exist_ok=True)
    artifact_id = f"{image_sha256[:16]}_{time.time_ns()}"
    bbox_path = directory / f"sam2_bbox_{artifact_id}.png"
    mask_path = directory / f"sam2_mask_{artifact_id}.png"
    overlay_path = directory / f"sam2_overlay_{artifact_id}.png"

    source = np.asarray(image)
    selected = np.asarray(mask, dtype=bool)
    boxed = source.copy()
    x1, y1, x2, y2 = bbox_xyxy
    cv2.rectangle(boxed, (x1, y1), (x2, y2), (0, 255, 0), 2)

    binary = selected.astype(np.uint8) * 255
    overlay = source.copy()
    overlay_color = np.asarray([40, 40, 240], dtype=np.uint8)
    overlay[selected] = (0.45 * overlay[selected] + 0.55 * overlay_color).astype(
        np.uint8
    )
    contours, _ = cv2.findContours(
        binary,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )
    cv2.drawContours(overlay, contours, -1, (0, 255, 255), 2)
    cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 255, 0), 2)

    for label, path, rendered in (
        ("框选", bbox_path, boxed),
        ("掩码", mask_path, binary),
        ("分割叠加", overlay_path, overlay),
    ):
        if not cv2.imwrite(str(path), rendered):
            raise OSError(f"OpenCV 写入{label}图片失败: {path}")
    return bbox_path, mask_path, overlay_path
