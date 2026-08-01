"""Stereo rectification and CREStereo backend adapters."""

from __future__ import annotations

import importlib
import json
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

import numpy as np

from rpent_traditional_grasp.image_trace import pixel_sha256, save_stereo_pngs
from rpent_traditional_grasp.logging import get_logger
from rpent_traditional_grasp.models import StereoObservation

logger = get_logger("stereo")


class StereoCamera(Protocol):
    """Raw synchronized stereo camera interface."""

    def capture_stereo(self) -> tuple[np.ndarray, np.ndarray, float]:
        """Return left image, right image and timestamp seconds."""


class DisparityBackend(Protocol):
    """Metric-independent disparity inference backend."""

    def predict_disparity(
        self, left_rectified: np.ndarray, right_rectified: np.ndarray
    ) -> np.ndarray:
        """Return disparity pixels in the rectified left image."""


@dataclass(slots=True)
class StereoCalibration:
    """Stereo intrinsics/extrinsics needed by OpenCV rectification."""

    image_size: tuple[int, int]
    camera_matrix_left: np.ndarray
    distortion_left: np.ndarray
    camera_matrix_right: np.ndarray
    distortion_right: np.ndarray
    rotation: np.ndarray
    translation_m: np.ndarray

    @classmethod
    def from_json(cls, path: str | Path) -> StereoCalibration:
        config_path = Path(path)
        try:
            raw = json.loads(config_path.read_text(encoding="utf-8"))
            calibration = cls.from_mapping(raw)
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            logger.exception("读取双目标定失败: path=%s", config_path)
            raise ValueError(f"无效双目标定文件: {config_path}") from exc
        calibration.validate()
        logger.info(
            "双目标定已加载: path=%s size=%sx%s baseline=%.4fm",
            config_path,
            calibration.image_size[0],
            calibration.image_size[1],
            calibration.baseline_m,
        )
        return calibration

    @classmethod
    def from_mapping(cls, raw: dict[str, Any]) -> StereoCalibration:
        """Accept the project schema or legacy K1/K2/D1/D2/R/T names."""
        legacy = "K1" in raw
        image_size = raw.get("image_shape") if legacy else raw.get("image_size")
        if image_size is None:
            raise ValueError("双目标定缺少 image_size")
        calibration = cls(
            image_size=tuple(int(value) for value in image_size),
            camera_matrix_left=np.asarray(
                raw["K1"] if legacy else raw["camera_matrix_left"],
                dtype=np.float64,
            ).reshape(3, 3),
            distortion_left=np.asarray(
                raw.get("D1", []) if legacy else raw.get("distortion_left", []),
                dtype=np.float64,
            ),
            camera_matrix_right=np.asarray(
                raw["K2"] if legacy else raw["camera_matrix_right"],
                dtype=np.float64,
            ).reshape(3, 3),
            distortion_right=np.asarray(
                raw.get("D2", []) if legacy else raw.get("distortion_right", []),
                dtype=np.float64,
            ),
            rotation=np.asarray(
                raw["R"] if legacy else raw["rotation"], dtype=np.float64
            ).reshape(3, 3),
            translation_m=np.asarray(
                raw["T"] if legacy else raw["translation_m"], dtype=np.float64
            ).reshape(3, 1),
        )
        calibration.validate()
        return calibration

    @property
    def baseline_m(self) -> float:
        return float(np.linalg.norm(self.translation_m))

    def validate(self) -> None:
        width, height = self.image_size
        if width <= 0 or height <= 0:
            raise ValueError("双目标定 image_size 无效")
        if self.camera_matrix_left[0, 0] <= 0:
            raise ValueError("左相机焦距无效")
        if self.camera_matrix_right[0, 0] <= 0:
            raise ValueError("右相机焦距无效")
        if not 0.01 <= self.baseline_m <= 0.5:
            raise ValueError(f"双目基线异常: {self.baseline_m:.4f}m")
        if not np.allclose(self.rotation.T @ self.rotation, np.eye(3), atol=1e-3):
            raise ValueError("双目标定 rotation 不是有效旋转矩阵")


class RectifiedStereoPipeline:
    """Rectify synchronized images, infer disparity, and convert to meters."""

    def __init__(
        self,
        camera: StereoCamera,
        disparity_backend: DisparityBackend,
        calibration: StereoCalibration,
        artifact_dir: str | Path | None = None,
    ) -> None:
        self.camera = camera
        self.disparity_backend = disparity_backend
        self.calibration = calibration
        self.artifact_dir = Path(artifact_dir) if artifact_dir else None
        self._maps: tuple[np.ndarray, ...] | None = None
        self._projection_left: np.ndarray | None = None

    def capture(self) -> StereoObservation:
        """Capture and process one stereo frame with diagnostic timing."""
        started = time.perf_counter()
        try:
            left, right, timestamp_s = self.camera.capture_stereo()
            left_rectified, right_rectified, projection = self._rectify(left, right)
            if self.artifact_dir is not None:
                self._save_rectified_pair(
                    left_rectified,
                    right_rectified,
                    timestamp_s,
                )
            disparity = np.asarray(
                self.disparity_backend.predict_disparity(
                    left_rectified, right_rectified
                ),
                dtype=np.float32,
            )
        except Exception as exc:
            logger.exception("双目采集或 CREStereo 推理失败")
            raise RuntimeError("双目深度计算失败") from exc
        if disparity.shape != left_rectified.shape[:2]:
            raise ValueError(
                f"视差尺寸不匹配: {disparity.shape} != " f"{left_rectified.shape[:2]}"
            )
        fx = float(projection[0, 0])
        depth_m = np.full(disparity.shape, np.nan, dtype=np.float32)
        valid = np.isfinite(disparity) & (disparity > 1e-6)
        depth_m[valid] = fx * self.calibration.baseline_m / disparity[valid]
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        logger.info(
            "双目深度完成: valid_ratio=%.3f elapsed_ms=%.1f",
            float(np.count_nonzero(valid) / valid.size),
            elapsed_ms,
        )
        return StereoObservation(
            left=left_rectified,
            right=right_rectified,
            depth_m=depth_m,
            projection_left=projection,
            timestamp_s=float(timestamp_s),
        )

    def _save_rectified_pair(
        self,
        left: np.ndarray,
        right: np.ndarray,
        timestamp_s: float,
    ) -> None:
        assert self.artifact_dir is not None
        left_sha256 = pixel_sha256(left)
        right_sha256 = pixel_sha256(right)
        try:
            left_path, right_path = save_stereo_pngs(
                self.artifact_dir,
                stage="rectified",
                timestamp_s=timestamp_s,
                left=left,
                right=right,
            )
        except Exception:
            logger.exception(
                "保存校正双目帧失败: artifact_dir=%s timestamp_s=%.6f "
                "left_sha256=%s right_sha256=%s",
                self.artifact_dir,
                timestamp_s,
                left_sha256,
                right_sha256,
            )
            return
        logger.info(
            "校正双目帧已保存: timestamp_s=%.6f left_path=%s "
            "right_path=%s left_sha256=%s right_sha256=%s",
            timestamp_s,
            left_path,
            right_path,
            left_sha256,
            right_sha256,
        )

    def _rectify(
        self, left: np.ndarray, right: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        try:
            import cv2
        except ImportError as exc:
            raise RuntimeError("实时双目矫正需要安装 opencv-python") from exc
        width, height = self.calibration.image_size
        if left.shape[:2] != (height, width) or right.shape[:2] != (height, width):
            raise ValueError(
                f"双目图像尺寸必须是 {(width, height)}，"
                f"实际 left={left.shape[:2]} right={right.shape[:2]}"
            )
        if self._maps is None or self._projection_left is None:
            (
                rect_left,
                rect_right,
                proj_left,
                proj_right,
                _,
                _,
                _,
            ) = cv2.stereoRectify(
                self.calibration.camera_matrix_left,
                self.calibration.distortion_left,
                self.calibration.camera_matrix_right,
                self.calibration.distortion_right,
                (width, height),
                self.calibration.rotation,
                self.calibration.translation_m,
                flags=cv2.CALIB_ZERO_DISPARITY,
                alpha=0,
            )
            left_maps = cv2.initUndistortRectifyMap(
                self.calibration.camera_matrix_left,
                self.calibration.distortion_left,
                rect_left,
                proj_left,
                (width, height),
                cv2.CV_32FC1,
            )
            right_maps = cv2.initUndistortRectifyMap(
                self.calibration.camera_matrix_right,
                self.calibration.distortion_right,
                rect_right,
                proj_right,
                (width, height),
                cv2.CV_32FC1,
            )
            self._maps = (*left_maps, *right_maps)
            self._projection_left = np.asarray(proj_left, dtype=np.float64)
        map_lx, map_ly, map_rx, map_ry = self._maps
        return (
            cv2.remap(left, map_lx, map_ly, cv2.INTER_LINEAR),
            cv2.remap(right, map_rx, map_ry, cv2.INTER_LINEAR),
            self._projection_left.copy(),
        )


class ExternalCREStereoBackend:
    """Lazy adapter for a deployment-provided CREStereo implementation."""

    def __init__(
        self,
        repository: str | Path,
        model_path: str | Path,
        module_name: str,
        class_name: str = "CREStereo",
        device: str = "cuda",
    ) -> None:
        self.repository = Path(repository)
        self.model_path = Path(model_path)
        self.module_name = module_name
        self.class_name = class_name
        self.device = device
        self._model: Any = None

    def predict_disparity(
        self, left_rectified: np.ndarray, right_rectified: np.ndarray
    ) -> np.ndarray:
        model = self._load()
        for method_name in ("predict_disparity", "infer", "predict"):
            method = getattr(model, method_name, None)
            if method is not None:
                return np.asarray(method(left_rectified, right_rectified))
        if callable(model):
            return np.asarray(model(left_rectified, right_rectified))
        raise RuntimeError(
            f"{self.class_name} 缺少 predict_disparity/infer/predict 方法"
        )

    def _load(self) -> Any:
        if self._model is not None:
            return self._model
        repository = str(self.repository.resolve())
        if repository not in sys.path:
            sys.path.insert(0, repository)
        try:
            module = importlib.import_module(self.module_name)
            type_ = getattr(module, self.class_name)
            try:
                self._model = type_(model_path=str(self.model_path), device=self.device)
            except TypeError:
                self._model = type_(str(self.model_path))
        except Exception as exc:
            logger.exception(
                "加载外部 CREStereo 失败: repo=%s module=%s class=%s",
                self.repository,
                self.module_name,
                self.class_name,
            )
            raise RuntimeError("无法加载外部 CREStereo 后端") from exc
        logger.info(
            "CREStereo 后端已加载: module=%s class=%s device=%s",
            self.module_name,
            self.class_name,
            self.device,
        )
        return self._model


class StaticStereoSource:
    """Deterministic source used by local tests and recorded-frame replay."""

    def __init__(self, observation: StereoObservation) -> None:
        self.observation = observation

    def capture(self) -> StereoObservation:
        return self.observation
