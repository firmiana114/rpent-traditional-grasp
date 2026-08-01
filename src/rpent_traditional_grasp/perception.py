"""YOLO-World and SAM2 adapters with lazy deployment imports."""

from __future__ import annotations

import importlib.util
import os
import sys
import time
from collections.abc import Sequence
from pathlib import Path
from typing import Any, Protocol

import numpy as np

from rpent_traditional_grasp.image_trace import (
    mask_bbox_xyxy,
    pixel_sha256,
    save_segmentation_pngs,
)
from rpent_traditional_grasp.logging import get_logger
from rpent_traditional_grasp.models import Detection

# Model loading must never mutate Thor's shared Python environment.
os.environ["YOLO_AUTOINSTALL"] = "false"

logger = get_logger("perception")


class Detector(Protocol):
    """Open-vocabulary object detector."""

    def detect(self, image: np.ndarray, prompts: Sequence[str]) -> list[Detection]:
        """Return matching detections."""


class Segmenter(Protocol):
    """Box-prompted binary segmenter."""

    def segment(self, image: np.ndarray, detection: Detection) -> np.ndarray:
        """Return a boolean mask in image coordinates."""


class YoloWorldDetector:
    """Ultralytics YOLO-World adapter for Thor's engine or dynamic PT model."""

    def __init__(
        self,
        model_path: str | Path,
        fallback_pt_path: str | Path | None = None,
        confidence: float = 0.45,
        iou: float = 0.45,
        device: str | int | None = None,
    ) -> None:
        self.model_path = Path(model_path)
        self.fallback_pt_path = Path(fallback_pt_path) if fallback_pt_path else None
        self.confidence = confidence
        self.iou = iou
        self.device = device
        self._model: Any = None
        self._dynamic_classes = False

    def detect(self, image: np.ndarray, prompts: Sequence[str]) -> list[Detection]:
        model = self._load(prompts)
        if self._dynamic_classes:
            model.set_classes(list(prompts))
        started = time.perf_counter()
        try:
            results = model.predict(
                image,
                conf=self.confidence,
                iou=self.iou,
                device=self.device,
                verbose=False,
            )
        except Exception as exc:
            logger.exception("YOLO-World 推理失败: model=%s", self.model_path)
            raise RuntimeError("YOLO-World 推理失败") from exc

        detections: list[Detection] = []
        wanted = {prompt.lower() for prompt in prompts}
        for result in results:
            names = result.names
            boxes = getattr(result, "boxes", None)
            if boxes is None:
                continue
            xyxy = boxes.xyxy.detach().cpu().numpy()
            conf = boxes.conf.detach().cpu().numpy()
            classes = boxes.cls.detach().cpu().numpy().astype(int)
            for bounds, score, class_index in zip(xyxy, conf, classes):
                class_name = str(names[class_index])
                if self._dynamic_classes or class_name.lower() in wanted:
                    detections.append(
                        Detection(
                            class_name=class_name,
                            confidence=float(score),
                            bbox_xyxy=tuple(round(v) for v in bounds),
                        )
                    )
        detections.sort(key=lambda item: item.confidence, reverse=True)
        logger.info(
            "YOLO-World 检测完成: prompts=%d matches=%d elapsed_ms=%.1f",
            len(prompts),
            len(detections),
            (time.perf_counter() - started) * 1000.0,
        )
        return detections

    def _load(self, prompts: Sequence[str]) -> Any:
        if self._model is not None:
            return self._model
        selected = self.model_path
        try:
            from ultralytics import YOLO, YOLOWorld

            if not selected.exists() and self.fallback_pt_path:
                selected = self.fallback_pt_path
            if not selected.exists():
                raise FileNotFoundError(selected)
            if (
                selected.suffix.lower() == ".engine"
                and importlib.util.find_spec("tensorrt") is None
            ):
                if self.fallback_pt_path is None or not self.fallback_pt_path.is_file():
                    raise RuntimeError(
                        "TensorRT Python 绑定不可用且没有 YOLO-World PT 回退"
                    )
                logger.warning(
                    "TensorRT Python 绑定不可用，回退 YOLO-World PT: "
                    "engine=%s fallback=%s",
                    selected,
                    self.fallback_pt_path,
                )
                selected = self.fallback_pt_path
            if selected.suffix == ".pt":
                self._model = YOLOWorld(str(selected))
                self._dynamic_classes = True
            else:
                self._model = YOLO(str(selected), task="detect")
                self._dynamic_classes = False
        except Exception as exc:
            logger.exception(
                "加载 YOLO-World 失败: primary=%s fallback=%s",
                self.model_path,
                self.fallback_pt_path,
            )
            raise RuntimeError("无法加载 YOLO-World 模型") from exc
        logger.info(
            "YOLO-World 已加载: model=%s dynamic_classes=%s prompts=%d",
            selected,
            self._dynamic_classes,
            len(prompts),
        )
        return self._model


class Sam2BoxSegmenter:
    """SAM2 box-prompt segmenter loaded from Thor's external checkout."""

    def __init__(
        self,
        repository: str | Path,
        checkpoint: str | Path,
        model_config: str,
        device: str = "cuda",
        artifact_dir: str | Path | None = None,
    ) -> None:
        self.repository = Path(repository)
        self.checkpoint = Path(checkpoint)
        self.model_config = model_config
        self.device = device
        self.artifact_dir = Path(artifact_dir) if artifact_dir else None
        self._predictor: Any = None

    def segment(self, image: np.ndarray, detection: Detection) -> np.ndarray:
        predictor = self._load()
        started = time.perf_counter()
        image_sha256 = pixel_sha256(image)
        try:
            predictor.set_image(image)
            masks, scores, _ = predictor.predict(
                point_coords=None,
                point_labels=None,
                box=np.asarray(detection.bbox_xyxy, dtype=np.float32),
                multimask_output=True,
            )
        except Exception as exc:
            logger.exception(
                "SAM2 分割失败: class=%s bbox=%s",
                detection.class_name,
                detection.bbox_xyxy,
            )
            raise RuntimeError("SAM2 分割失败") from exc
        if len(masks) == 0:
            raise RuntimeError("SAM2 未返回掩码")
        best_index = int(np.argmax(np.asarray(scores)))
        mask = np.asarray(masks[best_index], dtype=bool)
        artifact_paths: tuple[Path, Path, Path] | None = None
        if self.artifact_dir is not None:
            try:
                artifact_paths = save_segmentation_pngs(
                    self.artifact_dir,
                    image=image,
                    image_sha256=image_sha256,
                    bbox_xyxy=detection.bbox_xyxy,
                    mask=mask,
                )
            except Exception:
                logger.exception(
                    "保存 SAM2 诊断图片失败: artifact_dir=%s class=%s "
                    "bbox=%s image_sha256=%s",
                    self.artifact_dir,
                    detection.class_name,
                    detection.bbox_xyxy,
                    image_sha256,
                )
        logger.info(
            "SAM2 分割完成: class=%s bbox=%s image_shape=%s "
            "image_dtype=%s image_sha256=%s scores=%s selected=%d "
            "mask_bbox=%s pixels=%d score=%.3f elapsed_ms=%.1f",
            detection.class_name,
            detection.bbox_xyxy,
            image.shape,
            image.dtype,
            image_sha256,
            [round(float(score), 6) for score in scores],
            best_index,
            mask_bbox_xyxy(mask),
            int(np.count_nonzero(mask)),
            float(scores[best_index]),
            (time.perf_counter() - started) * 1000.0,
        )
        if artifact_paths is not None:
            logger.info(
                "SAM2 诊断图片已保存: image_sha256=%s bbox_path=%s "
                "mask_path=%s overlay_path=%s",
                image_sha256,
                *artifact_paths,
            )
        return mask

    def _load(self) -> Any:
        if self._predictor is not None:
            return self._predictor
        repository = str(self.repository.resolve())
        if repository not in sys.path:
            sys.path.insert(0, repository)
        try:
            from sam2.build_sam import build_sam2
            from sam2.sam2_image_predictor import SAM2ImagePredictor

            model = build_sam2(
                self.model_config,
                str(self.checkpoint),
                device=self.device,
            )
            self._predictor = SAM2ImagePredictor(model)
        except Exception as exc:
            logger.exception(
                "加载 SAM2 失败: repo=%s checkpoint=%s config=%s",
                self.repository,
                self.checkpoint,
                self.model_config,
            )
            raise RuntimeError("无法加载 SAM2") from exc
        logger.info(
            "SAM2 已加载: checkpoint=%s device=%s",
            self.checkpoint,
            self.device,
        )
        return self._predictor


class StaticDetector:
    """Deterministic detector for local replay tests."""

    def __init__(self, detections: list[Detection]) -> None:
        self.detections = detections

    def detect(self, image: np.ndarray, prompts: Sequence[str]) -> list[Detection]:
        del image, prompts
        return list(self.detections)


class StaticSegmenter:
    """Deterministic segmenter for local replay tests."""

    def __init__(self, mask: np.ndarray) -> None:
        self.mask = np.asarray(mask, dtype=bool)

    def segment(self, image: np.ndarray, detection: Detection) -> np.ndarray:
        del image, detection
        return self.mask.copy()
