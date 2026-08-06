"""YOLO-World and SAM2 adapters with lazy deployment imports."""

from __future__ import annotations

import importlib.util
import math
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
        self.last_artifacts: dict[str, str] = {}

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
        self.last_artifacts = {}
        if self.artifact_dir is not None:
            try:
                artifact_paths = save_segmentation_pngs(
                    self.artifact_dir,
                    image=image,
                    image_sha256=image_sha256,
                    bbox_xyxy=detection.bbox_xyxy,
                    mask=mask,
                )
                self.last_artifacts = {
                    "bbox_image_path": str(artifact_paths[0]),
                    "mask_image_path": str(artifact_paths[1]),
                    "overlay_image_path": str(artifact_paths[2]),
                    "result_image_path": str(artifact_paths[2]),
                }
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


class VlmDetector:
    """Open-vocabulary detector backed by a vision-language model over HTTP.

    YOLO-World cannot tell one drink from another here. Measured on the field
    scene with three bottles (cola lying down, Fanta and Sprite upright), over
    ten runs each: YOLO located all three boxes well but labelled every one of
    them ``bottle``, and the cola ranked third by confidence, so picking the
    top-scoring detection selected the wrong bottle every single time -- 0/10.
    Naming the brands did not help; with only brand classes offered it labelled
    all three ``fanta bottle``, and Chinese class names produced no detections
    at all because the CLIP text encoder behind YOLO-World is English-only.
    The same VLM on the same image scored 10/10 at IoU 0.96.

    The trade is latency: about 1.6 s against YOLO's 8 ms. That is affordable
    because grounding runs once per pick, not once per perception cycle, and
    grasping the wrong bottle costs far more than a second.
    """

    #: Qwen reports boxes on a 0-1000 grid rather than in pixels.
    _NORMALIZED_GRID = 1000.0
    COORDINATE_SPACES = ("normalized_1000", "pixel", "auto")

    def __init__(
        self,
        endpoint: str,
        model: str,
        *,
        timeout_s: float = 60.0,
        max_tokens: int = 128,
        coordinate_space: str = "normalized_1000",
        weights_path: str | Path | None = None,
    ) -> None:
        if coordinate_space not in self.COORDINATE_SPACES:
            raise ValueError(
                "coordinate_space 必须是 " + "、".join(self.COORDINATE_SPACES)
            )
        self.endpoint = endpoint.rstrip("/")
        self.model = model
        self.timeout_s = float(timeout_s)
        self.max_tokens = int(max_tokens)
        self.coordinate_space = coordinate_space
        self.weights_path = str(weights_path) if weights_path else None

    def detect(self, image: np.ndarray, prompts: Sequence[str]) -> list[Detection]:
        """Ground the first prompt; later prompts are generic fallbacks.

        ``prompts`` arrives as the task target followed by generic words like
        "bottle". Those extras exist to give YOLO-World something inside its
        vocabulary to match, and handing them to a VLM only invites it to
        return whatever bottle it sees first, which is the failure being fixed.
        """
        if not prompts:
            return []
        target = str(prompts[0]).strip()
        if not target:
            return []
        height, width = int(image.shape[0]), int(image.shape[1])
        started = time.perf_counter()
        try:
            payload = self._request(image, target, width, height)
        except Exception:
            logger.exception(
                "VLM 检测请求失败: endpoint=%s model=%s target=%s",
                self.endpoint,
                self.model,
                target,
            )
            return []
        boxes = self._parse(payload, width, height)
        logger.info(
            "VLM 检测完成: target=%s matches=%d elapsed_ms=%.1f endpoint=%s",
            target,
            len(boxes),
            (time.perf_counter() - started) * 1000.0,
            self.endpoint,
        )
        if not boxes:
            logger.warning(
                "VLM 未框出目标: target=%s 原始回复=%s",
                target,
                str(payload)[:200],
            )
        # A VLM reports no score. 1.0 keeps the downstream "highest confidence
        # first" ordering meaningful for the single box it returns; it is not a
        # calibrated probability and must not be compared against YOLO scores.
        return [
            Detection(class_name=target, confidence=1.0, bbox_xyxy=box)
            for box in boxes
        ]

    def _request(
        self, image: np.ndarray, target: str, width: int, height: int
    ) -> str:
        import base64
        import json
        import urllib.request

        try:
            import cv2
        except ImportError as exc:  # pragma: no cover - deployment dependency
            raise RuntimeError("VLM 检测需要安装 OpenCV 以编码图片") from exc

        ok, buffer = cv2.imencode(".png", np.asarray(image))
        if not ok:
            raise RuntimeError("VLM 检测图片编码失败")
        encoded = base64.b64encode(buffer.tobytes()).decode("ascii")
        instruction = (
            f"The image is {width}x{height}. Find exactly one object: {target}. "
            'Output ONLY compact JSON {"bbox":[x1,y1,x2,y2]} with no explanation. '
            'If it is absent output {"bbox":null}.'
        )
        body = {
            "model": self.model,
            "temperature": 0,
            "max_tokens": self.max_tokens,
            # Left on, the model narrates its reasoning and exhausts the token
            # budget before it ever emits the JSON.
            "chat_template_kwargs": {"enable_thinking": False},
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": "data:image/png;base64," + encoded
                            },
                        },
                        {"type": "text", "text": instruction},
                    ],
                }
            ],
        }
        request = urllib.request.Request(
            self.endpoint + "/chat/completions",
            data=json.dumps(body).encode("utf-8"),
            headers={"Content-Type": "application/json"},
        )
        with urllib.request.urlopen(request, timeout=self.timeout_s) as response:
            parsed = json.loads(response.read().decode("utf-8"))
        return parsed["choices"][0]["message"]["content"]

    def _parse(
        self, content: str, width: int, height: int
    ) -> list[tuple[int, int, int, int]]:
        import json

        text = str(content or "")
        try:
            start, end = text.index("{"), text.rindex("}")
        except ValueError:
            return []
        try:
            data = json.loads(text[start : end + 1])
        except json.JSONDecodeError:
            return []
        # The key name is not stable across replies: the same model and prompt
        # has returned "bbox" and "bbox_2d" on consecutive calls.
        raw = None
        for key in ("bbox", "bbox_2d", "box", "boxes"):
            if isinstance(data, dict) and data.get(key):
                raw = data[key]
                break
        if raw is None:
            return []
        if raw and isinstance(raw[0], (list, tuple)):
            raw = raw[0]
        if len(raw) != 4:
            return []
        try:
            values = [float(v) for v in raw]
        except (TypeError, ValueError):
            return []
        if not all(math.isfinite(v) for v in values):
            return []
        # Which space the numbers are in cannot be recovered from the numbers:
        # the measured reply [316,369,463,431] is a valid 640x480 pixel box AND
        # the correct answer once scaled from the 0-1000 grid, and only the
        # scaled reading matches the bottle. So the space is configured, not
        # guessed. The deployed Qwen3.5 emits the 0-1000 grid, hence the
        # default; "auto" exists only for models whose convention is unknown
        # and merely rules out values that overflow the grid.
        space = self.coordinate_space
        if space == "auto":
            space = "pixel" if max(values) > self._NORMALIZED_GRID else "normalized_1000"
        if space == "normalized_1000":
            scale = (width, height, width, height)
            values = [v * s / self._NORMALIZED_GRID for v, s in zip(values, scale)]
        x1, y1, x2, y2 = (int(round(v)) for v in values)
        x1, x2 = sorted((max(0, min(x1, width)), max(0, min(x2, width))))
        y1, y2 = sorted((max(0, min(y1, height)), max(0, min(y2, height))))
        if x2 - x1 < 2 or y2 - y1 < 2:
            return []
        return [(x1, y1, x2, y2)]


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
