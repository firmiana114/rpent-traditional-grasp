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


_CUDA_PROVIDER = "CUDAExecutionProvider"
# onnxruntime providers that actually run inference on local accelerators.
# AzureExecutionProvider is a remote-inference entry point, not an accelerator,
# and CPU-only builds still advertise it, so it must stay out of this set.
_ACCELERATED_PROVIDERS = frozenset(
    {
        "CUDAExecutionProvider",
        "TensorrtExecutionProvider",
        "NvTensorRTRTXExecutionProvider",
        "ROCMExecutionProvider",
        "MIGraphXExecutionProvider",
        "OpenVINOExecutionProvider",
        "CoreMLExecutionProvider",
        "DmlExecutionProvider",
        "QNNExecutionProvider",
    }
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
        self.execution_providers: list[str] = []

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
        self.execution_providers = self._ensure_execution_providers()
        return self._model

    def _ensure_execution_providers(self) -> list[str]:
        """Pin the session to an accelerator when one exists, else state the fallback.

        The deployment-provided CREStereo classes hardcode their own provider
        list, so a runtime without CUDA silently degrades to the CPU provider
        and costs roughly two orders of magnitude in latency. Whether CREStereo
        can use CUDA is decided by onnxruntime rather than by the Torch device
        string, so any device except an explicit ``cpu`` override queries the
        runtime and re-selects ``CUDAExecutionProvider`` on a session that landed
        on the CPU. It deliberately never forces TensorRT: that provider builds
        an engine on first use and the vendor class owns the engine cache path,
        so an already-active TensorRT session is left untouched.
        """
        session = getattr(self._model, "session", None)
        providers = getattr(session, "get_providers", None)
        if providers is None:
            logger.info(
                "CREStereo 后端未暴露 onnxruntime 会话，跳过执行提供者检查: "
                "module=%s class=%s device=%s",
                self.module_name,
                self.class_name,
                self.device,
            )
            return []
        try:
            active = list(providers())
        except Exception:
            logger.exception(
                "读取 CREStereo 执行提供者失败: module=%s class=%s",
                self.module_name,
                self.class_name,
            )
            return []
        if any(name in _ACCELERATED_PROVIDERS for name in active):
            logger.info(
                "CREStereo 使用加速执行提供者: active=%s device=%s",
                ",".join(active),
                self.device,
            )
            return active

        available: list[str] = []
        if self.device != "cpu":
            try:
                import onnxruntime

                available = list(onnxruntime.get_available_providers())
            except Exception:
                logger.exception("读取 onnxruntime 可用执行提供者失败")
            if _CUDA_PROVIDER in available:
                forced = [_CUDA_PROVIDER, "CPUExecutionProvider"]
                try:
                    session.set_providers(forced)
                    active = list(providers())
                except Exception:
                    logger.exception(
                        "切换 CREStereo 到 CUDA 失败，将继续使用当前提供者: "
                        "requested=%s model=%s",
                        ",".join(forced),
                        self.model_path,
                    )
                else:
                    logger.info(
                        "CREStereo 已强制切换到 CUDA: active=%s "
                        "vendor_default=CPU module=%s",
                        ",".join(active),
                        self.module_name,
                    )
                    return active

        logger.warning(
            "CREStereo 回退到纯 CPU 推理，深度耗时会高出约两个数量级: "
            "active=%s device=%s available=%s model=%s",
            ",".join(active) or "none",
            self.device,
            ",".join(available) or "unqueried",
            self.model_path,
        )
        return active


class TensorRTCREStereoBackend:
    """Run CREStereo from a prebuilt TensorRT engine, falling back on failure.

    ``ExternalCREStereoBackend`` reaches CUDA only if onnxruntime happens to
    ship an accelerated build, and the deployed planning interpreter ships a
    CPU-only one, which costs roughly two orders of magnitude per frame. A
    serialized engine bypasses onnxruntime entirely: it needs only the
    TensorRT runtime, which that interpreter does have.

    An engine is bound to the TensorRT major version and the GPU that built
    it, so it can never be assumed loadable. Every failure here therefore
    degrades to ``fallback`` rather than raising: a stale engine after a
    JetPack upgrade must slow grasping down, not break it.
    """

    def __init__(
        self,
        engine_path: str | Path,
        *,
        fallback: DisparityBackend | None = None,
        input_size: tuple[int, int] = (640, 480),
    ) -> None:
        self.engine_path = Path(engine_path)
        self.fallback = fallback
        self.input_width, self.input_height = input_size
        self.execution_providers: list[str] = []
        self._engine: Any = None
        self._context: Any = None
        self._bindings: dict[str, Any] = {}
        self._stream: Any = None
        self._cudart: Any = None
        self._output_name: str = ""
        self._load_failed = False

    def predict_disparity(
        self, left_rectified: np.ndarray, right_rectified: np.ndarray
    ) -> np.ndarray:
        if not self._load():
            if self.fallback is None:
                raise RuntimeError(
                    f"CREStereo TensorRT 引擎不可用且无回退后端: {self.engine_path}"
                )
            return np.asarray(
                self.fallback.predict_disparity(left_rectified, right_rectified)
            )
        started = time.perf_counter()
        left_tensor = self._prepare_input(left_rectified)
        right_tensor = self._prepare_input(right_rectified)
        try:
            output = self._infer(left_tensor, right_tensor)
        except Exception:
            logger.exception(
                "CREStereo TensorRT 推理失败，本帧改用回退后端: engine=%s",
                self.engine_path,
            )
            self._load_failed = True
            if self.fallback is None:
                raise
            return np.asarray(
                self.fallback.predict_disparity(left_rectified, right_rectified)
            )
        # CREStereo emits a two-channel flow field; horizontal disparity is
        # channel 0 and the vendor implementation discards channel 1.
        disparity = np.squeeze(output[:, 0, :, :])
        logger.info(
            "CREStereo TensorRT 推理完成: elapsed_ms=%.1f shape=%s",
            (time.perf_counter() - started) * 1000.0,
            "x".join(str(dim) for dim in disparity.shape),
        )
        return disparity

    def _prepare_input(self, image: np.ndarray) -> np.ndarray:
        """Match the vendor preprocessing byte for byte: BGR->RGB, NCHW, raw 0-255."""
        import cv2

        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        if rgb.shape[1] != self.input_width or rgb.shape[0] != self.input_height:
            rgb = cv2.resize(rgb, (self.input_width, self.input_height))
        return np.ascontiguousarray(
            rgb.transpose(2, 0, 1)[np.newaxis, :, :, :].astype(np.float32)
        )

    def _load(self) -> bool:
        if self._context is not None:
            return True
        if self._load_failed:
            return False
        try:
            self._build_context()
        except Exception:
            logger.exception(
                "加载 CREStereo TensorRT 引擎失败，将退回 ONNX 后端: engine=%s",
                self.engine_path,
            )
            self._load_failed = True
            return False
        return True

    def _build_context(self) -> None:
        import tensorrt as trt

        cudart = _import_cudart()
        if not self.engine_path.exists():
            raise FileNotFoundError(f"CREStereo 引擎不存在: {self.engine_path}")

        runtime = trt.Runtime(trt.Logger(trt.Logger.ERROR))
        engine = runtime.deserialize_cuda_engine(self.engine_path.read_bytes())
        if engine is None:
            # Almost always a TensorRT major-version change; the engine must be
            # rebuilt by scripts/build_crestereo_engine.py on this machine.
            raise RuntimeError(
                "引擎反序列化失败，通常是 TensorRT 版本或 GPU 与编译时不一致: "
                f"tensorrt={trt.__version__} engine={self.engine_path}"
            )
        context = engine.create_execution_context()
        if context is None:
            raise RuntimeError(f"无法创建 TensorRT 执行上下文: {self.engine_path}")

        status, stream = cudart.cudaStreamCreate()
        _check_cuda(status, "cudaStreamCreate")

        bindings: dict[str, Any] = {}
        inputs: list[str] = []
        output_name = ""
        for index in range(engine.num_io_tensors):
            name = engine.get_tensor_name(index)
            shape = tuple(engine.get_tensor_shape(name))
            dtype = np.dtype(trt.nptype(engine.get_tensor_dtype(name)))
            nbytes = int(np.prod(shape)) * dtype.itemsize
            status, device_ptr = cudart.cudaMalloc(nbytes)
            _check_cuda(status, f"cudaMalloc({name})")
            bindings[name] = {
                "ptr": device_ptr,
                "shape": shape,
                "dtype": dtype,
                "nbytes": nbytes,
            }
            context.set_tensor_address(name, int(device_ptr))
            if engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
                inputs.append(name)
            else:
                output_name = name

        if len(inputs) != 2 or not output_name:
            raise RuntimeError(
                "引擎接口与 CREStereo 不符，需要两个输入和一个输出: "
                f"inputs={inputs} output={output_name!r}"
            )

        self._engine = engine
        self._context = context
        self._bindings = bindings
        self._stream = stream
        self._cudart = cudart
        self._input_names = inputs
        self._output_name = output_name
        # The engine's own input geometry wins over the configured default:
        # a rebuild at another resolution must not silently feed wrong pixels.
        _, _, height, width = bindings[inputs[0]]["shape"]
        self.input_width, self.input_height = int(width), int(height)
        self.execution_providers = ["TensorRT"]
        logger.info(
            "CREStereo TensorRT 引擎已加载: engine=%s tensorrt=%s inputs=%s "
            "output=%s input_size=%dx%d",
            self.engine_path,
            trt.__version__,
            ",".join(inputs),
            output_name,
            self.input_width,
            self.input_height,
        )

    def _infer(self, left_tensor: np.ndarray, right_tensor: np.ndarray) -> np.ndarray:
        cudart = self._cudart
        kind_h2d = cudart.cudaMemcpyKind.cudaMemcpyHostToDevice
        kind_d2h = cudart.cudaMemcpyKind.cudaMemcpyDeviceToHost
        for name, tensor in zip(self._input_names, (left_tensor, right_tensor)):
            binding = self._bindings[name]
            status = cudart.cudaMemcpyAsync(
                binding["ptr"],
                tensor.ctypes.data,
                binding["nbytes"],
                kind_h2d,
                self._stream,
            )[0]
            _check_cuda(status, f"cudaMemcpyAsync(H2D,{name})")

        if not self._context.execute_async_v3(stream_handle=int(self._stream)):
            raise RuntimeError("TensorRT execute_async_v3 返回失败")

        out = self._bindings[self._output_name]
        host = np.empty(out["shape"], dtype=out["dtype"])
        status = cudart.cudaMemcpyAsync(
            host.ctypes.data, out["ptr"], out["nbytes"], kind_d2h, self._stream
        )[0]
        _check_cuda(status, "cudaMemcpyAsync(D2H,output)")
        _check_cuda(cudart.cudaStreamSynchronize(self._stream)[0], "cudaStreamSynchronize")
        return host

    def close(self) -> None:
        """Release device memory; safe to call more than once."""
        cudart = self._cudart
        if cudart is None:
            return
        for binding in self._bindings.values():
            cudart.cudaFree(binding["ptr"])
        if self._stream is not None:
            cudart.cudaStreamDestroy(self._stream)
        self._bindings = {}
        self._stream = None
        self._context = None
        self._engine = None


def _import_cudart() -> Any:
    """Return the cuda-python runtime module across its two import layouts."""
    try:
        from cuda.bindings import runtime as cudart
    except ImportError:  # cuda-python < 12.8 kept it at the top level.
        from cuda import cudart  # type: ignore[no-redef]
    return cudart


def _check_cuda(status: Any, operation: str) -> None:
    if int(status) != 0:
        raise RuntimeError(f"CUDA 调用失败: op={operation} status={int(status)}")


class StaticStereoSource:
    """Deterministic source used by local tests and recorded-frame replay."""

    def __init__(self, observation: StereoObservation) -> None:
        self.observation = observation

    def capture(self) -> StereoObservation:
        return self.observation
