"""Thor-specific adapters kept outside the upper RPent Git repository."""

from __future__ import annotations

import json
import os
import sys
import time
from collections.abc import Callable
from pathlib import Path
from typing import Any

import numpy as np

from rpent_traditional_grasp.api import TraditionalGraspAPI
from rpent_traditional_grasp.config import TraditionalGraspConfig
from rpent_traditional_grasp.execution import ArmExecutor, MockArmExecutor
from rpent_traditional_grasp.ik import MockIKSolver, TracIKProcess
from rpent_traditional_grasp.image_trace import pixel_sha256, save_stereo_pngs
from rpent_traditional_grasp.logging import get_logger
from rpent_traditional_grasp.models import IKPath
from rpent_traditional_grasp.perception import Sam2BoxSegmenter, YoloWorldDetector
from rpent_traditional_grasp.stereo import (
    ExternalCREStereoBackend,
    RectifiedStereoPipeline,
    StereoCalibration,
)

logger = get_logger("thor")


class ImagePairStereoCamera:
    """Load one explicit left/right image pair without opening a camera."""

    def __init__(
        self,
        left_image_path: str | Path,
        right_image_path: str | Path,
    ) -> None:
        self.left_image_path = Path(left_image_path)
        self.right_image_path = Path(right_image_path)

    def capture_stereo(self) -> tuple[np.ndarray, np.ndarray, float]:
        try:
            import cv2
        except ImportError as exc:
            logger.exception("读取离线双目图片需要安装 OpenCV")
            raise RuntimeError("读取离线双目图片需要安装 OpenCV") from exc

        missing = [
            str(path)
            for path in (self.left_image_path, self.right_image_path)
            if not path.is_file()
        ]
        if missing:
            raise FileNotFoundError("离线双目图片不存在: " + ", ".join(missing))
        left = cv2.imread(str(self.left_image_path), cv2.IMREAD_COLOR)
        right = cv2.imread(str(self.right_image_path), cv2.IMREAD_COLOR)
        if left is None or right is None:
            logger.error(
                "离线双目图片解码失败: left=%s right=%s",
                self.left_image_path,
                self.right_image_path,
            )
            raise ValueError("离线双目图片解码失败")
        if left.shape != right.shape:
            raise ValueError(
                f"左右目图片尺寸不一致: left={left.shape} right={right.shape}"
            )
        timestamp_s = max(
            self.left_image_path.stat().st_mtime,
            self.right_image_path.stat().st_mtime,
        )
        logger.info(
            "离线双目图片已加载: left=%s right=%s size=%dx%d",
            self.left_image_path,
            self.right_image_path,
            left.shape[1],
            left.shape[0],
        )
        return left, right, float(timestamp_s)


class ThorStereoCamera:
    """Capture the side-by-side BGR stream already used on Thor."""

    def __init__(
        self,
        host: str = "192.168.123.164",
        port: int = 55555,
        teleimager_source: str | Path = (
            "/home/aiot/fuchengjia/Projects/xr_teleoperate/" "teleop/teleimager/src"
        ),
        capture_timeout_s: float = 2.0,
        poll_interval_s: float = 0.01,
        artifact_dir: str | Path | None = None,
    ) -> None:
        self.host = host
        self.port = int(port)
        self.teleimager_source = Path(teleimager_source)
        self.capture_timeout_s = float(capture_timeout_s)
        self.poll_interval_s = float(poll_interval_s)
        self.artifact_dir = Path(artifact_dir) if artifact_dir else None
        if self.capture_timeout_s <= 0.0:
            raise ValueError("capture_timeout_s 必须大于零")
        if self.poll_interval_s <= 0.0:
            raise ValueError("poll_interval_s 必须大于零")
        self._subscriber: Any = None

    def capture_stereo(self) -> tuple[np.ndarray, np.ndarray, float]:
        if self._subscriber is None:
            source = str(self.teleimager_source)
            if source not in sys.path:
                sys.path.insert(0, source)
            try:
                from teleimager.image_client import ZMQ_SubscriberManager

                self._subscriber = ZMQ_SubscriberManager.get_instance()
            except Exception as exc:
                logger.exception(
                    "初始化 Thor 双目订阅失败: host=%s port=%d source=%s",
                    self.host,
                    self.port,
                    self.teleimager_source,
                )
                raise RuntimeError("无法初始化 Thor 双目订阅") from exc
        deadline = time.monotonic() + self.capture_timeout_s
        attempts = 0
        image = None
        while image is None:
            attempts += 1
            frame = self._subscriber.subscribe(self.host, self.port, request_bgr=True)
            image = getattr(frame, "bgr", None) if frame is not None else None
            if image is not None:
                break
            remaining_s = deadline - time.monotonic()
            if remaining_s <= 0.0:
                logger.warning(
                    "Thor 双目流取帧超时: host=%s port=%d timeout_s=%.2f "
                    "attempts=%d",
                    self.host,
                    self.port,
                    self.capture_timeout_s,
                    attempts,
                )
                raise RuntimeError(f"Thor 双目流未返回图像: {self.host}:{self.port}")
            time.sleep(min(self.poll_interval_s, remaining_s))
        image = np.asarray(image)
        if image.ndim != 3 or image.shape[1] < 8 or image.shape[1] % 2:
            raise ValueError(f"Thor 双目拼接图尺寸无效: {image.shape}")
        midpoint = image.shape[1] // 2
        timestamp_s = time.time()
        left = image[:, :midpoint]
        right = image[:, midpoint:]
        left_sha256 = pixel_sha256(left)
        right_sha256 = pixel_sha256(right)
        logger.info(
            "Thor 双目帧已采集: size=%dx%d attempts=%d timestamp_s=%.6f "
            "left_sha256=%s right_sha256=%s",
            midpoint,
            image.shape[0],
            attempts,
            timestamp_s,
            left_sha256,
            right_sha256,
        )
        if self.artifact_dir is not None:
            try:
                left_path, right_path = save_stereo_pngs(
                    self.artifact_dir,
                    stage="raw",
                    timestamp_s=timestamp_s,
                    left=left,
                    right=right,
                )
            except Exception:
                logger.exception(
                    "保存 Thor 原始双目帧失败: artifact_dir=%s "
                    "timestamp_s=%.6f left_sha256=%s right_sha256=%s",
                    self.artifact_dir,
                    timestamp_s,
                    left_sha256,
                    right_sha256,
                )
            else:
                logger.info(
                    "Thor 原始双目帧已保存: timestamp_s=%.6f "
                    "left_path=%s right_path=%s left_sha256=%s "
                    "right_sha256=%s",
                    timestamp_s,
                    left_path,
                    right_path,
                    left_sha256,
                    right_sha256,
                )
        return left, right, timestamp_s


class InjectedCapXExecutor:
    """Use an already-owned CapX controller without constructing or homing it.

    ``capx.arm.controller.ArmController`` homes in its constructor on the
    currently deployed Thor version. This adapter deliberately requires a
    controller created and authorized by the hosting process.
    """

    def __init__(
        self,
        controller: Any,
        *,
        min_point_duration_s: float = 0.12,
    ) -> None:
        required = {
            "get_state",
            "move_joints",
            "open_gripper",
            "close_gripper",
        }
        missing = sorted(name for name in required if not hasattr(controller, name))
        if missing:
            raise ValueError("注入的 CapX 控制器缺少方法: " + ", ".join(missing))
        self.controller = controller
        self.min_point_duration_s = float(min_point_duration_s)

    @property
    def is_hardware(self) -> bool:
        return True

    def current_joints(self, arm: str) -> np.ndarray:
        state = self.controller.get_state()
        if not isinstance(state, dict) or not state.get("success"):
            raise RuntimeError(f"读取 CapX 关节状态失败: {state}")
        current = np.asarray(state.get("current_q"), dtype=np.float64)
        if current.shape != (14,):
            raise RuntimeError(f"CapX current_q 尺寸无效: {current.shape}")
        return current[:7].copy() if arm == "left" else current[7:].copy()

    def execute_joint_path(self, path: IKPath) -> None:
        for index, joints in enumerate(path.positions):
            result = self.controller.move_joints(
                joints,
                arm_side=path.arm,
                duration=self.min_point_duration_s,
            )
            if not isinstance(result, dict) or not result.get("success"):
                raise RuntimeError(
                    f"CapX 关节点执行失败: arm={path.arm} "
                    f"index={index} result={result}"
                )
        logger.info(
            "CapX 七轴路径执行完成: arm=%s points=%d",
            path.arm,
            len(path.positions),
        )

    def close_gripper(self, arm: str) -> bool:
        result = self.controller.close_gripper(
            arm, require_contact=True, settle_time=3.0
        )
        if not isinstance(result, dict) or not result.get("success"):
            return False
        return True

    def open_gripper(self, arm: str) -> None:
        result = self.controller.open_gripper(arm, settle_time=0.5)
        if not isinstance(result, dict) or not result.get("success"):
            raise RuntimeError(f"CapX 夹爪张开失败: arm={arm} result={result}")


class CallbackCollisionChecker:
    """Connect a deployment-owned collision checker without hiding its result."""

    def __init__(self, callback: Callable[[IKPath], tuple[bool, str]]) -> None:
        self.callback = callback

    def check_path(self, path: IKPath) -> tuple[bool, str]:
        return self.callback(path)


def select_perception_device() -> tuple[str, int | str]:
    """Pick the perception device so Thor keeps CUDA and other hosts still run.

    Returns the Torch device string used by SAM2 and the external CREStereo
    adapter, plus the matching Ultralytics device argument. Set
    ``RPENT_TRADITIONAL_GRASP_DEVICE`` to force ``cuda``/``mps``/``cpu``.
    """
    override = (os.getenv("RPENT_TRADITIONAL_GRASP_DEVICE") or "").strip().lower()
    if override:
        if override not in {"cuda", "mps", "cpu"}:
            raise ValueError(
                "RPENT_TRADITIONAL_GRASP_DEVICE 必须是 cuda、mps 或 cpu，"
                f"实际为 {override}"
            )
        selected = override
        reason = "environment_override"
    else:
        try:
            import torch

            if torch.cuda.is_available():
                selected, reason = "cuda", "torch.cuda.is_available"
            elif torch.backends.mps.is_available():
                selected, reason = "mps", "torch.backends.mps.is_available"
            else:
                selected, reason = "cpu", "no_accelerator_detected"
        except Exception:
            logger.exception("探测 Torch 加速设备失败，回退 CPU")
            selected, reason = "cpu", "torch_probe_failed"
    ultralytics_device: int | str = 0 if selected == "cuda" else selected
    logger.info(
        "感知设备已选择: device=%s ultralytics_device=%s reason=%s",
        selected,
        ultralytics_device,
        reason,
    )
    return selected, ultralytics_device


def build_thor_shadow_api(
    config_path: str | Path,
    *,
    host: str = "192.168.123.164",
    port: int = 55555,
    left_image: str | Path | None = None,
    right_image: str | Path | None = None,
    online_camera: bool = False,
    perception_only: bool = False,
    planning_executor: ArmExecutor | None = None,
) -> TraditionalGraspAPI:
    """Build the real Thor perception stack with simulated, no-motion arms."""
    path = Path(config_path).resolve()
    config = TraditionalGraspConfig.from_json(path)
    if config.safety.mode == "live" or config.safety.allow_motion:
        raise ValueError("shadow 构建器拒绝 live 或 allow_motion 配置")

    resources = config.resources
    artifact_dir = os.getenv("RPENT_TRADITIONAL_GRASP_ARTIFACT_DIR")
    stereo_calibration_path = _resolve(resources.stereo_calibration, path.parent)
    transform_path = _resolve(resources.camera_to_body, path.parent)
    calibration = StereoCalibration.from_json(stereo_calibration_path)
    camera_to_body = load_transform(transform_path)
    has_left = left_image is not None
    has_right = right_image is not None
    if has_left != has_right:
        raise ValueError("left_image 与 right_image 必须同时提供")
    if online_camera and has_left:
        raise ValueError("在线相机与离线图片模式不能同时启用")
    if has_left:
        camera = ImagePairStereoCamera(
            _resolve(left_image, path.parent),
            _resolve(right_image, path.parent),
        )
        source_name = "image_pair"
    elif online_camera:
        camera = ThorStereoCamera(
            host=host,
            port=port,
            artifact_dir=artifact_dir,
        )
        source_name = "online_camera"
    else:
        raise ValueError(
            "必须提供 left_image/right_image；在线采集需显式设置 online_camera"
        )
    torch_device, ultralytics_device = select_perception_device()
    disparity = ExternalCREStereoBackend(
        resources.crestereo_repo,
        resources.crestereo_model,
        module_name=resources.crestereo_module,
        class_name=resources.crestereo_class,
        device=torch_device,
    )
    stereo_source = RectifiedStereoPipeline(
        camera,
        disparity,
        calibration,
        artifact_dir=(
            os.getenv("RPENT_TRADITIONAL_GRASP_ARTIFACT_DIR") if online_camera else None
        ),
    )
    detector = YoloWorldDetector(
        resources.yolo_model,
        resources.yolo_pt_model,
        confidence=config.perception.detection_confidence,
        iou=config.perception.detection_iou,
        device=ultralytics_device,
    )
    segmenter = Sam2BoxSegmenter(
        resources.sam2_repo,
        resources.sam2_checkpoint,
        resources.sam2_config,
        device=torch_device,
        artifact_dir=artifact_dir,
    )
    if perception_only:
        ik_solvers = {
            "left": MockIKSolver("left"),
            "right": MockIKSolver("right"),
        }
        ik_backend = "mock_perception_only"
    else:
        project_root = Path(__file__).resolve().parents[2]
        left_binary = _resolve(resources.left_ik_binary, project_root)
        right_binary = _resolve(resources.right_ik_binary, project_root)
        ik_solvers = {
            "left": TracIKProcess(
                left_binary,
                project_root / "robot/chains/g1_left_arm.chain",
                "left",
                config.planner.ik_timeout_s,
                config.planner.ik_tolerance,
            ),
            "right": TracIKProcess(
                right_binary,
                project_root / "robot/chains/g1_right_arm.chain",
                "right",
                config.planner.ik_timeout_s,
                config.planner.ik_tolerance,
            ),
        }
        ik_backend = "trac_ik"
    logger.info(
        "Thor shadow 栈已构建: config=%s source=%s ik=%s host=%s port=%d",
        path,
        source_name,
        ik_backend,
        host,
        port,
    )
    return TraditionalGraspAPI(
        config=config,
        stereo_source=stereo_source,
        detector=detector,
        segmenter=segmenter,
        ik_solvers=ik_solvers,
        executor=planning_executor or MockArmExecutor(contact_detected=False),
        camera_to_body=camera_to_body,
        collision_checker=None,
    )


def load_transform(path: str | Path) -> np.ndarray:
    """Load and validate a 4x4 transform JSON artifact."""
    transform_path = Path(path)
    try:
        raw = json.loads(transform_path.read_text(encoding="utf-8"))
        transform = np.asarray(raw.get("transform", raw), dtype=np.float64)
    except (OSError, ValueError, TypeError, json.JSONDecodeError) as exc:
        logger.exception("读取相机外参失败: path=%s", transform_path)
        raise ValueError(f"无效相机外参: {transform_path}") from exc
    if transform.shape != (4, 4):
        raise ValueError(f"相机外参必须是 4x4: shape={transform.shape}")
    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-4):
        raise ValueError("相机外参旋转矩阵不正交")
    if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-8):
        raise ValueError("相机外参齐次末行无效")
    logger.info("相机外参已加载: path=%s", transform_path)
    return transform


def _resolve(value: str | Path, base: Path) -> Path:
    path = Path(value).expanduser()
    return path if path.is_absolute() else (base / path).resolve()
