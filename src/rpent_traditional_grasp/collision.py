"""Self-collision pre-filter that reuses the parent project's checker.

The parent project vetoes execution with a pinocchio/hpp-fcl full-body
self-collision check. Ranking candidates without that knowledge put a colliding
trajectory first on 2026-08-02 14:18 and the whole batch was rejected, so the
planner consults the very same checker while ranking. It lives in another
interpreter (only the parent's has pinocchio), hence a persistent subprocess
mirroring the TRAC-IK adapter rather than an import.
"""

from __future__ import annotations

import json
import selectors
import subprocess
import threading
from collections.abc import Callable
from pathlib import Path

import numpy as np

from rpent_traditional_grasp.logging import get_logger
from rpent_traditional_grasp.models import IKPath

logger = get_logger("collision")

WORKER_SCRIPT = Path(__file__).resolve().parents[2] / "scripts" / "collision_worker.py"


class SubprocessSelfCollisionChecker:
    """Run the parent's authoritative self-collision checker out of process."""

    def __init__(
        self,
        python_executable: str | Path,
        repo_root: str | Path,
        joint_state: Callable[[], np.ndarray],
        *,
        module: str = "robots.air_robot.collision",
        class_name: str = "PinocchioSelfCollisionChecker",
        urdf_path: str | Path = "",
        worker_script: str | Path = WORKER_SCRIPT,
        startup_timeout_s: float = 60.0,
        response_timeout_s: float = 30.0,
    ) -> None:
        self.python_executable = Path(python_executable)
        self.repo_root = Path(repo_root)
        self.joint_state = joint_state
        self.module = module
        self.class_name = class_name
        self.urdf_path = str(urdf_path or "")
        self.worker_script = Path(worker_script)
        self.startup_timeout_s = float(startup_timeout_s)
        self.response_timeout_s = float(response_timeout_s)
        self.metadata: dict[str, object] = {}
        self._process: subprocess.Popen[str] | None = None
        self._lock = threading.RLock()

    def check_path(self, path: IKPath) -> tuple[bool, str]:
        """Return whether the sampled joint path is self-collision free."""
        positions = [
            [float(value) for value in np.asarray(point, dtype=np.float64).ravel()]
            for point in path.positions
        ]
        current_q = np.asarray(self.joint_state(), dtype=np.float64).ravel()
        if current_q.shape != (14,) or not np.all(np.isfinite(current_q)):
            raise ValueError("自碰撞检查需要 14 个有限的当前关节角")
        request = {
            "arm_side": path.arm,
            "positions": positions,
            "current_q": [float(value) for value in current_q],
        }
        reply = self._request(json.dumps(request, ensure_ascii=False))
        if not reply.get("ok"):
            raise RuntimeError(f"自碰撞检查失败: {reply.get('error')}")
        result = reply.get("result")
        if not isinstance(result, dict):
            raise RuntimeError("自碰撞检查返回了非法结果")
        safe = bool(result.get("safe"))
        detail = str(result.get("detail", ""))
        if not safe:
            detail = (
                f"{detail} (sample {result.get('sample_index')}"
                f"/{result.get('sample_count')})"
            )
        return safe, detail

    def close(self) -> None:
        with self._lock:
            process = self._process
            self._process = None
            if process is None:
                return
            try:
                if process.stdin:
                    process.stdin.write("QUIT\n")
                    process.stdin.flush()
                process.wait(timeout=2.0)
            except (BrokenPipeError, OSError, subprocess.TimeoutExpired):
                process.terminate()
                try:
                    process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    process.kill()
            logger.info("自碰撞检查子进程已关闭")

    def __enter__(self) -> SubprocessSelfCollisionChecker:
        self._start()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def _start(self) -> subprocess.Popen[str]:
        if self._process is not None and self._process.poll() is None:
            return self._process
        if not self.python_executable.is_file():
            raise FileNotFoundError(
                f"自碰撞检查解释器不存在: {self.python_executable}"
            )
        if not self.worker_script.is_file():
            raise FileNotFoundError(f"自碰撞检查工作脚本不存在: {self.worker_script}")
        if not self.repo_root.is_dir():
            raise FileNotFoundError(f"父项目根目录不存在: {self.repo_root}")
        command = [
            str(self.python_executable),
            str(self.worker_script),
            "--repo",
            str(self.repo_root),
            "--module",
            self.module,
            "--class-name",
            self.class_name,
        ]
        if self.urdf_path:
            command += ["--urdf", self.urdf_path]
        try:
            process = subprocess.Popen(
                command,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
            )
        except OSError as exc:
            logger.exception(
                "启动自碰撞检查子进程失败: python=%s repo=%s",
                self.python_executable,
                self.repo_root,
            )
            raise RuntimeError("无法启动自碰撞检查子进程") from exc
        self._process = process
        # Building the pinocchio model and all collision pairs takes seconds.
        ready = self._readline(process, self.startup_timeout_s)
        if not ready.startswith("READY "):
            error = process.stderr.readline().strip() if process.stderr else ""
            self.close()
            raise RuntimeError(
                f"自碰撞检查握手失败: response={ready[:200]!r} stderr={error[:300]!r}"
            )
        self.metadata = json.loads(ready[6:])
        logger.info(
            "自碰撞检查子进程已启动: pid=%s checker=%s pairs=%s urdf_sha256=%s",
            process.pid,
            self.metadata.get("checker"),
            self.metadata.get("collision_pair_count"),
            str(self.metadata.get("urdf_sha256"))[:16],
        )
        return process

    def _request(self, payload: str) -> dict[str, object]:
        with self._lock:
            process = self._start()
            if process.stdin is None:
                raise RuntimeError("自碰撞检查 stdin 不可用")
            try:
                process.stdin.write(payload + "\n")
                process.stdin.flush()
                line = self._readline(process, self.response_timeout_s)
            except (BrokenPipeError, OSError) as exc:
                logger.exception("自碰撞检查通信失败")
                self.close()
                raise RuntimeError("自碰撞检查通信失败") from exc
            try:
                return json.loads(line)
            except json.JSONDecodeError as exc:
                raise RuntimeError(f"自碰撞检查返回非 JSON: {line[:200]!r}") from exc

    def _readline(self, process: subprocess.Popen[str], timeout_s: float) -> str:
        if process.stdout is None:
            raise RuntimeError("自碰撞检查 stdout 不可用")
        selector = selectors.DefaultSelector()
        selector.register(process.stdout, selectors.EVENT_READ)
        try:
            if not selector.select(timeout_s):
                self.close()
                raise TimeoutError(f"自碰撞检查响应超时 ({timeout_s:.1f}s)")
            line = process.stdout.readline()
        finally:
            selector.close()
        if not line:
            error = process.stderr.readline().strip() if process.stderr else ""
            raise RuntimeError(f"自碰撞检查子进程已退出: stderr={error[:300]!r}")
        return line.strip()


def build_collision_checker(
    resources: object,
    joint_state: Callable[[], np.ndarray],
) -> SubprocessSelfCollisionChecker | None:
    """Build the checker when configured, otherwise degrade to no pre-filter.

    A missing checker must not block planning: the parent still applies the
    authoritative gate, so the only loss is that ranking stays collision-blind.
    """
    python_executable = str(getattr(resources, "collision_checker_python", "") or "")
    repo_root = str(getattr(resources, "collision_checker_repo", "") or "")
    if not python_executable or not repo_root:
        logger.warning(
            "未配置自碰撞预筛（collision_checker_python/repo 为空），"
            "候选排序将对自碰撞无感，父项目仍会做权威校验"
        )
        return None
    checker = SubprocessSelfCollisionChecker(
        python_executable,
        repo_root,
        joint_state,
        module=str(getattr(resources, "collision_checker_module", "")
                   or "robots.air_robot.collision"),
        class_name=str(getattr(resources, "collision_checker_class", "")
                       or "PinocchioSelfCollisionChecker"),
        urdf_path=str(getattr(resources, "collision_urdf", "") or ""),
    )
    try:
        checker._start()
    except Exception:
        logger.exception(
            "自碰撞预筛启动失败，退化为无预筛: python=%s repo=%s",
            python_executable,
            repo_root,
        )
        checker.close()
        return None
    return checker
