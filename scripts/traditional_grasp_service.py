#!/usr/bin/env python3
"""Serve online stereo-to-joint-plan requests over JSON lines; never move robot."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
from contextlib import redirect_stdout
from pathlib import Path
from typing import TextIO

from rpent_traditional_grasp.execution import PlanningArmExecutor
from rpent_traditional_grasp.logging import configure_logging, get_logger
from rpent_traditional_grasp.service import TraditionalGraspPlanningService
from rpent_traditional_grasp.thor import build_thor_shadow_api

logger = get_logger("service_cli")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Persistent Thor traditional-grasp planner with no motion output."
    )
    parser.add_argument("--config", required=True)
    parser.add_argument("--host", default="192.168.123.164")
    parser.add_argument("--port", type=int, default=55555)
    parser.add_argument("--plan-ttl-s", type=float, default=15.0)
    # Both are required together: a half-configured pair would silently fall
    # back to the camera and the caller would never learn its images were unused.
    parser.add_argument(
        "--left-image",
        default=None,
        help=(
            "Read the left frame from this file instead of opening the camera. "
            "Used by the MuJoCo simulation, which renders the pair from another "
            "process. Requires --right-image."
        ),
    )
    parser.add_argument("--right-image", default=None)
    return parser


def _isolate_protocol_stdout() -> TextIO:
    """Reserve the original stdout for JSON and redirect fd 1 noise to stderr."""
    sys.stdout.flush()
    protocol_fd = os.dup(sys.stdout.fileno())
    protocol_stdout = os.fdopen(
        protocol_fd,
        "w",
        buffering=1,
        encoding="utf-8",
    )
    os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
    return protocol_stdout


def main() -> int:
    args = _parser().parse_args()
    if bool(args.left_image) != bool(args.right_image):
        raise SystemExit("--left-image 与 --right-image 必须成对提供")
    protocol_stdout = _isolate_protocol_stdout()
    configure_logging()
    root = Path(__file__).resolve().parents[1]
    config_path = Path(args.config).resolve()
    executor = PlanningArmExecutor()
    with redirect_stdout(sys.stderr):
        # Simulation renders the MuJoCo stereo pair to files because the
        # simulated world lives in the arm service process, not this one. The
        # camera stays the default so the robot path is untouched.
        use_images = bool(args.left_image and args.right_image)
        api = build_thor_shadow_api(
            config_path,
            host=args.host,
            port=args.port,
            left_image=args.left_image,
            right_image=args.right_image,
            online_camera=not use_images,
            planning_executor=executor,
        )
    service = TraditionalGraspPlanningService(
        api,
        executor,
        code_revision=_git_revision(root),
        config_sha256=hashlib.sha256(config_path.read_bytes()).hexdigest(),
        plan_ttl_s=args.plan_ttl_s,
    )
    logger.info(
        "传统抓取规划服务已启动: config=%s host=%s port=%d motion=False "
        "stdout_protocol_isolated=True",
        config_path,
        args.host,
        args.port,
    )
    for line in sys.stdin:
        should_close = False
        try:
            request = json.loads(line)
            should_close = request.get("operation") == "close"
            with redirect_stdout(sys.stderr):
                result = service.handle(request)
            response = {"success": True, "result": result}
        except Exception as exc:
            logger.exception("传统抓取规划服务请求失败")
            response = {
                "success": False,
                "error": f"{type(exc).__name__}: {exc}",
            }
        print(
            json.dumps(response, ensure_ascii=False),
            file=protocol_stdout,
            flush=True,
        )
        if should_close:
            protocol_stdout.close()
            return 0
    service.close()
    protocol_stdout.close()
    return 0


def _git_revision(root: Path) -> str:
    try:
        completed = subprocess.run(
            ["git", "-C", str(root), "rev-parse", "HEAD"],
            check=True,
            capture_output=True,
            text=True,
            timeout=2.0,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        logger.warning("无法读取 traditional_grasp Git 提交: %s", exc)
        return "unknown"
    return completed.stdout.strip() or "unknown"


if __name__ == "__main__":
    raise SystemExit(main())
