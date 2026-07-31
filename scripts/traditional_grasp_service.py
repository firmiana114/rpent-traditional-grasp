#!/usr/bin/env python3
"""Serve online stereo-to-joint-plan requests over JSON lines; never move robot."""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from contextlib import redirect_stdout
from pathlib import Path

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
    return parser


def main() -> int:
    args = _parser().parse_args()
    configure_logging()
    root = Path(__file__).resolve().parents[1]
    config_path = Path(args.config).resolve()
    executor = PlanningArmExecutor()
    with redirect_stdout(sys.stderr):
        api = build_thor_shadow_api(
            config_path,
            host=args.host,
            port=args.port,
            online_camera=True,
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
        "传统抓取规划服务已启动: config=%s host=%s port=%d motion=False",
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
        print(json.dumps(response, ensure_ascii=False), flush=True)
        if should_close:
            return 0
    service.close()
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
