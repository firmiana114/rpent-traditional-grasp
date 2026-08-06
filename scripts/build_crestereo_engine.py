#!/usr/bin/env python3
"""Compile the CREStereo ONNX into a TensorRT engine for this exact runtime.

A serialized engine is not portable: it is tied to the TensorRT major version
and the GPU it was built on. Shipping one in the repository would therefore be
useless at best and silently wrong at worst, so the engine is always built on
the target machine by the very interpreter that will later deserialize it.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

# Operators run this by hand after deploying a new runtime, outside the service
# launcher that normally puts the package on PYTHONPATH.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from rpent_traditional_grasp.logging import configure_logging, get_logger

logger = get_logger("build_crestereo_engine")

# The deployed CREStereo (init_iter5) takes a rectified pair at a fixed size,
# so the engine needs no optimization profile beyond the static shapes baked
# into the ONNX graph.
_DEFAULT_WORKSPACE_MB = 4096


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Build the CREStereo TensorRT engine used by pick_object."
    )
    parser.add_argument(
        "--config",
        default=None,
        help="Traditional-grasp JSON config; supplies the ONNX and engine paths.",
    )
    parser.add_argument("--onnx", default=None, help="Override the ONNX source path.")
    parser.add_argument("--engine", default=None, help="Override the engine output path.")
    parser.add_argument(
        "--workspace-mb",
        type=int,
        default=_DEFAULT_WORKSPACE_MB,
        help="Scratch memory TensorRT may use while searching for kernels.",
    )
    parser.add_argument(
        "--fp32",
        action="store_true",
        help="Build without FP16. Slower, but useful when comparing accuracy.",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Rebuild even when the engine already exists and looks current.",
    )
    return parser


def _resolve_paths(args: argparse.Namespace) -> tuple[Path, Path]:
    onnx = args.onnx
    engine = args.engine
    if (onnx is None or engine is None) and args.config:
        payload = json.loads(Path(args.config).read_text(encoding="utf-8"))
        resources = payload.get("resources", {})
        onnx = onnx or resources.get("crestereo_model")
        engine = engine or resources.get("crestereo_engine")
    if not onnx:
        raise SystemExit("需要 --onnx 或配置中的 resources.crestereo_model")
    if not engine:
        raise SystemExit("需要 --engine 或配置中的 resources.crestereo_engine")
    return Path(onnx), Path(engine)


def build(
    onnx_path: Path,
    engine_path: Path,
    *,
    workspace_mb: int = _DEFAULT_WORKSPACE_MB,
    fp16: bool = True,
) -> Path:
    """Serialize ``onnx_path`` into ``engine_path`` and return the engine path."""
    import tensorrt as trt

    if not onnx_path.exists():
        raise FileNotFoundError(f"CREStereo ONNX 不存在: {onnx_path}")

    logger.info(
        "开始编译 CREStereo TensorRT 引擎: onnx=%s engine=%s tensorrt=%s "
        "fp16=%s workspace_mb=%d",
        onnx_path,
        engine_path,
        trt.__version__,
        fp16,
        workspace_mb,
    )
    started = time.perf_counter()

    trt_logger = trt.Logger(trt.Logger.WARNING)
    builder = trt.Builder(trt_logger)
    # Explicit batch became the only mode in TensorRT 10 and the flag was
    # dropped in 11, so it is set only where it still exists.
    explicit_batch = getattr(
        trt.NetworkDefinitionCreationFlag, "EXPLICIT_BATCH", None
    )
    flags = 0 if explicit_batch is None else 1 << int(explicit_batch)
    network = builder.create_network(flags)
    parser = trt.OnnxParser(network, trt_logger)
    if not parser.parse(onnx_path.read_bytes()):
        for index in range(parser.num_errors):
            logger.error("ONNX 解析错误 %d: %s", index, parser.get_error(index))
        raise RuntimeError(f"无法解析 CREStereo ONNX: {onnx_path}")

    config = builder.create_builder_config()
    config.set_memory_pool_limit(
        trt.MemoryPoolType.WORKSPACE, workspace_mb * 1024 * 1024
    )
    if fp16:
        # TensorRT 11 builds strongly-typed networks only: precision is taken
        # from the ONNX graph and the FP16 builder flag no longer exists. A
        # FP32 graph therefore yields a FP32 engine, which is still two orders
        # of magnitude faster than the CPU onnxruntime this replaces.
        fp16_flag = getattr(trt.BuilderFlag, "FP16", None)
        if fp16_flag is None:
            logger.info(
                "TensorRT %s 为强类型构建，精度由 ONNX 图决定，忽略 fp16 请求",
                trt.__version__,
            )
        else:
            if not getattr(builder, "platform_has_fast_fp16", True):
                logger.warning("当前平台无快速 FP16，仍按请求启用，可能不会更快")
            config.set_flag(fp16_flag)

    for index in range(network.num_inputs):
        tensor = network.get_input(index)
        logger.info("引擎输入: name=%s shape=%s dtype=%s", tensor.name, tensor.shape, tensor.dtype)
    for index in range(network.num_outputs):
        tensor = network.get_output(index)
        logger.info("引擎输出: name=%s shape=%s dtype=%s", tensor.name, tensor.shape, tensor.dtype)

    blob = builder.build_serialized_network(network, config)
    if blob is None:
        raise RuntimeError("TensorRT 引擎编译失败，未产出序列化结果")

    engine_path.parent.mkdir(parents=True, exist_ok=True)
    # Write through a sibling temporary file: a half-written engine that a
    # planning service picks up would fail deserialization at grasp time.
    staging = engine_path.with_suffix(engine_path.suffix + ".partial")
    staging.write_bytes(bytes(blob))
    os.replace(staging, engine_path)

    logger.info(
        "CREStereo TensorRT 引擎已生成: engine=%s size_mb=%.1f elapsed_s=%.1f",
        engine_path,
        engine_path.stat().st_size / 1e6,
        time.perf_counter() - started,
    )
    return engine_path


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    configure_logging()
    onnx_path, engine_path = _resolve_paths(args)
    if engine_path.exists() and not args.force:
        logger.info(
            "引擎已存在，跳过编译（用 --force 强制重建）: engine=%s size_mb=%.1f",
            engine_path,
            engine_path.stat().st_size / 1e6,
        )
        return 0
    build(
        onnx_path,
        engine_path,
        workspace_mb=args.workspace_mb,
        fp16=not args.fp32,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
