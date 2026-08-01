from __future__ import annotations

import logging
from pathlib import Path

import pytest

from rpent_traditional_grasp.stereo import ExternalCREStereoBackend

_FAKE_BACKEND_TEMPLATE = """
class _FakeSession:
    def get_providers(self):
        return {providers!r}


class CREStereo:
    def __init__(self, model_path, device="cuda"):
        self.model_path = model_path
        self.device = device
        self.session = _FakeSession()
"""


def _build_backend(
    tmp_path: Path,
    module_name: str,
    providers: list[str],
) -> ExternalCREStereoBackend:
    repository = tmp_path / "vendor"
    repository.mkdir()
    (repository / f"{module_name}.py").write_text(
        _FAKE_BACKEND_TEMPLATE.format(providers=providers),
        encoding="utf-8",
    )
    return ExternalCREStereoBackend(
        repository,
        tmp_path / "model.onnx",
        module_name=module_name,
        class_name="CREStereo",
    )


def test_accelerated_providers_are_logged_without_warning(
    tmp_path: Path,
    caplog: pytest.LogCaptureFixture,
) -> None:
    backend = _build_backend(
        tmp_path,
        "fake_crestereo_gpu",
        ["TensorrtExecutionProvider", "CUDAExecutionProvider", "CPUExecutionProvider"],
    )
    with caplog.at_level(logging.INFO, logger="rpent_traditional_grasp.stereo"):
        backend._load()
    assert "TensorrtExecutionProvider" in caplog.text
    assert not [record for record in caplog.records if record.levelno >= logging.WARNING]


def test_cpu_only_providers_emit_fallback_warning(
    tmp_path: Path,
    caplog: pytest.LogCaptureFixture,
) -> None:
    backend = _build_backend(
        tmp_path,
        "fake_crestereo_cpu",
        ["AzureExecutionProvider", "CPUExecutionProvider"],
    )
    with caplog.at_level(logging.INFO, logger="rpent_traditional_grasp.stereo"):
        backend._load()
    warnings = [
        record for record in caplog.records if record.levelno == logging.WARNING
    ]
    assert len(warnings) == 1
    assert "回退到纯 CPU 推理" in warnings[0].getMessage()


def test_backend_without_session_skips_provider_check(
    tmp_path: Path,
    caplog: pytest.LogCaptureFixture,
) -> None:
    repository = tmp_path / "vendor"
    repository.mkdir()
    (repository / "fake_crestereo_bare.py").write_text(
        "class CREStereo:\n"
        "    def __init__(self, model_path, device='cuda'):\n"
        "        self.model_path = model_path\n",
        encoding="utf-8",
    )
    backend = ExternalCREStereoBackend(
        repository,
        tmp_path / "model.onnx",
        module_name="fake_crestereo_bare",
        class_name="CREStereo",
    )
    with caplog.at_level(logging.INFO, logger="rpent_traditional_grasp.stereo"):
        backend._load()
    assert "跳过执行提供者检查" in caplog.text
    assert not [record for record in caplog.records if record.levelno >= logging.WARNING]
