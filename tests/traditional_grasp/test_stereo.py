from __future__ import annotations

import logging
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from rpent_traditional_grasp.stereo import ExternalCREStereoBackend

_FAKE_BACKEND_TEMPLATE = """
class _FakeSession:
    def __init__(self, providers):
        self.providers = list(providers)
        self.set_provider_calls = []
        self.fail_on_set = {fail_on_set!r}

    def get_providers(self):
        return list(self.providers)

    def set_providers(self, providers, provider_options=None):
        self.set_provider_calls.append(list(providers))
        if self.fail_on_set:
            raise RuntimeError("provider switch rejected")
        self.providers = list(providers)


class CREStereo:
    def __init__(self, model_path, device="cuda"):
        self.model_path = model_path
        self.device = device
        self.session = _FakeSession({providers!r})
"""


def _build_backend(
    tmp_path: Path,
    module_name: str,
    providers: list[str],
    *,
    device: str = "cuda",
    fail_on_set: bool = False,
) -> ExternalCREStereoBackend:
    repository = tmp_path / "vendor"
    repository.mkdir(exist_ok=True)
    (repository / f"{module_name}.py").write_text(
        _FAKE_BACKEND_TEMPLATE.format(providers=providers, fail_on_set=fail_on_set),
        encoding="utf-8",
    )
    return ExternalCREStereoBackend(
        repository,
        tmp_path / "model.onnx",
        module_name=module_name,
        class_name="CREStereo",
        device=device,
    )


def _stub_runtime(
    monkeypatch: pytest.MonkeyPatch,
    available: list[str],
) -> None:
    monkeypatch.setitem(
        sys.modules,
        "onnxruntime",
        SimpleNamespace(get_available_providers=lambda: list(available)),
    )


def _warnings(caplog: pytest.LogCaptureFixture) -> list[logging.LogRecord]:
    return [record for record in caplog.records if record.levelno >= logging.WARNING]


def test_active_accelerator_is_left_untouched(
    tmp_path: Path,
    caplog: pytest.LogCaptureFixture,
) -> None:
    backend = _build_backend(
        tmp_path,
        "fake_crestereo_gpu",
        ["TensorrtExecutionProvider", "CUDAExecutionProvider", "CPUExecutionProvider"],
    )
    with caplog.at_level(logging.INFO, logger="rpent_traditional_grasp.stereo"):
        model = backend._load()

    assert backend.execution_providers[0] == "TensorrtExecutionProvider"
    # Re-selecting would discard the vendor's TensorRT engine cache options.
    assert model.session.set_provider_calls == []
    assert "使用加速执行提供者" in caplog.text
    assert not _warnings(caplog)


def test_cpu_only_session_is_switched_to_cuda_when_the_runtime_has_it(
    tmp_path: Path,
    caplog: pytest.LogCaptureFixture,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _stub_runtime(monkeypatch, ["CUDAExecutionProvider", "CPUExecutionProvider"])
    backend = _build_backend(
        tmp_path,
        "fake_crestereo_switch",
        ["CPUExecutionProvider"],
    )
    with caplog.at_level(logging.INFO, logger="rpent_traditional_grasp.stereo"):
        model = backend._load()

    assert model.session.set_provider_calls == [
        ["CUDAExecutionProvider", "CPUExecutionProvider"]
    ]
    assert backend.execution_providers == [
        "CUDAExecutionProvider",
        "CPUExecutionProvider",
    ]
    assert "已强制切换到 CUDA" in caplog.text
    assert not _warnings(caplog)


def test_cpu_fallback_warns_when_the_runtime_has_no_cuda(
    tmp_path: Path,
    caplog: pytest.LogCaptureFixture,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    # A CPU-only onnxruntime still advertises Azure, which is a remote-inference
    # entry point rather than a local accelerator.
    _stub_runtime(monkeypatch, ["AzureExecutionProvider", "CPUExecutionProvider"])
    backend = _build_backend(
        tmp_path,
        "fake_crestereo_cpu",
        ["AzureExecutionProvider", "CPUExecutionProvider"],
    )
    with caplog.at_level(logging.INFO, logger="rpent_traditional_grasp.stereo"):
        model = backend._load()

    assert model.session.set_provider_calls == []
    warnings = _warnings(caplog)
    assert len(warnings) == 1
    assert "回退到纯 CPU 推理" in warnings[0].getMessage()


def test_explicit_cpu_override_never_queries_or_forces_the_runtime(
    tmp_path: Path,
    caplog: pytest.LogCaptureFixture,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    def _explode() -> list[str]:
        raise AssertionError("显式 cpu 覆盖不应查询 onnxruntime 提供者")

    monkeypatch.setitem(
        sys.modules,
        "onnxruntime",
        SimpleNamespace(get_available_providers=_explode),
    )
    backend = _build_backend(
        tmp_path,
        "fake_crestereo_cpu_override",
        ["CPUExecutionProvider"],
        device="cpu",
    )
    with caplog.at_level(logging.INFO, logger="rpent_traditional_grasp.stereo"):
        model = backend._load()

    assert model.session.set_provider_calls == []
    assert "unqueried" in _warnings(caplog)[0].getMessage()


def test_mps_device_still_takes_cuda_when_onnxruntime_offers_it(
    tmp_path: Path,
    caplog: pytest.LogCaptureFixture,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    # CREStereo runs on onnxruntime, so a Torch-derived "mps" device must not
    # stop it from using a CUDA-capable runtime.
    _stub_runtime(monkeypatch, ["CUDAExecutionProvider", "CPUExecutionProvider"])
    backend = _build_backend(
        tmp_path,
        "fake_crestereo_mps",
        ["CPUExecutionProvider"],
        device="mps",
    )
    with caplog.at_level(logging.INFO, logger="rpent_traditional_grasp.stereo"):
        backend._load()

    assert backend.execution_providers == [
        "CUDAExecutionProvider",
        "CPUExecutionProvider",
    ]
    assert not _warnings(caplog)


def test_failed_cuda_switch_degrades_to_the_cpu_warning(
    tmp_path: Path,
    caplog: pytest.LogCaptureFixture,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _stub_runtime(monkeypatch, ["CUDAExecutionProvider", "CPUExecutionProvider"])
    backend = _build_backend(
        tmp_path,
        "fake_crestereo_reject",
        ["CPUExecutionProvider"],
        fail_on_set=True,
    )
    with caplog.at_level(logging.INFO, logger="rpent_traditional_grasp.stereo"):
        model = backend._load()

    assert model.session.set_provider_calls == [
        ["CUDAExecutionProvider", "CPUExecutionProvider"]
    ]
    assert backend.execution_providers == ["CPUExecutionProvider"]
    messages = [record.getMessage() for record in _warnings(caplog)]
    assert any("切换 CREStereo 到 CUDA 失败" in message for message in messages)
    assert any("回退到纯 CPU 推理" in message for message in messages)


def test_backend_without_session_skips_provider_selection(
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

    assert backend.execution_providers == []
    assert "跳过执行提供者检查" in caplog.text
    assert not _warnings(caplog)
