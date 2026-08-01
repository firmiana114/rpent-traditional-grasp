from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys


def test_protocol_stdout_isolated_from_python_and_native_noise() -> None:
    root = Path(__file__).resolve().parents[2]
    script = root / "scripts" / "traditional_grasp_service.py"
    code = f"""
import importlib.util
import json
import os

spec = importlib.util.spec_from_file_location("traditional_grasp_service", {str(script)!r})
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
protocol = module._isolate_protocol_stdout()
print("python runtime noise", flush=True)
os.write(1, b"native runtime noise\\n")
print(json.dumps({{"success": True}}), file=protocol, flush=True)
protocol.close()
"""
    env = os.environ.copy()
    env["PYTHONPATH"] = os.pathsep.join(
        [str(root / "src"), env.get("PYTHONPATH", "")]
    )

    completed = subprocess.run(
        [sys.executable, "-c", code],
        check=True,
        capture_output=True,
        text=True,
        env=env,
    )

    assert completed.stdout.strip() == '{"success": true}'
    assert "python runtime noise" in completed.stderr
    assert "native runtime noise" in completed.stderr
