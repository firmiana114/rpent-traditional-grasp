#!/usr/bin/env bash
# Run the no-motion stereo-image-to-XYZ test on macOS with local model assets.

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/.." && pwd)"
# Local perception venv with torch/ultralytics/onnxruntime/CLIP installed; the
# weights and third-party inference code live outside the repository and are
# referenced by macos.example.json under rpent-models.
macos_python="${MACOS_PERCEPTION_PYTHON:-/Users/firmiana/project/.venvs/rpent-traditional-grasp-macos/bin/python}"
sam2_repo="${MACOS_SAM2_REPO:-/Users/firmiana/project/rpent-models/vendor/sam2_repo}"

log_info() {
  printf '%s [INFO] %s\n' "$(date '+%Y-%m-%dT%H:%M:%S%z')" "$*" >&2
}

if ! test -x "${macos_python}"; then
  log_info "感知解释器不可执行: ${macos_python}"
  exit 2
fi

if ! test -d "${sam2_repo}/sam2"; then
  log_info "缺少本机 SAM2 仓库: ${sam2_repo}"
  exit 2
fi

export PYTHONPATH="${sam2_repo}:${project_root}/src"
export YOLO_AUTOINSTALL=false
log_info "启动本机图片到 XYZ 测试；不采集相机、不自动安装、不发送运动"
exec "${macos_python}" "${project_root}/scripts/run_image_to_xyz.py" \
  --config "${project_root}/macos.example.json" "$@"
