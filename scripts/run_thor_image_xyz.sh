#!/usr/bin/env bash
# Run the no-motion stereo-image-to-XYZ test with Thor's existing packages.

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/.." && pwd)"
yolo_python="${THOR_YOLO_PYTHON:-/home/aiot/miniconda3/envs/yolo_world/bin/python}"
abot_packages="${THOR_ABOT_PACKAGES:-/home/aiot/miniconda3/envs/abot-claw/lib/python3.10/site-packages}"
deps_root="$(mktemp -d "${TMPDIR:-/tmp}/rpent-image-xyz-deps.XXXXXX")"

log_info() {
  printf '%s [INFO] %s\n' "$(date '+%Y-%m-%dT%H:%M:%S%z')" "$*" >&2
}

cleanup() {
  rm -rf -- "${deps_root}"
}
trap cleanup EXIT

if ! test -x "${yolo_python}"; then
  log_info "yolo_world Python 不可执行: ${yolo_python}"
  exit 2
fi

for package_name in hydra iopath portalocker; do
  source_path="${abot_packages}/${package_name}"
  if ! test -d "${source_path}"; then
    log_info "缺少 Thor 现有纯 Python 依赖: ${source_path}"
    exit 2
  fi
  ln -s "${source_path}" "${deps_root}/${package_name}"
done

export PYTHONPATH="${deps_root}:${project_root}/src"
export YOLO_AUTOINSTALL=false
log_info "启动图片到 XYZ 测试；不采集相机、不自动安装、不发送运动"
"${yolo_python}" "${project_root}/scripts/run_image_to_xyz.py" "$@"
