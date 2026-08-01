#!/usr/bin/env bash
# Build standalone TRAC-IK on Apple Silicon macOS with Homebrew dependencies.

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/.." && pwd)"
native_root="${project_root}/native"
build_root="${native_root}/build"

log_info() {
  printf '%s [INFO] %s\n' "$(date '+%Y-%m-%dT%H:%M:%S%z')" "$*"
}

if [[ "$(uname -s)" != "Darwin" ]]; then
  log_info "该脚本仅用于 macOS；Linux/Thor 请使用 scripts/build_native_thor.sh"
  exit 2
fi

for command_name in brew cmake ctest ninja pkg-config; do
  if ! command -v "${command_name}" >/dev/null 2>&1; then
    log_info "缺少构建命令: ${command_name}"
    exit 2
  fi
done

required_formulae=(eigen orocos-kdl nlopt pkgconf)
missing_formulae=()
for formula in "${required_formulae[@]}"; do
  if ! brew list --versions "${formula}" >/dev/null 2>&1; then
    missing_formulae+=("${formula}")
  fi
done
if (( ${#missing_formulae[@]} > 0 )); then
  log_info "缺少 Homebrew 依赖: ${missing_formulae[*]}"
  log_info "请先执行: brew install ${missing_formulae[*]}"
  exit 2
fi

cmake_prefix_path="$(brew --prefix eigen);$(brew --prefix orocos-kdl);$(brew --prefix nlopt)"
log_info "配置 Apple Silicon standalone TRAC-IK"
cmake \
  -S "${native_root}" \
  -B "${build_root}" \
  -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="${cmake_prefix_path}"

log_info "编译 standalone TRAC-IK"
cmake --build "${build_root}" --parallel

log_info "运行左右臂 IK/FK 原生自检"
ctest --test-dir "${build_root}" --output-on-failure
log_info "macOS 原生构建与自检通过: ${build_root}/g1_trac_ik"
