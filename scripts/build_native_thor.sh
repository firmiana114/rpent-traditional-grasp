#!/usr/bin/env bash
# Build standalone TRAC-IK on Thor without installing system packages.

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/.." && pwd)"
native_root="${project_root}/native"
deps_root="${native_root}/.deps"
build_root="${native_root}/build"
download_root="$(mktemp -d "${TMPDIR:-/tmp}/rpent-trac-ik-debs.XXXXXX")"
chmod 0755 "${download_root}"

log_info() {
  printf '%s [INFO] %s\n' "$(date '+%Y-%m-%dT%H:%M:%S%z')" "$*"
}

cleanup() {
  rm -rf -- "${download_root}"
}
trap cleanup EXIT

packages=(
  libeigen3-dev
  liborocos-kdl-dev
  liborocos-kdl1.5
  libnlopt-cxx-dev
  libnlopt-cxx0
  libnlopt-dev
  libnlopt0
)

for command_name in apt-get cmake ctest dpkg-deb ninja; do
  if ! command -v "${command_name}" >/dev/null 2>&1; then
    log_info "缺少构建命令: ${command_name}"
    exit 2
  fi
done

log_info "下载 KDL、NLopt 与 Eigen 开发包到临时目录，不修改系统包数据库"
(
  cd -- "${download_root}"
  apt-get download "${packages[@]}"
)

mkdir -p -- "${deps_root}"
for archive in "${download_root}"/*.deb; do
  log_info "解包依赖: $(basename -- "${archive}")"
  dpkg-deb -x "${archive}" "${deps_root}"
done

log_info "配置 standalone TRAC-IK"
cmake \
  -S "${native_root}" \
  -B "${build_root}" \
  -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DRPENT_DEPS_ROOT="${deps_root}"

log_info "编译 standalone TRAC-IK"
cmake --build "${build_root}" --parallel

log_info "运行左右臂 IK/FK 原生自检"
ctest --test-dir "${build_root}" --output-on-failure
log_info "Thor 原生构建与自检通过"
