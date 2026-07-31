#!/usr/bin/env bash
# Run stage two: stereo images to the final gripper TCP XYZ, without motion.

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "${script_dir}/run_thor_image_xyz.sh" --result-kind gripper_xyz "$@"
