#!/usr/bin/env bash
# Foreground offline replay; Ctrl-C stops both processes and drains VIO output.
set -eo pipefail
if [[ $# -lt 2 ]]; then
    echo "Usage: $0 RECORDING_DIRECTORY NEW_OUTPUT_DIRECTORY [name:=value ...]" >&2
    exit 2
fi
session_dir=$(realpath -e -- "$1")
output_dir=$(realpath -m -- "$2")
shift 2
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=/dev/null
source "$script_dir/sdk_paths.sh"
workspace_root=${SPACEMIT_ROBOT_WORKSPACE_ROOT:-$(spacemit_robot_default_workspace "$script_dir")}
spacemit_robot_sdk_paths "$workspace_root"
# This setup file is selected by the deployment, outside the source tree.
# shellcheck source=/dev/null
source "${ROS_SETUP:-/opt/ros/humble/setup.bash}"
# shellcheck source=/dev/null
source "$SPACEMIT_ROBOT_WORKSPACE_SETUP"
set -u
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-1}
mpp_dir=${MPP_LIBRARY_DIR:-${SPACEMIT_ROBOT_SDK_PREFIX}/lib}
vio_dir=${SPACEMIT_VSLAM_LIBRARY_DIR:-${SPACEMIT_ROBOT_SDK_PREFIX}/lib}
export LD_LIBRARY_PATH="${mpp_dir}:${vio_dir}:${SPACEMIT_ROBOT_SDK_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
unset LD_PRELOAD
if [[ -e "$output_dir" || -e "${output_dir}.launch.log" ]]; then
    echo "Replay output/log already exists: $output_dir" >&2
    exit 2
fi
mkdir -p -- "$(dirname -- "$output_dir")"
input_check_options=(--allow-legacy-video-summary)
ros2 run bringup check_vio_recording.py "$session_dir" "${input_check_options[@]}"
set +e
ros2 launch bringup vio_replay_launch.py \
    session_directory:="$session_dir" output_directory:="$output_dir" \
    camera_ld_preload:="${mpp_dir}/libmpp.so.1" "$@" 2>&1 | tee "${output_dir}.launch.log"
pipeline_status=("${PIPESTATUS[@]}")
set -e
[[ ! -d "$output_dir" ]] || mv -- "${output_dir}.launch.log" "$output_dir/launch.log"
ros2 run bringup check_vio_replay.py "$session_dir" "$output_dir"
[[ ${pipeline_status[1]} -eq 0 ]] || exit "${pipeline_status[1]}"
exit "${pipeline_status[0]}"
