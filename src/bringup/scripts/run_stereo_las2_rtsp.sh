#!/usr/bin/env bash
# Start or stop the live camera -> LAS2 -> RTSP diagnostic pipeline.
set -eo pipefail

script_path=$(readlink -f "${BASH_SOURCE[0]}")
script_dir=$(cd "$(dirname "${script_path}")" && pwd)
# shellcheck source=/dev/null
source "${script_dir}/sdk_paths.sh"
workspace_root=${SPACEMIT_ROBOT_WORKSPACE_ROOT:-$(spacemit_robot_default_workspace "$script_dir")}
run_dir=${SPACEMIT_ROBOT_RUN_DIR:-${workspace_root}/.run/stereo_las2_rtsp}

spacemit_robot_sdk_paths "$workspace_root"
workspace_setup=$SPACEMIT_ROBOT_WORKSPACE_SETUP

ros_setup=${ROS_SETUP:-/opt/ros/humble/setup.bash}
# sdk_paths.sh exports the uppercase SDK overlay environment variable.
# shellcheck disable=SC2153
mpp_messages_setup=${MPP_MESSAGES_SETUP}
mpp_library_dir=${MPP_LIBRARY_DIR:-${SPACEMIT_ROBOT_SDK_PREFIX}/lib}
las2_library_dir=${LAS2_LIBRARY_DIR:-${SPACEMIT_ROBOT_SDK_PREFIX}/lib}
opencv_library_dir=${LAS2_OPENCV_LIBRARY_DIR:-/usr/lib/riscv64-linux-gnu}
ort_library_dir=${LAS2_ORT_LIBRARY_DIR}
las2_preload=$(spacemit_robot_las2_preload "$ort_library_dir")
IFS=: read -r -a las2_runtime_files <<< "$las2_preload"
model_path=${LAS2_MODEL_PATH}
CAMERA_CONFIG_DIR="${SPACEMIT_ROBOT_SDK_PREFIX}/share/camera_io/config"
DEFAULT_CAMERA_CONFIG="${CAMERA_CONFIG_DIR}/usb_stereo_4000x1200.json"
calibration_file=${SPACEMIT_ROBOT_CALIBRATION_FILE:-${CAMERA_CONFIG_DIR}/0.yaml}
camera_config_source=${SPACEMIT_ROBOT_CAMERA_CONFIG_FILE:-${DEFAULT_CAMERA_CONFIG}}
instance_count=${LAS2_INSTANCE_COUNT:-2}
threads_per_instance=${LAS2_THREADS_PER_INSTANCE:-4}
core_affinity=${LAS2_CORE_AFFINITY:-8;9;10;11/12;13;14;15}
preview_visualization=${LAS2_PREVIEW_VISUALIZATION:-disparity}
preview_disparity_min=${LAS2_PREVIEW_DISPARITY_MIN:-0.0}
preview_disparity_max=${LAS2_PREVIEW_DISPARITY_MAX:-128.0}
rtsp_url=${SPACEMIT_ROBOT_RTSP_URL:-rtsp://0.0.0.0:8554/stereo}
rtsp_fps=${SPACEMIT_ROBOT_RTSP_FPS:-30}

default_host_ip=$(ip -4 -o address show scope global 2>/dev/null |
    awk '$4 ~ /^10\./ {sub(/\/.*/, "", $4); print $4; exit}')
if [[ -z ${default_host_ip} ]]; then
    default_host_ip=$(hostname -I | awk '{print $1}')
fi
host_ip=${SPACEMIT_ROBOT_HOST_IP:-${default_host_ip}}

stop_stack()
{
    if [[ ! -s "${run_dir}/stack.pid" ]]; then
        return
    fi
    local pgid
    pgid=$(<"${run_dir}/stack.pid")
    kill -INT -- "-${pgid}" 2>/dev/null || true
    for _ in 1 2 3 4 5; do
        sleep 1
        pgrep -g "${pgid}" >/dev/null 2>&1 || break
    done
    if pgrep -g "${pgid}" >/dev/null 2>&1; then
        kill -TERM -- "-${pgid}" 2>/dev/null || true
    fi
    rm -f "${run_dir}/stack.pid"
}

action=start
case ${1:-} in
    "")
        ;;
    --stop)
        action=stop
        ;;
    --check)
        action=check
        ;;
    *)
        echo "Usage: $(basename "$0") [--stop|--check]" >&2
        exit 2
        ;;
esac

# MPP needs CMA physical-address discovery through debugfs, and LAS2 allocates
# its reusable OpenCL/EP output ring from the root-only system DMA heap on K3.
# Elevate the complete process group so camera, inference, and RTSP all use the
# same MPP/VB ownership context.  Preserve only supported runtime overrides.
if [[ ${action} != check ]] && ((EUID != 0)); then
    if ! command -v sudo >/dev/null 2>&1; then
        echo "This K3 pipeline requires root access, but sudo is unavailable" >&2
        exit 1
    fi
    echo "K3 DMA/V2D access requires root; requesting sudo authentication"
    PRESERVED_VARIABLES=(
        SPACEMIT_ROBOT_SDK_ROOT SPACEMIT_ROBOT_SDK_PREFIX SPACEMIT_ROBOT_WORKSPACE_ROOT
        SPACEMIT_ROBOT_RUN_DIR SPACEMIT_ROBOT_WORKSPACE_SETUP ROS_SETUP MPP_MESSAGES_SETUP
        MPP_LIBRARY_DIR LAS2_LIBRARY_DIR LAS2_OPENCV_LIBRARY_DIR LAS2_ORT_LIBRARY_DIR
        LAS2_MODEL_PATH SPACEMIT_ROBOT_CALIBRATION_FILE SPACEMIT_ROBOT_CAMERA_CONFIG_FILE
        LAS2_INSTANCE_COUNT LAS2_THREADS_PER_INSTANCE LAS2_CORE_AFFINITY
        LAS2_PREVIEW_VISUALIZATION LAS2_PREVIEW_DISPARITY_MIN LAS2_PREVIEW_DISPARITY_MAX
        SPACEMIT_ROBOT_RTSP_URL SPACEMIT_ROBOT_RTSP_FPS SPACEMIT_ROBOT_HOST_IP
        SPACEMIT_ROBOT_CAMERA_DEVICE ROS_DOMAIN_ID
    )
    PRESERVED_ENVIRONMENT=$(IFS=,; printf '%s' "${PRESERVED_VARIABLES[*]}")
    exec sudo --preserve-env="${PRESERVED_ENVIRONMENT}" -- "${script_path}" "$@"
fi

if [[ ${action} == stop ]]; then
    stop_stack
    exit 0
fi

for required in \
    "${ros_setup}" \
    "${mpp_messages_setup}" \
    "${workspace_setup}" \
    "${mpp_library_dir}/libmpp.so.1" \
    "${las2_library_dir}/libstereo_matching_pipeline.so" \
    "${las2_runtime_files[@]}" \
    "${model_path}" \
    "${camera_config_source}" \
    "${calibration_file}"; do
    [[ -e ${required} ]] || {
        echo "Missing required runtime path: ${required}" >&2
        exit 1
    }
done

if [[ ${action} == check ]]; then
    echo "Stereo/LAS2/RTSP runtime paths are valid: ${workspace_root}"
    exit 0
fi

stop_stack
mkdir -p "${run_dir}"

camera_device=${SPACEMIT_ROBOT_CAMERA_DEVICE:-}
if [[ -z ${camera_device} ]]; then
    for video_device in /dev/video*; do
        if v4l2-ctl -d "${video_device}" --list-formats-ext 2>/dev/null |
            grep -q "4000x1200"; then
            camera_device=${video_device}
            break
        fi
    done
fi
if [[ -z ${camera_device} ]]; then
    echo "No 4000x1200 UVC stereo camera was found" >&2
    exit 1
fi

camera_config="${run_dir}/camera.json"
python3 - "${camera_config_source}" "${camera_config}" \
    "${camera_device}" "${calibration_file}" <<'PY'
import json
import sys

source, destination, device, calibration = sys.argv[1:]
with open(source, encoding="utf-8") as stream:
    config = json.load(stream)
config["camera"]["device"] = device
config["calibration"]["file"] = calibration
with open(destination, "w", encoding="utf-8") as stream:
    json.dump(config, stream, indent=2)
    stream.write("\n")
PY

if command -v spacemit-tcm-smi >/dev/null 2>&1; then
    env -u LD_PRELOAD spacemit-tcm-smi -c
fi

set +u
# This setup file is selected by the deployment, outside the source tree.
# shellcheck source=/dev/null
source "${ros_setup}"
# This setup file is selected by the deployment, outside the source tree.
# shellcheck source=/dev/null
source "${mpp_messages_setup}"
# This setup file is selected by the deployment, outside the source tree.
# shellcheck source=/dev/null
source "${workspace_setup}"
set -u

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-1}
export LD_LIBRARY_PATH="${mpp_library_dir}:${las2_library_dir}:\
${opencv_library_dir}:${ort_library_dir}:${LD_LIBRARY_PATH:-}"
unset LD_PRELOAD

setsid ros2 launch bringup stereo_las2_rtsp_launch.py \
    camera_config_file:="${camera_config}" \
    camera_ld_preload:="${mpp_library_dir}/libmpp.so.1" \
    las2_ld_preload:="${las2_preload}" \
    calibration_file:="${calibration_file}" \
    model_path:="${model_path}" \
    las2_instance_count:="${instance_count}" \
    las2_threads_per_instance:="${threads_per_instance}" \
    las2_core_affinity:="${core_affinity}" \
    preview_visualization:="${preview_visualization}" \
    preview_disparity_min:="${preview_disparity_min}" \
    preview_disparity_max:="${preview_disparity_max}" \
    rtsp_url:="${rtsp_url}" \
    rtsp_fps:="${rtsp_fps}" \
    > "${run_dir}/launch.log" 2>&1 < /dev/null &
echo $! > "${run_dir}/stack.pid"

echo "Stereo/LAS2/RTSP started: pgid=$(<"${run_dir}/stack.pid")"
echo "Camera: ${camera_device}"
echo "RTSP: ${rtsp_url/0.0.0.0/${host_ip}}"
echo "Log: ${run_dir}/launch.log"
