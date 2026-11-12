#!/usr/bin/env bash
# Start or stop live stereo/VIO/LAS2/ESDF with a left-camera RTSP output.
set -eo pipefail
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-1}

script_path=$(readlink -f "${BASH_SOURCE[0]}")
script_dir=$(cd "$(dirname "${script_path}")" && pwd)
# shellcheck source=/dev/null
source "${script_dir}/sdk_paths.sh"
workspace_root=${SPACEMIT_ROBOT_WORKSPACE_ROOT:-$(spacemit_robot_default_workspace "$script_dir")}
run_dir=${SPACEMIT_ROBOT_RUN_DIR:-/tmp/headless_esdf_rtsp}
supervisor="${script_dir}/run_detached_launch.py"

spacemit_robot_sdk_paths "$workspace_root"
workspace_setup=$SPACEMIT_ROBOT_WORKSPACE_SETUP

ros_setup=${ROS_SETUP:-/opt/ros/humble/setup.bash}
# sdk_paths.sh exports the uppercase SDK overlay environment variable.
# shellcheck disable=SC2153
mpp_messages_setup=${MPP_MESSAGES_SETUP}
mpp_library_dir=${MPP_LIBRARY_DIR:-${SPACEMIT_ROBOT_SDK_PREFIX}/lib}
spacemit_vslam_library_dir=${SPACEMIT_VSLAM_LIBRARY_DIR:-${SPACEMIT_ROBOT_SDK_PREFIX}/lib}
las2_library_dir=${LAS2_LIBRARY_DIR:-${SPACEMIT_ROBOT_SDK_PREFIX}/lib}
opencv_library_dir=${LAS2_OPENCV_LIBRARY_DIR:-/usr/lib/riscv64-linux-gnu}
ort_library_dir=${LAS2_ORT_LIBRARY_DIR}
las2_preload=$(spacemit_robot_las2_preload "$ort_library_dir")
IFS=: read -r -a las2_runtime_files <<< "$las2_preload"
model_path=${LAS2_MODEL_PATH}
calibration_file=${SPACEMIT_ROBOT_CALIBRATION_FILE:-${SPACEMIT_ROBOT_SDK_PREFIX}/share/camera_io/config/0.yaml}
camera_config_source=${SPACEMIT_ROBOT_CAMERA_CONFIG_FILE:-${SPACEMIT_ROBOT_SDK_PREFIX}/share/camera_io/config/usb_stereo_4000x1200.json}
display_width=${ESDF_STREAM_WIDTH:-1280}
display_height=${ESDF_STREAM_HEIGHT:-720}
slice_thickness_m=${ESDF_SLICE_THICKNESS_M:-0.20}
slice_height_offset_m=${ESDF_SLICE_HEIGHT_OFFSET_M:--0.10}
max_integration_distance_m=${ESDF_MAX_INTEGRATION_DISTANCE_M:-3.0}
invalid_depth_decay_factor=${ESDF_INVALID_DEPTH_DECAY_FACTOR:-0.8}
host_ip=${SPACEMIT_ROBOT_HOST_IP:-$(hostname -I | awk '{print $1}')}
las2_instance_count=${LAS2_INSTANCE_COUNT:-1}
las2_threads_per_instance=${LAS2_THREADS_PER_INSTANCE:-8}
las2_core_affinity=${LAS2_CORE_AFFINITY:-8;9;10;11;12;13;14;15}
enable_mapping=${SPACEMIT_ROBOT_ENABLE_MAPPING:-true}
enable_rtsp=${SPACEMIT_ROBOT_ENABLE_RTSP:-true}
enable_esdf_rtsp=${SPACEMIT_ROBOT_ENABLE_ESDF_RTSP:-false}
publish_las2_preview=${SPACEMIT_ROBOT_PUBLISH_LAS2_PREVIEW:-false}
las2_preview_visualization=${LAS2_PREVIEW_VISUALIZATION:-depth}
las2_preview_disparity_min=${LAS2_PREVIEW_DISPARITY_MIN:-0.0}
las2_preview_disparity_max=${LAS2_PREVIEW_DISPARITY_MAX:-128.0}
rtsp_input_topic=${SPACEMIT_ROBOT_RTSP_INPUT_TOPIC:-/camera/left/frame_desc}
rtsp_url=${SPACEMIT_ROBOT_RTSP_URL:-rtsp://0.0.0.0:8554/left}

check_only=false
case ${1:-} in
    "")
        ;;
    --stop)
        exec python3 "$supervisor" stop "$run_dir"
        ;;
    --status)
        exec python3 "$supervisor" status "$run_dir"
        ;;
    --check)
        check_only=true
        ;;
    *)
        echo "Usage: $(basename "$0") [--stop|--check|--status]" >&2
        exit 2
        ;;
esac

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

if ${check_only}; then
    echo "Sensing runtime paths are valid for workspace: ${workspace_root}"
    echo "SpaceMIT ONNX Runtime library directory: ${ort_library_dir}"
    exit 0
fi

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
python3 "$supervisor" prepare "$run_dir"
camera_config="${run_dir}/camera.json"
camera_config_temp=$(mktemp "${run_dir}/.camera.XXXXXX")
trap 'rm -f -- "$camera_config_temp"' EXIT
python3 - "${camera_config_source}" "${camera_config_temp}" \
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

export LD_LIBRARY_PATH="${mpp_library_dir}:${spacemit_vslam_library_dir}:${las2_library_dir}:${opencv_library_dir}:${ort_library_dir}:${LD_LIBRARY_PATH:-}"
unset LD_PRELOAD

launch_file=vio_las2_esdf_launch.py
record_arguments=()
if [[ -n ${SPACEMIT_ROBOT_RECORD_SESSION_DIR:-} ]]; then
    launch_file=vio_record_launch.py
    record_arguments+=("session_directory:=${SPACEMIT_ROBOT_RECORD_SESSION_DIR}"
                       "record_vio:=${SPACEMIT_ROBOT_RECORD_VIO:-true}")
fi

bringup_share=$(ros2 pkg prefix --share bringup)
launch_path="${bringup_share}/launch/${launch_file}"
if [[ ! -f ${launch_path} ]]; then
    echo "Missing launch file: ${launch_path}" >&2
    exit 1
fi
# The pre-start shell expands its own camera configuration arguments.
# shellcheck disable=SC2016
python3 "$supervisor" start "$run_dir" --before-start bash -ec '
    if command -v spacemit-tcm-smi >/dev/null 2>&1; then
        env -u LD_PRELOAD spacemit-tcm-smi -c
    fi
    mv -fT -- "$1" "$2"
' prepare "$camera_config_temp" "$camera_config" -- \
    stack launch.log python3 "${script_dir}/run_checked_launch.py" \
    --status-file "${run_dir}/pipeline-status.json" "$launch_path" "${record_arguments[@]}" \
    camera_config_file:="${camera_config}" \
    camera_ld_preload:="${mpp_library_dir}/libmpp.so.1" \
    las2_ld_preload:="${las2_preload}" \
    model_path:="${model_path}" \
    calibration_file:="${calibration_file}" \
    las2_instance_count:="${las2_instance_count}" \
    las2_threads_per_instance:="${las2_threads_per_instance}" \
    las2_core_affinity:="${las2_core_affinity}" \
    asynchronous_processing:="${SPACEMIT_ROBOT_ASYNCHRONOUS_PROCESSING:-true}" \
    publish_las2_preview:="${publish_las2_preview}" \
    las2_preview_visualization:="${las2_preview_visualization}" \
    las2_preview_disparity_min:="${las2_preview_disparity_min}" \
    las2_preview_disparity_max:="${las2_preview_disparity_max}" \
    enable_mapping:="${enable_mapping}" \
    enable_rtsp:="${enable_rtsp}" \
    rtsp_input_topic:="${rtsp_input_topic}" \
    rtsp_url:="${rtsp_url}" \
    enable_esdf_rtsp:="${enable_esdf_rtsp}" \
    enable_gles_drm:="${enable_esdf_rtsp}" \
    headless_render:="${enable_esdf_rtsp}" \
    publish_render_output:="${enable_esdf_rtsp}" \
    render_fps:=30.0 \
    display_width:="${display_width}" \
    display_height:="${display_height}" \
    gravity_align_map:=true \
    follow_slice_height:=true \
    slice_thickness_m:="${slice_thickness_m}" \
    slice_height_offset_m:="${slice_height_offset_m}" \
    depth_subsampling_factor:=1 \
    max_integration_distance_m:="${max_integration_distance_m}" \
    projective_tsdf_integrator_invalid_depth_decay_factor:="${invalid_depth_decay_factor}" \
    truncation_distance_m:=0.15 \
    render_point_size:=8.0

echo "Headless reconstruction submitted. Use --status to inspect the launch state."
echo "SpaceMIT ONNX Runtime library directory: ${ort_library_dir}"
if [[ ${enable_rtsp} == true ]]; then
    echo "Primary video: ${rtsp_url/0.0.0.0/${host_ip}}"
fi
if [[ ${enable_esdf_rtsp} == true ]]; then
    echo "ESDF top slice (${slice_thickness_m} m thick, offset ${slice_height_offset_m} m): rtsp://${host_ip}:8555/esdf"
fi
echo "Log: ${run_dir}/launch.log"
