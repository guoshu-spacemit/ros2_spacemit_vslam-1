#!/usr/bin/env bash
# Start or stop the live MPP -> VIO -> LAS2 -> RVV spacemit_mapping -> Foxglove stack.
set -euo pipefail

script_path=$(readlink -f "${BASH_SOURCE[0]}")
script_dir=$(cd "$(dirname "${script_path}")" && pwd)
# shellcheck source=/dev/null
source "${script_dir}/sdk_paths.sh"
workspace_root=${SPACEMIT_ROBOT_WORKSPACE_ROOT:-$(spacemit_robot_default_workspace "$script_dir")}
run_dir=${SPACEMIT_ROBOT_RUN_DIR:-/tmp/vio_las2_esdf}
supervisor="${script_dir}/run_detached_launch.py"

case ${1:-} in
    --stop) exec python3 "$supervisor" stop "$run_dir" ;;
    --status) exec python3 "$supervisor" status "$run_dir" ;;
    "") ;;
    *)
        echo "Usage: $(basename "$0") [--stop|--status]" >&2
        exit 2
        ;;
esac

spacemit_robot_sdk_paths "$workspace_root"
workspace_setup=$SPACEMIT_ROBOT_WORKSPACE_SETUP

ros_setup=${ROS_SETUP:-/opt/ros/humble/setup.bash}
mpp_messages_setup=${MPP_MESSAGES_SETUP:-$SPACEMIT_ROBOT_WORKSPACE_SETUP}
mpp_library_dir=${MPP_LIBRARY_DIR:-${SPACEMIT_ROBOT_SDK_PREFIX}/lib}
spacemit_vslam_library_dir=${SPACEMIT_VSLAM_LIBRARY_DIR:-${SPACEMIT_ROBOT_SDK_PREFIX}/lib}
las2_library_dir=${LAS2_LIBRARY_DIR:-${SPACEMIT_ROBOT_SDK_PREFIX}/lib}
opencv_library_dir=${LAS2_OPENCV_LIBRARY_DIR:-/usr/lib/$(gcc -dumpmachine)}
ort_library_dir=${LAS2_ORT_LIBRARY_DIR}
las2_preload=$(spacemit_robot_las2_preload "$ort_library_dir")
IFS=: read -r -a las2_runtime_files <<< "$las2_preload"
model_path=${LAS2_MODEL_PATH:-}
calibration_file=${SPACEMIT_ROBOT_CALIBRATION_FILE:-${SPACEMIT_ROBOT_SDK_PREFIX}/share/camera_io/config/0.yaml}
camera_config_file=${SPACEMIT_ROBOT_CAMERA_CONFIG_FILE:-}
input_file=${SPACEMIT_ROBOT_INPUT_FILE:-}
las2_instance_count=${LAS2_INSTANCE_COUNT:-2}
las2_threads_per_instance=${LAS2_THREADS_PER_INSTANCE:-4}
las2_warmup_iterations=${LAS2_WARMUP_ITERATIONS:-2}
las2_core_affinity=${LAS2_CORE_AFFINITY:-8;9;10;11/12;13;14;15}
las2_preview_visualization=${LAS2_PREVIEW_VISUALIZATION:-depth}
las2_preview_disparity_min=${LAS2_PREVIEW_DISPARITY_MIN:-0.0}
las2_preview_disparity_max=${LAS2_PREVIEW_DISPARITY_MAX:-128.0}
las2_preview_depth_min_m=${LAS2_PREVIEW_DEPTH_MIN_M:-0.0}
las2_preview_depth_max_m=${LAS2_PREVIEW_DEPTH_MAX_M:-5.0}
asynchronous_processing=${SPACEMIT_ROBOT_ASYNCHRONOUS_PROCESSING:-true}
slice_thickness_m=${SPACEMIT_ROBOT_SLICE_THICKNESS_M:-0.05}
invalid_depth_decay_factor=${SPACEMIT_ROBOT_INVALID_DEPTH_DECAY_FACTOR:-${ESDF_INVALID_DEPTH_DECAY_FACTOR:-0.8}}
render_tsdf_interpolated_points=${SPACEMIT_ROBOT_RENDER_TSDF_INTERPOLATED_POINTS:-false}
record_enabled=${SPACEMIT_ROBOT_RECORD_ENABLED:-false}
record_use_source_timestamps=${SPACEMIT_ROBOT_RECORD_USE_SOURCE_TIMESTAMPS:-false}
record_timestamp_topic=${SPACEMIT_ROBOT_RECORD_TIMESTAMP_TOPIC:-/camera/full/frame_desc}
record_timestamp_use_frame_pts=${SPACEMIT_ROBOT_RECORD_TIMESTAMP_USE_FRAME_PTS:-true}
record_timestamp_origin_zero=${SPACEMIT_ROBOT_RECORD_TIMESTAMP_ORIGIN_ZERO:-}
record_output_pattern=${SPACEMIT_ROBOT_RECORD_OUTPUT_PATTERN:-${workspace_root}/recordings/esdf_%Y%m%d_%H%M%S.mp4}
record_fps=${SPACEMIT_ROBOT_RECORD_FPS:-60}
record_bitrate_bps=${SPACEMIT_ROBOT_RECORD_BITRATE_BPS:-8000000}
record_gop=${SPACEMIT_ROBOT_RECORD_GOP:-60}
record_pool_depth=${SPACEMIT_ROBOT_RECORD_POOL_DEPTH:-8}
record_venc_channel=${SPACEMIT_ROBOT_RECORD_VENC_CHANNEL:-2}
record_mux_channel=${SPACEMIT_ROBOT_RECORD_MUX_CHANNEL:-2}
record_wait_for_map=${SPACEMIT_ROBOT_RECORD_WAIT_FOR_MAP:-false}
foxglove_runner=${FOXGLOVE_RUNNER:-}
host_ip=${SPACEMIT_ROBOT_HOST_IP:-$(hostname -I | awk '{print $1}')}

if [[ -z ${record_timestamp_origin_zero} ]]; then
    if [[ -n ${SPACEMIT_ROBOT_INPUT_FILE:-} ]]; then
        record_timestamp_origin_zero=true
    else
        record_timestamp_origin_zero=false
    fi
fi

for runtime_variable in LAS2_MODEL_PATH SPACEMIT_ROBOT_CALIBRATION_FILE FOXGLOVE_RUNNER; do
    if [[ -z ${!runtime_variable:-} ]]; then
        echo "Set $runtime_variable to the required deployment file." >&2
        exit 1
    fi
done

for required in \
    "${ros_setup}" \
    "${mpp_messages_setup}" \
    "${workspace_setup}" \
    "${mpp_library_dir}/libmpp.so.1" \
    "${las2_library_dir}/libstereo_matching_pipeline.so" \
    "${las2_runtime_files[@]}" \
    "${model_path}" \
    "${calibration_file}"; do
    [[ -e ${required} ]] || {
        echo "Missing required reconstruction runtime path: ${required}" >&2
        exit 1
    }
done
if [[ ! -f ${foxglove_runner} || ! -x ${foxglove_runner} ]]; then
    echo "Foxglove runner must be executable: ${foxglove_runner}" >&2
    exit 1
fi
if [[ -n ${input_file} && ! -f ${input_file} ]]; then
    echo "Missing input file: ${input_file}" >&2
    exit 1
fi

set +u
# This setup file is selected by the deployment, outside the source tree.
# shellcheck source=/dev/null
source "${ros_setup}"
# This setup file is selected by the deployment, outside the source tree.
# shellcheck source=/dev/null
source "${mpp_messages_setup}"
# shellcheck source=/dev/null
source "${workspace_setup}"
set -u

bringup_share=$(ros2 pkg prefix --share bringup)
launch_path="${bringup_share}/launch/vio_las2_esdf_launch.py"
if [[ ! -f ${launch_path} ]]; then
    echo "Missing launch file: ${launch_path}" >&2
    exit 1
fi
export LD_LIBRARY_PATH="${SPACEMIT_ROBOT_SDK_PREFIX}/lib:${mpp_library_dir}:${spacemit_vslam_library_dir}:${las2_library_dir}:${opencv_library_dir}:${ort_library_dir}:${LD_LIBRARY_PATH:-}"
unset LD_PRELOAD

launch_arguments=(
    input_file:="${input_file}"
    camera_ld_preload:="${mpp_library_dir}/libmpp.so.1"
    las2_ld_preload:="${las2_preload}"
    model_path:="${model_path}"
    calibration_file:="${calibration_file}"
    las2_instance_count:="${las2_instance_count}"
    las2_threads_per_instance:="${las2_threads_per_instance}"
    las2_warmup_iterations:="${las2_warmup_iterations}"
    las2_core_affinity:="${las2_core_affinity}"
    las2_preview_visualization:="${las2_preview_visualization}"
    las2_preview_disparity_min:="${las2_preview_disparity_min}"
    las2_preview_disparity_max:="${las2_preview_disparity_max}"
    las2_preview_depth_min_m:="${las2_preview_depth_min_m}"
    las2_preview_depth_max_m:="${las2_preview_depth_max_m}"
    asynchronous_processing:="${asynchronous_processing}"
    record_enabled:="${record_enabled}"
    record_use_source_timestamps:="${record_use_source_timestamps}"
    record_timestamp_topic:="${record_timestamp_topic}"
    record_timestamp_use_frame_pts:="${record_timestamp_use_frame_pts}"
    record_timestamp_origin_zero:="${record_timestamp_origin_zero}"
    record_output_pattern:="${record_output_pattern}"
    record_fps:="${record_fps}"
    record_bitrate_bps:="${record_bitrate_bps}"
    record_gop:="${record_gop}"
    record_pool_depth:="${record_pool_depth}"
    record_venc_channel:="${record_venc_channel}"
    record_mux_channel:="${record_mux_channel}"
    record_wait_for_map:="${record_wait_for_map}"
    gravity_align_map:=true
    follow_slice_height:=true
    slice_thickness_m:="${slice_thickness_m}"
    projective_tsdf_integrator_invalid_depth_decay_factor:="${invalid_depth_decay_factor}"
    depth_subsampling_factor:=1
    max_integration_distance_m:=3.0
    truncation_distance_m:=0.15
    publish_las2_preview:=true
    render_tsdf_interpolated_points:="${render_tsdf_interpolated_points}"
    render_max_blocks:=8192
    render_point_size:=8.0
)
if [[ -n ${camera_config_file} ]]; then
    launch_arguments+=(camera_config_file:="${camera_config_file}")
fi

# Clear TCM only after the previous stack has released its resources.
python3 "$supervisor" start "$run_dir" --before-start bash -ec '
    if command -v spacemit-tcm-smi >/dev/null 2>&1; then
        env -u LD_PRELOAD spacemit-tcm-smi -c
    fi
' -- \
    stack launch.log python3 "${script_dir}/run_checked_launch.py" \
    --status-file "${run_dir}/pipeline-status.json" "$launch_path" "${launch_arguments[@]}" \
    ::: bridge foxglove.log env -u LD_PRELOAD -u LD_LIBRARY_PATH \
    ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    TOPIC_WHITELIST="['^/vio/pose/image$','^/vio/pose/imu$','^/vio/trajectory$','^/vio/visualization$','^/tf$','^/tf_static$','^/esdf/colored_pointcloud$','^/rosout$']" \
    BEST_EFFORT_QOS_TOPIC_WHITELIST="['^/vio/pose/image$','^/vio/pose/imu$','^/vio/trajectory$','^/esdf/colored_pointcloud$']" \
    "${foxglove_runner}" --domain-id "${ROS_DOMAIN_ID:-0}"

echo "RVV reconstruction submitted. Use --status to inspect the launch state."
echo "SpaceMIT ONNX Runtime library directory: ${ort_library_dir}"
echo "Foxglove: ws://${host_ip}:8765"
echo "Logs: ${run_dir}/launch.log and ${run_dir}/foxglove.log"
