#!/usr/bin/env bash
# Start the live USB-stereo VIO, RTSP, and Foxglove stack on the K3 board.
set -euo pipefail

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
run_dir=${SPACEMIT_ROBOT_RUN_DIR:-/tmp/live_vio}
case ${1:-} in
    --stop|--status)
        exec python3 "$script_dir/run_detached_launch.py" "${1#--}" "$run_dir"
        ;;
    "")
        ;;
    *)
        echo "Usage: $(basename "$0") [--stop|--status]" >&2
        exit 2
        ;;
esac

# shellcheck source=/dev/null
source "${script_dir}/sdk_paths.sh"
workspace_root=${SPACEMIT_ROBOT_WORKSPACE_ROOT:-$(spacemit_robot_default_workspace "$script_dir")}
spacemit_robot_sdk_paths "$workspace_root"

ros_setup=${ROS_SETUP:-/opt/ros/humble/setup.bash}
mpp_messages_setup=${MPP_MESSAGES_SETUP:-$SPACEMIT_ROBOT_WORKSPACE_SETUP}
mpp_library_dir=${MPP_LIBRARY_DIR:-${SPACEMIT_ROBOT_SDK_PREFIX}/lib}
spacemit_vslam_library_dir=${SPACEMIT_VSLAM_LIBRARY_DIR:-${SPACEMIT_ROBOT_SDK_PREFIX}/lib}
foxglove_runner=${FOXGLOVE_RUNNER:-}
domain_id=${ROS_DOMAIN_ID:-0}
host_ip=${SPACEMIT_ROBOT_HOST_IP:-$(hostname -I | awk '{print $1}')}

camera_executable="${SPACEMIT_ROBOT_SDK_PREFIX}/lib/camera_io/camera_mpp_publisher"
vio_executable="${SPACEMIT_ROBOT_SDK_PREFIX}/lib/vio/vio_node"
rtsp_executable="${SPACEMIT_ROBOT_SDK_PREFIX}/lib/mpp_rtsp/mpp_rtsp_node"
camera_config=${CAMERA_CONFIG_FILE:-"${SPACEMIT_ROBOT_SDK_PREFIX}/share/camera_io/config/usb_stereo_4000x1200.json"}
calibration_file=${CALIBRATION_FILE:-"${SPACEMIT_ROBOT_SDK_PREFIX}/share/camera_io/config/0.yaml"}

if [[ -z ${FOXGLOVE_RUNNER:-} ]]; then
    echo "Set FOXGLOVE_RUNNER to the required deployment file." >&2
    exit 1
fi

for required in "$ros_setup" "$mpp_messages_setup" "$SPACEMIT_ROBOT_WORKSPACE_SETUP" \
    "$camera_config" "$calibration_file" "$mpp_library_dir/libmpp.so.1"; do
    [[ -f $required ]] || { echo "Missing required file: $required" >&2; exit 1; }
done
for executable in "$camera_executable" "$vio_executable" "$rtsp_executable" "$foxglove_runner"; do
    [[ -f $executable && -x $executable ]] || {
        echo "Executable required: $executable" >&2
        exit 1
    }
done

# ROS setup scripts read optional variables without default values, which is
# incompatible with this launcher's `set -u` policy.
set +u
# This setup file is selected by the deployment, outside the source tree.
# shellcheck source=/dev/null
source "${ros_setup}"
# This setup file is selected by the deployment, outside the source tree.
# shellcheck source=/dev/null
source "${mpp_messages_setup}"
# shellcheck source=/dev/null
source "$SPACEMIT_ROBOT_WORKSPACE_SETUP"
set -u
export ROS_DOMAIN_ID=${domain_id}
export LD_LIBRARY_PATH="${mpp_library_dir}:${spacemit_vslam_library_dir}:${LD_LIBRARY_PATH:-}"
export LD_PRELOAD="${mpp_library_dir}/libmpp.so.1${LD_PRELOAD:+:${LD_PRELOAD}}"

python3 "$script_dir/run_detached_launch.py" start "$run_dir" -- \
    live live.log "${camera_executable}" --ros-args \
        -r __node:=camera_io \
        -p "config_file:=${camera_config}" -p autostart:=true \
    ::: vio live.log "${vio_executable}" --ros-args \
        -r __node:=vio \
        -p "calibration_file:=${calibration_file}" \
        -p "imu_calibration_file:=${calibration_file}" \
        -p "camera_config_file:=${camera_config}" \
        -p image_width:=640 -p image_height:=400 \
        -p use_imu:=true -p asynchronous_processing:=true \
        -p slam_enabled:=true \
        -p image_queue_depth:=1 -p sync_queue_depth:=2 \
        -p publish_features:=false \
    ::: rtsp live.log "${rtsp_executable}" --ros-args \
        -r __node:=mpp_rtsp \
        -p input_topic:=/camera/left/frame_desc \
        -p fps:=60 -p codec:=h264 \
        -p rtsp_url:=rtsp://0.0.0.0:8554/live \
        -p overlay_enabled:=false -p feature_overlay_enabled:=false \
    ::: foxglove foxglove.log env \
        ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
        TOPIC_WHITELIST="['^/vio/pose/image$','^/vio/pose/imu$','^/vio/trajectory$','^/vio/visualization$','^/slam/pose/image$','^/slam/trajectory$','^/tf$','^/tf_static$','^/rosout$']" \
        BEST_EFFORT_QOS_TOPIC_WHITELIST="['^/vio/pose/image$','^/vio/pose/imu$','^/vio/trajectory$','^/slam/pose/image$','^/slam/trajectory$']" \
        "${foxglove_runner}" --domain-id "${ROS_DOMAIN_ID}"

echo 'Live VIO submitted. Use --status to inspect the launch state.'
echo "Foxglove: ws://${host_ip}:8765"
echo "RTSP:     rtsp://${host_ip}:8554/live"
echo "Logs:     ${run_dir}/live.log and ${run_dir}/foxglove.log"
