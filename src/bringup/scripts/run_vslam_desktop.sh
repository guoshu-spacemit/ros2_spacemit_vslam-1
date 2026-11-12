#!/usr/bin/env bash
# Foreground VIO/depth/mapping and its optional desktop viewer.
set -euo pipefail

mode=all
case ${1:-} in
    --backend-only)
        mode=backend
        shift
        ;;
    --viewer-only)
        mode=viewer
        shift
        ;;
esac

if [[ ${1:-} == --help || ${1:-} == -h ]]; then
    cat <<'EOF'
Usage: ros2 run bringup run_vslam_desktop.sh [--backend-only|--viewer-only] [name:=value ...]

Default: start the complete pipeline and a regular 1280x720 desktop window.
Run it from the logged-in desktop user's terminal with the SDK overlay loaded.
Pass camera_config_file, calibration_file and imu_calibration_file for your rig;
other vio_las2_esdf_launch.py arguments are forwarded, for example:
  camera_fps:=30 desktop_fullscreen:=true display_width:=1920 display_height:=1080

--backend-only starts capture/VIO/LAS2/mapping without a viewer or DISPLAY.
Use an existing authorized hardware account if the SDK requires it.
--viewer-only subscribes to the running backend, without capture or inference.
Keep the backend and viewer under the same effective runtime UID. A privileged
viewer can set SPACEMIT_DESKTOP_UID to the logged-in desktop user's numeric UID
while preserving DISPLAY/XAUTHORITY; this changes no X11 access rules.
Its node parameters are forwarded directly:
  input_topic:=/esdf/render/frame_desc window_width:=1280 window_height:=720
  fullscreen:=false title:="SpacemiT VIO / Depth / TSDF / ESDF"

Use the same ROS_DOMAIN_ID in both terminals. Both commands stay in foreground.
Ctrl+C stops the corresponding process. The desktop service stays running;
DISPLAY/XAUTHORITY are inherited from the current desktop session.
EOF
    exit 0
fi

if [[ $mode != backend && -z ${DISPLAY:-} ]]; then
    echo 'No X11/Xwayland display. Run this command in the logged-in desktop terminal.' >&2
    exit 1
fi
if ! command -v ros2 >/dev/null 2>&1; then
    echo 'ROS 2 is not loaded. Source the SDK environment and run sros2_setup first.' >&2
    exit 1
fi

script_dir=$(cd -- "$(dirname -- "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)
# shellcheck source=/dev/null
source "$script_dir/sdk_paths.sh"
workspace_root=${SPACEMIT_ROBOT_WORKSPACE_ROOT:-$(spacemit_robot_default_workspace "$script_dir")}
if [[ -z ${SPACEMIT_ROBOT_SDK_PREFIX:-} && -n ${PREFIX:-} ]]; then
    export SPACEMIT_ROBOT_SDK_PREFIX=$PREFIX
fi
spacemit_robot_sdk_paths "$workspace_root"

if [[ $mode == viewer ]]; then
    viewer_parameters=(
        -p input_topic:=/esdf/render/frame_desc
        -p window_width:=1280 -p window_height:=720
        -p fullscreen:=false
        -p 'title:=SpacemiT VIO / Depth / TSDF / ESDF'
    )
    for argument in "$@"; do
        if [[ $argument != *:=* || -z ${argument%%:=*} ]]; then
            echo 'Viewer arguments must be node parameters in name:=value form.' >&2
            exit 2
        fi
        viewer_parameters+=(-p "$argument")
    done
    exec env LD_PRELOAD= ros2 run esdf_drm esdf_desktop_viewer \
        --ros-args "${viewer_parameters[@]}"
fi

viewer_enabled=true
[[ $mode != backend ]] || viewer_enabled=false
las2_preload=$(spacemit_robot_las2_preload "$LAS2_ORT_LIBRARY_DIR")
exec ros2 launch bringup vio_las2_esdf_launch.py \
    model_path:="$LAS2_MODEL_PATH" \
    camera_ld_preload:="$MPP_LIBRARY_DIR/libmpp.so.1" \
    las2_ld_preload:="$las2_preload" \
    application_cpu_set:=0-7 camera_fps:=30 desktop_fullscreen:=false \
    display_width:=1280 display_height:=720 \
    "$@" \
    desktop_viewer_enabled:="$viewer_enabled" headless_render:=true publish_render_output:=true
