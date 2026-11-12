#!/usr/bin/env bash
# One restart-safe live reconstruction + FPV transmitter per workspace.
set -euo pipefail
self=$(realpath -e -- "${BASH_SOURCE[0]}")
script_dir=$(dirname -- "$self")
fail() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }
usage() {
    cat <<'EOF'
Usage: run_reconstruction_video_transmission.sh {start|restart|stop|status|check}
       [--workspace DIR] [--config FILE]
Defaults: workspace and dependency paths are resolved from the SDK checkout
          SPACEMIT_ROBOT_VIDEO_SESSION_CONFIG=$workspace/config/reconstruction_video_transmission.json
Camera + VO/VIO + LAS2 + TSDF/ESDF + complete 1920x1080/30 FPS FPV picture.
check loads configuration/libraries and launch arguments without opening hardware.
Transient systemd service survives SSH disconnect; stop only targets this workspace.
Runtime overrides: ROS_SETUP_FILE, SPACEMIT_ROBOT_WORKSPACE_SETUP; other paths belong in JSON.
EOF
}
[[ $# -gt 0 ]] || { usage; exit 2; }
action=$1
shift
[[ $action != --help && $action != -h ]] || { usage; exit 0; }

load_session_environment() {
    local setup_status environment_values values key value i
    export ROS_LOG_DIR="$session/ros"
    mkdir -p "$ROS_LOG_DIR" || return $?
    unset LD_PRELOAD AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
    unset LD_LIBRARY_PATH PYTHONPATH ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
    unset MPP_PLUGIN_DIR __EGL_VENDOR_LIBRARY_FILENAMES GBM_BACKENDS_PATH
    unset LIBGL_DRIVERS_PATH GBM_BACKEND
    set +u
    # This setup file is selected by the deployment, outside the source tree.
    # shellcheck source=/dev/null
    source "${ROS_SETUP_FILE:-/opt/ros/humble/setup.bash}" || {
        setup_status=$?; set -u; return "$setup_status"
    }
    if [[ -f $workspace/setup_dependencies.bash ]]; then
        # shellcheck source=/dev/null
        source "$workspace/setup_dependencies.bash" || {
            setup_status=$?; set -u; return "$setup_status"
        }
    fi
    if [[ -n ${SPACEMIT_ROBOT_WORKSPACE_SETUP:-} ]]; then
        # This setup file is selected by the deployment, outside the source tree.
        # shellcheck source=/dev/null
        source "$SPACEMIT_ROBOT_WORKSPACE_SETUP" || {
            setup_status=$?; set -u; return "$setup_status"
        }
    elif [[ -f $workspace/install/setup.bash ]]; then
        # shellcheck source=/dev/null
        source "$workspace/install/setup.bash" || {
            setup_status=$?; set -u; return "$setup_status"
        }
    else
        # The cross-build bundle has a merged prefix, without host setup hooks.
        export AMENT_PREFIX_PATH="$workspace/install:${AMENT_PREFIX_PATH:-}"
        export LD_LIBRARY_PATH="$workspace/lib:$workspace/install/lib:${LD_LIBRARY_PATH:-}"
    fi
    set -u
    environment_values=$(python3 - "$session/runtime-environment.json" <<'PY'
import json
import sys
for key, value in json.load(open(sys.argv[1], encoding='utf-8')).items():
    print(key)
    print(value)
PY
    ) || return $?
    values=()
    if [[ -n $environment_values ]]; then
        mapfile -t values <<< "$environment_values" || return $?
    fi
    for ((i=0; i<${#values[@]}; i+=2)); do
        key=${values[i]}
        value=${values[i+1]}
        if [[ $key == LD_LIBRARY_PATH ]]; then
            value="$value:${LD_LIBRARY_PATH:-}"
        fi
        export "$key=$value" || return $?
    done
    export ROS_DOMAIN_ID
    ROS_DOMAIN_ID=$(python3 - "$session/session.json" <<'PY'
import json
import sys
print(json.load(open(sys.argv[1], encoding='utf-8'))['ros_domain_id'])
PY
    ) || return $?
    export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}
}

read_options() {
    local options_text
    options_text=$(python3 - "$session/launch-options.json" <<'PY'
import json
import sys
for key, value in json.load(open(sys.argv[1], encoding='utf-8')).items():
    print(f'{key}:={value}')
PY
    ) || return $?
    [[ -n $options_text ]] || { printf 'ERROR: Empty launch options snapshot\n' >&2; return 1; }
    mapfile -t launch_options <<< "$options_text" || return $?
}

if [[ $action == _run ]]; then
    [[ $# == 2 ]] || fail 'Invalid internal service arguments'
    workspace=$1
    session=$2
    trap 'python3 - "$session/pipeline-status.json" <<"PY"
import json
import os
from pathlib import Path
import sys
path = Path(sys.argv[1])
temporary = path.with_name(path.name + ".new")
temporary.write_text(json.dumps({"state": "failed", "reason": "Service setup failed; see launch.log"}) + "\n")
os.replace(temporary, path)
PY
    ' EXIT
    load_session_environment || exit $?
    read_options || exit $?
    share=$(ros2 pkg prefix --share bringup) || exit $?
    affinity=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["cpu_affinity"])' "$session/session.json") || exit $?
    exec taskset -c "$affinity" python3 "$script_dir/run_checked_launch.py" \
        --status-file "$session/pipeline-status.json" \
        "$share/launch/reconstruction_video_transmission_launch.py" "${launch_options[@]}"
fi

case $action in start|restart|stop|status|check) ;; *) usage; exit 2 ;; esac
# shellcheck source=/dev/null
source "$script_dir/sdk_paths.sh"
workspace=${SPACEMIT_ROBOT_WORKSPACE_ROOT:-$(spacemit_robot_default_workspace "$script_dir")}
profile=${SPACEMIT_ROBOT_VIDEO_SESSION_CONFIG:-}
while (( $# )); do
    [[ $# -ge 2 ]] || fail "Missing value for $1"
    case $1 in
        --workspace) workspace=$2 ;;
        --config)
            [[ $action != stop && $action != status ]] || fail "$action takes only --workspace"
            profile=$2 ;;
        *) fail "Unknown option: $1" ;;
    esac
    shift 2
done
workspace=$(realpath -e -- "$workspace")
spacemit_robot_sdk_paths "$workspace"
profile=${profile:-$workspace/config/reconstruction_video_transmission.json}
[[ $EUID == 0 ]] || fail 'Run as root for K3 DMA/DRM access'
runtime="$workspace/.run/reconstruction_video_transmission"
mkdir -p "$runtime"
unit="robot_perception-fpv-$(printf '%s' "$workspace" | sha256sum | cut -c1-12).service"
exec 9>"$runtime/control.lock"
flock -n 9 || fail 'Another controller command is in progress'
state() { systemctl show "$unit" -p ActiveState --value 2>/dev/null || true; }
stop_session() {
    case $(state) in
        active|activating|deactivating|reloading) systemctl stop "$unit" ;;
    esac
}
if [[ $action == stop ]]; then
    stop_session
    echo 'Stopped (or already stopped). Session logs retained.'
    exit 0
fi
if [[ $action == status ]]; then
    systemctl show "$unit" -p ActiveState -p SubState -p MainPID -p Result || true
    [[ -f $runtime/current-session ]] || { echo 'No session submitted.'; exit 0; }
    session=$(<"$runtime/current-session")
    printf 'Session: %s\nLog: %s/launch.log\n' "$session" "$session"
    python3 - "$session/pipeline-status.json" <<'PY'
import json
from pathlib import Path
import sys
path = Path(sys.argv[1])
if path.exists():
    status = json.loads(path.read_text())
    print(json.dumps(status, indent=2))
    sys.exit(1 if status['state'] == 'failed' else 0)
print('Supervisor status not yet written; inspect launch.log.')
PY
    exit $?
fi
for tool in systemctl systemd-run taskset python3; do
    command -v "$tool" >/dev/null || fail "Missing $tool"
done
[[ -f $profile ]] || fail "Deployment profile missing: $profile"
[[ -f $SPACEMIT_ROBOT_WORKSPACE_SETUP ]] || fail 'SDK setup file missing'
export ROS_SETUP_FILE
ROS_SETUP_FILE=$(realpath -e -- "${ROS_SETUP_FILE:-/opt/ros/humble/setup.bash}")
if [[ -n ${SPACEMIT_ROBOT_WORKSPACE_SETUP:-} ]]; then
    export SPACEMIT_ROBOT_WORKSPACE_SETUP
    SPACEMIT_ROBOT_WORKSPACE_SETUP=$(realpath -e -- "$SPACEMIT_ROBOT_WORKSPACE_SETUP")
fi
session="$runtime/${action}_$(date +%Y%m%d_%H%M%S_%N)"
python3 "$script_dir/reconstruction_video_transmission_config.py" "$profile" "$session"
(
    load_session_environment || exit $?
    read_options || exit $?
    share=$(ros2 pkg prefix --share bringup) || exit $?
    python3 "$script_dir/reconstruction_video_transmission_config.py" "$profile" "$session" \
        --validate-arguments "$share/launch/reconstruction_video_transmission_launch.py" \
        "$share/launch/vio_las2_esdf_launch.py" || exit $?
    ros2 launch bringup reconstruction_video_transmission_launch.py \
        "${launch_options[@]}" --show-args >"$session/launch-arguments-help.txt" || exit $?
    python3 - "$session/session.json" "$session/camera.json" "$session/launch-options.json" <<'PY' || exit $?
import json
import sys
with open(sys.argv[1], encoding='utf-8') as stream:
    session = json.load(stream)
with open(sys.argv[2], encoding='utf-8') as stream:
    camera = json.load(stream)
with open(sys.argv[3], encoding='utf-8') as stream:
    options = json.load(stream)
print(f'Resolved camera: {options["camera_config_file"]} '
      f'({camera["camera"]["fps"]} FPS, {camera["timestamp"]["source"]})')
print(f'Resolved calibration source: {session["resources"]["calibration_file"]}')
print(f'Resolved odometry: {session["odometry_mode"]}; use_imu={options["use_imu"]}; '
      f'display-only IMU={options["display_gravity_imu_topic"] or "disabled"}')
print(f'Resolved model: {options["model_path"]}')
print(f'Resolved FPV config: {options["video_config_file"]}')
PY
    video_config=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["video_config_file"])' "$session/launch-options.json") || exit $?
    ros2 run video_transmission video_transmission_node --ros-args \
        -p "config_file:=$video_config" -p check_config_only:=true || exit $?
    affinity=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["cpu_affinity"])' "$session/session.json") || exit $?
    taskset -c "$affinity" true || exit $?
    python3 "$script_dir/reconstruction_video_transmission_config.py" "$profile" "$session" --network-check || exit $?
) >"$session/launch-check.log" 2>&1 || {
    cat "$session/launch-check.log" >&2
    fail "Preflight failed: $session"
}
if [[ $action == check ]]; then
    printf 'CHECK PASS (no camera, renderer, encoder or link started): %s\n' "$session"
    exit 0
fi
device=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["camera"]["device"])' "$session/camera.json")
[[ -c $device ]] || fail "Camera not connected: $device"
headless=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["headless_render"])' "$session/session.json")
if [[ $headless == False ]] && systemctl is-active --quiet display-manager.service; then
    fail 'Desktop owns DRM; stop the display manager explicitly for local DRM display'
fi
case $(state) in
    active|activating|deactivating|reloading)
        [[ $action == restart ]] || fail 'Already running; use restart or stop'
        stop_session ;;
esac
# Stop only our own unit. Other complete ROBOT/camera sessions still conflict.
# Preserve exact kernel comm-name matching for existing session ownership.
# shellcheck disable=SC2009
if ps -e -o comm= | grep -Eq '^(camera_mpp_publ|vio_node|las2_node|esdf_drm_no|video_trans)$'; then
    fail 'Another camera/VIO/LAS2/render/FPV session exists; stop its own launcher first'
fi
private_mpp_ipc=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["private_mpp_ipc"])' "$session/session.json")
# Preserve exact kernel comm-name matching for existing session ownership.
# shellcheck disable=SC2009
if [[ $private_mpp_ipc == False ]] && ps -e -o comm= | grep -Eq '^(fpv_sky|go2_fpv)'; then
    fail 'External FPV session exists; enable private_mpp_ipc or stop its own launcher'
fi
# IPC isolation cannot make two users of the same UVC capture node safe.
python3 - "$device" <<'PY'
import os
from pathlib import Path
import stat
import sys
device = os.stat(sys.argv[1]).st_rdev
owners = []
for process in Path('/proc').iterdir():
    if not process.name.isdigit():
        continue
    try:
        descriptors = list((process / 'fd').iterdir())
    except OSError:
        continue
    for descriptor in descriptors:
        try:
            info = descriptor.stat()
            if stat.S_ISCHR(info.st_mode) and info.st_rdev == device:
                owners.append(f'{process.name} {(process / "comm").read_text().strip()}')
                break
        except OSError:
            continue
if owners:
    raise SystemExit(f'Camera already open by {", ".join(owners)}; stop its owning session first')
PY
python3 "$script_dir/reconstruction_video_transmission_config.py" "$profile" "$session" --network-configure
ipc_properties=()
if [[ $private_mpp_ipc == True ]]; then
    ipc_properties=(--property=PrivateTmp=yes --property=PrivateIPC=yes
        "--property=TemporaryFileSystem=/dev/shm:rw,nosuid,nodev,mode=1777,size=256M")
fi
service_home=$(python3 -c 'import os,pwd; print(pwd.getpwuid(os.geteuid()).pw_dir)')
systemd-run --quiet --collect --unit="$unit" --service-type=exec \
    "${ipc_properties[@]}" \
    --property=KillSignal=SIGINT --property=KillMode=control-group \
    --property=TimeoutStopSec=60 --property=Restart=no \
    --property="WorkingDirectory=$workspace" \
    --property="StandardOutput=append:$session/launch.log" --property=StandardError=inherit \
    --setenv="HOME=$service_home" \
    --setenv="ROS_LOG_DIR=$session/ros" --setenv="ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}" \
    --setenv="ROS_SETUP_FILE=$ROS_SETUP_FILE" \
    --setenv="SPACEMIT_ROBOT_WORKSPACE_SETUP=${SPACEMIT_ROBOT_WORKSPACE_SETUP:-}" \
    /bin/bash "$self" _run "$workspace" "$session" 9>&-
printf '%s\n' "$session" >"$runtime/current-session.new"
mv -- "$runtime/current-session.new" "$runtime/current-session"
printf 'Launch submitted: complete 1920x1080/30 FPS render → FPV\nSession: %s\nLog: %s/launch.log\n' "$session" "$session"
echo 'Submission is not hardware readiness. Use status and launch.log; stop sends SIGINT to all children.'
