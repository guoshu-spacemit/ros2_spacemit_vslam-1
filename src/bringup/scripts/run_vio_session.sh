#!/usr/bin/env bash
# One detached live VIO session per workspace; no algorithm or hardware tuning.
set -euo pipefail
self=$(realpath -e -- "${BASH_SOURCE[0]}")
script_dir=$(dirname -- "$self")
fail() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }
usage() {
    cat <<'EOF'
Usage: run_vio_session.sh {start|restart|stop|status|check|control|play|pause|step} [options]
  --source live|replay|step (default live; step starts paused)
  --session DIR     Original recording directory for replay/step
  --mode auto|vo|vio (default auto: camera profile / original recording)
  --workspace DIR   Built workspace (or SPACEMIT_ROBOT_WORKSPACE_ROOT)
  --config FILE     Live profile JSON (or SPACEMIT_ROBOT_SESSION_CONFIG)
  --mapping on|off  LAS2 + TSDF + ESDF (default off)
  --slam on|off     Global SLAM / loop closure (default off)
  --record on|off   Original MJPEG + IMU + timestamps (default off)
  --agc on|off      Fixed short exposure + auto gain (default on for live; off for replay)
  --features on|off Feature image + all diagnostic text together (default off)
  --fps 30|60       Camera capture mode (default 60)

DP left image + trajectory remain active with mapping off.
The public run_vio_session.sh defaults to ROS_DOMAIN_ID=1 and ROS_LOCALHOST_ONLY=1.
Direct internal-controller use defaults to domain 0. Stop gracefully to close recordings.
check evaluates configuration/launch without opening the camera or DRM.
In a TTY, run_vio_session.sh start/restart --source step automatically opens keyboard control.
Space: play/pause; Down: next frame; Right: forward 5s; Q: detach (not stop).
Set SPACEMIT_ROBOT_REPLAY_KEYBOARD=false to keep step startup noninteractive.
Runtime overrides: ROS_SETUP_FILE, SPACEMIT_ROBOT_WORKSPACE_SETUP, SPACEMIT_ROBOT_CPU_AFFINITY (0-7).
The workspace may provide setup_dependencies.bash for MPP/EP/library paths.
EOF
}
[[ $# -gt 0 ]] || { usage; exit 2; }
action=$1
shift
[[ $action != --help && $action != -h ]] || { usage; exit 0; }

runtime_environment() {
    local requested_domain=${ROS_DOMAIN_ID:-0}
    local requested_localhost=${ROS_LOCALHOST_ONLY:-0}
    # Do not accidentally inherit another workspace's runtime libraries.
    unset LD_PRELOAD AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
    unset LD_LIBRARY_PATH PYTHONPATH ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
    set +u
    # This setup file is selected by the deployment, outside the source tree.
    # shellcheck source=/dev/null
    source "${ROS_SETUP_FILE:-/opt/ros/humble/setup.bash}"
    if [[ -f $workspace/setup_dependencies.bash ]]; then
        # shellcheck source=/dev/null
        source "$workspace/setup_dependencies.bash"
    fi
    # This setup file is selected by the deployment, outside the source tree.
    # shellcheck source=/dev/null
    source "$SPACEMIT_ROBOT_WORKSPACE_SETUP"
    # shellcheck source=/dev/null
    source "$script_dir/sdk_paths.sh"
    spacemit_robot_sdk_paths "$workspace"
    set -u
    export ROS_DOMAIN_ID="$requested_domain" ROS_LOCALHOST_ONLY="$requested_localhost"
}

if [[ $action == _run ]]; then
    [[ $# == 2 ]] || fail 'Invalid internal session arguments'
    workspace=$1
    export SPACEMIT_ROBOT_VIO_SESSION_DIR=$2
    # Also retain setup/affinity failures before the checked supervisor starts.
    trap 'python3 - "$SPACEMIT_ROBOT_VIO_SESSION_DIR/pipeline-status.json" <<"PY"
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
    runtime_environment
    taskset -c "${SPACEMIT_ROBOT_CPU_AFFINITY:-0-7}" true
    if [[ $(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("source", "live"))' \
            "$SPACEMIT_ROBOT_VIO_SESSION_DIR/session.json") != live ]]; then
        export SPACEMIT_ROBOT_WORKSPACE_ROOT="$workspace"
        exec taskset -c "${SPACEMIT_ROBOT_CPU_AFFINITY:-0-7}" python3 \
            "$script_dir/replay_session_config.py" "$SPACEMIT_ROBOT_VIO_SESSION_DIR" --run
    fi
    python3 "$script_dir/calibration_bundle.py" "$SPACEMIT_ROBOT_VIO_SESSION_DIR"
    launch_share=$(ros2 pkg prefix --share bringup)
    [[ -f $launch_share/launch/vio_session_launch.py ]] || fail 'Session launch missing'
    exec taskset -c "${SPACEMIT_ROBOT_CPU_AFFINITY:-0-7}" python3 \
        "$script_dir/run_checked_launch.py" \
        --status-file "$SPACEMIT_ROBOT_VIO_SESSION_DIR/pipeline-status.json" \
        "$launch_share/launch/vio_session_launch.py"
fi

case $action in start|restart|stop|status|check|control|play|pause|step) ;; *) usage; exit 2 ;; esac
workspace=${SPACEMIT_ROBOT_WORKSPACE_ROOT:-}
profile=${SPACEMIT_ROBOT_SESSION_CONFIG:-}
mapping=off
slam=off
record=off
agc=on
agc_explicit=false
fps=60
source_kind=live
source_session=
mode=auto
fps_explicit=false
display_options=(--features off)
while (( $# )); do
    [[ $# -ge 2 ]] || fail "Missing value for $1"
    case $1 in
        --source) source_kind=$2 ;;
        --session) source_session=$2 ;;
        --mode) mode=$2 ;;
        --workspace) workspace=$2 ;;
        --config)
            [[ $action != stop && $action != status ]] || fail "$action takes only --workspace"
            profile=$2 ;;
        --mapping|--slam|--record|--agc)
            [[ $action != stop && $action != status ]] || fail "$action takes only --workspace"
            [[ $2 == on || $2 == off ]] || fail "$1 requires on or off"
            case $1 in
                --mapping) mapping=$2 ;; --slam) slam=$2 ;; --record) record=$2 ;;
                --agc) agc=$2; agc_explicit=true ;;
            esac ;;
        --features)
            [[ $action != stop && $action != status ]] || fail "$action takes only --workspace"
            [[ $2 == on || $2 == off ]] || fail "$1 requires on or off"
            display_options+=("$1" "$2") ;;
        --fps)
            [[ $action != stop && $action != status ]] || fail "$action takes only --workspace"
            [[ $2 == 30 || $2 == 60 ]] || fail '--fps requires 30 or 60'
            fps=$2; fps_explicit=true ;;
        *) fail "Unknown option: $1" ;;
    esac
    shift 2
done
[[ $source_kind == live || $source_kind == replay || $source_kind == step ]] || fail 'Invalid --source'
[[ $mode == auto || $mode == vo || $mode == vio ]] || fail 'Invalid --mode'
[[ -n $workspace ]] || fail 'Supply --workspace or SPACEMIT_ROBOT_WORKSPACE_ROOT'
workspace=$(realpath -e -- "$workspace")
# shellcheck source=/dev/null
source "$script_dir/sdk_paths.sh"
spacemit_robot_sdk_paths "$workspace"
[[ $EUID == 0 ]] || fail 'Run as root for K3 DMA/DRM access'
runtime="$workspace/.run/vio_session"
mkdir -p "$runtime"
unit="robot_perception-vio-$(printf '%s' "$workspace" | sha256sum | cut -c1-12).service"
exec 9>"$runtime/control.lock"
flock -n 9 || fail 'Another controller command is in progress'
state() { systemctl show "$unit" -p ActiveState --value 2>/dev/null || true; }
stop_session() {
    case $(state) in
        active|activating|deactivating|reloading) systemctl stop "$unit" ;;
    esac
}
case $action in
    control|play|pause|step)
        [[ -f $runtime/current-session ]] || fail 'No session'
        session=$(<"$runtime/current-session")
        export ROS_DOMAIN_ID
        ROS_DOMAIN_ID=$(python3 - "$session/session.json" <<'PY'
import json, sys
value = json.load(open(sys.argv[1]))
if value.get('source', 'live') == 'live':
    raise SystemExit('Current session is live, not replay')
print(value['domain'])
PY
        )
        export ROS_LOCALHOST_ONLY
        ROS_LOCALHOST_ONLY=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("localhost_only", "0"))' "$session/session.json")
        runtime_environment
        flock -u 9
        if [[ $action == control ]]; then
            keyboard_options=()
            if [[ ${SPACEMIT_ROBOT_REPLAY_UI:-plain} == tui ]]; then
                keyboard_options+=(--ui tui)
            else
                printf '\n逐帧回放控制 · 空格 播放/暂停 · ↓ 单帧 · → 前进5秒 · Q 返回（不停止任务）\n'
            fi
            if [[ -n ${SPACEMIT_ROBOT_REPLAY_KEYBOARD_RESULT:-} ]]; then
                keyboard_options+=(--result-file "$SPACEMIT_ROBOT_REPLAY_KEYBOARD_RESULT")
            fi
            exec python3 "$script_dir/replay_keyboard.py" \
                --connect-timeout-sec "${SPACEMIT_ROBOT_REPLAY_CONNECT_TIMEOUT_SEC:-120}" \
                "${keyboard_options[@]}"
        fi
        [[ $action != play ]] || action=resume
        exec python3 "$script_dir/replay_command.py" "$action" ;;
esac
if [[ $action == status ]]; then
    systemctl show "$unit" -p ActiveState -p SubState -p MainPID -p Result || true
    [[ -f $runtime/current-session ]] || { echo 'No session submitted.'; exit 0; }
    session=$(<"$runtime/current-session")
    printf 'Session: %s\nLog: %s/launch.log\n' "$session" "$session"
    python3 - "$session" <<'PY'
import json
from pathlib import Path
import sys
directory = Path(sys.argv[1])
if json.loads((directory / 'session.json').read_text())['record']:
    print(f'Recording: {directory}')
status = directory / 'pipeline-status.json'
for name in ('result/replay_summary.json', 'result/replay_check.json'):
    path = directory / name
    if path.is_file():
        print(path.read_text())
if status.exists():
    result = json.loads(status.read_text())
    print(json.dumps(result, indent=2))
    sys.exit(1 if result['state'].startswith('failed') else 0)
print('Pipeline status not yet written; inspect launch.log, not a readiness confirmation.')
PY
    exit $?
fi
if [[ $action == stop ]]; then
    stop_session
    echo 'Stopped (or already stopped). Logs and recordings retained.'
    exit 0
fi
if [[ $source_kind == live ]]; then
    [[ -f $profile ]] || fail 'Supply --config or SPACEMIT_ROBOT_SESSION_CONFIG'
    [[ -z $source_session ]] || fail 'Replay options cannot be used with live input'
else
    [[ $agc_explicit == true ]] || agc=off
    [[ -d $source_session ]] || fail '--session must be an existing recording directory'
    [[ $record == off && $agc == off ]] || fail 'Recording/AGC options are live-only'
    [[ $fps_explicit == false && $slam == off ]] || fail 'Replay preserves recorded FPS/SLAM; do not override them here'
fi
domain=${ROS_DOMAIN_ID:-0}
[[ $domain =~ ^[0-9]{1,3}$ ]] || fail 'Invalid ROS_DOMAIN_ID'
(( 10#$domain <= 232 )) || fail 'ROS_DOMAIN_ID must be 0..232'
export ROS_DOMAIN_ID=$((10#$domain))
[[ -f $SPACEMIT_ROBOT_WORKSPACE_SETUP ]] || fail 'Workspace setup missing'
export ROS_SETUP_FILE
ROS_SETUP_FILE=$(realpath -e -- "${ROS_SETUP_FILE:-/opt/ros/humble/setup.bash}")
export SPACEMIT_ROBOT_WORKSPACE_SETUP
SPACEMIT_ROBOT_WORKSPACE_SETUP=$(realpath -e -- "$SPACEMIT_ROBOT_WORKSPACE_SETUP")
for tool in systemctl systemd-run taskset; do command -v "$tool" >/dev/null || fail "Missing $tool"; done
taskset -c "${SPACEMIT_ROBOT_CPU_AFFINITY:-0-7}" true || fail 'Invalid/unavailable SPACEMIT_ROBOT_CPU_AFFINITY'
session="$runtime/${action}_$(date +%Y%m%d_%H%M%S_%N)"
if [[ $record == on && $action != check ]]; then
    session="$workspace/recordings/recording_$(date +%Y%m%d_%H%M%S_%N)"
fi
if [[ $source_kind == live ]]; then
python3 "$script_dir/vio_session_config.py" "$profile" "$session" --mode "$mode" \
    --mapping "$mapping" --slam "$slam" --record "$record" --fps "$fps" \
    --domain "$ROS_DOMAIN_ID" --agc "$agc" \
    "${display_options[@]}"
(
    runtime_environment
    python3 "$script_dir/check_vio_runtime.py" --mapping "$mapping"
    export SPACEMIT_ROBOT_VIO_SESSION_DIR="$session"
    ros2 launch bringup vio_session_launch.py --show-args
) >"$session/launch-check.log" 2>&1 || {
    cat "$session/launch-check.log" >&2; fail "Launch validation failed: $session";
}
else
    runtime_environment
    python3 "$script_dir/check_vio_runtime.py" --mapping "$mapping"
    python3 "$script_dir/check_vio_recording.py" "$source_session" --allow-legacy-video-summary
    replay_options=()
    [[ $source_kind != step ]] || replay_options+=(--paused)
    python3 "$script_dir/replay_session_config.py" "$session" --source "$source_session" \
        --profile "$profile" --mode "$mode" --mapping "$mapping" \
        --domain "$ROS_DOMAIN_ID" "${display_options[@]}" "${replay_options[@]}"
fi
python3 - "$session/session.json" <<'PY'
import json, os, sys
from pathlib import Path
path = Path(sys.argv[1])
value = json.loads(path.read_text())
value['localhost_only'] = os.environ.get('ROS_LOCALHOST_ONLY', '0')
path.write_text(json.dumps(value, indent=2) + '\n')
PY
if [[ $action == check ]]; then
    printf 'CHECK PASS (no hardware opened): %s\n' "$session"
    exit 0
fi
if [[ $source_kind == live ]]; then
    device=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["camera"]["device"])' "$session/camera.json")
    [[ -c $device ]] || fail "Camera not connected: $device"
fi
if systemctl is-active --quiet display-manager.service; then
    fail 'Desktop owns DRM; stop the display manager explicitly before starting'
fi
case $(state) in
    active|activating|deactivating|reloading)
        [[ $action == restart ]] || fail 'Already running; use restart or stop'
        stop_session ;;
esac
# Preserve exact kernel comm-name matching for existing session ownership.
# shellcheck disable=SC2009
if ps -C camera_mpp_publisher,session_replay_publisher,vio_node,esdf_drm_node -o pid= | grep -q '[0-9]'; then
    fail 'Another camera/VIO/DRM session exists; stop its own launcher first'
fi
sdk_environment=()
for name in SPACEMIT_ROBOT_SDK_ROOT SPACEMIT_ROBOT_SDK_PREFIX SROBOTIS_ROOT \
    MPP_LIBRARY_DIR MPP_PLUGIN_DIR SPACEMIT_VSLAM_LIBRARY_DIR SPACEMIT_MAPPING_LIBRARY_DIR \
    LAS2_LIBRARY_DIR LAS2_ORT_LIBRARY_DIR LAS2_ORT_ROOT; do
    [[ -z ${!name:-} ]] || sdk_environment+=("--setenv=$name=${!name}")
done
systemd-run --quiet --collect --unit="$unit" --service-type=exec \
    --property=KillSignal=SIGINT --property=KillMode=control-group \
    --property=TimeoutStopSec=60 --property=Restart=no \
    --property="WorkingDirectory=$workspace" \
    --property="StandardOutput=append:$session/launch.log" --property=StandardError=inherit \
    --setenv="ROS_LOG_DIR=$session/ros-log" --setenv="ROS_DOMAIN_ID=$ROS_DOMAIN_ID" \
    --setenv="ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}" \
    --setenv="ROS_SETUP_FILE=${ROS_SETUP_FILE:-/opt/ros/humble/setup.bash}" \
    --setenv="SPACEMIT_ROBOT_WORKSPACE_SETUP=$SPACEMIT_ROBOT_WORKSPACE_SETUP" \
    --setenv="SPACEMIT_ROBOT_CPU_AFFINITY=${SPACEMIT_ROBOT_CPU_AFFINITY:-0-7}" \
    "${sdk_environment[@]}" \
    /bin/bash "$self" _run "$workspace" "$session" 9>&-
printf '%s\n' "$session" >"$runtime/current-session.new"
mv -- "$runtime/current-session.new" "$runtime/current-session"
printf 'Source=%s mode=%s (auto preserves configuration)\n' "$source_kind" "$mode"
[[ $source_kind == live ]] || fps=recorded
printf 'Launch submitted: mapping=%s slam=%s record=%s fps=%s domain=%s agc=%s\n' \
    "$mapping" "$slam" "$record" "$fps" "$ROS_DOMAIN_ID" "$agc"
printf 'Session: %s\nLog: %s/launch.log\n' "$session" "$session"
[[ $record == off ]] || printf 'Recording: %s\n' "$session"
echo 'Submission is not readiness. Use status / launch.log; stop drains the recorder.'
