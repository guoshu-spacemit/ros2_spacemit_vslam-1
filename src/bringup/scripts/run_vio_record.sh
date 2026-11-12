#!/usr/bin/env bash
# Same runtime overrides as run_headless_esdf_rtsp.sh; dedicated recording profile.
set -euo pipefail
SCRIPT_DIR=$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")
export SPACEMIT_ROBOT_RUN_DIR=${SPACEMIT_ROBOT_RUN_DIR:-/tmp/vio_record}
# The recording launch controls mapping and display through enable_screen.
export SPACEMIT_ROBOT_ENABLE_RTSP=false SPACEMIT_ROBOT_ENABLE_ESDF_RTSP=false
export SPACEMIT_ROBOT_PUBLISH_LAS2_PREVIEW=false

if [[ -z ${SPACEMIT_ROBOT_RECORD_SESSION_DIR:-} ]]; then
    RECORD_OUTPUT_DIR=${SPACEMIT_ROBOT_RECORD_OUTPUT_DIR:-}
    if [[ -z ${RECORD_OUTPUT_DIR} ]]; then
        WORKSPACE_ROOT=${SPACEMIT_ROBOT_WORKSPACE_ROOT:?Set SPACEMIT_ROBOT_WORKSPACE_ROOT}
        RECORD_OUTPUT_DIR="${WORKSPACE_ROOT}/recordings"
    fi
    RECORDING_NAME="recording_$(date +%Y%m%d_%H%M%S_%N)"
    SPACEMIT_ROBOT_RECORD_SESSION_DIR="${RECORD_OUTPUT_DIR}/${RECORDING_NAME}"
fi
export SPACEMIT_ROBOT_RECORD_SESSION_DIR
exec "${SCRIPT_DIR}/run_headless_esdf_rtsp.sh" "$@"
