#!/usr/bin/env bash
# Shared dependency paths for an SDK checkout or a relocated product release.

# Select the packaged runtime, or the explicitly supplied external ORT/EP pair.
spacemit_robot_las2_preload() {
    local directory=${1:?Expected the LAS2 runtime library directory}
    if [[ -f $directory/libstereo_matching_runtime.so ]]; then
        printf '%s\n' "$directory/libstereo_matching_runtime.so"
    else
        printf '%s:%s\n' "$directory/libonnxruntime.so.1" "$directory/libspacemit_ep.so.2"
    fi
}

# Print the source workspace or release root for the supplied script directory.
spacemit_robot_default_workspace() {
    local scripts=${1:?Expected the bringup script directory}
    # The six-package source workspace has one less directory level than the
    # delivered src/<project>/bringup layout. Keep installed launchers compatible.
    if [[ -f $scripts/../../../src/bringup/package.xml ]]; then
        (cd -- "$scripts/../../.." && pwd)
    else
        (cd -- "$scripts/../../../.." && pwd)
    fi
}

# Export existing SDK/release dependency paths; never install or modify libraries.
# The workspace argument supplies the relocation fallback when no SDK is found.
spacemit_robot_sdk_paths() {
    local workspace=${1:?Expected the perception workspace}
    local helper_dir candidate sdk_root prefix
    helper_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
    sdk_root=${SPACEMIT_ROBOT_SDK_ROOT:-${SROBOTIS_ROOT:-}}
    if [[ -z $sdk_root ]]; then
        for candidate in "${SPACEMIT_ROBOT_SOURCE_ROOT:-}" "$helper_dir" "$workspace"; do
            [[ -n $candidate ]] || continue
            candidate=$(realpath -m -- "$candidate")
            while [[ $candidate != / ]]; do
                if [[ -d $candidate/components/multimedia/mpp &&
                      -d $candidate/components/perception/stereo_matching ]]; then
                    sdk_root=$candidate
                    break 2
                fi
                candidate=$(dirname -- "$candidate")
            done
        done
    fi
    prefix=${SPACEMIT_ROBOT_SDK_PREFIX:-${PREFIX:-}}
    if [[ -z $prefix ]]; then
        if [[ -n ${SPACEMIT_ROBOT_BUNDLE_ROOT:-} ]]; then
            prefix=$SPACEMIT_ROBOT_BUNDLE_ROOT
        elif [[ -d $workspace/lib ]]; then
            prefix=$workspace
        elif [[ -f $helper_dir/../../share/ament_index/resource_index/packages/bringup ]]; then
            prefix=$(cd -- "$helper_dir/../.." && pwd)
        elif [[ -n $sdk_root ]]; then
            prefix=$sdk_root/output/staging
        else
            prefix=$workspace/install
        fi
    fi
    export SPACEMIT_ROBOT_SDK_PREFIX=$prefix
    export SPACEMIT_ROBOT_WORKSPACE_SETUP=${SPACEMIT_ROBOT_WORKSPACE_SETUP:-$prefix/setup.bash}
    if [[ -n $sdk_root ]]; then
        export SPACEMIT_ROBOT_SDK_ROOT=$sdk_root
    fi
    export MPP_LIBRARY_DIR=${MPP_LIBRARY_DIR:-$prefix/lib}
    export SPACEMIT_VSLAM_LIBRARY_DIR=${SPACEMIT_VSLAM_LIBRARY_DIR:-$prefix/lib}
    export SPACEMIT_MAPPING_LIBRARY_DIR=${SPACEMIT_MAPPING_LIBRARY_DIR:-$prefix/lib}
    export LAS2_LIBRARY_DIR=${LAS2_LIBRARY_DIR:-$prefix/lib}
    local default_ort_dir=${LAS2_ORT_ROOT:+$LAS2_ORT_ROOT/lib}
    export LAS2_ORT_LIBRARY_DIR=${LAS2_ORT_LIBRARY_DIR:-$default_ort_dir}
    export LAS2_ORT_LIBRARY_DIR=${LAS2_ORT_LIBRARY_DIR:-$prefix/lib}
    export MPP_PLUGIN_DIR=${MPP_PLUGIN_DIR:-$MPP_LIBRARY_DIR}
    if [[ -z ${MPP_MESSAGES_SETUP:-} ]]; then
        if [[ -f $prefix/share/mpp_msgs/local_setup.bash ]]; then
            export MPP_MESSAGES_SETUP=$prefix/share/mpp_msgs/local_setup.bash
        else
            MPP_MESSAGES_SETUP=${SPACEMIT_ROBOT_WORKSPACE_SETUP:-$workspace/install/setup.bash}
            export MPP_MESSAGES_SETUP
        fi
    fi
    export LAS2_MODEL_PATH="${LAS2_MODEL_PATH:-$HOME/.cache/models/vision/LAS2_M_288x384.fp16.onnx}"
    local library_path entry existing
    local -a library_entries unique_entries
    library_path="$MPP_LIBRARY_DIR:$SPACEMIT_VSLAM_LIBRARY_DIR"
    library_path+=":$SPACEMIT_MAPPING_LIBRARY_DIR:$LAS2_LIBRARY_DIR"
    library_path+=":$LAS2_ORT_LIBRARY_DIR:${LD_LIBRARY_PATH:-}"
    IFS=: read -r -a library_entries <<< "$library_path"
    unique_entries=()
    for entry in "${library_entries[@]}"; do
        [[ -n $entry ]] || continue
        existing=false
        for candidate in "${unique_entries[@]}"; do
            [[ $candidate != "$entry" ]] || { existing=true; break; }
        done
        [[ $existing == true ]] || unique_entries+=("$entry")
    done
    LD_LIBRARY_PATH=$(IFS=:; printf '%s' "${unique_entries[*]}")
    export LD_LIBRARY_PATH
}
