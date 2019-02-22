#!/bin/bash

# Copyright (c) 2015, Isaac I. Y. Saito
# Copyright (c) 2017, Mathias Lüdtke
# All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
## Greatly inspired by JSK travis https://github.com/jsk-ros-pkg/jsk_travis

# source_tests.sh script runs integration tests for the target ROS packages.
# It is dependent on environment variables that need to be exported in advance
# (As of version 0.4.4 most of them are defined in env.sh).


function exec_in_workspace {
    local extend=$1; shift
    local path=$1; shift
    ( { [ ! -e "$extend/setup.bash" ] || source "$extend/setup.bash"; } && cd "$path" && exec "$@")
}

function setup_rosdep {
    # Setup rosdep
    rosdep --version
    if ! [ -d /etc/ros/rosdep/sources.list.d ]; then
        sudo rosdep init
    fi

    update_opts=()
    case "$ROS_DISTRO" in
    "jade")
        if rosdep update --help | grep -q -- --include-eol-distros; then
          update_opts+=(--include-eol-distros)
        fi
        ;;
    esac

    ici_retry 2 rosdep update "${update_opts[@]}"
}

function resolve_scheme {
    local url=$1; shift
    if [[ $url =~ ([^:]+):([^#]+)#(.+) ]]; then
        local fragment="${BASH_REMATCH[3]}"
        local repo=${BASH_REMATCH[2]}
        local name=${repo##*/}
        local scheme=${BASH_REMATCH[1]}

        case "$scheme" in
            bitbucket)
                echo "$name" "git" "https://bitbucket.org/$repo" "$fragment"
                ;;
            github)
                echo "$name" "git" "https://github.com/$repo" "$fragment"
                ;;
            gitlab)
                echo "$name" "git" "https://gitlab.com/$repo" "$fragment"
                ;;
            git+*)
                echo "$name" "git" "$scheme:$repo" "$fragment"
                ;;
            *)
                echo "$name" "$scheme" "$scheme:$repo" "$fragment"
                ;;
        esac
    else
        error "could not parse URL '$url'"
    fi

}

function vcs_import_repository {
    local sourcespace=$1; shift
    local url=$1; shift

    local -a parts
    parts=($(resolve_scheme "$url"))
    vcs import "$sourcespace" <<< "{repositories: {'${parts[0]}': {type: '${parts[1]}', url: '${parts[2]}', version: '${parts[3]}'}}}"
}

function vcs_import_file {
    local sourcespace=$1; shift
    local file=$1; shift

    if [ -e "$TARGET_REPO_PATH/$file.$ROS_DISTRO" ]; then
        # install (maybe unreleased version) dependencies from source for specific ros version
        vcs import "$sourcespace" < "$TARGET_REPO_PATH/$file.$ROS_DISTRO"
    elif [ -e "$TARGET_REPO_PATH/$file" ]; then
        # install (maybe unreleased version) dependencies from source
        vcs import "$sourcespace" < "$TARGET_REPO_PATH/$file"
    else
        error "UPSTREAM_WORKSPACE file '$file[.$ROS_DISTRO]' does not exist"
    fi
}

function prepare_sourcespace {
    local sourcespace="$1"; shift

    mkdir -p "$sourcespace"

    for source in "$@"; do
        case "$source" in
        debian)
            echo "Obtain deb binary for sources packages."
            ;;
        file) # When UPSTREAM_WORKSPACE is file, the dependended packages that need to be built from source are downloaded based on $ROSINSTALL_FILENAME file.
            vcs_import_file "$sourcespace" "$ROSINSTALL_FILENAME"
            ;;
        git* | bitbucket*)
            vcs_import_repository "$sourcespace" "$source"
            ;;
        http://* | https://*) # When UPSTREAM_WORKSPACE is an http url, use it directly
            set -o pipefail
            wget -O- -q "$source" | vcs import "$sourcespace"
            set +o pipefail
            ;;
        -*)
            rm -rf "${sourcespace:?}/${source:1}"
            ;;
        /*)
            if [ -d "$source" ]; then
                cp -a "$source" "$sourcespace"
            else
                error "'$source' is not a directory"
            fi
            ;;
        "")
            error "source is empty string"
            ;;
        *)
            if [ -d "$TARGET_REPO_PATH/$source" ]; then
                cp -a "$TARGET_REPO_PATH/$source" "$sourcespace"
            elif [ -f "$TARGET_REPO_PATH/$source" ]; then
                vcs_import_file "$sourcespace" "$source"
            else
                error "cannot read source from '$source'"
            fi
            ;;
        esac
    done
}

function install_dependencies {
    local extend=$1; shift
    local skip_keys=$1; shift
    rosdep_opts=(-q --from-paths "$@" --ignore-src -y)
    if [ -n "$skip_keys" ]; then
      rosdep_opts+=(--skip-keys "$skip_keys")
    fi
    set -o pipefail # fail if rosdep install fails
    exec_in_workspace "$extend" "." rosdep install "${rosdep_opts[@]}" | { grep "executing command" || true; }
    set +o pipefail
}

function build_workspace {
    local name=$1; shift
    local extend=$1; shift
    local ws=$1; shift

    local ws_env="${name^^}_WORKSPACE"
    local sources=("$@" ${!ws_env})
    ici_run "setup_${name}_workspace" prepare_sourcespace "$ws/src" "${sources[@]}"
    ici_run "install_${name}_dependencies" install_dependencies "$extend" "$ROSDEP_SKIP_KEYS" "$ws/src"
    ici_run "build_${name}_workspace" builder_run_build "$extend" "$ws"
}

function test_workspace {
    local name=$1; shift
    local extend=$1; shift
    local ws=$1; shift

    ici_run "run_${name}_test" builder_run_tests "$extend" "$ws"
    builder_test_results "$extend" "$ws"
}
