#!/bin/bash

# Copyright (c) 2015, Isaac I. Y. Saito
# Copyright (c) 2017, Mathias Luedtke
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

# execute BEFORE_SCRIPT in repository, exit on errors

function run_catkin_lint {
    local path=$1; shift
    sudo pip install catkin-lint
    catkin_lint --explain "$@" "$path" && echo "catkin_lint passed." || error "catkin_lint failed by either/both errors and/or warnings"
}

function prepare_source_tests {
    ici_time_start setup_apt
    sudo apt-get update -qq

    # If more DEBs needed during preparation, define ADDITIONAL_DEBS variable where you list the name of DEB(S, delimitted by whitespace)
    if [ "$ADDITIONAL_DEBS" ]; then
        sudo apt-get install -qq -y $ADDITIONAL_DEBS || error "One or more additional deb installation is failed. Exiting."
    fi
    ici_time_end  # setup_apt

    if [ "$CCACHE_DIR" ]; then
        ici_time_start setup_ccache
        sudo apt-get install -qq -y ccache || error "Could not install ccache. Exiting."
        export PATH="/usr/lib/ccache:$PATH"
        ici_time_end  # setup_ccache
    fi

    ici_time_start setup_rosdep

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

    ici_time_end  # setup_rosdep
}

function vcs_import_file {
    local ws=$1; shift
    local file=$1; shift

    if [ -e "$TARGET_REPO_PATH/$file.$ROS_DISTRO" ]; then
        # install (maybe unreleased version) dependencies from source for specific ros version
        vcs import "$ws" < "$TARGET_REPO_PATH/$file.$ROS_DISTRO"
    elif [ -e "$TARGET_REPO_PATH/$file" ]; then
        # install (maybe unreleased version) dependencies from source
        vcs import "$ws" < "$TARGET_REPO_PATH/$file"
    else
        error "UPSTREAM_WORKSPACE file '$file[.$ROS_DISTRO]' does not exist"
    fi

}

function prepare_upstream {
    local sourcespace="$1"; shift

    mkdir -p "$sourcespace"

    for upstream in "$@"; do
        case "$upstream" in
        debian)
            echo "Obtain deb binary for upstream packages."
            ;;
        file) # When UPSTREAM_WORKSPACE is file, the dependended packages that need to be built from source are downloaded based on $ROSINSTALL_FILENAME file.
            vcs_import_file "$sourcespace" "$ROSINSTALL_FILENAME"
            ;;
        http://* | https://*) # When UPSTREAM_WORKSPACE is an http url, use it directly
            set -o pipefail
            wget -O- -q "$upstream" | vcs import "$sourcespace"
            set +o pipefail
            ;;
        -*)
            rm -rf  "$sourcespace/${upstream:1}"
            ;;
        *)
            vcs_import_file "$sourcespace" "$upstream"
            ;;
        esac
    done
}

function build_workspace {
    local name=$1; shift
    local ws=$1; shift
    local extend=$1; shift

    ici_time_start "setup_${name}_workspace"
    mkdir -p "$ws/src"
    prepare_upstream "$ws/src" $*
    ici_time_end # "setup_$name_workspace"

    ici_time_start "${name}_rosdep_install"

    rosdep_opts=(-q --from-paths "$ws/src" --ignore-src --rosdistro $ROS_DISTRO -y)
    if [ -n "$ROSDEP_SKIP_KEYS" ]; then
      rosdep_opts+=(--skip-keys "$ROSDEP_SKIP_KEYS")
    fi
    set -o pipefail # fail if rosdep install fails
    ([ -e "$extend/setup.bash" ] && source "$extend/setup.bash"; exec rosdep install "${rosdep_opts[@]}") #| { grep "executing command" || true; }
    set +o pipefail
    ici_time_end  # {name}_rosdep_install

    ici_time_start  "build_${name}_workspace"
    (source "$extend/setup.bash"; cd "$ws"; exec colcon build)
    ici_time_end  # ${name}_rosdep_install
}

function test_workspace {
    local name=$1; shift
    local ws=$1; shift
    local extend=$1; shift

    ici_time_start "run_${name}_tests"
    (source "$extend/setup.bash"; cd "$ws"; exec colcon test)
    (source "$extend/setup.bash"; cd "$ws"; exec colcon test-result --all )
    ici_time_end # "run_${name}_tests"
}

function run_source_tests {
    ici_require_run_in_docker # this script must be run in docker

    prepare_source_tests

    local OPT_VI
    if [ "$VERBOSE_OUTPUT" == true ]; then
        OPT_VI="-vi"
    fi
    local OPT_RUN_V
    if [ "$VERBOSE_TESTS" != false ]; then
        OPT_RUN_V="-v"
    fi

    export CATKIN_WORKSPACE=~/catkin_ws
    local upstream_workspace=~/upstream_ws
    local extend="/opt/ros/$ROS_DISTRO"

    if [ -n "$UPSTREAM_WORKSPACE" ]; then
        build_workspace upstream "$upstream_workspace" "$extend" $UPSTREAM_WORKSPACE
        extend="$upstream_workspace/install"
    fi

    mkdir -p "$CATKIN_WORKSPACE/src"
    cp -a $TARGET_REPO_PATH "$CATKIN_WORKSPACE/src"

    if [ "${USE_MOCKUP// }" != "" ]; then
        if [ ! -d "$TARGET_REPO_PATH/$USE_MOCKUP" ]; then
            error "mockup directory '$USE_MOCKUP' does not exist"
        fi
        ln -sf "$TARGET_REPO_PATH/$USE_MOCKUP" "$CATKIN_WORKSPACE/src"
    fi
    if [ "$CATKIN_LINT" == "true" ] || [ "$CATKIN_LINT" == "pedantic" ]; then
        ici_time_start catkin_lint
        local catkin_lint_args=($CATKIN_LINT_ARGS)
        if [ "$CATKIN_LINT" == "pedantic" ]; then
        	catkin_lint_args+=(--strict -W2)
        fi
        run_catkin_lint "$TARGET_REPO_PATH" "${catkin_lint_args[@]}"
        ici_time_end  # catkin_lint
    fi
    build_workspace target "$CATKIN_WORKSPACE" "$extend"

    if [ "$NOT_TEST_BUILD" != "true" ]; then
        test_workspace target "$CATKIN_WORKSPACE" "$extend"
    fi

    extend="$CATKIN_WORKSPACE/install"

    if [ -n "$DOWNSTREAM_WORKSPACE" ]; then
        local downstream_workspace=~/downstream_ws
        build_workspace downstream "$downstream_workspace" "$extend" $DOWNSTREAM_WORKSPACE

        if [ "$NOT_TEST_DOWNSTREAM" != "true" ]; then
            test_workspace downstream "$downstream_workspace" "$extend"
        fi
    fi
}
