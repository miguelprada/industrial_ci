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
    source /opt/ros/$ROS_DISTRO/setup.bash
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
        *)
            vcs_import_file "$sourcespace" "$upstream"
            ;;
        esac
    done
}

function run_source_tests {
    ici_require_run_in_docker # this script must be run in docker

    prepare_source_tests

    local opt_vi
    if [ "$VERBOSE_OUTPUT" == true ]; then
        opt_vi="-vi"
    fi
    local opt_run_v
    if [ "$VERBOSE_TESTS" != false ]; then
        opt_run_v="-v"
    fi

    ici_time_start setup_workspace
    export CATKIN_WORKSPACE=~/catkin_ws
    prepare_upstream "$CATKIN_WORKSPACE/src/_upstream" $UPSTREAM_WORKSPACE

    for p in $(catkin_topological_order "${TARGET_REPO_PATH}" --only-names); do
        local dup=$(catkin_find_pkg "$p" "$CATKIN_WORKSPACE/src" 2> /dev/null)
        if [ -n "$dup" ]; then
            ici_warn "removing duplicated package '$p' ($dup)"
            rm -rf "$dup"
        fi
    done

    # TARGET_REPO_PATH is the path of the downstream repository that we are testing. copy it to the catkin workspace
    cp -a $TARGET_REPO_PATH "$CATKIN_WORKSPACE/src"

    if [ "${USE_MOCKUP// }" != "" ]; then
        if [ ! -d "$TARGET_REPO_PATH/$USE_MOCKUP" ]; then
            error "mockup directory '$USE_MOCKUP' does not exist"
        fi
        ln -sf "$TARGET_REPO_PATH/$USE_MOCKUP" "$CATKIN_WORKSPACE/src"
    fi

    catkin config -w "$CATKIN_WORKSPACE"  --install
    if [ -n "$CATKIN_CONFIG" ]; then eval catkin config -w "$CATKIN_WORKSPACE" $CATKIN_CONFIG; fi
    ici_time_end  # setup_workspace

    if [ "${BEFORE_SCRIPT// }" != "" ]; then
      ici_time_start before_script
      bash -e -c "cd $TARGET_REPO_PATH; ${BEFORE_SCRIPT}"
      ici_time_end  # before_script
    fi

    ici_time_start rosdep_install

    rosdep_opts=(-q --from-paths "$CATKIN_WORKSPACE/src" --ignore-src --rosdistro $ROS_DISTRO -y)
    if [ -n "$ROSDEP_SKIP_KEYS" ]; then
      rosdep_opts+=(--skip-keys "$ROSDEP_SKIP_KEYS")
    fi
    set -o pipefail # fail if rosdep install fails
    rosdep install "${rosdep_opts[@]}" | { grep "executing command" || true; }
    set +o pipefail

    ici_time_end  # rosdep_install

    if [ "$CATKIN_LINT" == "true" ] || [ "$CATKIN_LINT" == "pedantic" ]; then
        ici_time_start catkin_lint
        local catkin_lint_args=($CATKIN_LINT_ARGS)
        if [ "$CATKIN_LINT" == "pedantic" ]; then
        	catkin_lint_args+=(--strict -W2)
        fi
        run_catkin_lint "$TARGET_REPO_PATH" "${catkin_lint_args[@]}"
        ici_time_end  # catkin_lint
    fi


    # for catkin
    if [ "${TARGET_PKGS// }" == "" ]; then export TARGET_PKGS=`catkin_topological_order ${TARGET_REPO_PATH} --only-names`; fi
    # fall-back to all workspace packages if target repo does not contain any packages (#232)
    if [ "${TARGET_PKGS// }" == "" ]; then export TARGET_PKGS=`catkin_topological_order "$CATKIN_WORKSPACE/src" --only-names`; fi
    if [ "${PKGS_DOWNSTREAM// }" == "" ]; then export PKGS_DOWNSTREAM=$( [ "${BUILD_PKGS_WHITELIST// }" == "" ] && echo "$TARGET_PKGS" || echo "$BUILD_PKGS_WHITELIST"); fi

    ici_time_start catkin_build
    catkin build -w "$CATKIN_WORKSPACE" $OPT_VI --summarize  --no-status $BUILD_PKGS_WHITELIST $CATKIN_PARALLEL_JOBS --make-args $ROS_PARALLEL_JOBS
    ici_time_end  # catkin_build

    if [ "$NOT_TEST_BUILD" != "true" ]; then
        ici_time_start catkin_build_downstream_pkgs
        catkin build -w "$CATKIN_WORKSPACE" $OPT_VI --summarize  --no-status $PKGS_DOWNSTREAM $CATKIN_PARALLEL_JOBS --make-args $ROS_PARALLEL_JOBS
        ici_time_end  # catkin_build_downstream_pkgs

        ici_time_start catkin_build_tests
        catkin build  -w "$CATKIN_WORKSPACE" --no-deps --catkin-make-args tests -- $OPT_VI --summarize  --no-status $PKGS_DOWNSTREAM $CATKIN_PARALLEL_JOBS --make-args $ROS_PARALLEL_JOBS --

        ici_time_end  # catkin_build_tests
        ici_time_start catkin_run_tests
        catkin build -w "$CATKIN_WORKSPACE" --no-deps --catkin-make-args run_tests -- $OPT_RUN_V --no-status $PKGS_DOWNSTREAM $CATKIN_PARALLEL_TEST_JOBS --make-args $ROS_PARALLEL_TEST_JOBS --
        catkin_test_results --verbose "$CATKIN_WORKSPACE" || error
        ici_time_end  # catkin_run_tests
    fi

}
