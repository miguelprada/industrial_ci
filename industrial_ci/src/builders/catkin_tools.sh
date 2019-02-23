#!/bin/bash

# Copyright (c) 2019, Mathias Lüdtke
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

function builder_setup {
  install_pkgs_for_command catkin python-catkin-tools
}

function builder_run_build {
    local extend=$1; shift
    local ws=$1; shift
    exec_in_workspace "$extend" "$ws" catkin config --install
    if [ -n "$CATKIN_CONFIG" ]; then exec_in_workspace "$extend" "$ws" eval catkin config $CATKIN_CONFIG; fi
    exec_in_workspace "$extend" "$ws" catkin build $OPT_VI --summarize  --no-status
}

function builder_run_tests {
    local extend=$1; shift
    local ws=$1; shift
    exec_in_workspace "$extend" "$ws" catkin build --catkin-make-args run_tests -- $OPT_RUN_V --no-status
}

function builder_test_results {
    local extend=$1; shift
    local ws=$1; shift
    exec_in_workspace "$extend" "$ws" catkin_test_results --verbose
}
