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
    apt-get -qq install --no-install-recommends -y \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool
    if [ $(lsb_release -sc) = "xenial" ]; then
        pip3 install -U setuptools
    fi
}

function builder_run_build {
    local extend=$1; shift
    local ws=$1; shift
    if [ ! -e "$extend/local_setup.bash" ]; then
      ln -s "$extend/setup.bash" "$extend/local_setup.bash"
    fi
    COLCON_PREFIX_PATH="$extend" exec_in_workspace "$extend" "$ws" colcon build --event-handlers status-
}

function builder_run_test {
    local extend=$1; shift
    local ws=$1; shift
    exec_in_workspace "$extend" "$ws" colcon test --event-handlers status-
}

function builder_test_results {
    local extend=$1; shift
    local ws=$1; shift
    exec_in_workspace "$extend" "$ws" colcon test-result --verbose
}
