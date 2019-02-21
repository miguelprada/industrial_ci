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

ici_mark_deprecated ROSINSTALL_FILENAME "Please migrate to new UPSTREAM_WORKSPACE format"
if [ ! "$ROSINSTALL_FILENAME" ]; then export ROSINSTALL_FILENAME=".travis.rosinstall"; fi

# variables in docker.env without default will be exported with empty string
# this might break the build, e.g. for Makefile which rely on these variables
if [ -z "${CC}" ]; then unset CC; fi
if [ -z "${CFLAGS}" ]; then unset CFLAGS; fi
if [ -z "${CPPFLAGS}" ]; then unset CPPFLAGS; fi
if [ -z "${CXX}" ]; then unset CXX; fi
if [ -z "${CXXFLAGS}" ]; then unset CXXLAGS; fi

function  ros1_defaults {
    DEFAULT_OS_CODE_NAME=$1
    ROS1_DISTRO=${ROS1_DISTRO:-$ROS_DISTRO}
    ROS1_REPOSITORY_PATH=${ROS1_REPOSITORY_PATH:-$ROS_REPOSITORY_PATH}
    ROS1_REPO=${ROS1_REPO:-${ROS_REPO:-ros}}
}
function  ros2_defaults {
    DEFAULT_OS_CODE_NAME=$1
    ROS2_DISTRO=${ROS2_DISTRO:-$ROS_DISTRO}
    ROS2_REPOSITORY_PATH=${ROS2_REPOSITORY_PATH:-$ROS_REPOSITORY_PATH}
    ROS2_REPO=${ROS2_REPO:-${ROS_REPO:-ros2}}
}

DEFAULT_OS_CODE_NAME=""
case "$ROS_DISTRO" in
"indigo"|"jade")
    ros1_defaults "trusty"
    ;;
"kinetic"|"lunar")
    ros1_defaults "xenial"
    ;;
"melodic")
    ros1_defaults "bionic"
    ;;
"ardent"|"bouncy"|"crystal")
    ros2_defaults "bionic"
    ;;
esac


# If not specified, use ROS Shadow repository http://wiki.ros.org/ShadowRepository
if [ ! "$ROS1_REPOSITORY_PATH" ]; then
    case "${ROS1_REPO}" in
    "building")
        ROS1_REPOSITORY_PATH="http://repositories.ros.org/ubuntu/building/"
        ;;
    "ros"|"main")
        ROS1_REPOSITORY_PATH="http://packages.ros.org/ros/ubuntu"
        ;;
    "ros-shadow-fixed"|"testing")
        ROS1_REPOSITORY_PATH="http://packages.ros.org/ros-shadow-fixed/ubuntu"
        ;;
    *)
        if [ -n "$ROS1_DISTRO" ]; then
            error "ROS1 repo '$ROS1_REPO' is not supported"
        fi
        ;;
    esac
fi

if [ ! "$ROS2_REPOSITORY_PATH" ]; then
    case "${ROS2_REPO}" in
    "ros2"|"main")
        ROS2_REPOSITORY_PATH="http://packages.ros.org/ros2/ubuntu"
        ;;
    "ros2-testing"|"testing")
        ROS2_REPOSITORY_PATH="http://packages.ros.org/ros2-testing/ubuntu"
        ;;
    *)
        if [ -n "$ROS2_DISTRO" ]; then
            error "ROS2 repo '$ROS2_REPO' is not supported"
        fi
        ;;
    esac
fi

export OS_CODE_NAME
export OS_NAME
export DOCKER_BASE_IMAGE

# exit with error if OS_NAME is set, but OS_CODE_NAME is not.
# assume ubuntu as default
if [ -z "$OS_NAME" ]; then
    OS_NAME=ubuntu
elif [ -z "$OS_CODE_NAME" ]; then
    error "please specify OS_CODE_NAME"
fi

if [ -z "$OS_CODE_NAME" ]; then
    case "$ROS_DISTRO" in
    "")
        if [ -n "$DOCKER_IMAGE" ] || [ -n "$DOCKER_BASE_IMAGE" ]; then
          # try to reed ROS_DISTRO from imgae
          if [ "$DOCKER_PULL" != false ]; then
            docker pull "${DOCKER_IMAGE:-$DOCKER_BASE_IMAGE}"
          fi
          export ROS_DISTRO=$(docker image inspect --format "{{.Config.Env}}" "${DOCKER_IMAGE:-$DOCKER_BASE_IMAGE}" | grep -o -P "(?<=ROS_DISTRO=)[a-z]*")
        fi
        if [ -z "$ROS_DISTRO" ]; then
            error "Please specify ROS_DISTRO"
        fi
        ;;
    *)
        if [ -z "$DEFAULT_OS_CODE_NAME" ]; then
            error "ROS distro '$ROS_DISTRO' is not supported"
        fi
        OS_CODE_NAME=$DEFAULT_OS_CODE_NAME
        ;;
    esac
fi

if [ -z "$DOCKER_BASE_IMAGE" ]; then
    DOCKER_BASE_IMAGE="$OS_NAME:$OS_CODE_NAME" # scheme works for all supported OS images
fi


export TERM=${TERM:-dumb}
