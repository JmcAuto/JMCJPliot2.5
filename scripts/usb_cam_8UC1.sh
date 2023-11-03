#!/usr/bin/env bash

###############################################################################
# Copyright 2017 The JmcAuto Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################


DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cd "${DIR}/.."

source "${DIR}/jmc_auto_base.sh"

function start() {
    source /opt/ros/indigo/setup.bash
    LOG="${JMC_AUTO_ROOT_DIR}/data/log/usb_cam.out"
    CMD="roslaunch usb_cam start_cam_with_orb.launch"
    NUM_PROCESSES="$(pgrep -c -f "usb_cam_node")"
    if [ "${NUM_PROCESSES}" -eq 0 ]; then
       eval "nohup ${CMD} </dev/null >${LOG} 2>&1 &"
    fi
}

function stop() {
    pkill -9 -f start_leopard
    pkill -9 -f usb_cam_node
}

# run command_name module_name
function run() {
    case $1 in
        start)
            start
            ;;
        stop)
            stop
            ;;
        *)
            start
            ;;
    esac
}

run "$1"
