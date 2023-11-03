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
    NUM_PROCESSES="$(pgrep -c -f "modules/drivers/radar/conti_radar/conti_radar")"
    if [ "${NUM_PROCESSES}" -eq 0 ]; then
      eval "nohup ${JMC_AUTO_BIN_PREFIX}/modules/drivers/radar/conti_radar/conti_radar \
          --flagfile=modules/drivers/radar/conti_radar/conf/front_radar.conf \
          --log_dir=${JMC_AUTO_ROOT_DIR}/data/log $@ </dev/null >${JMC_AUTO_ROOT_DIR}/data/log/radar_front.out 2>&1 &"
      eval "nohup ${JMC_AUTO_BIN_PREFIX}/modules/drivers/radar/conti_radar/conti_radar \
          --flagfile=modules/drivers/radar/conti_radar/conf/left_radar.conf \
          --log_dir=${JMC_AUTO_ROOT_DIR}/data/log $@ </dev/null >${JMC_AUTO_ROOT_DIR}/data/log/radar_left.out 2>&1 &"
      eval "nohup ${JMC_AUTO_BIN_PREFIX}/modules/drivers/radar/conti_radar/conti_radar \
          --flagfile=modules/drivers/radar/conti_radar/conf/right_radar.conf \
          --log_dir=${JMC_AUTO_ROOT_DIR}/data/log $@ </dev/null >${JMC_AUTO_ROOT_DIR}/data/log/radar_right.out 2>&1 &"
    fi
}

function stop() {
    pkill -SIGKILL -f "modules/drivers/radar/conti_radar/conti_radar"
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
