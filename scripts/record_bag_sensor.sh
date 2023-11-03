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

source "${DIR}/jmc_auto_base.sh"

function start() {
  decide_task_dir $@
  cd "${TASK_DIR}"

  # Start recording.
  record_bag_env_log
  LOG="/tmp/jmc_auto_record.out"
  NUM_PROCESSES="$(pgrep -c -f "rosbag record")"
  if [ "${NUM_PROCESSES}" -eq 0 ]; then
    nohup rosbag record --split --duration=1m -b 2048  \
        /jmc_auto/sensor/camera/traffic/image_short \
        /jmc_auto/sensor/camera/traffic/image_long \
        /jmc_auto/sensor/conti_radar \
        /jmc_auto/sensor/delphi_esr \
        /jmc_auto/sensor/gnss/best_pose \
        /jmc_auto/sensor/gnss/corrected_imu \
        /jmc_auto/sensor/gnss/gnss_status \
        /jmc_auto/sensor/gnss/imu \
        /jmc_auto/sensor/gnss/raw_data \
        /jmc_auto/sensor/gnss/ins_stat \
        /jmc_auto/sensor/gnss/odometry \
        /jmc_auto/sensor/gnss/rtk_eph \
        /jmc_auto/sensor/gnss/rtk_obs \
        /jmc_auto/sensor/mobileye \
        /jmc_auto/sensor/velodyne64/compensator/PointCloud2 \
        /jmc_auto/canbus/chassis \
        /jmc_auto/canbus/chassis_detail \
        /jmc_auto/control \
        /jmc_auto/control/pad \
        /jmc_auto/perception/obstacles \
        /jmc_auto/perception/traffic_light \
        /jmc_auto/planning \
        /jmc_auto/prediction \
        /jmc_auto/routing_request \
        /jmc_auto/routing_response \
        /jmc_auto/localization/pose \
        /jmc_auto/drive_event \
        /tf \
        /tf_static \
        /jmc_auto/monitor \
        /jmc_auto/monitor/static_info </dev/null >"${LOG}" 2>&1 &
    fi
}

function stop() {
  pkill -SIGINT -f record
}

function help() {
  echo "Usage:"
  echo "$0 [start]                     Record bag to data/bag."
  echo "$0 stop                        Stop recording."
  echo "$0 help                        Show this help message."
}

case $1 in
  start)
    shift
    start $@
    ;;
  stop)
    shift
    stop $@
    ;;
  help)
    shift
    help $@
    ;;
  *)
    start $@
    ;;
esac
