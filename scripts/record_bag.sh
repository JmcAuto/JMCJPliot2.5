#####################################################
# File Name: record_bag.sh
# Author: Leo
# mail: yli97@jmc.com.cn
# Created Time: 四  8/13 14:05:09 2020
#####################################################

#!/bin/bash

JMC_AUTO_ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
source ${JMC_AUTO_ROOT_DIR}/scripts/jmc_auto_base.sh

function start() {
  cd "${TASK_DIR}"

  # Start recording.
  #record_bag_env_log
  LOG="${JMC_AUTO_ROOT_DIR}/data/log/record.out"
  NUM_PROCESSES="$(pgrep -c -f "rosbag record")"
  if [ "${NUM_PROCESSES}" -eq 0 ]; then
    if [ "$1" == "SIMPLE" ]; then
      nohup rosbag record --split --duration=1m -b 2048 \
          /jmc_auto/sensor/gnss/best_pose \
          /jmc_auto/sensor/gnss/corrected_imu \
          /jmc_auto/sensor/gnss/gnss_status \
          /jmc_auto/sensor/gnss/heading \
          /jmc_auto/sensor/gnss/imu \
          /jmc_auto/sensor/gnss/ins_stat \
          /jmc_auto/sensor/gnss/ins_status \
          /jmc_auto/sensor/gnss/odometry \
          /jmc_auto/sensor/gnss/raw_data \
          /jmc_auto/sensor/gnss/rtcm_data \
          /jmc_auto/sensor/gnss/rtk_eph \
          /jmc_auto/sensor/gnss/rtk_obs \
          /jmc_auto/sensor/gnss/stream_status \
          /jmc_auto/sensor/radar_front \
          /jmc_auto/sensor/radar_left \
          /jmc_auto/sensor/radar_right \
          /jmc_auto/localization/pose \
          /jmc_auto/canbus/chassis \
          /jmc_auto/canbus/chassis_detail \
          </dev/null >"${LOG}" 2>&1 &
    elif [ "$1" == "FULL" ]; then
      nohup rosbag record --split --duration=1m -b 2048 \
          /jmc_auto/sensor/gnss/best_pose \
          /jmc_auto/sensor/gnss/corrected_imu \
          /jmc_auto/sensor/gnss/gnss_status \
          /jmc_auto/sensor/gnss/heading \
          /jmc_auto/sensor/gnss/imu \
          /jmc_auto/sensor/gnss/ins_stat \
          /jmc_auto/sensor/gnss/ins_status \
          /jmc_auto/sensor/gnss/odometry \
          /jmc_auto/sensor/gnss/raw_data \
          /jmc_auto/sensor/gnss/rtcm_data \
          /jmc_auto/sensor/gnss/rtk_eph \
          /jmc_auto/sensor/gnss/rtk_obs \
          /jmc_auto/sensor/gnss/stream_status \
          /jmc_auto/sensor/radar_front \
          /jmc_auto/sensor/radar_left \
          /jmc_auto/sensor/radar_right \
          /jmc_auto/sensor/velodyne64/PointCloud2 \
          /jmc_auto/sensor/velodyne64/compensator/PointCloud2 \
          /jmc_auto/localization/pose \
          /jmc_auto/canbus/chassis \
          /jmc_auto/canbus/chassis_detail \
          </dev/null >"${LOG}" 2>&1 &
    fi
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
