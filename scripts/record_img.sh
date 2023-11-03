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
  source /opt/ros/indigo/setup.bash
  bash ${JMC_AUTO_ROOT_DIR}/scripts/usb_cam.sh
  cd "${TASK_DIR}"

  # Start recording.
    LOG="${JMC_AUTO_ROOT_DIR}/data/log/img_record.out"
    NUM_PROCESSES="$(pgrep -c -f "getimg")"
    if [ ! -e "${TASK_DIR}/image_short/" ]; then
	mkdir ${TASK_DIR}/image_short/
    fi
    if [ ! -e "${TASK_DIR}/image_long/" ]; then
        mkdir ${TASK_DIR}/image_long/
    fi
    if [ ! -e "${TASK_DIR}/image_right/" ]; then
        mkdir ${TASK_DIR}/image_right/
    fi
    if [ "${NUM_PROCESSES}" -eq 0 ]; then
        nohup python ${JMC_AUTO_ROOT_DIR}/scripts/getimg_short.py ${TASK_DIR}/image_short/ \
            </dev/null >"${LOG}" 2>&1 &
        nohup python ${JMC_AUTO_ROOT_DIR}/scripts/getimg_long.py ${TASK_DIR}/image_long/ \
            </dev/null >>"${LOG}" 2>&1 &
        nohup python ${JMC_AUTO_ROOT_DIR}/scripts/getimg_right.py ${TASK_DIR}/image_right/ \
            </dev/null >>"${LOG}" 2>&1 &
    fi
}

function stop() {
  pkill -SIGINT -f getimg
  bash ${JMC_AUTO_ROOT_DIR}/scripts/usb_cam.sh stop
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
