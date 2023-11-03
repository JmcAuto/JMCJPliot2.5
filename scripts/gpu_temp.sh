#!/usr/bin/env bash
#####################################################
# File Name: gpu_temp.sh
# Author: Leo
# mail: yli97@jmc.com.cn
# Created Time: Friday, May 27, 2022 AM11:28:20 HKT
#####################################################


DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cd "${DIR}/.."

source "${DIR}/jmc_auto_base.sh"

function do_loop() {
    while [[ 1 ]]
    do
        nvidia-smi -q |grep -E "Time|Temp"
        echo "============================================"
        sleep 10
    done
}

function start() {
    LOG="${JMC_AUTO_ROOT_DIR}/data/log/gpu_temp.out"
    NUM_PROCESSES="$(pgrep -c -f "nvidia-smi")"
    if [ "${NUM_PROCESSES}" -eq 0 ]; then
        eval "do_loop </dev/null >${LOG} 2>&1 &"
    fi
}

function stop() {
    pkill -9 -f gpu_temp
}

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
