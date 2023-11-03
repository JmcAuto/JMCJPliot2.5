#!/usr/bin/env bash

JMC_AUTO_ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

RED='\033[0;31m'
YELLOW='\e[33m'
NO_COLOR='\033[0m'

function info() {
  (>&2 echo -e "[\e[34m\e[1mINFO\e[0m] $*")
}

function error() {
  (>&2 echo -e "[${RED}ERROR${NO_COLOR}] $*")
}

function warning() {
  (>&2 echo -e "${YELLOW}[WARNING] $*${NO_COLOR}")
}

function ok() {
  (>&2 echo -e "[\e[32m\e[1m OK \e[0m] $*")
}

function print_delim() {
  echo '============================'
}

function get_now() {
  echo $(date +%s)
}

function print_time() {
  END_TIME=$(get_now)
  ELAPSED_TIME=$(echo "$END_TIME - $START_TIME" | bc -l)
  MESSAGE="Took ${ELAPSED_TIME} seconds"
  info "${MESSAGE}"
}

function success() {
  print_delim
  ok "$1"
  print_time
  print_delim
}

function fail() {
  print_delim
  error "$1"
  print_time
  print_delim
  exit -1
}

function check_in_docker() {
  if [ -f /.dockerenv ]; then
    JMC_AUTO_IN_DOCKER=true
  else
    JMC_AUTO_IN_DOCKER=false
  fi
  export JMC_AUTO_IN_DOCKER
}

function set_lib_path() {
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${JMC_AUTO_ROOT_DIR}/third_party/avm/lib
  if [ "$RELEASE_DOCKER" == 1 ];then
    source /jmcauto/ros/setup.bash
    export QT_X11_NO_MITSHM=1
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/jmcauto/lib:/usr/local/jmc_auto/local_integ/lib:${HOME}/secure_upgrade/depend_lib
    PY_LIB_PATH=/jmcauto/lib
    PY_TOOLS_PATH=/jmcauto/modules/tools
  else
    local ROS_SETUP="/home/tmp/ros/setup.bash"
    if [ -e "${ROS_SETUP}" ]; then
      source "${ROS_SETUP}"
    fi
    PY_LIB_PATH=${JMC_AUTO_ROOT_DIR}/py_proto
    PY_TOOLS_PATH=${JMC_AUTO_ROOT_DIR}/modules/tools
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/jmcauto/lib:/jmcauto/bazel-genfiles/external/caffe/lib:${HOME}/secure_upgrade/depend_lib
  fi
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/jmcauto/lib:/usr/local/jmc_auto/local_integ/lib
  export LD_LIBRARY_PATH=/usr/local/adolc/lib64:$LD_LIBRARY_PATH
  #export LD_LIBRARY_PATH=/usr/local/Qt5.5.1/5.5/gcc_64/lib:$LD_LIBRARY_PATH
  #export LD_LIBRARY_PATH=/usr/local/fast-rtps/lib:$LD_LIBRARY_PATH
  export LD_LIBRARY_PATH=/usr/local/jmc_auto/adv_plat/lib:$LD_LIBRARY_PATH
  export LD_LIBRARY_PATH=/usr/local/jmc_auto/boost/lib:$LD_LIBRARY_PATH
  export LD_LIBRARY_PATH=/usr/local/jmc_auto/jsoncpp/lib:$LD_LIBRARY_PATH
  export LD_LIBRARY_PATH=/usr/local/jmc_auto/undistort/lib:$LD_LIBRARY_PATH
  export LD_LIBRARY_PATH=/usr/local/ffmpeg/lib:$LD_LIBRARY_PATH
  #export LD_LIBRARY_PATH=/usr/local/jmc_auto/paddlepaddle_dep/mkldnn/lib/:$LD_LIBRARY_PATH
  export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

  PY_LIB_PATH=${PY_LIB_PATH}:/usr/local/jmc_auto/snowboy/Python
  export PYTHONPATH=${PY_LIB_PATH}:${PY_TOOLS_PATH}:${PYTHONPATH}
  export PATH=$PATH:/home/tmp/JmcPercetionTool
  if [ -e /usr/local/cuda-8.0/ ];then
    export PATH=/usr/local/cuda-8.0/bin:$PATH
    export LD_LIBRARY_PATH=/usr/local/cuda-8.0/lib64:$LD_LIBRARY_PATH
    export C_INCLUDE_PATH=/usr/local/cuda-8.0/include:$C_INCLUDE_PATH
    export CPLUS_INCLUDE_PATH=/usr/local/cuda-8.0/include:$CPLUS_INCLUDE_PATH
  fi
}

function create_data_dir() {
  local DATA_DIR=""
  if [ "$RELEASE_DOCKER" != "1" ];then
    DATA_DIR="${JMC_AUTO_ROOT_DIR}/data"
  else
    DATA_DIR="${HOME}/data"
  fi

  mkdir -p "${DATA_DIR}/log"
  mkdir -p "${DATA_DIR}/bag"
  mkdir -p "${DATA_DIR}/core"
  mkdir -p "${DATA_DIR}/csv"
}

function determine_bin_prefix() {
  JMC_AUTO_BIN_PREFIX=$JMC_AUTO_ROOT_DIR
  if [ -e "${JMC_AUTO_ROOT_DIR}/bazel-bin" ]; then
    JMC_AUTO_BIN_PREFIX="${JMC_AUTO_ROOT_DIR}/bazel-bin"
  fi
  export JMC_AUTO_BIN_PREFIX
}

function find_device() {
  # ${1} = device pattern
  local device_list=$(find /dev -name "${1}")
  if [ -z "${device_list}" ]; then
    warning "Failed to find device with pattern \"${1}\" ..."
  else
    local devices=""
    for device in $(find /dev -name "${1}"); do
      ok "Found device: ${device}."
      devices="${devices} --device ${device}:${device}"
    done
    echo "${devices}"
  fi
}

function setup_device() {
  if [ $(uname -s) != "Linux" ]; then
    echo "Not on Linux, skip mapping devices."
    return
  fi

  # setup CAN device
  for INDEX in `seq 0 3`
  do
    # soft link if sensorbox exist
    if [ -e /dev/zynq_can${INDEX} ] &&  [ ! -e /dev/can${INDEX} ]; then
      sudo ln -s /dev/zynq_can${INDEX} /dev/can${INDEX}
    fi
    if [ ! -e /dev/can${INDEX} ]; then
      sudo mknod --mode=a+rw /dev/can${INDEX} c 52 $INDEX
    fi
  done

  MACHINE_ARCH=$(uname -m)
  if [ "$MACHINE_ARCH" == 'aarch64' ]; then
    sudo ip link set can0 type can bitrate 500000
    sudo ip link set can0 up
  fi

  # setup nvidia device
  sudo /sbin/modprobe nvidia
  sudo /sbin/modprobe nvidia-uvm
  if [ ! -e /dev/nvidia0 ];then
    sudo mknod -m 666 /dev/nvidia0 c 195 0
  fi
  if [ ! -e /dev/nvidiactl ];then
    sudo mknod -m 666 /dev/nvidiactl c 195 255
  fi
  if [ ! -e /dev/nvidia-uvm ];then
    sudo mknod -m 666 /dev/nvidia-uvm c 243 0
  fi
  if [ ! -e /dev/nvidia-uvm-tools ];then
    sudo mknod -m 666 /dev/nvidia-uvm-tools c 243 1
  fi

  if [ ! -e /dev/nvidia-uvm-tools ];then
    sudo mknod -m 666 /dev/nvidia-uvm-tools c 243 1
  fi

  #setup GPS device
  if [ -e /dev/novatel0 ]; then
    sudo chmod a+rw /dev/novatel0
  fi
  if [ -e /dev/novatel1 ]; then
    sudo chmod a+rw /dev/novatel1
  fi
  if [ -e /dev/novatel2 ]; then
    sudo chmod a+rw /dev/novatel2
  fi
  if [ -e /dev/ttyACM0 ]; then
    sudo chmod a+rw /dev/ttyACM0
  fi
  if [ -e /dev/ttyUSB0 ]; then
    sudo chmod a+rw /dev/ttyUSB0
  fi
  if [ -e /dev/ttyUSB1 ]; then
    sudo chmod a+rw /dev/ttyUSB1
  fi
  if [ -e /dev/ttyUSB2 ]; then
    sudo chmod a+rw /dev/ttyUSB2
  fi

  # setup camera device
  if [ -e /dev/camera/obstacle ]; then
    sudo chmod a+rw /dev/camera/obstacle
  fi
  if [ -e /dev/camera/trafficlights ]; then
    sudo chmod a+rw /dev/camera/trafficlights
  fi
  if [ -e /dev/camera/lanemark ]; then
    sudo chmod a+rw /dev/camera/lanemark
  fi
  if [ -e /dev/video0 ]; then
    sudo chmod a+rw /dev/video0
  fi
}

function decide_task_dir() {
  # Try to find largest NVMe drive.
  DISK="$(df | grep "^/dev/nvme" | sort -nr -k 4 | \
      awk '{print substr($0, index($0, $6))}')"

  # Try to find largest external drive.
  if [ -z "${DISK}" ]; then
    DISK="$(df | grep "/media/${USER}" | sort -nr -k 4 | \
        awk '{print substr($0, index($0, $6))}')"
  fi

  if [ -z "${DISK}" ]; then
    echo "Cannot find portable disk. Fallback to jmcauto data dir."
    DISK="/jmcauto"
  fi

  # Create task dir.
  BAG_PATH="${DISK}/data/bag"
  TASK_ID=$(date +%Y-%m-%d-%H-%M-%S)
  TASK_DIR="${BAG_PATH}/${TASK_ID}"
  mkdir -p "${TASK_DIR}"

  echo "Record bag to ${TASK_DIR}..."
  export TASK_ID="${TASK_ID}"
  export TASK_DIR="${TASK_DIR}"
}

function is_stopped_customized_path() {
  MODULE_PATH=$1
  MODULE=$2
  NUM_PROCESSES="$(pgrep -u $DOCKER_USER_ID -c -f "modules/${MODULE_PATH}/${MODULE}")"
  if [ "${NUM_PROCESSES}" -eq 0 ]; then
    return 1
  else
    return 0
  fi
}

function start_customized_path() {
  MODULE_PATH=$1
  MODULE=$2
  shift 2

  LOG="${JMC_AUTO_ROOT_DIR}/data/log/${MODULE}.out"
  is_stopped_customized_path "${MODULE_PATH}" "${MODULE}"
  if [ $? -eq 1 ]; then
    eval "nohup ${JMC_AUTO_BIN_PREFIX}/modules/${MODULE_PATH}/${MODULE} \
        --flagfile=modules/${MODULE_PATH}/conf/${MODULE}.conf \
        --log_dir=${JMC_AUTO_ROOT_DIR}/data/log $@ </dev/null >${LOG} 2>&1 &"
    sleep 0.5
    is_stopped_customized_path "${MODULE_PATH}" "${MODULE}"
    if [ $? -eq 0 ]; then
      echo "Launched module ${MODULE}."
      return 0
    else
      echo "Could not launch module ${MODULE}. Is it already built?"
      return 1
    fi
  else
    echo "Module ${MODULE} is already running - skipping."
    return 2
  fi
}

function start() {
  MODULE=$1
  shift

  start_customized_path $MODULE $MODULE "$@"
}

function start_prof_customized_path() {
  MODULE_PATH=$1
  MODULE=$2
  shift 2

  echo "Make sure you have built with 'bash jmc_auto.sh build_prof'"
  LOG="${JMC_AUTO_ROOT_DIR}/data/log/${MODULE}.out"
  is_stopped_customized_path "${MODULE_PATH}" "${MODULE}"
  if [ $? -eq 1 ]; then
    PROF_FILE="/tmp/$MODULE.prof"
    rm -rf $PROF_FILE
    BINARY=${JMC_AUTO_BIN_PREFIX}/modules/${MODULE_PATH}/${MODULE}
    eval "CPUPROFILE=$PROF_FILE $BINARY \
        --flagfile=modules/${MODULE_PATH}/conf/${MODULE}.conf \
        --log_dir=${JMC_AUTO_ROOT_DIR}/data/log $@ </dev/null >${LOG} 2>&1 &"
    sleep 0.5
    is_stopped_customized_path "${MODULE_PATH}" "${MODULE}"
    if [ $? -eq 0 ]; then
      echo -e "Launched module ${MODULE} in prof mode. \nExport profile by command:"
      echo -e "${YELLOW}google-pprof --pdf $BINARY $PROF_FILE > ${MODULE}_prof.pdf${NO_COLOR}"
      return 0
    else
      echo "Could not launch module ${MODULE}. Is it already built?"
      return 1
    fi
  else
    echo "Module ${MODULE} is already running - skipping."
    return 2
  fi
}

function start_prof() {
  MODULE=$1
  shift

  start_prof_customized_path $MODULE $MODULE "$@"
}

function start_fe_customized_path() {
  MODULE_PATH=$1
  MODULE=$2
  shift 2

  eval "${JMC_AUTO_BIN_PREFIX}/modules/${MODULE_PATH}/${MODULE} \
      --flagfile=modules/${MODULE_PATH}/conf/${MODULE}.conf \
      --alsologtostderr --log_dir=${JMC_AUTO_ROOT_DIR}/data/log $@"
}

function start_fe() {
  MODULE=$1
  shift

  start_fe_customized_path $MODULE $MODULE "$@"
}

function start_gdb_customized_path() {
  MODULE_PATH=$1
  MODULE=$2
  shift 2

  eval "gdb --args ${JMC_AUTO_BIN_PREFIX}/modules/${MODULE_PATH}/${MODULE} \
      --flagfile=modules/${MODULE_PATH}/conf/${MODULE}.conf \
      --log_dir=${JMC_AUTO_ROOT_DIR}/data/log $@"
}

function start_gdb() {
  MODULE=$1
  shift

  start_gdb_customized_path $MODULE $MODULE "$@"
}

function stop_customized_path() {
  MODULE_PATH=$1
  MODULE=$2

  pkill -u $DOCKER_USER_ID -SIGKILL -f "modules/${MODULE_PATH}/${MODULE}"
  if [ $? -eq 0 ]; then
    echo "Successfully stopped module ${MODULE}."
  else
    echo "Module ${MODULE} is not running - skipping."
  fi
}

function stop() {
  MODULE=$1
  stop_customized_path $MODULE $MODULE
}

# Note: This 'help' function here will overwrite the bash builtin command 'help'.
# TODO: add a command to query known modules.
function help() {
cat <<EOF
Invoke ". scripts/jmc_auto_base.sh" within docker to add the following commands to the environment:
Usage: COMMAND [<module_name>]

COMMANDS:
  help:      show this help message
  start:     start the module in background
  start_fe:  start the module without putting in background
  start_gdb: start the module with gdb
  stop:      stop the module
EOF
}

function run_customized_path() {
  local module_path=$1
  local module=$2
  local cmd=$3
  shift 3
  case $cmd in
    start)
      start_customized_path $module_path $module "$@"
      ;;
    start_fe)
      start_fe_customized_path $module_path $module "$@"
      ;;
    start_gdb)
      start_gdb_customized_path $module_path $module "$@"
      ;;
    start_prof)
      start_prof_customized_path $module_path $module "$@"
      ;;
    stop)
      stop_customized_path $module_path $module
      ;;
    help)
      help
      ;;
    *)
      start_customized_path $module_path $module $cmd "$@"
    ;;
  esac
}

# Write log to a file about the env when record a bag.
function record_bag_env_log() {
  if [ -z "${TASK_ID}" ]; then
    TASK_ID=$(date +%Y-%m-%d-%H-%M)
  fi

  git status >/dev/null 2>&1
  if [ $? -ne 0 ]; then
    echo "Not in Git repo, maybe because you are in release container."
    echo "Skip log environment."
    return
  fi

  commit=$(git log -1)
  echo -e "Date:$(date)\n" >> Bag_Env_$TASK_ID.log
  git branch | awk '/\*/ { print "current branch: " $2; }'  >> Bag_Env_$TASK_ID.log
  echo -e "\nNewest commit:\n$commit"  >> Bag_Env_$TASK_ID.log
  echo -e "\ngit diff:" >> Bag_Env_$TASK_ID.log
  git diff >> Bag_Env_$TASK_ID.log
  echo -e "\n\n\n\n" >> Bag_Env_$TASK_ID.log
  echo -e "git diff --staged:" >> Bag_Env_$TASK_ID.log
  git diff --staged >> Bag_Env_$TASK_ID.log
}

# run command_name module_name
function run() {
  local module=$1
  shift
  run_customized_path $module $module "$@"
}

check_in_docker
create_data_dir

#if [ -z $JMC_AUTO_BASE_SOURCED ]; then
  set_lib_path
  determine_bin_prefix
  export JMC_AUTO_BASE_SOURCED=1
#fi

#auto complete
function _jmc_auto() {
  local cur=${COMP_WORDS[COMP_CWORD]};
  local prev=${COMP_WORDS[COMP_CWORD-1]};

  if [[ ${prev} == *"jmc_auto.sh" ]] ; then
    COMPREPLY=( $(compgen -W "build build_gpu build_opt_gpu build_velodyne build_usbcam release clean help" -- ${cur}) )
    return 0
  fi

  case $prev in
    build)
      cd ${JMC_AUTO_ROOT_DIR}/modules
      COMPREPLY=( $(compgen -o dirnames -- ${cur}) )
      cd ${JMC_AUTO_ROOT_DIR}
      ;;
    build_gpu)
      cd ${JMC_AUTO_ROOT_DIR}/modules
      COMPREPLY=( $(compgen -o dirnames -- ${cur}) )
      cd ${JMC_AUTO_ROOT_DIR}
      ;;
    build_opt_gpu)
      cd ${JMC_AUTO_ROOT_DIR}/modules
      COMPREPLY=( $(compgen -o dirnames -- ${cur}) )
      cd ${JMC_AUTO_ROOT_DIR}
      ;;

    build_py)
      cd ${JMC_AUTO_ROOT_DIR}/modules
      COMPREPLY=( $(compgen -o dirnames -- ${cur}) )
      cd ${JMC_AUTO_ROOT_DIR}
      ;;
    build_velodyne)
      COMPREPLY=()
      ;;
    build_usbcam)
      COMPREPLY=()
      ;;
    release)
      COMPREPLY=()
      ;;
    clean)
      COMPREPLY=()
      ;;
    help)
      COMPREPLY=()
      ;;
    esac
    return 0
}

complete -o default -F _jmc_auto bash
complete -F _jmc_auto jmc_auto.sh
