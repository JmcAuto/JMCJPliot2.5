#!/usr/bin/env bash

#=================================================
#                   Utils
#=================================================

function source_jmc_auto_base() {
  DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
  cd "${DIR}"

  source "${DIR}/scripts/jmc_auto_base.sh"
}

function jmc_auto_check_system_config() {
  # check docker environment
#  if [ ${MACHINE_ARCH} == "x86_64" ] && [ ${JMC_AUTO_IN_DOCKER} != "true" ]; then
#    error "Must run $0 in docker."
#    exit 0
#  fi

  # check global filee
  if [ ! -e "/jmcauto/modules/common/data/global_flagfile.txt" ]; then
    cp /jmcauto/modules/common/data/global_flagfile /jmcauto/modules/common/data/global_flagfile.txt
  fi


  # check operating system
  OP_SYSTEM=$(uname -s)
  case $OP_SYSTEM in
    "Linux")
      echo "System check passed. Build continue ..."

      # check system configuration
      DEFAULT_MEM_SIZE="2.0"
      MEM_SIZE=$(free | grep Mem | awk '{printf("%0.2f", $2 / 1024.0 / 1024.0)}')
      if (( $(echo "$MEM_SIZE < $DEFAULT_MEM_SIZE" | bc -l) )); then
         warning "System memory [${MEM_SIZE}G] is lower than minimum required memory size [2.0G]. Build could fail."
      fi
      ;;
    *)
      error "Unsupported system: ${OP_SYSTEM}."
      error "Please use Linux, we recommend Ubuntu 14.04."
      exit 1
      ;;
  esac
}

function check_machine_arch() {
  # the machine type, currently support x86_64, aarch64
  MACHINE_ARCH=$(uname -m)

  # Generate WORKSPACE file based on marchine architecture
  if [ "$MACHINE_ARCH" == 'x86_64' ]; then
    sed "s/MACHINE_ARCH/x86_64/g" WORKSPACE.in > WORKSPACE
  elif [ "$MACHINE_ARCH" == 'aarch64' ]; then
    sed "s/MACHINE_ARCH/aarch64/g" WORKSPACE.in > WORKSPACE
  else
    fail "Unknown machine architecture $MACHINE_ARCH"
    exit 1
  fi

  #setup vtk folder name for different systems.
  VTK_VERSION=$(find /usr/include/ -type d  -name "vtk-*" | tail -n1 | cut -d '-' -f 2)
  sed -i "s/VTK_VERSION/${VTK_VERSION}/g" WORKSPACE
}

function check_esd_files() {
  CAN_CARD="fake_can"

  if [ -f ./third_party/can_card_library/esd_can/include/ntcan.h \
      -a -f ./third_party/can_card_library/esd_can/lib/libntcan.so.4 \
      -a -f ./third_party/can_card_library/esd_can/lib/libntcan.so.4.0.1 ]; then
      USE_ESD_CAN=true
      CAN_CARD="esd_can"
  else
      warning "ESD CAN library supplied by ESD Electronics does not exist. If you need ESD CAN, please refer to third_party/can_card_library/esd_can/README.md."
      USE_ESD_CAN=false
  fi
}

function generate_build_targets() {
  if [ -z $NOT_BUILD_PERCEPTION ] ; then
    if [ "$1" ] ; then
        BUILD_TARGETS=`bazel query $(echo "//modules/$1/...")`
        if [ $? -ne 0 ]; then
          ISFAIL=1
        fi
        shift
    else BUILD_TARGETS=`bazel query //modules/...  except //modules/drivers/gnss/... except //modules/drivers/lidar_velodyne/... except //modules/drivers/radar/conti_radar/... except //modules/remotecontrol/... except //modules/third_party_perception/... except //modules/map/relative_map/...`
        if [ $? -ne 0 ]; then
          ISFAIL=1
        fi
    fi
  else
    info 'Skip building perception module!'
    BUILD_TARGETS=`bazel query //modules/... except //modules/perception/... except //modules/calibration/lidar_ex_checker/...`
    if [ $? -ne 0 ]; then
      ISTAIL=1
    fi
  fi

  if [ "${ISFAIL}" == 1 ]; then
    fail 'Build failed!'
  fi

  if ! $USE_ESD_CAN; then
    BUILD_TARGETS=$(echo $BUILD_TARGETS |tr ' ' '\n' | grep -v "esd")
  fi
  #skip msf for non x86_64 platforms
  if [ ${MACHINE_ARCH} != "x86_64" ]; then
    BUILD_TARGETS=$(echo $BUILD_TARGETS |tr ' ' '\n' | grep -v "msf")
  fi
}

#=================================================
#              Build functions
#=================================================

function build() {
  info "Start building, please wait ..."
  MACHINE_ARCH=$(uname -m)
  info "Building on $MACHINE_ARCH..."
  ISFAIL=0
  JOB_ARG=""
  if [ "$MACHINE_ARCH" == 'aarch64' ]; then
    JOB_ARG="--jobs=3"
  fi

  generate_build_targets $1
  echo "$BUILD_TARGETS" | xargs bazel build $JOB_ARG $DEFINES -c $BUILD_CMD
  if [ $? -ne 0 ]; then
    fail 'Build failed!'
  fi

  # Build python proto
  build_py_proto $1

  # Clear KV DB and update commit_id after compiling.
  #rm -fr data/kv_db
  #python modules/tools/common/kv_db.py put \
  #    "jmc_auto:data:commit_id" "$(git rev-parse HEAD)"

  if [ $? -eq 0 ]; then
    success 'Build passed!'
  fi
}

function jmc_auto_build_dbg() {
  BUILD_CMD="dbg"
  build $@
}

function jmc_auto_build_opt() {
  BUILD_CMD="opt"
  build $@
}

function build_py_proto() {
#  if [ -d "./py_proto" ];then
#    rm -rf py_proto
#  fi
  if [ ! -d "./py_proto"  ] ; then
    mkdir py_proto
  fi

  if [ -e './bazel-out/host/bin/external/com_github_google_protobuf/protoc' ]; then
    PROTOC='./bazel-out/host/bin/external/com_github_google_protobuf/protoc'
  elif [ -e './bazel-out/host/bin/external/com_google_protobuf/protoc' ]; then
    PROTOC='./bazel-out/host/bin/external/com_google_protobuf/protoc'
  else
    PROTOC='protoc'
  fi

  if [ "$1" ] ; then
    find modules/common -name "*.proto" \
        | grep -v node_modules \
        | xargs ${PROTOC} --python_out=py_proto
    if [ -d "modules/$1"  ] ; then
      find modules/$1 -name "*.proto" \
          | grep -v node_modules \
          | xargs ${PROTOC} --python_out=py_proto
    fi
    if [ -d "modules/drivers/$1"  ] ; then
      find modules/drivers/$1 -name "*.proto" \
          | grep -v node_modules \
          | xargs ${PROTOC} --python_out=py_proto
    fi
  else
    find modules/ -name "*.proto" \
        | grep -v node_modules \
        | xargs ${PROTOC} --python_out=py_proto
  fi

  find py_proto/* -type d -exec touch "{}/__init__.py" \;
}

function release() {
  RELEASE_DIR="${HOME}/.cache/jmcauto_release"
  if [ -d "${RELEASE_DIR}" ]; then
    rm -rf "${RELEASE_DIR}"
    rm -rf "${DIR}/release"
  fi
  JMCAUTO_RELEASE_DIR="${RELEASE_DIR}/jmcauto"
  mkdir -p "${JMCAUTO_RELEASE_DIR}"
  ln -fs ${JMCAUTO_RELEASE_DIR} ${DIR}/release

  # Find binaries and convert from //path:target to path/target
  QUERIES=$(bazel query "kind(cc_binary, //modules/...)" | sed 's/^\/\///' | sed 's/:/\//')
  BINARIES=$(echo ${QUERIES} |tr ' ' '\n' |grep -v '\.so')
  # Copy binaries to release dir.
  for BIN in ${BINARIES}; do
    SRC_PATH="bazel-bin/${BIN}"
    DST_PATH="${JMCAUTO_RELEASE_DIR}/${BIN}"
    if [ -e "${SRC_PATH}" ]; then
      mkdir -p "$(dirname "${DST_PATH}")"
      cp "${SRC_PATH}" "${DST_PATH}"
    fi
  done

#  LIBRARIES=$(echo ${QUERIES} |tr ' ' '\n' |grep '\.so')
#  # Copy libraries to release dir.
  LIB_DIR="${JMCAUTO_RELEASE_DIR}/lib"
  mkdir "${LIB_DIR}"
#  for LIB in ${LIBRARIES}; do
#    SRC_PATH2="bazel-bin/${LIB}"
#    if [ -e "${SRC_PATH2}" ]; then
#      cp "${SRC_PATH2}" "${LIB_DIR}"
#    fi
#  done

  # modules data and conf
  CONFS=$(find modules/ \( -path modules/dreamview/frontend/node_modules \
    -o -path modules/perception/model/traffic_light \) -prune -o -name "conf" -print)
  DATAS=$(find modules/ \( -path modules/dreamview/frontend/node_modules \
    -o -path modules/perception/model/traffic_light \) -prune -o -name "data" -print)
  OTHER=("modules/tools"
         "modules/perception/model"
         "modules/perception/obstacle/camera/detector/yolo_camera_detector/jmc_model")
  rm -rf test/*
  for conf in $CONFS; do
    mkdir -p $JMCAUTO_RELEASE_DIR/$conf
    rsync -a $conf/* $JMCAUTO_RELEASE_DIR/$conf
  done
  for data in $DATAS; do
    mkdir -p $JMCAUTO_RELEASE_DIR/$data
    rsync -a $data/* $JMCAUTO_RELEASE_DIR/$data
  done
  # Other
  for path in "${OTHER[@]}"; do
    mkdir -p $JMCAUTO_RELEASE_DIR/$path
    rsync -a --exclude traffic_light $path/* $JMCAUTO_RELEASE_DIR/$path
  done

  # dreamview frontend
  #cp -a modules/dreamview/frontend $JMCAUTO_RELEASE_DIR/modules/dreamview
  rsync -a --exclude node_modules modules/dreamview/frontend $JMCAUTO_RELEASE_DIR/modules/dreamview

  # remove all pyc file in modules/
  find modules/ -name "*.pyc" | xargs -I {} rm {}

  # scripts
  cp -r scripts ${JMCAUTO_RELEASE_DIR}

  # lib
  #if $USE_ESD_CAN; then
  #  warn_proprietary_sw
  #fi
  cp -r third_party/can_card_library/*/lib/* $LIB_DIR
  cp -r third_party/avm/lib/* $LIB_DIR
  cp -r bazel-genfiles/external $LIB_DIR
  cp -r py_proto/modules $LIB_DIR
  if [ -e modules/perception/cuda_util/cmake_build/libcuda_util.so ]; then
    cp modules/perception/cuda_util/cmake_build/libcuda_util.so $LIB_DIR
  fi

  # doc
  cp -r docs "${JMCAUTO_RELEASE_DIR}"
  #cp LICENSE "${JMCAUTO_RELEASE_DIR}"
  #cp third_party/ACKNOWLEDGEMENT.txt "${JMCAUTO_RELEASE_DIR}"

  # release info
  META="${JMCAUTO_RELEASE_DIR}/meta.ini"
  echo "git_commit: $(git rev-parse HEAD)" >> $META
  #echo "car_type: LINCOLN.MKZ" >> $META
  echo "version: $(git describe --tags `git rev-list --tags --max-count=1`)" >> $META
  echo "arch: ${MACHINE_ARCH}" >> $META
  echo "release" > ${JMCAUTO_RELEASE_DIR}/.USER_NAME
  echo "--dreamview_module_name=dreamview" > ${JMCAUTO_RELEASE_DIR}/.DREAMVIEW.txt
  echo "--server_ports=8889" >> ${JMCAUTO_RELEASE_DIR}/.DREAMVIEW.txt
}


function check() {
  bash $0 build && bash $0 "test" && bash $0 lint

  if [ $? -eq 0 ]; then
    success 'Check passed!'
    return 0
  else
    fail 'Check failed!'
    return 1
  fi
}

function clean() {
  bazel clean --async
}

function buildify() {
  local buildifier_url=https://github.com/bazelbuild/buildtools/releases/download/0.4.5/buildifier
  wget $buildifier_url -O ~/.buildifier
  chmod +x ~/.buildifier
  find . -name '*BUILD' -type f -exec ~/.buildifier -showlog -mode=fix {} +
  if [ $? -eq 0 ]; then
    success 'Buildify worked!'
  else
    fail 'Buildify failed!'
  fi
  rm ~/.buildifier
}

function build_fe() {
  cd modules/dreamview/frontend
  yarn build
}

function gen_doc() {
  rm -rf docs/doxygen
  doxygen jmc_auto.doxygen
}

function version() {
  commit=$(git log -1 --pretty=%H)
  date=$(git log -1 --pretty=%cd)
  echo "Commit: ${commit}"
  echo "Date: ${date}"
}

function build_velodyne() {
  CURRENT_PATH=$(pwd)
  if [ -d "${ROS_ROOT}" ]; then
    ROS_PATH="${ROS_ROOT}/../.."
  else
    warning "ROS not found. Run apolllo.sh build first."
    exit 1
  fi

  source "${ROS_PATH}/setup.bash"

  cd modules
  catkin_make_isolated --install --source drivers/velodyne \
    --install-space "${ROS_PATH}" -DCMAKE_BUILD_TYPE=Release \
    --cmake-args --no-warn-unused-cli
  find "${ROS_PATH}" -name "*.pyc" -print0 | xargs -0 rm -rf
  cd -

  rm -rf modules/.catkin_workspace
  rm -rf modules/build_isolated/
  rm -rf modules/devel_isolated/
}

function build_lslidar() {
  CURRENT_PATH=$(pwd)
  if [ -d "${ROS_ROOT}" ]; then
    ROS_PATH="${ROS_ROOT}/../.."
  else
    warning "ROS not found. Run apolllo.sh build first."
    exit 1
  fi

  source "${ROS_PATH}/setup.bash"

  cd modules
  catkin_make_isolated --install --source drivers/lslidar_jmc_auto\
    --install-space "${ROS_PATH}" -DCMAKE_BUILD_TYPE=Release \
    --cmake-args --no-warn-unused-cli
  find "${ROS_PATH}" -name "*.pyc" -print0 | xargs -0 rm -rf
  cd -

  rm -rf modules/.catkin_workspace
  rm -rf modules/build_isolated/
  rm -rf modules/devel_isolated/
}

function build_rslidar() {
  CURRENT_PATH=$(pwd)
  if [ -d "${ROS_ROOT}" ]; then
    ROS_PATH="${ROS_ROOT}/../.."
  else
    warning "ROS not found. Run apolllo.sh build first."
    exit 1
  fi

  source "${ROS_PATH}/setup.bash"

  cd modules
  catkin_make_isolated --install --source drivers/rslidar \
    --install-space "${ROS_PATH}" -DCMAKE_BUILD_TYPE=Release \
    --cmake-args --no-warn-unused-cli
  find "${ROS_PATH}" -name "*.pyc" -print0 | xargs -0 rm -rf
  cd -

  rm -rf modules/.catkin_workspace
  rm -rf modules/build_isolated/
  rm -rf modules/devel_isolated/
}

function build_usbcam() {
  CURRENT_PATH=$(pwd)
  if [ -d "${ROS_ROOT}" ]; then
    ROS_PATH="${ROS_ROOT}/../.."
  else
    warning "ROS not found. Run apolllo.sh build first."
    exit 1
  fi

  source "${ROS_PATH}/setup.bash"

  cd modules
  catkin_make_isolated --install --source drivers/usb_cam \
    --install-space "${ROS_PATH}" -DCMAKE_BUILD_TYPE=Release \
    --cmake-args --no-warn-unused-cli
  find "${ROS_PATH}" -name "*.pyc" -print0 | xargs -0 rm -rf
  cd -

  rm -rf modules/.catkin_workspace
  rm -rf modules/build_isolated/
  rm -rf modules/devel_isolated/
}

function build_rtspcam() {
  CURRENT_PATH=$(pwd)
  if [ -d "${ROS_ROOT}" ]; then
    ROS_PATH="${ROS_ROOT}/../.."
  else
    warning "ROS not found. Run apolllo.sh build first."
    exit 1
  fi

  source "${ROS_PATH}/setup.bash"

  cd modules
  catkin_make_isolated --install --source drivers/rtsp_cam \
    --install-space "${ROS_PATH}" -DCMAKE_BUILD_TYPE=Release \
    --cmake-args --no-warn-unused-cli
  find "${ROS_PATH}" -name "*.pyc" -print0 | xargs -0 rm -rf
  cd -

  rm -rf modules/.catkin_workspace
  rm -rf modules/build_isolated/
  rm -rf modules/devel_isolated/
}


function print_usage() {
  RED='\033[0;31m'
  BLUE='\033[0;34m'
  BOLD='\033[1m'
  NONE='\033[0m'

  echo -e "\n${RED}Usage${NONE}:
  .${BOLD}/jmc_auto.sh${NONE} [OPTION]"

  echo -e "\n${RED}Options${NONE}:
  ${BLUE}build [module_name]${NONE}: run build for all modules or specified module.
  ${BLUE}build_py [module_name]${NONE}: run build proto to proto.py for all modules or specified module.
  ${BLUE}build_opt${NONE}: build optimized binary for the code
  ${BLUE}build_gpu${NONE}: run build only with Caffe GPU mode support
  ${BLUE}build_velodyne${NONE}: build velodyne driver
  ${BLUE}build_lslidar${NONE}: build lslidar driver
  ${BLUE}build_rslidar${NONE}: build rslidar driver
  ${BLUE}build_usbcam${NONE}: build usb camera driver
  ${BLUE}build_rtspcam${NONE}: build rtsp camera driver
  ${BLUE}build_opt_gpu${NONE}: build optimized binary with Caffe GPU mode support
  ${BLUE}build_fe${NONE}: compile frontend javascript code, this requires all the node_modules to be installed already
  ${BLUE}build_no_perception [dbg|opt]${NONE}: run build build skip building perception module, useful when some perception dependencies are not satisified, e.g., CUDA, CUDNN, LIDAR, etc.
  ${BLUE}build_prof${NONE}: build for gprof support.
  ${BLUE}buildify${NONE}: fix style of BUILD files
  ${BLUE}release${NONE}: pack build file into release folder.
  ${BLUE}clean${NONE}: run Bazel clean
  ${BLUE}doc${NONE}: generate doxygen document
  ${BLUE}usage${NONE}: print this menu
  ${BLUE}version${NONE}: display current commit and date
  "
}

function main() {
  source_jmc_auto_base
  check_machine_arch
  jmc_auto_check_system_config
  check_esd_files

  DEFINES="--define ARCH=${MACHINE_ARCH} --define CAN_CARD=${CAN_CARD} --cxxopt=-DUSE_ESD_CAN=${USE_ESD_CAN}"

  if [ ${MACHINE_ARCH} == "x86_64" ]; then
    DEFINES="${DEFINES} --copt=-mavx2"
  fi

  local cmd=$1
  shift

  START_TIME=$(get_now)
  case $cmd in
    build)
      DEFINES="${DEFINES} --cxxopt=-DCPU_ONLY"
      jmc_auto_build_dbg $@
      ;;
    build_prof)
      DEFINES="${DEFINES} --config=cpu_prof --cxxopt=-DCPU_ONLY"
      jmc_auto_build_dbg $@
      ;;
    build_no_perception)
      DEFINES="${DEFINES} --cxxopt=-DCPU_ONLY"
      NOT_BUILD_PERCEPTION=true
      if [ "$1" == "opt" ]; then
        shift
        jmc_auto_build_opt $@
      elif [ "$1" == "dbg" ]; then
        shift
        jmc_auto_build_dbg $@
      fi
      ;;
    build_opt)
      DEFINES="${DEFINES} --cxxopt=-DCPU_ONLY"
      jmc_auto_build_opt $@
      ;;
    build_gpu)
      DEFINES="${DEFINES} --cxxopt=-DUSE_CAFFE_GPU"
      jmc_auto_build_dbg $@
      ;;
    build_opt_gpu)
      DEFINES="${DEFINES} --cxxopt=-DUSE_CAFFE_GPU"
      jmc_auto_build_opt $@
      ;;
    release)
        release 1
      ;;
    build_fe)
      build_fe
      ;;
    buildify)
      buildify
      ;;
    build_py)
      build_py_proto $@
      ;;
    build_velodyne)
      build_velodyne
      ;;
    build_lslidar)
      build_lslidar
      ;;
    build_rslidar)
      build_rslidar
      ;;
    build_usbcam)
      build_usbcam
      ;;
    build_rtspcam)
      build_rtspcam
      ;;
    doc)
      gen_doc
      ;;
    clean)
      clean
      ;;
    version)
      version
      ;;
    help)
      print_usage
      ;;
    *)
      print_usage
      ;;
  esac
}

main $@
