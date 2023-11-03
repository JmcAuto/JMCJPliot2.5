#! /bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "${DIR}/.."

source "${DIR}/jmc_auto_base.sh"

$JMC_AUTO_BIN_PREFIX/modules/localization/msf/local_tool/local_visualization/online_visual/online_local_visualizer \
    --flagfile=/jmcauto/modules/localization/conf/localization.conf \
    --log_dir=/jmcauto/data/log
