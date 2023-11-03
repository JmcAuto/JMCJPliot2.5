#!/usr/bin/env bash

###############################################################################
# Copyright 2018 The Apollo Authors. All Rights Reserved.
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

# Fail on first error.
set -e

USER_NAME=caros

adduser --disabled-password --gecos '' ${USER_NAME}
usermod -aG sudo ${USER_NAME}
echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

echo """
export PATH=/jmcauto/scripts:/usr/local/miniconda2/bin/:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /jmcauto/scripts/jmc_auto_base.sh
ulimit -c unlimited
""" >> /home/${USER_NAME}/.bashrc
