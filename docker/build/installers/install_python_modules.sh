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

cd "$(dirname "${BASH_SOURCE[0]}")"

apt-get install -y \
   libgeos-dev \
   libleveldb-dev \
   python-matplotlib \
   python-pip \
   python-psutil \
   python-scipy \
   python-software-properties \
   python3-psutil

#python -m pip install --upgrade --user pip
#export PROJ_DIR=/usr/local
#pip install -r py27_requirements.txt
pip install -i https://pypi.tuna.tsinghua.edu.cn/simple pip -U
#pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple
pip install -i https://pypi.tuna.tsinghua.edu.cn/simple -r py27_requirements.txt

echo_supervisord_conf > /etc/supervisord.conf
