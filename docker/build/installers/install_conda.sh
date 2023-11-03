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

# Install Miniconda2.
CONDA_PATH=/usr/local/miniconda2
#wget https://repo.continuum.io/miniconda/Miniconda2-latest-Linux-x86_64.sh
bash Miniconda2-latest-Linux-x86_64.sh -b -p ${CONDA_PATH}

# Set Chinese sources
${CONDA_PATH}/bin/conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/free/
${CONDA_PATH}/bin/conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/main/
${CONDA_PATH}/bin/conda config --set show_channel_urls yes

# Init python 2.7 environment.
${CONDA_PATH}/bin/conda env update -f py27_conda.yml


# Clean up.
rm -f Miniconda2-latest-Linux-x86_64.sh

