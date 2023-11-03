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

JMC_AUTO_LIB_PATH=/usr/local/jmc_auto
# Expected file structure:
#  ${JMC_AUTO_LIB_PATH}/
#    - jsoncpp
#      - lib/*.so
#    - adv_plat
#      - include/*.h
#      - lib/*.a

mkdir -p ${JMC_AUTO_LIB_PATH}

# Install jsoncpp.
#wget https://apollocache.blob.core.windows.net/apollo-cache/jsoncpp.zip
unzip jsoncpp.zip
mv jsoncpp ${JMC_AUTO_LIB_PATH}

# Install adv plat.
#wget https://apollocache.blob.core.windows.net/apollo-cache/adv_plat.zip
unzip adv_plat.zip
mv adv_plat ${JMC_AUTO_LIB_PATH}

# Clean up.
rm -fr jsoncpp.zip adv_plat.zip
