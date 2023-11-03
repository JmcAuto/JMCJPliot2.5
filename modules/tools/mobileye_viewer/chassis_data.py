#!/usr/bin/env python

###############################################################################
# Copyright 2017 The JmcAuto Authors. All Rights Reserved.
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

import sys
sys.path.append("/home/tmp/ros/lib/python2.7/dist-packages/")
import threading


class ChassisData:
    def __init__(self, chassis_pb=None):
        self.chassis_pb = chassis_pb

    def update(self, chassis_pb):
        self.chassis_pb = chassis_pb

    def is_auto(self):
        if self.chassis_pb is None:
            return False
        if self.chassis_pb.driving_mode is None:
            return False
        if self.chassis_pb.driving_mode == \
                self.chassis_pb.COMPLETE_AUTO_DRIVE:
            return True
        return False
