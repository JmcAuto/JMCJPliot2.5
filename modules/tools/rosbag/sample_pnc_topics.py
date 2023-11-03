#!/usr/bin/env python

###############################################################################
# Copyright 2018 The JmcAuto Authors. All Rights Reserved.
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

"""
Sample PNC topics. For each /path/to/a.bag, will generate
/path/to/pnc_sample/a.bag.

Usage:
    ./sample_pnc_topics.py <bag_path>
        <bag_path>    Support * and ?.
Example:
    ./sample_pnc_topics.py '/mnt/nfs/public_test/2018-04-??/*/mkz8/*/*.bag'
"""

import sys
sys.path.append("/home/tmp/ros/lib/python2.7/dist-packages/")
import glob
import os
import sys

import glog
import rosbag


class SamplePNC(object):
    """Sample bags to contain PNC related topics only."""
    TOPICS = [
        '/jmc_auto/sensor/conti_radar',
        '/jmc_auto/sensor/delphi_esr',
        '/jmc_auto/sensor/gnss/best_pose',
        '/jmc_auto/sensor/gnss/corrected_imu',
        '/jmc_auto/sensor/gnss/gnss_status',
        '/jmc_auto/sensor/gnss/imu',
        '/jmc_auto/sensor/gnss/ins_stat',
        '/jmc_auto/sensor/gnss/odometry',
        '/jmc_auto/sensor/gnss/rtk_eph',
        '/jmc_auto/sensor/gnss/rtk_obs',
        '/jmc_auto/sensor/mobileye',
        '/jmc_auto/canbus/chassis',
        '/jmc_auto/canbus/chassis_detail',
        '/jmc_auto/control',
        '/jmc_auto/control/pad',
        '/jmc_auto/navigation',
        '/jmc_auto/perception/obstacles',
        '/jmc_auto/perception/traffic_light',
        '/jmc_auto/planning',
        '/jmc_auto/prediction',
        '/jmc_auto/routing_request',
        '/jmc_auto/routing_response',
        '/jmc_auto/localization/pose',
        '/jmc_auto/drive_event',
        '/tf',
        '/tf_static',
        '/jmc_auto/monitor',
        '/jmc_auto/monitor/system_status',
        '/jmc_auto/monitor/static_info',
    ]

    @classmethod
    def process_bags(cls, bags):
        for bag_file in bags:
            output_dir = os.path.join(os.path.dirname(bag_file), 'pnc_sample')
            output_bag = os.path.join(output_dir, os.path.basename(bag_file))
            if os.path.exists(output_bag):
                glog.info('Skip {} which has been processed'.format(bag_file))
                continue

            glog.info('Processing bag {}'.format(bag_file))
            if not os.path.exists(output_dir):
                os.makedirs(output_dir)
            with rosbag.Bag(bag_file, 'r') as bag_in:
                with rosbag.Bag(output_bag, 'w') as bag_out:
                    for topic, msg, t in bag_in.read_messages(
                            topics=SamplePNC.TOPICS):
                        bag_out.write(topic, msg, t)

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: %s <bag_path> ..." % sys.argv[0])
        sys.exit(1)

    bags = sorted(sum([glob.glob(arg) for arg in sys.argv[1:]], []))
    SamplePNC.process_bags(bags)
