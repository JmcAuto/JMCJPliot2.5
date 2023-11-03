#!/usr/bin/env python3

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
Start jmc_auto data recorder.

It lists all available disks mounted under /media, and prioritize them in order:
   - Disk#1. Largest NVME disk
   - Disk#2. Smaller NVME disk
   - ...
   - Disk#x. Largest Non-NVME disk
   - Disk#y. Smaller Non-NVME disk
   - ...

1. If we have NVME disk, it will be used to record all data.
2. If we have Non-NVME disk, it will only record SMALL_TOPICS, unless '--all' is
   specified.
3. If no external disks are available, we will take '/jmcauto' as a
   'Non-NVME disk' and follow the rule above.

Run with '--help' to see more options.
"""

import argparse
import datetime
import os
import subprocess
import sys

import psutil


SMALL_TOPICS = [
    '/jmc_auto/canbus/chassis',
    '/jmc_auto/canbus/chassis_detail',
    '/jmc_auto/control',
    '/jmc_auto/control/pad',
    '/jmc_auto/drive_event',
    '/jmc_auto/guardian',
    '/jmc_auto/localization/pose',
    '/jmc_auto/localization/msf_gnss',
    '/jmc_auto/localization/msf_lidar',
    '/jmc_auto/localization/msf_status',
    '/jmc_auto/monitor',
    '/jmc_auto/monitor/static_info',
    '/jmc_auto/monitor/system_status',
    '/jmc_auto/navigation',
    '/jmc_auto/perception/obstacles',
    '/jmc_auto/perception/traffic_light',
    '/jmc_auto/planning',
    '/jmc_auto/prediction',
    '/jmc_auto/relative_map',
    '/jmc_auto/routing_request',
    '/jmc_auto/routing_response',
    '/jmc_auto/sensor/conti_radar',
    '/jmc_auto/sensor/delphi_esr',
    '/jmc_auto/sensor/gnss/best_pose',
    '/jmc_auto/sensor/gnss/corrected_imu',
    '/jmc_auto/sensor/gnss/gnss_status',
    '/jmc_auto/sensor/gnss/imu',
    '/jmc_auto/sensor/gnss/ins_stat',
    '/jmc_auto/sensor/gnss/odometry',
    '/jmc_auto/sensor/gnss/raw_data',
    '/jmc_auto/sensor/gnss/rtk_eph',
    '/jmc_auto/sensor/gnss/rtk_obs',
    '/jmc_auto/sensor/mobileye',
    '/tf',
    '/tf_static',
]

LARGE_TOPICS = [
    '/jmc_auto/sensor/camera/traffic/image_short',
    '/jmc_auto/sensor/camera/traffic/image_long',
    '/jmc_auto/sensor/camera/obstacle/front_6mm',
    '/jmc_auto/sensor/velodyne64/compensator/PointCloud2',
    '/jmc_auto/sensor/velodyne16/compensator/PointCloud2',
]


MIN_DISK_SIZE = 2**35  # 32GB


def shell_cmd(cmd, alert_on_failure=True):
    """Execute shell command and return (ret-code, stdout, stderr)."""
    print('SHELL > {}'.format(cmd))
    proc = subprocess.Popen(cmd, shell=True, close_fds=True,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    ret = proc.wait()
    stdout = proc.stdout.read().decode('utf-8') if proc.stdout else None
    stderr = proc.stderr.read().decode('utf-8') if proc.stderr else None
    if alert_on_failure and stderr and ret != 0:
        sys.stderr.write('{}\n'.format(stderr))
    return (ret, stdout, stderr)


class ArgManager(object):
    """Arguments manager."""

    def __init__(self):
        self.parser = argparse.ArgumentParser(
            description="Manage jmc_auto data recording.")
        self.parser.add_argument('--start', default=False, action="store_true",
                                 help='Start recorder. It is the default '
                                 'action if no other actions are triggered. In '
                                 'that case, the False value is ignored.')
        self.parser.add_argument('--stop', default=False, action="store_true",
                                 help='Stop recorder.')
        self.parser.add_argument('--all', default=False, action="store_true",
                                 help='Record all topics even without high '
                                 'performance disks.')
        self.parser.add_argument('--split_duration', default="1m",
                                 help='Duration to split bags, will be applied '
                                 'as parameter to "rosbag record --duration".')
        self._args = None

    def args(self):
        """Get parsed args."""
        if self._args is None:
           self._args = self.parser.parse_args()
        return self._args


class DiskManager(object):
    """Disk manager."""

    def __init__(self):
        """Manage disks."""
        disks = []
        for disk in psutil.disk_partitions():
            if not disk.mountpoint.startswith('/media/'):
                continue
            disks.append({
                'mountpoint': disk.mountpoint,
                'available_size': DiskManager.disk_avail_size(disk.mountpoint),
                'is_nvme': disk.device.startswith('/dev/nvme'),
            })
        # Prefer NVME disks and then larger disks.
        self.disks = sorted(
            disks, reverse=True,
            key=lambda disk: (disk['is_nvme'], disk['available_size']))

    @staticmethod
    def disk_avail_size(disk_path):
        """Get disk available size."""
        statvfs = os.statvfs(disk_path)
        return statvfs.f_frsize * statvfs.f_bavail


class Recorder(object):
    """Data recorder."""
    kEventCollector='modules/data/tools/event_collector_main'


    def __init__(self, args):
        self.args = args
        self.disk_manager = DiskManager()

    def start(self):
        """Start recording."""
        if Recorder.is_running():
            print('Another data recorder is running, skip.')
            return

        disks = self.disk_manager.disks
        # To record all topics if
        # 1. User requested with '--all' argument.
        # 2. Or we have a NVME disk.
        record_all = self.args.all or (len(disks) > 0 and disks[0]['is_nvme'])
        # Use the best disk, or fallback '/jmcauto' if none available.
        disk_to_use = '/jmcauto'
        available_size = 0
        if len(disks) > 0:
            disk_to_use = disks[0]['mountpoint']
            available_size = disks[0]['available_size']
        else:
            available_size = DiskManager.disk_avail_size(disk_to_use)
        if available_size < MIN_DISK_SIZE:
            print('Insufficient disk space, stop recording: {} with {}'.format(
                disk_to_use, available_size))
            return
        if record_all:
            self.record_task(disk_to_use, SMALL_TOPICS + LARGE_TOPICS)
        else:
            self.record_task(disk_to_use, SMALL_TOPICS)

    def stop(self):
        """Stop recording."""
        shell_cmd('kill -TERM $(pgrep -f "rosbag/record" | grep -v pgrep)')
        shell_cmd('kill -INT $(pgrep -f "{}" | grep -v pgrep)'.format(
            Recorder.kEventCollector))

    def record_task(self, disk, topics):
        """Record tasks into the <disk>/data/bag/<task_id> directory."""
        task_id = datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
        task_dir = os.path.join(disk, 'data/bag', task_id)
        print('Recording bag to {}'.format(task_dir))

        log_file = '/jmcauto/data/log/jmc_auto_record.out'
        topics_str = ' '.join(topics)

        os.makedirs(task_dir)
        cmd = '''
            cd "{}"
            source /jmcauto/scripts/jmc_auto_base.sh
            nohup rosbag record --split --duration={} -b 2048 {} >{} 2>&1 &
            nohup ${{JMC_AUTO_BIN_PREFIX}}/{} >/dev/null 2>&1 &
        '''.format(task_dir, self.args.split_duration, topics_str, log_file,
                   Recorder.kEventCollector)
        shell_cmd(cmd)

    @staticmethod
    def is_running():
        """Test if the given process running."""
        _, stdout, _ = shell_cmd('pgrep -c -f "rosbag/record"', False)
        # If stdout is the pgrep command itself, no such process is running.
        return stdout.strip() != '1' if stdout else False


def main():
    """Main entry."""
    arg_manager = ArgManager()
    args = arg_manager.args()
    recorder = Recorder(args)
    if args.stop:
        recorder.stop()
    else:
        recorder.start()

if __name__ == '__main__':
    main()
