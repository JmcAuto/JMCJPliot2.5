#!/usr/bin/env python

import argparse
import atexit
import logging
import os
import sys
sys.path.append("/home/tmp/ros/lib/python2.7/dist-packages/")

import rospy
from gflags import FLAGS

from modules.localization.proto import localization_pb2
from modules.canbus.proto import chassis_pb2
from modules.localization.proto import gps_pb2
from modules.drivers.proto import ttecontiradar_pb2

class LocalizationPublisher(object):
    def __init__(self):
        self.localization = localization_pb2.LocalizationEstimate()
        self.localization_pub = rospy.Publisher('/jmc_auto/localization/pose', localization_pb2.LocalizationEstimate, queue_size=1) 
        self.sequence_num = 0
        self.terminating = False
        self.position_x = 389125.558278886
        self.position_y = 3155629.89290818
        self.position_z = 0
        self.orientation_x = 0
        self.orientation_y = 0
        self.orientation_z = 0
        self.orientation_w = 0
        self.linear_velocity_x = 0 
        self.linear_velocity_y = 0
        self.linear_velocity_z = 0

    # def localization_callback(self, data):
    #     """
    #     New message received
    #     """
    #     self.localization.CopyFrom(data)
    #     self.position_x = self.localization.pose.position.x
    #     self.position_y = self.localization.pose.position.y
    #     self.position_z = self.localization.pose.position.z
    #     self.orientation_x = self.localization.pose.orientation.qx
    #     self.orientation_y = self.localization.pose.orientation.qy
    #     self.orientation_z = self.localization.pose.orientation.qz
    #     self.orientation_w = self.localization.pose.orientation.qw
    #     self.linear_velocity_x = self.localization.pose.linear_velocity.x
    #     self.linear_velocity_y = self.localization.pose.linear_velocity.y
    #     self.linear_velocity_z = self.localization.pose.linear_velocity.z

    def localization_callback(self):
        localization = localization_pb2.LocalizationEstimate()
        now = rospy.get_time()
        localization.header.timestamp_sec = now
        localization.pose.position.x = self.position_x
        localization.pose.position.y = self.position_y
        localization.pose.position.z = self.position_z
        localization.pose.orientation.qx = self.orientation_x
        localization.pose.orientation.qy = self.orientation_y
        localization.pose.orientation.qz = self.orientation_z
        localization.pose.orientation.qw = self.orientation_w
        localization.pose.linear_velocity.x = self.linear_velocity_x
        localization.pose.linear_velocity.y = self.linear_velocity_y
        localization.pose.linear_velocity.z = self.linear_velocity_z
        rospy.loginfo("%s"%localization)
        self.localization_pub.publish(localization)

    def shutdown(self):
        """
        shutdown rosnode
        """
        self.terminating = True
        self.logger.info("Shutting Down...")
        self.file_handler.close()
        rospy.sleep(0.1)


class ChassisPublisher(object):
    def __init__(self):
        self.speed_mps = 0
        self.chassis = chassis_pb2.Chassis()
        self.chassis_pub = rospy.Publisher('/jmc_auto/canbus/chassis', chassis_pb2.Chassis, queue_size=1)
    def chassis_callback(self):
        chassis = chassis_pb2.Chassis()
        now = rospy.get_time()
        chassis.speed_mps=self.chassis
        chassis.steering_percentage=0

class TtePublisher(object):
    def __init__(self):
        self.tte_pub = rospy.Publisher('/jmc_auto/sensor/tte_conti_radar', ttecontiradar_pb2.TteContiRadar, queue_size=1)
    def tte_callback(self):
        tte = ttecontiradar_pb2.TteContiRadar
        now = rospy.get_time()
        tte.debug_apareardistanceinfo_457.aparrs_distance=2
        tte.debug_rightusslot_ptab_467.rightusslot_pta_x=2
        tte.debug_rightusslot_ptab_467.rightusslot_pta_y=2
        tte.debug_rightusslot_ptab_467.rightusslot_ptb_x=2
        tte.debug_rightusslot_ptab_467.rightusslot_ptb_y=2

        tte.debug_rightvplslot_ptab_463.rightvplslot_pta_x=2
        tte.debug_rightvplslot_ptab_463.rightvplslot_pta_y=2
        tte.debug_rightvplslot_ptab_463.rightvplslot_ptb_x=2
        tte.debug_rightvplslot_ptab_463.rightvplslot_ptb_y=2

        tte.debug_rightvplslot_ptcd_464.rightvplslot_ptc_x=2
        tte.debug_rightvplslot_ptcd_464.rightvplslot_ptc_y=2
        tte.debug_rightvplslot_ptcd_464.rightvplslot_ptd_x=2
        tte.debug_rightvplslot_ptcd_464.rightvplslot_ptd_y=2


def main():
    """
    Main rosnode
    """
    rospy.init_node('odom_publisher', anonymous=True)
    Localization = LocalizationPublisher()
    Chassis=ChassisPublisher()
    tte=TtePublisher()
    # rospy.Subscriber('/jmc_auto/localization/pose', localization_pb2.LocalizationEstimate, Localization.localization_callback)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        Localization.localization_callback()
        Chassis.chassis_callback()
        tte.tte_callback()
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
