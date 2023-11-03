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

class LocalizationPublisher(object):
    def __init__(self):
        self.localization = localization_pb2.LocalizationEstimate()
        self.localization_pub = rospy.Publisher('/jmc_auto/localization/pose', localization_pb2.LocalizationEstimate, queue_size=1) 
        self.sequence_num = 0
        self.terminating = False
        #self.position_x = 389116.61
        #self.position_y = 3155643.26
        #
        #self.position_x = 389125.558278886
        #self.position_y = 3155629.89290818
        #
        #self.position_x = 389130.558278886
        #self.position_y = 3155611.89290818
        #1
        #self.position_x = 389121.52
        #self.position_y = 3155565.27
        #2
        #self.position_x = 389115.40
        #self.position_y = 3155640.01
        #2.1
        #self.position_x = 389116.40
        #self.position_y = 3155641.01
	#3
	#self.position_x = 389117.86
        #self.position_y = 3155639.75
  	#jinzong_test
	#self.position_x = 389140.40
        #self.position_y = 3155633.75 
        #guangchang
        self.position_x = 389059.998278886
        self.position_y = 3155570.89490818
        #test
        #self.position_x = 389179.028278886
        #self.position_y = 3155561.20290818
        #
        #self.position_x = 389291.558278886
        #self.position_y = 3155675.89290818
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
        localization.pose.angular_velocity_vrf.x = 0
        localization.pose.angular_velocity_vrf.y = 0
        localization.pose.angular_velocity_vrf.z = 0
        localization.pose.linear_acceleration_vrf.x = 0
        localization.pose.linear_acceleration_vrf.y = 0
        localization.pose.linear_acceleration_vrf.z = 0
        localization.pose.heading= 0.039569967
        #localization.pose.heading=1.61036634
        #localization.pose.heading= 4.72
        #localization.pose.heading=3.18
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
        chassis.speed_mps=0
        rospy.loginfo("%s"%chassis)
        self.chassis_pub.publish(chassis)



def main():
    """
    Main rosnode
    """
    rospy.init_node('odom_publisher', anonymous=True)
    Localization = LocalizationPublisher()
    Chassis=ChassisPublisher()
    # rospy.Subscriber('/jmc_auto/localization/pose', localization_pb2.LocalizationEstimate, Localization.localization_callback)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        Localization.localization_callback()
        Chassis.chassis_callback()
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
