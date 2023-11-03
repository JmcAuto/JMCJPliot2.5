#!/usr/bin/env python
# coding: utf-8

import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

# path = '/home/leo/test/test/' #存放图片的位置
fpath = sys.argv[1]
if fpath[-1] != "/":
    path = fpath + "/"
else:
    path = fpath


class ImageCreator():

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        timestr = "%.6f" % data.header.stamp.to_sec()
        # %.6f表示小数点后带有6位，可根据精确度需要修改；
        image_name = timestr + ".jpg"
        cv2.imwrite(path + image_name, cv_image)

    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node('listener_right')
        self.image_sub = rospy.Subscriber("/cam_right/image_raw",Image, self.callback)
        rospy.spin()


if __name__ == '__main__':
    try:
        ImageCreator()
    except rospy.ROSInterruptException:
        pass
