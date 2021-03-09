#!/usr/bin/env python
import rospy
import sys
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import datetime


class ImageCrop:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/camera/depth/image_raw", Image, self.image_callback, queue_size=1)

    def image_callback(self, ros_image):
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, 'passthrough')
        except CvBridgeError, e:
            rospy.logerr(e)
        
        dt_now = datetime.datetime.now()
        date_time = dt_now.isoformat().replace(':', '-')
        filename = "depth_image.png"
        filepath = "../data/" + date_time + "_" + filename
        cv2.imwrite(filepath, frame)
        print("create "+ filepath)
        #cv2.waitKey(1)


if __name__ == '__main__':
    try:
        rospy.init_node('image_crop')
        print("create image_crop")

        ImageCrop()
        rospy.spin()

    except rospy.ROSInterruptException:
        print("program interrupted before completion")
