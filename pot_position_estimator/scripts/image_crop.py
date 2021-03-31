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
        self.depth_sub = rospy.Subscriber(
            "/camera/depth/image_raw", Image, self.depth_callback, queue_size=1)
        self.color_sub = rospy.Subscriber(
            "/camera/color/image_raw", Image, self.color_callback, queue_size=1)

    def depth_callback(self, ros_image):
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, 'passthrough')
        except CvBridgeError, e:
            rospy.logerr(e)
        
        dt_now = datetime.datetime.now()
        date_time = dt_now.isoformat().replace(':', '-')
        filename = "depth_image.png"
        filepath = "../data/" + date_time + "_" + filename
        cv2.imwrite(filepath, frame)

        filename_npy = "depth_image.npy"
        filepath = "../data/" + date_time + "_" + filename_npy
        depth_array = np.array(frame, dtype=np.float32)
        np.save(filepath, depth_array)

        print("create "+ filepath)
        #cv2.waitKey(1)

    def color_callback(self, ros_image):
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, 'passthrough')
        except CvBridgeError, e:
            rospy.logerr(e)

        dt_now = datetime.datetime.now()
        date_time = dt_now.isoformat().replace(':', '-')
        filename = "color_image.png"
        filepath = "../data/" + date_time + "_" + filename
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        cv2.imwrite(filepath, frame)

        print("create " + filepath)
        #cv2.waitKey(1)

if __name__ == '__main__':
    try:
        rospy.init_node('image_crop')
        print("create image_crop")

        ImageCrop()
        rospy.spin()

    except rospy.ROSInterruptException:
        print("program interrupted before completion")
