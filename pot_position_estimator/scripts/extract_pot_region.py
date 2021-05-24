#!/usr/bin/env python
import rospy
import sys
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class ExtractPotRegion():
    def __init__(self):
        self.bridge = CvBridge()
        self.color_sub = rospy.Subscriber(
            "/camera/color/image_raw", Image, self.color_callback, queue_size=1)
        self.color_pub = rospy.Publisher('image_with_BBox', Image, queue_size=1)

    def color_callback(self, ros_image):
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, 'passthrough')
        except CvBridgeError, e:
            rospy.logerr(e)

        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        h = hsv[:, :, 0]
        s = hsv[:, :, 1]

        mask = np.zeros(h.shape, dtype=np.uint8)
        mask[((h < 20) | (h > 200)) & (s > 128)] = 255

        kernel=cv2.getStructuringElement(cv2.MORPH_RECT,(25,25))
        mask = cv2.dilate(mask,kernel,iterations = 1)
        mask = cv2.erode(mask,kernel,iterations = 1)

        contours, _= cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[1:]
        
        display = np.zeros(mask.shape, dtype=np.uint8)
        for c in contours:
            for elem in c:
                display[elem[0,1],elem[0,0]]=255

        rects = []
        for contour in contours:
            approx = cv2.convexHull(contour)
            rect = cv2.boundingRect(approx)
            rects.append(rect)

        for rect in rects:
            x, y, w, h = rect
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), thickness=2)

        pub_image = self.bridge.cv2_to_imgmsg(frame, encoding="rgb8")
        self.color_pub.publish(pub_image)



if __name__ == '__main__':
    try:
        rospy.init_node('extract_pot_region')
        print("create extract_pot_region")

        ExtractPotRegion()
        rospy.spin()

    except rospy.ROSInterruptException:
        print("program interrupted before completion")
