#!/usr/bin/env python
import rospy
import sys
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

import pyrealsense2


class ExtractPotRegion():
    def __init__(self):
        self.bridge = CvBridge()
        self._intrinsics = pyrealsense2.intrinsics()
        self.rects = []
        self.color_sub = rospy.Subscriber(
            "/camera/color/image_raw", Image, self.color_callback, queue_size=1)
        self.depth_sub = rospy.Subscriber(
            "/camera/depth/image_raw", Image, self.depth_callback, queue_size=1)
        self.camerainfo_sub = rospy.Subscriber(
            "/camera/depth/camera_info", CameraInfo, self.camerainfo_callback, queue_size=1)
        self.color_pub = rospy.Publisher('image_with_BBox', Image, queue_size=1)
        '''
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.create_BBox()
            for rect in self.rects:
                x, y, w, h = rect
                print(self.estimate_pot_position(x+w//2,y+h//2))
            r.sleep()
            '''

    def camerainfo_callback(self, cameraInfo):
        self._intrinsics.width = cameraInfo.width
        self._intrinsics.height = cameraInfo.height
        self._intrinsics.ppx = cameraInfo.K[2]
        self._intrinsics.ppy = cameraInfo.K[5]
        self._intrinsics.fx = cameraInfo.K[0]
        self._intrinsics.fy = cameraInfo.K[4]
        #self._intrinsics.model = cameraInfo.distortion_model
        if cameraInfo.distortion_model == 'plumb_bob':
            self._intrinsics.model = pyrealsense2.distortion.brown_conrady
        elif cameraInfo.distortion_model == 'equidistant':
            self._intrinsics.model = pyrealsense2.distortion.kannala_brandt4
        #print([i for i in cameraInfo.D])
        if [i for i in cameraInfo.D] == []:
            self._intrinsics.coeffs = [0,0,0,0,0]
        else:
            self._intrinsics.coeffs = [i for i in cameraInfo.D]

    def depth_callback(self, ros_image):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(ros_image, 'passthrough')
        except CvBridgeError, e:
            rospy.logerr(e)
        self.depth_image = np.array(self.depth_image, dtype=np.float32)

    def color_callback(self, ros_image):
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(ros_image, 'passthrough')
        except CvBridgeError, e:
            rospy.logerr(e)

    
        self.create_BBox()
        for rect in self.rects:
            x, y, w, h = rect
            print(self.estimate_pot_position(x+w//2,y+h//2))

    def create_BBox(self): # creates self.rects[]
        frame = self.color_image.copy()
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

        self.rects = []
        for contour in contours:
            approx = cv2.convexHull(contour)
            rect = cv2.boundingRect(approx)
            self.rects.append(rect)

        for rect in self.rects:
            x, y, w, h = rect
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), thickness=2)

        pub_image = self.bridge.cv2_to_imgmsg(frame, encoding="rgb8")
        self.color_pub.publish(pub_image)

    def estimate_pot_position(self, x, y):
        #print(self.depth_image[y,x])
        y, z, x = pyrealsense2.rs2_deproject_pixel_to_point(self._intrinsics, [y, x], self.depth_image[y,x])
        return [x/1000,-y/1000,-z/1000]

if __name__ == '__main__':
    try:
        rospy.init_node('extract_pot_region')
        print("create extract_pot_region")

        ExtractPotRegion()
        rospy.spin()

    except rospy.ROSInterruptException:
        print("program interrupted before completion")
