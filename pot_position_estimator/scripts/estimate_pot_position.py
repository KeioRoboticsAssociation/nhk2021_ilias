#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import message_filters
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import sys
import cv2
import numpy as np
import pyrealsense2

class EstimatePotPosition():
    def __init__(self):
        self.bridge = CvBridge()
        self._intrinsics = pyrealsense2.intrinsics()
        self.rects = []

        self.camerainfo_sub = rospy.Subscriber(
            "/camera/depth/camera_info", CameraInfo, self.camerainfo_callback, queue_size=1)
        self.color_pub = rospy.Publisher('image_with_BBox', Image, queue_size=1)
        self.points_pub = rospy.Publisher("estimate_pot_points", Marker, queue_size = 1)

        # https://qiita.com/nabion/items/319d4ffdc3d87bfb0076
        self.color_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
        self.depth_sub = message_filters.Subscriber("/camera/depth/image_raw", Image)
        fps = 10.
        delay = 1.0 / fps * 0.5 # how long is synchronized delay accepted
        mf = message_filters.ApproximateTimeSynchronizer([self.color_sub, self.depth_sub], 1, delay)
        mf.registerCallback(self.image_callback)

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

    def image_callback(self, color, depth):
        # get color image
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(color, 'passthrough')
        except CvBridgeError, e:
            rospy.logerr(e)

        # get depth image
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(depth, 'passthrough')
        except CvBridgeError, e:
            rospy.logerr(e)
        self.depth_image = np.array(self.depth_image, dtype=np.float32)        

        # processing
        self.create_BBox()
        self.pred()


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

    def pred(self):
        for rect in self.rects:
            x, y, w, h = rect
            p_x, p_y, p_z = self.estimate_pot_position(x+w//2,y+h//2)
            print(str(p_x) + " " + str(p_y) + " " + str(p_z))
            self.points_pub.publish(self.set_marker(p_x, p_y, p_z))

    def estimate_pot_position(self, x, y):
        #print(self.depth_image[y,x])
        y, z, x = pyrealsense2.rs2_deproject_pixel_to_point(self._intrinsics, [y, x], self.depth_image[y,x])
        return [x/1000,-y/1000,-z/1000]

    def set_marker(self, x, y, z):
        marker_data = Marker()
        marker_data.header.frame_id = "camera_link"
        marker_data.header.stamp = rospy.Time.now()

        marker_data.ns = "basic_shapes"
        marker_data.id = 0

        marker_data.action = Marker.ADD

        marker_data.pose.position.x = x
        marker_data.pose.position.y = y
        marker_data.pose.position.z = z

        marker_data.pose.orientation.x=0.0
        marker_data.pose.orientation.y=0.0
        marker_data.pose.orientation.z=1.0
        marker_data.pose.orientation.w=0.0

        marker_data.color.r = 1.0
        marker_data.color.g = 0.0
        marker_data.color.b = 0.0
        marker_data.color.a = 1.0

        marker_data.scale.x = 0.3
        marker_data.scale.y = 0.3
        marker_data.scale.z = 0.3

        marker_data.lifetime = rospy.Duration()

        marker_data.type = Marker.SPHERE

        return marker_data


if __name__ == '__main__':
    try:
        rospy.init_node('estimate_pot_position')
        print("create estimate_pot_position_node")

        EstimatePotPosition()
        rospy.spin()

    except rospy.ROSInterruptException:
        print("program interrupted before completion")
