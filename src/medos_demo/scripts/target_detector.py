#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class ImageTargetDetectorNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('image_target_detector_node', anonymous=True)

        self.camera_ctrl_pub = rospy.Publisher('/bebop/camera_control', Twist, queue_size=10)
        self.image_sub = rospy.Subscriber('/bebop/image_raw', Image, self.on_receive_raw_image)
        self.image_pub = rospy.Publisher('/bebop/image_target_detections', Image, queue_size=10)

        self.bridge = CvBridge()
        self.color_blob_size_threshold = 500 # number of pixels required in blob

        # Wait for publisher to connect before sending camera command
        timeout = rospy.Time.now() + rospy.Duration(7.0)  # wait up to 7 seconds
        while self.camera_ctrl_pub.get_num_connections() == 0:
            if rospy.Time.now() > timeout:
                rospy.logwarn("Timed out waiting for /bebop/camera_control subscribers.")
                break
            rospy.loginfo("Waiting for /bebop/camera_control subscriber...")
            rospy.sleep(0.1)

        # publish a camera control message to point camera downward
        # rostopic pub --once /bebop/camera_control geometry_msgs/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 10.0, z: 0.0}}'
        downward_viewpoint = Twist()
        downward_viewpoint.angular.y = -83.0 # degrees (docs say 83 deg is as far is it goes)
        self.camera_ctrl_pub.publish(downward_viewpoint)

        rospy.loginfo("Target Detector Node initialized.")

    def on_receive_raw_image(self, msg):
        try:
            # Convert ROS image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", str(e))
            return

        # Convert to HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # search for color blobs
        mask = self._get_target_mask(hsv_image=hsv)

        # Find contours
        _, contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            if cv2.contourArea(contour) > self.color_blob_size_threshold:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 0, 255), 2)  # red boxes

        # Convert back to ROS Image message and publish
        try:
            output_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            self.image_pub.publish(output_msg)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", str(e))

    def _get_target_mask(self, hsv_image):

        neon_pink_lower = np.array([130, 50, 50])
        neon_pink_upper = np.array([175, 255, 255])

        neon_yellow_lower = np.array([25, 100, 100])
        neon_yellow_upper = np.array([35, 255, 255])

        neon_green_lower = np.array([35, 40, 40])
        neon_green_upper = np.array([85, 255, 255])

        pink_mask   = cv2.inRange(hsv_image, neon_pink_lower,   neon_pink_upper)
        green_mask  = cv2.inRange(hsv_image, neon_green_lower,  neon_green_upper)
        yellow_mask = cv2.inRange(hsv_image, neon_yellow_lower, neon_yellow_upper)

        combined_mask = cv2.bitwise_or(pink_mask, green_mask)
        combined_mask = cv2.bitwise_or(combined_mask, yellow_mask)

        return combined_mask

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        print("starting node...")
        node = ImageTargetDetectorNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
