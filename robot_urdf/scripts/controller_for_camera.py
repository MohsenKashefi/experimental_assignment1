#!/usr/bin/env python3

# ROS libraries
import rospy
import time
import math

# Python libs
import sys

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

import imutils

# OpenCV
import cv2

# ROS Messages
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import CompressedImage, CameraInfo

class CameraFixController:

    def __init__(self):
        rospy.init_node('camera_fix_controller')

        # Initialization of variables
        self.CameraCenter = Point()
        self.BoxCenter = Point()
        self.box_id_list = []  # List to hold detected box IDs
        self.box_centers_dict = {}  # Dictionary to hold center coordinates for each detected box
        self.current_box_id = 0
        self.info_gathering_mode = True  # Flag to control data gathering
        self.reached = False

        # Publishers
        self.image_pub = rospy.Publisher("/output/image_raw/compressed", CompressedImage, queue_size=1)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Subscribers
        rospy.Subscriber("/robot/camera1/image_raw/compressed", CompressedImage, self.controller_callback, queue_size=1)
        rospy.Subscriber("/robot/camera1/camera_info", CameraInfo, self.camera_callback, queue_size=1)
        rospy.Subscriber("/box/id_number", Int32, self.id_callback, queue_size=1)
        rospy.Subscriber("/box/center_loc", Point, self.center_callback, queue_size=1)

    def camera_callback(self, msg):
        """Find the center of the camera."""
        self.CameraCenter.x = msg.height / 2
        self.CameraCenter.y = msg.width / 2

    def id_callback(self, msg):
        """Update the ID of the detected box."""
        self.current_box_id = msg.data

    def center_callback(self, msg):
        """Update the center of the detected box."""
        self.BoxCenter.x = msg.x
        self.BoxCenter.y = msg.y

        if self.current_box_id and self.current_box_id not in self.box_id_list:
            self.box_centers_dict[self.current_box_id] = (msg.x, msg.y)

    def controller_callback(self, msg):
        """Main control logic for navigating and interacting with boxes."""
        np_arr = np.fromstring(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        vel = Twist()

        if self.info_gathering_mode:
            if len(self.box_id_list) < 5:
                vel.linear.x = 0
                vel.angular.z = 0.7

                if self.current_box_id and self.current_box_id not in self.box_id_list:
                    self.box_id_list.append(self.current_box_id)
                    self.box_id_list.sort()
                    rospy.loginfo(f"Updated box IDs: {self.box_id_list}")

            else:
                self.info_gathering_mode = False
                rospy.loginfo("Finished gathering all boxes. Starting interaction phase.")
        else:
            if len(self.box_id_list) > 0:
                self.current_box_id = self.box_id_list[0]
                target_x = self.BoxCenter.x
                target_y = self.BoxCenter.y
                rospy.loginfo(f"Targeting box {self.current_box_id}, Target: ({target_x}, {target_y}), Camera Center: ({self.CameraCenter.x}, {self.CameraCenter.y})")

                if abs(self.CameraCenter.x - target_x) < 10 and self.current_box_id == self.box_id_list[0]:
                    self.reached = True
                    vel.angular.z = 0
                    rospy.loginfo(f"Reached box {self.current_box_id}")

                    # Draw a circle around the box position on the image
                    cv2.circle(image_np, (int(target_x), int(target_y)), 20, (0, 255, 0), 3)

                    Image_msg = CompressedImage()
                    Image_msg.header.stamp = rospy.Time.now()
                    Image_msg.format = "jpeg"
                    Image_msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()

                    self.image_pub.publish(Image_msg)

                if self.reached:
                    self.box_id_list.pop(0)
                    self.reached = False
                    vel.angular.z = 0

                elif self.CameraCenter.x > target_x and self.current_box_id == self.box_id_list[0]:
                    vel.angular.z = 0.3
                    rospy.loginfo("Turn left")

                elif self.CameraCenter.x < target_x and self.current_box_id == self.box_id_list[0]:
                    vel.angular.z = -0.3
                    rospy.loginfo("Turn right")

                else:
                    vel.angular.z = 0.5
                    rospy.loginfo("Adjusting position")

            else:
                vel.angular.z = 0
                rospy.loginfo("All boxes processed.")

        self.velocity_publisher.publish(vel)

def main():
    CameraFixController()
    rospy.spin()

if __name__ == '__main__':
    main()

