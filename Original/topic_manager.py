#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import Image, LaserScan
from xycar_msgs.msg import XycarMotor
from cv_bridge import CvBridge

class TopicManager:
    def __init__(self):
        self.image = np.empty(shape=[0])
        self.ranges = None
        self.motor = None
        self.motor_msg = XycarMotor()
        self.bridge = CvBridge()
        
    def init_node(self):
        rospy.init_node('Track_Driver')
        rospy.Subscriber("/usb_cam/image_raw/", Image, self._camera_callback, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self._lidar_callback, queue_size=1)
        self.motor = rospy.Publisher('xycar_motor', XycarMotor, queue_size=1)
        
    def wait_for_topics(self):
        rospy.wait_for_message("/usb_cam/image_raw/", Image)
        print("Camera Ready")
        rospy.wait_for_message("/scan", LaserScan)
        print("Lidar Ready")
        
    def _camera_callback(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        
    def _lidar_callback(self, data):
        self.ranges = data.ranges[0:360]
        
    def drive(self, angle, speed):
        self.motor_msg.angle = float(angle)
        self.motor_msg.speed = float(speed)
        self.motor.publish(self.motor_msg)
        
    def get_image(self):
        return self.image
        
    def get_ranges(self):
        return self.ranges
