#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2
import time
from sensor_msgs.msg import Image
from xycar_msgs.msg import XycarMotor
from cv_bridge import CvBridge

bridge = CvBridge()
image = None
motor_pub = None
frame_count = 0
prev_angle = 0.0
white_lost_count = 0

def img_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")

def drive(angle, speed):
    msg = XycarMotor()
    msg.angle = float(angle)
    msg.speed = float(speed)
    motor_pub.publish(msg)

def start():
    global motor_pub, frame_count, prev_angle, white_lost_count

    rospy.init_node('xycar_robust')
    motor_pub = rospy.Publisher('/xycar_motor', XycarMotor, queue_size=1)
    rospy.Subscriber('/usb_cam/image_raw', Image, img_callback)
    rospy.sleep(1.0)

    print("▶▶▶ Robust Driving Mode Start")

    while not rospy.is_shutdown():
        if image is None:
            continue

        height, width = image.shape[:2]
        roi = image[int(height * 0.6):, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        lower_white = np.array([0, 0, 180])
        upper_white = np.array([180, 30, 255])
        white_mask = cv2.inRange(hsv, lower_white, upper_white)

        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        frame_count += 1
        if frame_count <= 50:
            angle = 0
            speed = 60
            drive(angle, speed)
            print(f"[INIT] Frame {frame_count} → 직진")
            cv2.imshow("original", image)
            cv2.imshow("white_mask", white_mask)
            cv2.imshow("yellow_mask", yellow_mask)
            cv2.waitKey(1)
            time.sleep(0.1)
            continue

        left_mask  = white_mask[:, :width//3]
        mid_mask   = white_mask[:, width//3:2*width//3]
        right_mask = white_mask[:, 2*width//3:]

        left_ratio  = cv2.countNonZero(left_mask)  / left_mask.size
        mid_ratio   = cv2.countNonZero(mid_mask)   / mid_mask.size
        right_ratio = cv2.countNonZero(right_mask) / right_mask.size
        total_white = cv2.countNonZero(white_mask)

        if total_white > 300:
            white_lost_count = 0
            error = (left_ratio - right_ratio) * 100
            angle = np.clip(error * 0.6, -30, 30)

            if left_ratio - right_ratio > 0.05:
                angle += 15
            elif right_ratio - left_ratio > 0.05:
                angle -= 15

            direction = "WHITE TRACK"
        else:
            white_lost_count += 1
            if white_lost_count >= 1:  # fallback 바로 진입
                M = cv2.moments(yellow_mask)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    error = cx - (width // 2)
                    angle = np.clip(error * 0.005, -25, 25)
                    direction = "YELLOW FALLBACK"
                else:
                    angle = -15  # 기본 회피각 부여
                    direction = "NO LINE → BASE ESCAPE"
            else:
                angle = prev_angle
                direction = f"WHITE LOST {white_lost_count}"

        # angle 변화 제한
        max_delta = 10
        if abs(angle - prev_angle) > max_delta:
            angle = prev_angle + np.sign(angle - prev_angle) * max_delta

        prev_angle = angle

        # 속도 결정
        abs_angle = abs(angle)
        if abs_angle < 5:
            speed = 80
        elif abs_angle < 10:
            speed = 60
        else:
            speed = 45
        if "HOLD" in direction or "NO LINE" in direction:
            speed = 30

        print(f"[INFO] Mode: {direction}, Angle: {angle:.2f}, Speed: {speed}, White: {total_white}, L/M/R: {left_ratio:.3f} {mid_ratio:.3f} {right_ratio:.3f}")

        cv2.imshow("original", image)
        cv2.imshow("white_mask", white_mask)
        cv2.imshow("yellow_mask", yellow_mask)
        cv2.waitKey(1)

        drive(angle, speed)
        time.sleep(0.1)

if __name__ == '__main__':
    start()
