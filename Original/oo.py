#!/usr/bin/env python
# -*- coding: utf-8 -*- 2

#=============================================
# 본 프로그램은 2025 제8회 국민대 자율주행 경진대회에서
# 예선과제를 수행하기 위한 파일입니다. 
# 예선과제 수행 용도로만 사용가능하며 외부유출은 금지됩니다.
#=============================================
import numpy as np
import cv2, rospy, time, math
from sensor_msgs.msg import Image
from xycar_msgs.msg import XycarMotor
from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt

#=============================================
# 프로그램 변수 선언부
#=============================================
image = np.empty(shape=[0])
ranges = None
motor = None
motor_msg = XycarMotor()
Fix_Speed = 70  # 기본 속도
bridge = CvBridge()

fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlim(-120, 120)
ax.set_ylim(-120, 120)
ax.set_aspect('equal')
lidar_points, = ax.plot([], [], 'bo')

#=============================================
# 콜백함수
#=============================================
def usbcam_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")

def lidar_callback(data):
    global ranges    
    ranges = data.ranges[0:360]

#=============================================
# 모터 발행
#=============================================
def drive(angle, speed):
    motor_msg.angle = float(angle)
    motor_msg.speed = float(speed)
    motor.publish(motor_msg)

#=============================================
# 직선 기울기 및 절편 계산                         slope < 0 이면 왼쪽 차선, slope < 0 이면 오른쪽 차선
#=============================================
def get_line_params(x1, y1, x2, y2):
    if x2 - x1 == 0:
        return float('inf'), x1
    slope = (y2 - y1) / (x2 - x1)
    intercept = y1 - slope * x1
    return slope, intercept

#=============================================
# 메인 함수
#=============================================
def start():
    global motor, image, ranges

    rospy.init_node('Track_Driver')
    rospy.Subscriber("/usb_cam/image_raw/", Image, usbcam_callback, queue_size=1)
    rospy.Subscriber("/scan", LaserScan, lidar_callback, queue_size=1)
    motor = rospy.Publisher('xycar_motor', XycarMotor, queue_size=1)

    rospy.wait_for_message("/usb_cam/image_raw/", Image)
    rospy.wait_for_message("/scan", LaserScan)

    plt.ion()
    plt.show()

    prev_angle = 0.0        # 이전 루프에서 계산된 조향각을 저장하는 변수
    roi_start_row_ratio = 0.6       # ROI 시작지점 비율(이미지의 하단 40%만 사용)
    P_GAIN = 0.35       #       P 제어기의 비례 계수. angle = error * P_GAIN = 조향값
    MAX_DELTA = 10      #       루프마다 변할 수 있는 최대 조향 각도

    lane_state = "CENTER"  # 차선 상태 초기값 설정 CENTER, LEFT, RIGHT

    while not rospy.is_shutdown():
        if image.shape[0] == 0:
            continue

        height, width, _ = image.shape
        roi_start_row = int(height * roi_start_row_ratio)
        roi_img = image[roi_start_row:, :]

        display_image = image.copy()
        cv2.rectangle(display_image, (0, roi_start_row), (width-1, height-1), (0, 255, 0), 2) # ROI 영역을 시각화(녹색 박스)

        # -------------------
        # 1. 중앙선 (노란색 점선) 검출
        # -------------------
        hsv_roi = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([35, 255, 255])
        yellow_mask = cv2.inRange(hsv_roi, lower_yellow, upper_yellow)

        # -------------------
        # 2. 실선 (흰색) 검출
        # -------------------
        gray_roi = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)
        _, white_mask = cv2.threshold(gray_roi, 200, 255, cv2.THRESH_BINARY)

        # -------------------
        # 3. Combine
        # -------------------
        combined_mask = cv2.bitwise_or(yellow_mask, white_mask)
        edges = cv2.Canny(combined_mask, 50, 150)

        # -------------------
        # 4. 차선 검출
        # -------------------
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 30, minLineLength=20, maxLineGap=10)
        left_xs, right_xs, center_xs = [], [], []

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                y1 += roi_start_row
                y2 += roi_start_row
                slope, _ = get_line_params(x1, y1, x2, y2)
                if abs(slope) < 0.3 or math.isinf(slope):
                    continue

                if slope < 0:
                    left_xs.extend([x1, x2])
                    cv2.line(display_image, (x1, y1), (x2, y2), (255, 0, 0), 2)
                elif slope > 0:
                    right_xs.extend([x1, x2])
                    cv2.line(display_image, (x1, y1), (x2, y2), (0, 0, 255), 2)

        # -------------------
        # 5. 차선 중심 계산
        # -------------------
        left_avg_x = int(np.mean(left_xs)) if left_xs else -1
        right_avg_x = int(np.mean(right_xs)) if right_xs else -1

        # -------------------
        # 6. Lane state 결정
        # -------------------
        # 실선은 넘어가지 않도록 - 양쪽 차선 사이만 허용
        if left_avg_x != -1 and right_avg_x != -1:
            lane_center = (left_avg_x + right_avg_x) // 2
            if lane_state == "CENTER":
                target_center_x = lane_center
            else:
                target_center_x = lane_center  # 차선 변경 후 CENTER로 복귀
                lane_state = "CENTER"
        elif left_avg_x != -1:
            if lane_state == "CENTER":
                target_center_x = left_avg_x + (width // 4)
                # 중앙선 점선만 인식된 경우에만 LEFT로 변경
                if np.sum(yellow_mask) > 5000:
                    lane_state = "LEFT"
            else:
                target_center_x = left_avg_x + (width // 4)
        elif right_avg_x != -1:
            if lane_state == "CENTER":
                target_center_x = right_avg_x - (width // 4)
                # 중앙선 점선만 인식된 경우에만 RIGHT로 변경
                if np.sum(yellow_mask) > 5000:
                    lane_state = "RIGHT"
            else:
                target_center_x = right_avg_x - (width // 4)
        else:
            target_center_x = width // 2  # fallback

        cv2.putText(display_image, f"Lane: {lane_state}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
        cv2.circle(display_image, (target_center_x, height - 10), 10, (0, 255, 255), -1)

        # -------------------
        # 7. 조향각 계산
        # -------------------
        error = target_center_x - (width // 2)
        angle = np.clip(error * P_GAIN, -35, 35)
        if abs(angle - prev_angle) > MAX_DELTA:
            angle = prev_angle + np.sign(angle - prev_angle) * MAX_DELTA
        prev_angle = angle

        abs_angle = abs(angle)
        speed = Fix_Speed
        if abs_angle < 5:
            speed = 70
        elif abs_angle < 10:
            speed = 70
        elif abs_angle < 20:
            speed = 70
        else:
            speed = 40

        print(f"[INFO] Lane: {lane_state}, Angle: {angle:.2f}, Speed: {speed}")

        cv2.imshow("Original with ROI", display_image)
        cv2.imshow("Yellow Mask", yellow_mask)
        cv2.imshow("White Mask", white_mask)
        cv2.imshow("Edges", edges)

        if ranges is not None:
            angles_rad = np.linspace(0, 2*np.pi, len(ranges))
            x_lidar = ranges * np.cos(angles_rad - np.pi/2)
            y_lidar = ranges * np.sin(angles_rad - np.pi/2)
            lidar_points.set_data(x_lidar, y_lidar)
            fig.canvas.draw_idle()
            plt.pause(0.01)

        drive(angle=angle, speed=speed)
        time.sleep(0.01)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            print("[INFO] ESC 키를 눌러 프로그램을 종료합니다.")
            break

    plt.close('all')
    rospy.signal_shutdown('User requested shutdown')

if __name__ == '__main__':
    start()