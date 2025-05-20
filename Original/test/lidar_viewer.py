#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import numpy as np

fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlim(-120, 120)
ax.set_ylim(-120, 120)
ax.set_aspect('equal')
lidar_points, = ax.plot([], [], 'bo')
ranges = None

def callback(data):
    global ranges
    print(f"Total number of points in data.ranges: {len(data.ranges)}")
    print(f"Angle min: {data.angle_min}, Angle max: {data.angle_max}, Angle increment: {data.angle_increment}")
    print(f"Range min: {data.range_min}, Range max: {data.range_max}") # <--- 이 줄을 추가했어요!

    # 예상 포인트 수 계산 (부동소수점 오차 감안)
    num_expected_points = int(round((data.angle_max - data.angle_min) / data.angle_increment)) + 1
    print(f"Calculated number of points based on angles: {num_expected_points}")
    
    # ranges = data.ranges[0:360] # 기존 코드
    # 만약 720개까지 가져오고 싶다면, 실제 길이를 고려해서 슬라이싱하는 것이 안전합니다.
    # 예를 들어, 최대 720개 또는 실제 길이 중 작은 값만큼 가져오도록 할 수 있습니다.
    # ranges = data.ranges[0:min(720, len(data.ranges))] 
    
    ranges = data.ranges[0:360] # 일단은 원래 코드로 둘게요!


def main():
    global ranges
    rospy.init_node('lidar_visualizer', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, callback)
    plt.ion()
    plt.show()
    rate = rospy.Rate(10) 
    print("Lidar Visualizer Ready ----------")
    
    while not rospy.is_shutdown():
        if ranges is not None:            
            angles = np.linspace(0,2*np.pi, len(ranges))+np.pi/2
            x = ranges * np.cos(angles)
            y = ranges * np.sin(angles)

            lidar_points.set_data(x, y)
            fig.canvas.draw_idle()
            plt.pause(0.01)         
        rate.sleep()

if __name__ == '__main__':
    main()
