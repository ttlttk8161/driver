#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
import threading
from topic_manager import TopicManager
from visualizer import Visualizer
from perception import Perception
from control import Control
from thread_manager import ThreadManager

Fix_Speed = 10

sensor_lock = threading.Lock()
perception_lock = threading.Lock()
control_lock = threading.Lock()

sensor_event = threading.Event()
perception_event = threading.Event()
control_event = threading.Event()

shared_data = {
    'image': None,
    'ranges': None,
    'lane_data': None,
    'obstacle_data': None,
    'vis_img': None,
    'angle': 0.0,
    'speed': Fix_Speed
}

def sensor_thread(topic_mgr):
    while not rospy.is_shutdown():
        image = topic_mgr.get_image()
        ranges = topic_mgr.get_ranges()
        with sensor_lock:
            shared_data['image'] = image
            shared_data['ranges'] = ranges
        sensor_event.set()
        time.sleep(0.01)

def perception_thread(perception):
    while not rospy.is_shutdown():
        sensor_event.wait()
        with sensor_lock:
            image = shared_data['image']
            ranges = shared_data['ranges']
        lane_data, vis_img = perception.process_image(image)
        obstacle_data = perception.process_lidar(ranges)
        with perception_lock:
            shared_data['lane_data'] = lane_data
            shared_data['obstacle_data'] = obstacle_data
            shared_data['vis_img'] = vis_img
        perception_event.set()
        sensor_event.clear()
        time.sleep(0.01)

def control_thread(control):
    while not rospy.is_shutdown():
        perception_event.wait()
        with perception_lock:
            lane_data = shared_data['lane_data']
            obstacle_data = shared_data['obstacle_data']
        angle, speed = control.calculate_control(lane_data, obstacle_data)
        with control_lock:
            shared_data['angle'] = angle
            shared_data['speed'] = speed
        control_event.set()
        perception_event.clear()
        time.sleep(0.01)

def actuator_thread(topic_mgr, visualizer):
    while not rospy.is_shutdown():
        control_event.wait()
        with sensor_lock:
            image = shared_data['image']
            ranges = shared_data['ranges']
        with control_lock:
            angle = shared_data['angle']
            speed = shared_data['speed']
        with perception_lock:
            vis_img = shared_data.get('vis_img', None)
        if vis_img is not None:
            visualizer.show_camera(vis_img)
        # perception의 show_lane_info로 별도 창에 차선 정보 시각화
        if image is not None and hasattr(image, 'size') and image.size > 0:
            from perception import Perception
            perception = shared_data.get('perception_instance', None)
            if perception is not None:
                perception.show_lane_info(image)
        topic_mgr.drive(angle=angle, speed=speed)
        control_event.clear()
        time.sleep(0.01)

def start():
    topic_mgr = TopicManager()
    visualizer = Visualizer()
    perception = Perception()
    control = Control()
    # perception 인스턴스를 shared_data에 저장하여 actuator_thread에서 사용
    shared_data['perception_instance'] = perception
    
    print("Start program")
    topic_mgr.init_node()
    topic_mgr.wait_for_topics()
    visualizer.init_lidar_plot()
    control.set_target_speed(Fix_Speed)
    control.set_speed_limits(5, 20)
    control.set_pid_gains(1.0, 0.0, 0.1)
    print("======================================")
    print(" S T A R T    D R I V I N G ...")
    print("======================================")

    tm = ThreadManager()
    tm.add_thread(sensor_thread, args=(topic_mgr,))
    tm.add_thread(perception_thread, args=(perception,))
    tm.add_thread(control_thread, args=(control,))
    tm.add_thread(actuator_thread, args=(topic_mgr, visualizer))
    tm.start_all()

    # 메인스레드에서 시각화 루프 실행
    while not rospy.is_shutdown():
        with sensor_lock:
            ranges = shared_data['ranges']
        visualizer.update_lidar(ranges)
        time.sleep(0.05)

if __name__ == '__main__':
    start()
