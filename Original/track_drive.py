#!/usr/bin/env python
# -*- coding: utf-8 -*- 2

import numpy as np
import cv2, rospy, time, os, math
from sensor_msgs.msg import Image
from xycar_msgs.msg import XycarMotor
from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan
# import matplotlib.pyplot as plt # Visualize.py로 이동

# Modules 시스템 import
import sys
import os # os 모듈 추가
import logging # 로깅 모듈 사용
from Modules.error_manager import error_manager, ErrorCode
sys.path.append(os.path.dirname(os.path.abspath(__file__))) # track_drive.py가 있는 디렉토리를 sys.path에 추가하여 바로 아래 Modules 패키지를 찾을 수 있도록 함
from Modules.main_system import MainSystem, load_dummy_config

# Custom stream to redirect print output to logger
class StreamToLogger(object):
   """
   Fake file-like stream object that redirects writes to a logger instance.
   """
   def __init__(self, logger, level):
      self.logger = logger
      self.level = level
      self.buffer = '' # Use a buffer to handle partial writes

   def write(self, message):
      self.buffer += message
      while '\n' in self.buffer:
          line, self.buffer = self.buffer.split('\n', 1)
          self.logger.log(self.level, line.rstrip()) # rstrip() to remove trailing newline if any

   def flush(self): # flush() is important for some print scenarios
      if self.buffer:
          self.logger.log(self.level, self.buffer.rstrip())
          self.buffer = ''

motor = None  # 모터노드
motor_msg = XycarMotor()  # 모터 토픽 메시지
Fix_Speed = 10  # 모터 속도 고정 상수값 
new_angle = 0  # 모터 조향각 초기값
new_speed = Fix_Speed  # 모터 속도 초기값
bridge = CvBridge()  # OpenCV 함수를 사용하기 위한 브릿지 

def start():
    global motor, bridge
    original_stdout = sys.stdout
    try:
        print("Start program --------------")
        rospy.init_node('Track_Driver')
        # 로깅 설정
        log_file_path = "/home/xytron/xycar_ws/src/kookmin/driver/Original/track_drive.log"
        logger = logging.getLogger()
        logger.setLevel(logging.INFO)
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        fh = logging.FileHandler(log_file_path, mode='a', encoding='utf-8')
        fh.setFormatter(formatter)
        logger.addHandler(fh)
        sh = logging.StreamHandler(original_stdout)
        sh.setFormatter(formatter)
        logger.addHandler(sh)
        # ROS 토픽 준비 대기
        motor = rospy.Publisher('/xycar_motor', XycarMotor, queue_size=1)
        try:
            rospy.wait_for_message("/usb_cam/image_raw/", Image)
            print("Track_Driver: Camera Topic Ready -------------- (MainSystem will subscribe)")
        except Exception as e:
            error_manager.handle(ErrorCode.ROS_TOPIC_TIMEOUT, f"카메라 토픽: {e}")
            raise
        try:
            rospy.wait_for_message("/scan", LaserScan)
            print("Track_Driver: Lidar Topic Ready ---------- (MainSystem or Visualize.py will subscribe)")
        except Exception as e:
            error_manager.handle(ErrorCode.ROS_TOPIC_TIMEOUT, f"라이다 토픽: {e}")
            raise
        # stdout 리디렉션
        try:
            sys.stdout = StreamToLogger(logger, logging.INFO)
            logging.info("Sys.stdout redirected to logger. Subsequent print() statements will be logged to file and console.")
        except Exception as e:
            error_manager.handle(ErrorCode.STDOUT_REDIRECT_FAIL, str(e))
        print("This is a test print after redirection. It should appear in the log file and on the console.")
        # MainSystem 설정 및 시작
        config = load_dummy_config()
        if "perception_config" not in config:
            config["perception_config"] = {}
        if "detection" not in config["perception_config"]:
            config["perception_config"]["detection"] = {}
        config["perception_config"]["detection"]["active_perception_algorithm"] = "hsv_lane_detection"  # 인지 알고리즘 활성화
        logging.info(f"Active perception algorithm set to: {config['perception_config']['detection']['active_perception_algorithm']}")
        config["ros_bridge"] = bridge
        config["ros_motor_publisher"] = motor
        config["ros_motor_msg_template"] = XycarMotor()
        try:
            autonomous_system = MainSystem(config=config)
        except Exception as e:
            error_manager.handle(ErrorCode.MAIN_SYSTEM_INIT_FAIL, str(e))
            raise
        rospy.on_shutdown(autonomous_system.stop)
        try:
            autonomous_system.start()
        except Exception as e:
            error_manager.handle(ErrorCode.MODULE_START_FAIL, str(e))
            raise
        print("===================================================")
        print(" S T A R T    D R I V I N G (Modular System)...")
        print(" LiDAR Visualization runs in Visualize.py (if launched).")
        print("===================================================")
        while not rospy.is_shutdown():
            try:
                rospy.sleep(0.1)
            except Exception as e:
                error_manager.handle(ErrorCode.MODULE_RUNTIME_EXCEPTION, str(e))
                break
    except Exception as e:
        error_manager.handle(ErrorCode.UNKNOWN, str(e))
        print(f"An unhandled exception occurred in start(): {e}")
    finally:
        if 'original_stdout' in locals() and sys.stdout != original_stdout:
            sys.stdout = original_stdout
        print(f"\nProgram finished.")
        if 'logger' in locals():
            logging.info("Program finished. Logging is being shut down.")
            logging.shutdown()

if __name__ == '__main__':
    start()