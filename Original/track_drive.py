#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 자율주행 시스템 메인 실행 파일
# 패키지 import
import numpy as np
import cv2, rospy, time, os, math
from sensor_msgs.msg import Image
from xycar_msgs.msg import XycarMotor
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge

# 모듈 시스템 import
import sys
import os
import logging
# Modules 패키지 경로 추가
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from Modules.main_system import MainSystem, load_dummy_config

# 프로그램 변수 선언

# Logger 스트림 클래스
class StreamToLogger(object):
   def __init__(self, logger, level):
      self.logger = logger
      self.level = level
      self.buffer = ''

   def write(self, message):
      self.buffer += message
      while '\n' in self.buffer:
          line, self.buffer = self.buffer.split('\n', 1)
          self.logger.log(self.level, line.rstrip())

   def flush(self):
      if self.buffer:
          self.logger.log(self.level, self.buffer.rstrip())
          self.buffer = ''

# 전역 변수 초기화
motor = None  # 모터노드
motor_msg = XycarMotor()  # 모터 토픽 메시지
Fix_Speed = 10  # 모터 속도 고정값
new_angle = 0  # 모터 조향각 초기값
new_speed = Fix_Speed  # 모터 속도 초기값
bridge = CvBridge()  # OpenCV 브릿지
             
# 메인 함수
def start():
    global motor, bridge

    original_stdout = sys.stdout
    logger = None
    
    try:
        print("Start program --------------")

        # ROS 노드 초기화
        rospy.init_node('Track_Driver')
        rospy.sleep(1.0)  # ROS 마스터와의 연결 안정화

        # 로깅 설정
        log_file_path = "/home/xytron/xycar_ws/src/kookmin/driver/Original/track_drive.log"
        logger = logging.getLogger('track_driver')
        logger.setLevel(logging.INFO)

        if logger.hasHandlers():
            logger.handlers.clear()

        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')

        # 파일 핸들러
        try:
            fh = logging.FileHandler(log_file_path, mode='a', encoding='utf-8')
            fh.setFormatter(formatter)
            logger.addHandler(fh)
        except Exception as e:
            print(f"Warning: Could not create log file handler: {e}")

        # 콘솔 핸들러
        sh = logging.StreamHandler(original_stdout)
        sh.setFormatter(formatter)
        logger.addHandler(sh)

        motor = rospy.Publisher('/xycar_motor', XycarMotor, queue_size=1)
            
        # 토픽 준비 대기 (슬래시 없는 토픽명으로 통일)
        rospy.wait_for_message("/usb_cam/image_raw", Image)
        print("Track_Driver: Camera Topic Ready -------------- (MainSystem will subscribe)")
        rospy.wait_for_message("/scan", LaserScan)
        print("Track_Driver: Lidar Topic Ready ---------- (MainSystem or Visualize.py will subscribe)")

        # stdout 리디렉션
        sys.stdout = StreamToLogger(logger, logging.INFO)
        logger.info("Sys.stdout redirected to logger. Subsequent print() statements will be logged to file and console.")
        print("This is a test print after redirection. It should appear in the log file and on the console.")
        
        # MainSystem 초기화
        config = load_dummy_config()

        # Perception Algorithm 설정
        if "perception_config" not in config:
            config["perception_config"] = {}
        if "detection" not in config["perception_config"]:
            config["perception_config"]["detection"] = {}
        logger.info(f"Active perception algorithm set to: {config['perception_config']['detection'].get('active_perception_algorithm', 'None')}")

        # Planning Algorithm 설정
        if "planning_config" not in config:
            config["planning_config"] = {}
        # path_planner
        if "path_planner" not in config["planning_config"]:
            config["planning_config"]["path_planner"] = {}
        if "active_strategy" not in config["planning_config"]["path_planner"]:
            config["planning_config"]["path_planner"]["active_strategy"] = "a_star_planner"
        logger.info(f"Active path planner strategy set to: {config['planning_config']['path_planner']['active_strategy']}")
        # decision_maker
        if "decision_maker" not in config["planning_config"]:
            config["planning_config"]["decision_maker"] = {}
        if "active_strategy" not in config["planning_config"]["decision_maker"]:
            config["planning_config"]["decision_maker"]["active_strategy"] = "rule_based_logic"
        logger.info(f"Active decision maker strategy set to: {config['planning_config']['decision_maker']['active_strategy']}")
        # action_planner
        if "action_planner" not in config["planning_config"]:
            config["planning_config"]["action_planner"] = {}
        if "active_strategy" not in config["planning_config"]["action_planner"]:
            config["planning_config"]["action_planner"]["active_strategy"] = "hsv_lane_following"
        logger.info(f"Active action planner strategy set to: {config['planning_config']['action_planner']['active_strategy']}")

        # Prediction Algorithm 설정
        if "prediction_config" not in config:
            config["prediction_config"] = {}
        if "active_prediction_strategy" not in config["prediction_config"]:
            config["prediction_config"]["active_prediction_strategy"] = "kalman_cv_prediction"
        logger.info(f"Active prediction strategy set to: {config['prediction_config']['active_prediction_strategy']}")

        # Control Algorithm 설정
        if "control_config" not in config:
            config["control_config"] = {}
        if "active_control_law" not in config["control_config"]:
            config["control_config"]["active_control_law"] = "basic_pid"
        logger.info(f"Active control law set to: {config['control_config']['active_control_law']}")

        # ROS 객체 추가
        config["ros_bridge"] = CvBridge()
        config["ros_motor_publisher"] = motor
        config["ros_motor_msg_template"] = XycarMotor()
        
        autonomous_system = MainSystem(config=config)
        
        # ROS 종료 시 시스템 정지
        rospy.on_shutdown(autonomous_system.stop)

        autonomous_system.start()
        
        print("===================================================")
        print(" S T A R T    D R I V I N G (Modular System)...")
        print(" LiDAR Visualization runs in Visualize.py (if launched).")
        print("===================================================")
        
        # 메인 루프
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            
    except KeyboardInterrupt:
        if logger:
            logger.info("Received KeyboardInterrupt. Shutting down gracefully.")
        else:
            print("Received KeyboardInterrupt. Shutting down gracefully.")
    except Exception as e:
        if logger:
            logger.critical(f"An unhandled exception occurred in start(): {e}", exc_info=True)
        else:
            original_stdout.write(f"CRITICAL ERROR (logger not init): {e}\n")
    finally:
        if 'autonomous_system' in locals() and autonomous_system is not None:
            if logger:
                logger.info("Executing autonomous_system.stop() in finally block.")
            autonomous_system.stop()
        
        # stdout 복원
        if sys.stdout != original_stdout:
            sys.stdout = original_stdout
        
        final_message = "\nProgram finished."
        if logger:
            logger.info(final_message + " Logging is being shut down.")
            for handler in logger.handlers[:]:
                handler.close()
                logger.removeHandler(handler)
        else:
            original_stdout.write(final_message + "\n")

# 메인함수 호출
if __name__ == '__main__':
    start()