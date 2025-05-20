#!/usr/bin/env python
# -*- coding: utf-8 -*- 2
#=============================================
# 본 프로그램은 2025 제8회 국민대 자율주행 경진대회에서
# 예선과제를 수행하기 위한 파일입니다. 
# 예선과제 수행 용도로만 사용가능하며 외부유출은 금지됩니다.
#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
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
# track_drive.py가 있는 디렉토리를 sys.path에 추가하여
# 바로 아래 Modules 패키지를 찾을 수 있도록 함
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from Modules.main_system import MainSystem, load_dummy_config

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================

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

# image = np.empty(shape=[0])  # MainSystem의 SensorInputManager가 처리하므로 제거
# ranges = None  # MainSystem의 SensorInputManager가 처리하므로 제거
motor = None  # 모터노드
motor_msg = XycarMotor()  # 모터 토픽 메시지
Fix_Speed = 10  # 모터 속도 고정 상수값 
new_angle = 0  # 모터 조향각 초기값
new_speed = Fix_Speed  # 모터 속도 초기값
bridge = CvBridge()  # OpenCV 함수를 사용하기 위한 브릿지 

#=============================================
# 라이다 스캔정보 시각화는 Visualize.py로 분리됨
#=============================================
# fig, ax = plt.subplots(figsize=(8, 8)) # Visualize.py로 이동
# (이하 Matplotlib 관련 변수 제거)

#=============================================
# 콜백함수 - 카메라 토픽을 처리하는 콜백함수
# MainSystem의 SensorInputManager가 직접 구독하므로 이 콜백은 제거합니다.
#=============================================
# def usbcam_callback(data):
#     pass # 제거
   
#=============================================
# 콜백함수 - 라이다 토픽을 받아서 처리하는 콜백함수
# MainSystem의 SensorInputManager가 직접 구독하거나 Visualize.py가 처리하므로 제거합니다.
#=============================================
# def lidar_callback(data):
#     pass # 제거
	
#=============================================
# 모터로 토픽을 발행하는 함수 
# MainSystem의 ControlModule -> VehicleInterface가 담당하므로 이 함수는 제거합니다.
#=============================================
# def drive(angle, speed):
#     pass # 제거
             
#=============================================
# 실질적인 메인 함수 
#=============================================
def start():
    global motor, bridge

    # Store original stdout for restoring it and for the console logger
    original_stdout = sys.stdout
    
    try:
        print("Start program --------------")

        #=========================================
        # 노드를 생성하고, 구독/발행할 토픽들을 선언합니다.
        #=========================================
        rospy.init_node('Track_Driver')

        # --- 로깅 설정 시작 ---
        log_file_path = "/home/xytron/xycar_ws/src/kookmin/driver/Original/track_drive.log"
        
        # 루트 로거 가져오기 및 레벨 설정
        logger = logging.getLogger() # Get the root logger
        logger.setLevel(logging.INFO) # Set the desired logging level

        # 다른 라이브러리에 의해 핸들러가 추가되었을 수 있으므로, 기존 핸들러 정리 (선택적)
        # if logger.hasHandlers():
        #    logger.handlers.clear()

        # 포매터 생성
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')

        # 파일 핸들러: 로그 파일에 기록
        fh = logging.FileHandler(log_file_path, mode='a', encoding='utf-8') # 'a' for append
        fh.setFormatter(formatter)
        logger.addHandler(fh)

        # 콘솔(터미널) 핸들러: 원래 터미널에도 로그 출력 (original_stdout 사용)
        sh = logging.StreamHandler(original_stdout) # Use the original stdout
        sh.setFormatter(formatter)
        logger.addHandler(sh)
        # --- 로깅 설정 끝 ---
        # rospy.Subscriber("/usb_cam/image_raw/",Image,usbcam_callback, queue_size=1) # MainSystem이 처리
        # rospy.Subscriber("/scan", LaserScan, lidar_callback, queue_size=1) # MainSystem 또는 Visualize.py가 처리
        motor = rospy.Publisher('/xycar_motor', XycarMotor, queue_size=1)
            
        #=========================================
        # 노드들로부터 첫번째 토픽들이 도착할 때까지 기다립니다.
        #=========================================
        rospy.wait_for_message("/usb_cam/image_raw/", Image)
        print("Track_Driver: Camera Topic Ready -------------- (MainSystem will subscribe)")
        rospy.wait_for_message("/scan", LaserScan)
        print("Track_Driver: Lidar Topic Ready ---------- (MainSystem or Visualize.py will subscribe)")

        # rospy.init_node() 이후에 sys.stdout을 StreamToLogger로 리디렉션
        sys.stdout = StreamToLogger(logger, logging.INFO)
        logging.info("Sys.stdout redirected to logger. Subsequent print() statements will be logged to file and console.")
        print("This is a test print after redirection. It should appear in the log file and on the console.")
        
        #=========================================
        # MainSystem 초기화 및 설정
        #=========================================
        # plt.ion() # Visualize.py로 이동
        # plt.show() # Visualize.py로 이동
        # print("Lidar Visualizer Ready ----------") # Visualize.py가 담당
        
        config = load_dummy_config() # 기본 더미 설정 로드

        # --- Perception Algorithm Selection ---
        # PerceptionModule에서 사용할 특정 알고리즘을 선택합니다.
        # None으로 설정하면 "작업을 수행하기 위한 모듈이 선택되지 않았습니다" 메시지가 PerceptionModule에서 출력됩니다.
        # 사용 가능한 알고리즘 이름 (예: "hsv_lane_detection", "canny_hough_lane_detection")은
        # main_system.py의 load_dummy_config 및 perception_module.py에 정의된 내용을 따릅니다.
        # 해당 알고리즘에 필요한 파라미터는 load_dummy_config에서 설정되어야 합니다.
        if "perception_config" not in config:
            config["perception_config"] = {}
        if "detection" not in config["perception_config"]:
            config["perception_config"]["detection"] = {}
        config["perception_config"]["detection"]["active_perception_algorithm"] = None # 여기에 원하는 알고리즘 이름 또는 None 설정
        logging.info(f"Active perception algorithm set to: {config['perception_config']['detection']['active_perception_algorithm']}")
        # --- Perception Algorithm Selection End ---

        # ROS 관련 객체를 config에 추가하여 MainSystem으로 전달
        config["ros_bridge"] = bridge # SensorInputManager에서 CvBridge 사용 위함
        config["ros_motor_publisher"] = motor # ControlModule의 VehicleInterface에서 사용
        config["ros_motor_msg_template"] = XycarMotor() # ControlModule에서 사용할 메시지 템플릿
        
        # HD Map 경로 설정 (필요시 실제 경로로 수정)
        # main_system.py의 if __name__ == "__main__": 블록에서 처리하던 것을 여기로 옮기거나,
        # MainSystem 생성자에서 경로를 필수로 받도록 수정할 수 있습니다.
        # 여기서는 MainSystem의 기본 dummy_map.hd 경로를 사용한다고 가정하고,
        # 필요시 config["hd_map_path"]를 여기서 덮어쓸 수 있습니다.
        # 예: config["hd_map_path"] = str(Path(__file__).resolve().parent.parent / "Modules/maps/my_custom_map.hd")

        autonomous_system = MainSystem(config=config)
        
        # ROS 종료 시 MainSystem 정지
        rospy.on_shutdown(autonomous_system.stop)

        autonomous_system.start() # MainSystem의 모든 모듈 스레드 시작
        
        print("===================================================")
        print(" S T A R T    D R I V I N G (Modular System)...")
        print(" LiDAR Visualization runs in Visualize.py (if launched).")
        print("===================================================")
        #=========================================
        # 메인 루프 (ROS 노드가 살아있도록 유지)
        #=========================================
        while not rospy.is_shutdown():
            # MainSystem의 스레드들이 백그라운드에서 실행됩니다.
            # 이 루프는 ROS 노드가 살아있도록 유지하는 역할만 합니다.
            rospy.sleep(0.1) 
    except Exception as e:
        print(f"An unhandled exception occurred in start(): {e}") # 로깅 대신 print 사용
    finally:
        # Restore original stdout before printing final messages directly to console
        if 'original_stdout' in locals() and sys.stdout != original_stdout : # Check if original_stdout was defined and redirection happened
            sys.stdout = original_stdout
        print(f"\nProgram finished.")
        if 'logger' in locals(): # Check if logger was defined
            logging.info("Program finished. Logging is being shut down.")
            logging.shutdown()

#=============================================
# 메인함수를 호출합니다.
# start() 함수가 실질적인 메인함수입니다.
#=============================================
if __name__ == '__main__':
    start()