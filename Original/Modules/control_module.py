import queue
import threading
import time
from .data_structures import ActionCommand, ControlActuatorCommands
import numpy as np # For np.clip
import math # For math.degrees
import logging

logger = logging.getLogger(__name__)

class VehicleInterface:
    """Dummy interface to simulate sending commands to a vehicle."""
    def __init__(self, config: dict, motor_publisher=None, motor_msg_template=None):
        self.config = config
        self.motor_publisher = motor_publisher
        self.motor_msg = motor_msg_template # This should be an instance of XycarMotor
        self.max_speed = self.config.get("max_xycar_speed", 50.0) # Xycar의 최대 속도값 (0-50)
        self.min_speed = self.config.get("min_xycar_speed", 0.0)
        self.max_angle = self.config.get("max_xycar_angle", 50.0) # Xycar의 최대 조향각 (절대값)

        if self.motor_publisher and self.motor_msg:
            logger.info(f"VehicleInterface: Initialized with ROS motor publisher. MaxSpeed: {self.max_speed}, MaxAngle: {self.max_angle}")
        else:
            logger.warning(f"VehicleInterface: Initialized (simulation mode - no ROS publisher). Config: {config}")

    def send_commands(self, steering: float, throttle: float, brake: float):
        if self.motor_publisher and self.motor_msg:
            # steering: 라디안 단위의 목표 조향각
            # throttle: 0.0 ~ 1.0 사이의 가속 명령
            # brake: 0.0 ~ 1.0 사이의 제동 명령 (1.0이 최대 제동)

            # 1. 조향각 변환 (라디안 -> Xycar 각도 단위, -50 ~ 50)
            # 차량의 전방을 기준으로 왼쪽이 +, 오른쪽이 - (Xycar 기준과 동일한지 확인 필요)
            # 일반적으로 로봇공학에서는 반시계방향(좌회전)이 +
            target_angle_deg = np.clip(math.degrees(steering), -self.max_angle, self.max_angle)

            # 2. 속도 결정 (throttle, brake -> Xycar 속도 단위, 0 ~ 50)
            target_speed = 0.0
            if brake > 0.1: # 브레이크가 일정 값 이상이면 감속/정지
                # 현재 속도에서 brake 값에 비례하여 감속하는 로직이 필요하나,
                # XycarMotor는 목표 속도를 직접 지정하므로, 여기서는 정지 또는 낮은 속도로 설정
                target_speed = self.min_speed # 또는 0으로 설정하여 정지
            else: # 브레이크가 약하면 throttle 값에 따라 속도 설정
                # throttle (0~1) 값을 Xycar 속도 (min_speed ~ max_speed)로 매핑
                target_speed = self.min_speed + throttle * (self.max_speed - self.min_speed)
            
            target_speed = np.clip(target_speed, self.min_speed, self.max_speed)
            
            self.motor_msg.angle = float(target_angle_deg)
            self.motor_msg.speed = float(target_speed)
            self.motor_publisher.publish(self.motor_msg)
            # logger.debug(f"VehicleInterface (ROS): Sent Angle: {self.motor_msg.angle:.2f}, Speed: {self.motor_msg.speed:.2f}") # Frequent
        else:
            # 시뮬레이션 모드 또는 오류 처리
            logger.info(f"VehicleInterface (Sim): Steering(rad): {steering:.2f}, Throttle: {throttle:.2f}, Brake: {brake:.2f}")

class ControlModule:
    def __init__(self, config: dict,
                 input_queue_planning: queue.Queue,
                 vehicle_interface_config: dict):
        self.config = config
        self.input_queue_planning = input_queue_planning

        self.active_law_name = self.config.get("active_control_law", "basic_pid") # Default strategy
        self.law_params = self.config.get(f"{self.active_law_name}_params", {})
        logger.info(f"ControlModule: Active control law: {self.active_law_name} with params: {self.law_params}")

        self.law_map = {
            "basic_pid": self._execute_basic_pid_control,
        }
        # vehicle_interface_config에 motor_publisher와 motor_msg_template이 주입되어야 함
        self.vehicle_interface = VehicleInterface(
            vehicle_interface_config,
            motor_publisher=vehicle_interface_config.get("ros_motor_publisher"),
            motor_msg_template=vehicle_interface_config.get("ros_motor_msg_template"))
        self._running = False
        self._thread = None

        # 로그 출력을 위한 마지막 값 저장 변수 및 임계값
        self.last_logged_velocity_mps = None
        self.last_logged_steering_angle_rad = None
        # Parameters for logging thresholds are now part of the strategy's params
        self.LOG_VELOCITY_THRESHOLD_MPS = self.law_params.get("log_velocity_threshold_mps", 0.05)
        self.LOG_ANGLE_THRESHOLD_RAD = self.law_params.get("log_angle_threshold_rad", 0.005)

        logger.info("ControlModule: Initialized.")

    def _execute_basic_pid_control(self, action: ActionCommand, params: dict) -> ControlActuatorCommands:
        # Placeholder for translating ActionCommand (velocity/angle) to low-level actuator commands
        # This would involve PID controllers or other vehicle dynamics models.

        # 로그 출력 조건 확인
        should_log = (
            self.last_logged_velocity_mps is None or
            abs(action.target_velocity_mps - self.last_logged_velocity_mps) > self.LOG_VELOCITY_THRESHOLD_MPS or
            abs(action.target_steering_angle_rad - self.last_logged_steering_angle_rad) > self.LOG_ANGLE_THRESHOLD_RAD
        )
        if should_log:
            logger.debug(f"BasicPIDControl: Translating action: Vel={action.target_velocity_mps:.2f} m/s, Angle={action.target_steering_angle_rad:.3f} rad")
            self.last_logged_velocity_mps = action.target_velocity_mps
            self.last_logged_steering_angle_rad = action.target_steering_angle_rad
        # Example: Proportional control (very basic)
        # Assume current speed is 0 for simplicity or needs feedback
        throttle_command = 0.0
        brake_command = 0.0

        # target_velocity_mps를 throttle/brake로 변환 (0~1 범위)
        # 이 로직은 차량 모델과 제어 전략에 따라 매우 달라질 수 있음.
        # 여기서는 간단한 비례 제어를 가정.
        max_module_speed_mps = params.get("max_control_speed_mps", 1.4) # m/s 단위의 최대 제어 속도
        
        if action.target_velocity_mps > 0.05: # 전진
            throttle_command = np.clip(action.target_velocity_mps / max_module_speed_mps, 0.0, 1.0)
        elif action.target_velocity_mps < -0.05: # 후진 또는 강한 제동 (Xycar는 후진 기능이 별도로 없음)
            brake_command = np.clip(abs(action.target_velocity_mps) / max_module_speed_mps, 0.0, 1.0)
        else: # 정지 또는 매우 낮은 속도
            brake_command = 0.2 # 약한 브레이크로 정지 유지 시도
        steering_command = action.target_steering_angle_rad # Pass through steering angle

        return ControlActuatorCommands(
            timestamp=action.timestamp,
            steering_command=steering_command,
            throttle_command=throttle_command,
            brake_command=brake_command
        )

    def run(self):
        logger.info(f"ControlModule: Thread started. Control Law: {self.active_law_name}")
        selected_law_method = self.law_map.get(self.active_law_name)

        while self._running:
            try:
                action_command: ActionCommand = self.input_queue_planning.get(timeout=1.0)
                if selected_law_method:
                    control_commands = selected_law_method(action_command, self.law_params)
                    self.vehicle_interface.send_commands(
                        control_commands.steering_command,
                        control_commands.throttle_command,
                        control_commands.brake_command
                    )
                else:
                    logger.warning(f"ControlModule: Control law '{self.active_law_name}' not found.")
                    # Fallback: send zero commands or hold last command? For safety, maybe zero.
                    self.vehicle_interface.send_commands(steering=0.0, throttle=0.0, brake=0.2) # Gentle brake

                self.input_queue_planning.task_done()
            except queue.Empty:
                if not self._running:
                    break
                continue
            except Exception as e:
                logger.error(f"ControlModule: Error processing command: {e}", exc_info=True)
        logger.info("ControlModule: Thread stopped.")

    def start(self):
        if not self._running:
            self._running = True
            self._thread = threading.Thread(target=self.run, name="ControlThread")
            self._thread.start()
            logger.info("ControlModule: Started.")

    def stop(self):
        if self._running:
            self._running = False
            if self._thread:
                self._thread.join(timeout=2.0)
            logger.info("ControlModule: Stopped.")