# 제어 모듈
import queue
import _queue
import threading
import time
from .optimized_data_structures import ActionCommand, ControlActuatorCommands
from .optimized_data_structures import DataPriority, PriorityQueueItem
from .performance_monitor import performance_timing, PerformanceTracker
import numpy as np
import math
import logging

logger = logging.getLogger(__name__)

# 성능 추적기 초기화
performance_tracker = PerformanceTracker("ControlModule")

class VehicleInterface:
    """차량 인터페이스"""
    def __init__(self, config: dict, motor_publisher=None, motor_msg_template=None):
        self.config = config
        self.motor_publisher = motor_publisher
        self.motor_msg = motor_msg_template
        self.max_speed = self.config.get("max_xycar_speed", 50.0)
        self.min_speed = self.config.get("min_xycar_speed", 0.0)
        self.max_angle = self.config.get("max_xycar_angle", 50.0)
        
        # 속도 변환 인수 설정
        self.xycar_mps_to_speed_unit_factor = self.config.get("xycar_mps_to_speed_unit_factor", 35.71)

        if self.motor_publisher and self.motor_msg:
            logger.info(f"VehicleInterface: Initialized with ROS motor publisher. MaxSpeed: {self.max_speed}, MaxAngle: {self.max_angle}, SpeedFactor: {self.xycar_mps_to_speed_unit_factor}")
        else:
            logger.warning(f"VehicleInterface: Initialized (simulation mode - no ROS publisher). Config: {config}")

    def send_commands(self, steering_rad: float, target_velocity_mps: float):
        """제어 명령 전송"""
        if self.motor_publisher and self.motor_msg:
            # 조향각 변환 (라디안 -> Xycar 각도 단위)
            target_angle_deg = np.clip(math.degrees(steering_rad), -self.max_angle, self.max_angle)

            # 속도 변환 (m/s -> Xycar 속도 단위)
            XYCAR_MPS_TO_SPEED_UNIT_FACTOR = self.config.get("xycar_mps_to_speed_unit_factor", 35.71) 
            
            target_xycar_speed = 0.0
            if target_velocity_mps > 0.01: # 전진
                target_xycar_speed = target_velocity_mps * XYCAR_MPS_TO_SPEED_UNIT_FACTOR
            elif target_velocity_mps < -0.01: # 후진 (Xycar는 후진을 지원하지 않으므로, 0으로 처리하거나 경고)
                logger.warning(f"VehicleInterface: Reverse speed ({target_velocity_mps} m/s) requested, but Xycar may not support reverse. Setting speed to 0.")
                target_xycar_speed = 0.0
            # else: 정지 (target_xycar_speed = 0.0)
            
            target_xycar_speed = np.clip(target_xycar_speed, self.min_speed, self.max_speed)

            self.motor_msg.angle = float(target_angle_deg)
            self.motor_msg.speed = float(target_xycar_speed)
            self.motor_publisher.publish(self.motor_msg)
            print(f"VehicleInterface: Published to /xycar_motor - Angle: {self.motor_msg.angle:.2f}, Speed: {self.motor_msg.speed:.2f}")
            # logger.debug(f"VehicleInterface (ROS): Sent Angle: {self.motor_msg.angle:.2f}, Speed: {self.motor_msg.speed:.2f}") # Frequent
        else:
            logger.info(f"VehicleInterface (Sim): Steering(rad): {steering_rad:.2f}, TargetVel(mps): {target_velocity_mps:.2f}")

class ControlModule:
    def __init__(self, config: dict,
                 input_queue_planning: queue.Queue,
                 vehicle_interface_config: dict):
        self.config = config
        self.input_queue_planning = input_queue_planning

        # 속도 변환 인수 설정 (Planning 모듈과 일치)
        self.xycar_mps_to_speed_unit_factor = self.config.get("xycar_mps_to_speed_unit_factor", 35.71)

        self.active_law_name = self.config.get("active_control_law", "basic_pid") # Default strategy
        self.law_params = self.config.get(f"{self.active_law_name}_params", {})
        
        logger.info(f"ControlModule: Using speed conversion factor: {self.xycar_mps_to_speed_unit_factor}")
        logger.info(f"ControlModule: Active control law: {self.active_law_name} with params: {self.law_params}")

        self.law_map = {
            "basic_pid": self._execute_basic_pid_control,
            "vehicle_model_pid": self._execute_vehicle_model_pid, # 새로운 제어 법칙 추가
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
        # Update logging thresholds from params, in case they changed
        self.LOG_VELOCITY_THRESHOLD_MPS = params.get("log_velocity_threshold_mps", 0.05)
        self.LOG_ANGLE_THRESHOLD_RAD = params.get("log_angle_threshold_rad", 0.005)

        if should_log:
            logger.debug(f"BasicPIDControl: Translating action: Vel={action.target_velocity_mps:.2f} m/s, Angle={action.target_steering_angle_rad:.3f} rad")
            self.last_logged_velocity_mps = action.target_velocity_mps
            self.last_logged_steering_angle_rad = action.target_steering_angle_rad

        return ControlActuatorCommands(
            timestamp=action.timestamp,
            steering_command_rad=action.target_steering_angle_rad,
            target_velocity_mps=action.target_velocity_mps
        )

    def _execute_vehicle_model_pid(self, action: ActionCommand, params: dict) -> ControlActuatorCommands:
        logger.debug(f"VehicleModelPID: Action: Vel={action.target_velocity_mps:.2f}, Angle={action.target_steering_angle_rad:.3f}. Params: {params}")

        kp_speed = params.get("kp_speed", 0.8)
        kp_steer_lat = params.get("kp_steer_lat", 0.7) # Using lateral P-gain for steering
        
        # For this P-controller, target_velocity_mps from ActionCommand is the direct target.
        # A more complex PID would calculate an error (target_velocity - current_velocity)
        # and then determine throttle/brake. Here, we assume ActionCommand's velocity is the desired output.
        final_target_velocity_mps = action.target_velocity_mps * kp_speed # Apply P-gain to velocity

        # --- Speed Control (P-controller) ---
        # The `final_target_velocity_mps` will be passed to VehicleInterface,
        # which will convert it to Xycar speed units.
        # No explicit throttle/brake calculation here, as we output target velocity.

        # --- Steering Control (P-controller) ---
        # Assuming target_steering_angle_rad is the desired steering output after high-level planning
        # A more complex model would use CTE (Cross-Track Error) and Yaw Error.
        # Here, we directly use the target_steering_angle_rad as if it's an "error" or desired output.
        steering_command_rad = action.target_steering_angle_rad * kp_steer_lat # Apply P gain

        logger.debug(f"VehicleModelPID Output: Steer(rad)={steering_command_rad:.3f}, TargetVel(mps)={final_target_velocity_mps:.2f}")
        return ControlActuatorCommands(
            timestamp=action.timestamp,
            steering_command_rad=steering_command_rad,
            target_velocity_mps=final_target_velocity_mps
        )


    @performance_timing
    def run(self):
        print("[CONTROL] run() 진입", flush=True)
        logger.info(f"ControlModule: Thread started. Active control law: {self.active_law_name}")
        selected_law_method = self.law_map.get(self.active_law_name)

        while self._running:
            print("[CONTROL] run() 루프 진입, 큐 get 시도", flush=True)
            try:
                # 우선순위 큐에서 액션 커맨드 수신
                queue_item = self.input_queue_planning.get(timeout=1.0)
                print(f"ControlModule: Got action command from planning queue", flush=True)
                # PriorityQueueItem에서 실제 데이터 추출
                if isinstance(queue_item, tuple) and len(queue_item) == 3:
                    # (priority_value, timestamp, data) 형태
                    _, _, action_command = queue_item
                elif hasattr(queue_item, 'data'):
                    # PriorityQueueItem 객체
                    action_command = queue_item.data
                else:
                    # 직접 ActionCommand 객체
                    action_command = queue_item
                print(f"ControlModule: Received action command - speed: {getattr(action_command, 'target_speed_kph', 0):.2f} kph, steering: {getattr(action_command, 'steering_angle_deg', 0):.2f} deg", flush=True)
                    
                if selected_law_method:
                    control_commands = selected_law_method(action_command, self.law_params)
                    self.vehicle_interface.send_commands(
                        steering_rad=control_commands.steering_command_rad,
                        target_velocity_mps=control_commands.target_velocity_mps
                    )
                    performance_tracker.record_metric("commands_processed", 1)
                else:
                    logger.warning(f"ControlModule: Control law '{self.active_law_name}' not found.")
                    # Fallback: send zero commands for safety
                    self.vehicle_interface.send_commands(steering_rad=0.0, target_velocity_mps=0.0)
                    performance_tracker.record_metric("fallback_commands", 1)
                
                self.input_queue_planning.task_done()
                
            except (queue.Empty, _queue.Empty):
                if not self._running:
                    break
                performance_tracker.record_metric("timeout_events", 1)
                continue
            except Exception as e:
                logger.error(f"ControlModule: Error processing command: {e}", exc_info=True)
                performance_tracker.record_metric("error_events", 1)
                
        logger.info("ControlModule: Thread stopped.")

    def start(self):
        if not self._running:
            self._running = True
            self._thread = threading.Thread(target=self.run, name="ControlThread")
            self._thread.start()
            logger.info(f"ControlModule: Started. Active control law: {self.active_law_name}")