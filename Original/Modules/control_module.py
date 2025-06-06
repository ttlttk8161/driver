import logging
from .data_structures import ActionCommand, ControlActuatorCommands
import numpy as np
import math
import threading

logger = logging.getLogger(__name__)

class VehicleInterface:
    def __init__(self, config: dict, motor_publisher=None, motor_msg_template=None):
        self.config = config
        self.motor_publisher = motor_publisher
        self.motor_msg = motor_msg_template
        self.max_speed = self.config.get("max_xycar_speed", 50.0)
        self.min_speed = self.config.get("min_xycar_speed", 0.0)
        self.max_angle = self.config.get("max_xycar_angle", 50.0)

    def send_commands(self, steering: float, throttle: float, brake: float):
        if self.motor_publisher and self.motor_msg:
            target_angle_deg = np.clip(math.degrees(steering), -self.max_angle, self.max_angle)
            if brake > 0.1:
                target_speed = self.min_speed
            else:
                target_speed = self.min_speed + throttle * (self.max_speed - self.min_speed)
            target_speed = np.clip(target_speed, self.min_speed, self.max_speed)
            self.motor_msg.angle = float(target_angle_deg)
            self.motor_msg.speed = float(target_speed)
            self.motor_publisher.publish(self.motor_msg)
        else:
            logger.info(f"VehicleInterface (Sim): Steering(rad): {steering:.2f}, Throttle: {throttle:.2f}, Brake: {brake:.2f}")

class ControlModule:
    def __init__(self, config: dict, input_queue_planning, vehicle_interface_config: dict):
        self.input_queue_planning = input_queue_planning
        self.vehicle_interface = VehicleInterface(
            vehicle_interface_config,
            motor_publisher=vehicle_interface_config.get("ros_motor_publisher"),
            motor_msg_template=vehicle_interface_config.get("ros_motor_msg_template")
        )
        self._running = False
        self._thread = None

    def _action_to_actuator(self, action: ActionCommand) -> ControlActuatorCommands:
        # 최소 변환: 속도/조향값을 actuator 명령으로 변환
        max_speed = 1.4  # m/s, 필요시 config에서 가져올 수 있음
        throttle = np.clip(action.target_velocity_mps / max_speed, 0.0, 1.0) if action.target_velocity_mps > 0.05 else 0.0
        brake = 0.2 if action.target_velocity_mps <= 0.05 else 0.0
        return ControlActuatorCommands(
            timestamp=action.timestamp,
            steering_command=action.target_steering_angle_rad,
            throttle_command=throttle,
            brake_command=brake
        )

    def run(self):
        self._running = True
        while self._running:
            try:
                action_command = self.input_queue_planning.get(block=False)
            except Exception:
                action_command = None
            if action_command is not None:
                control_cmd = self._action_to_actuator(action_command)
                self.vehicle_interface.send_commands(
                    control_cmd.steering_command,
                    control_cmd.throttle_command,
                    control_cmd.brake_command
                )
                self.input_queue_planning.task_done()

    def start(self):
        if not self._running:
            self._thread = threading.Thread(target=self.run, name="ControlThread")
            self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)