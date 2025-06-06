import threading
import queue
import time
import os # For creating dummy config if needed
import logging
from .thread_queue_manager import ThreadQueueManager

# Import module classes
from .sensor_input_module import SensorInputManager
from .perception_module import PerceptionModule
# from .localization_module import LocalizationModule
# from .prediction_module import PredictionModule
from .planning_module import PlanningModule
from .control_module import ControlModule
from .data_structures import SensorData # And others if directly used here
from .error_manager import error_manager, ErrorCode

# Dummy config function
def load_dummy_config() -> dict:

    return {
        "image_width": 640, # 카메라 이미지 너비 (PlanningModule에서 사용)
        "image_height": 480, # 카메라 이미지 높이
        "sensor_input_config": {
            "publish_rate_hz": 50 # 센서 데이터 발행 빈도
        },
        "perception_config": {
            "active_perception_algorithm": "hsv_lane_detection", # PerceptionModule에서 사용할 알고리즘
            },
        # "localization_config": {
        #     "active_localization_strategy": "placeholder_localization" # or "ekf_slam", "particle_filter"
        # },
        # "prediction_config": {
        #     "active_prediction_strategy": "simple_extrapolation" # or "kalman_filter_cv", "social_lstm"
        # },
        "planning_config": {
            "path_planner": {
                "active_strategy": "_spline_based_path_planning"
            },
            "decision_maker": {
                "active_strategy": "avoidance_lane_keep_decision"
            },
            "action_planner": {
                "active_strategy": "_calculate_waypoint_based_steering", # or "mppi", "dqn"
            },
        },
        "control_config": {
            "active_control_law": "basic_pid"
        },
        "vehicle_interface_config": { 
            "max_xycar_speed": 50.0, # Xycar의 최대 속도 유닛
            "min_xycar_speed": 0.0
        }
    }

class MainSystem:
    def __init__(self, config: dict):
        self.config = config
        try:
            self._initialize_queues()
            self._initialize_modules()
            self._threads = []
            logging.info("MainSystem: Initialized.")
        except Exception as e:
            error_manager.handle(ErrorCode.MAIN_SYSTEM_INIT_FAIL, str(e))
            raise

    def _initialize_queues(self):
        logging.info("MainSystem: Initializing queues...")
        self.sensor_to_perception_queue = ThreadQueueManager(maxsize=10, name="sensor_to_perception_queue")

        # Perception outputs to multiple modules
        self.perception_to_localization_queue = ThreadQueueManager(maxsize=5, name="perception_to_localization_queue")
        self.perception_to_prediction_queue = ThreadQueueManager(maxsize=5, name="perception_to_prediction_queue")
        self.perception_to_planning_queue = ThreadQueueManager(maxsize=5, name="perception_to_planning_queue") # For direct scene info to planning

        # # Localization outputs to multiple modules
        # self.localization_to_prediction_queue = ThreadQueueManager(maxsize=5, name="localization_to_prediction_queue")
        # self.localization_to_planning_queue = ThreadQueueManager(maxsize=5, name="localization_to_planning_queue")

        # self.prediction_to_planning_queue = ThreadQueueManager(maxsize=5, name="prediction_to_planning_queue")
        # self.planning_to_control_queue = ThreadQueueManager(maxsize=5, name="planning_to_control_queue")

        # Optional direct sensor input to localization (e.g., GNSS/IMU if not through perception)
        self.direct_sensor_to_localization_queue = ThreadQueueManager(maxsize=10, name="direct_sensor_to_localization_queue")

    def _initialize_modules(self):
        try:
            logging.info("MainSystem: Initializing modules...")
            # 1. Sensor Input
            ros_bridge_instance = self.config.get("ros_bridge")
            self.sensor_manager = SensorInputManager(
                self.config.get("sensor_input_config", {}),
                self.sensor_to_perception_queue,
                ros_bridge=ros_bridge_instance
            )

            # 2. Perception Module
            perception_output_queues = {
                "localization": self.perception_to_localization_queue,
                "prediction": self.perception_to_prediction_queue,
                "planning": self.perception_to_planning_queue
            }
            self.perception_module = PerceptionModule(
                self.config.get("perception_config", {}),
                self.sensor_to_perception_queue,
                perception_output_queues
            )

            # # 3. Localization Module
            # localization_output_queues = {
            #     "prediction": self.localization_to_prediction_queue,
            #     "planning": self.localization_to_planning_queue
            # }
            # self.localization_module = LocalizationModule(
            #     self.config.get("localization_config", {}),
            #     input_queue_perception=self.perception_to_localization_queue,
            #     input_queue_sensor=None,
            #     output_queues=localization_output_queues
            # )

            # # 4. Prediction Module (Behavioral)
            # self.prediction_module = PredictionModule(
            #     self.config.get("prediction_config", {}),
            #     input_queue_perception=self.perception_to_prediction_queue,
            #     input_queue_localization=self.localization_to_prediction_queue,
            #     output_queue=self.prediction_to_planning_queue
            # )

            # Planning Module
            planning_input_queues = {"localization": self.localization_to_planning_queue, "prediction": self.prediction_to_planning_queue, "perception": self.perception_to_planning_queue}
            self.planning_module = PlanningModule(
                planning_specific_config=self.config.get("planning_config", {}),
                overall_system_config=self.config,
                input_queues=planning_input_queues,
                output_queue_control=self.planning_to_control_queue
            )
 
            # 6. Control Module
            # track_drive.py에서 motor 퍼블리셔와 메시지 템플릿을 config 통해 전달받는다고 가정
            vehicle_if_config = self.config.get("vehicle_interface_config", {}).copy()
            vehicle_if_config["ros_motor_publisher"] = self.config.get("ros_motor_publisher")
            vehicle_if_config["ros_motor_msg_template"] = self.config.get("ros_motor_msg_template")
            self.control_module = ControlModule(
                self.config.get("control_config", {}),
                input_queue_planning=self.planning_to_control_queue,
                vehicle_interface_config=vehicle_if_config
            )

            self.modules = [
                self.sensor_manager,
                self.perception_module,
                # self.localization_module,
                # self.prediction_module,
                self.planning_module,
                self.control_module
            ]
        except Exception as e:
            error_manager.handle(ErrorCode.MODULE_START_FAIL, str(e))
            raise

    def start(self):
        logging.info("MainSystem: Starting all modules...")
        try:
            self.sensor_manager.start_sensors()
            time.sleep(0.5)
            for module in self.modules:
                if hasattr(module, 'start') and module != self.sensor_manager:
                    module.start()
                    self._threads.append(module._thread)
            logging.info("MainSystem: All modules started.")
        except Exception as e:
            error_manager.handle(ErrorCode.MODULE_START_FAIL, str(e))
            raise

    def stop(self):
        logging.info("MainSystem: Stopping all modules...")
        try:
            for module in reversed(self.modules):
                if hasattr(module, 'stop'):
                    logging.info(f"MainSystem: Stopping {module.__class__.__name__}...")
                    module.stop()
            for thread in self._threads:
                if thread and thread.is_alive():
                    logging.info(f"MainSystem: Joining thread {thread.name}...")
                    thread.join(timeout=5.0)
                    if thread.is_alive():
                        logging.warning(f"MainSystem: WARNING - Thread {thread.name} did not terminate.")
            logging.info("MainSystem: All modules stopped.")
        except Exception as e:
            error_manager.handle(ErrorCode.MODULE_RUNTIME_EXCEPTION, str(e))
            raise

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    logger = logging.getLogger(__name__)
    logger.info("=============== Autonomous Driving System Simulation ===============")

    config = load_dummy_config()
    system = MainSystem(config=config)

    try:
        system.start()

    except KeyboardInterrupt:
        logger.info("\nMainSystem: KeyboardInterrupt received. Shutting down...")
    finally:
        logger.info("\nMainSystem: Initiating shutdown sequence...")
        system.stop()
        logger.info("=============== System Simulation Ended ===============")