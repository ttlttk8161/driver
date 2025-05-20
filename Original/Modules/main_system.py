import threading
import queue
import time
import os # For creating dummy config if needed
import logging

# Import module classes
from .sensor_input_module import SensorInputManager
from .perception_module import PerceptionModule
from .localization_module import LocalizationModule, HDMapInterface # HDMapInterface is defined in localization_module
from .prediction_module import PredictionModule
from .planning_module import PlanningModule
from .control_module import ControlModule
from .data_structures import SensorData # And others if directly used here

# Dummy config function
def load_dummy_config() -> dict:
    # PerceptionModule에서 사용할 기본 알고리즘 설정.
    # track_drive.py에서 이 값을 오버라이드할 수 있습니다.
    # None으로 설정 시 PerceptionModule은 "작업을 수행하기 위한 모듈이 선택되지 않았습니다" 메시지를 출력합니다.
    default_active_perception_algorithm = None # 예: "hsv_lane_detection", "canny_hough_lane_detection", None

    detection_config = {
        "active_perception_algorithm": default_active_perception_algorithm,
        "debug_cv_show": True, # Perception 모듈의 cv2.imshow 사용 여부

        # 'hsv_lane_detection' 알고리즘을 위한 파라미터 블록
        "hsv_lane_detection_params": {
            "roi_y_start_ratio": 0.2, # HSV ROI용 (기존 0.8에서 수정)
            "lower_white_hsv": [0, 0, 180],
            "upper_white_hsv": [180, 30, 255],
            "lower_yellow_hsv": [20, 100, 100],
            "upper_yellow_hsv": [30, 255, 255],
            "white_pixel_threshold": 300, # 흰색 픽셀 감지 임계값 (주로 HSV 결과에 사용)
            "yellow_area_threshold": 100, # 노란색 영역 감지 임계값 (주로 HSV 결과에 사용)
        },

        # 'canny_hough_lane_detection' 알고리즘을 위한 파라미터 블록
        "canny_hough_lane_detection_params": {
            "canny_low_threshold": 50,
            "canny_high_threshold": 150,
            "hough_threshold": 20,
            "hough_min_line_length": 10,
            "hough_max_line_gap": 5,
            "roi_y_start_ratio": 0.5, # Canny/Hough용 ROI
        },

        # 'custom_block_example' 알고리즘을 위한 파라미터 블록 (예시)
        "custom_block_example_params": {
            "custom_param_1": 123,
            "custom_param_2": "test_value"
        }
        # 여기에 다른 알고리즘과 그 파라미터 블록을 추가할 수 있습니다.
    }

    return {
        "image_width": 640, # 카메라 이미지 너비 (PlanningModule에서 사용)
        "image_height": 480, # 카메라 이미지 높이
        "sensor_input_config": {
            "publish_rate_hz": 20 # 센서 데이터 발행 빈도
        },
        "perception_config": {
            "detection": detection_config,
            "scene_understanding": {}, "tracking": {}, "perception_prediction": {}
            },
        "hd_map_path": "path/to/dummy_map.osm", # Example path
        "localization_config": {
            "active_localization_strategy": "placeholder_localization", # or "ekf_slam", "particle_filter"
            "placeholder_localization_params": {
                "update_rate_hz": 10,
                "sim_step_x": 0.05
            },
            # "ekf_slam_params": { ... }
        },
        "prediction_config": {
            "active_prediction_strategy": "simple_extrapolation", # or "kalman_filter_cv", "social_lstm"
            "simple_extrapolation_params": {
                "prediction_horizon_sec": 2.0,
                "time_step_sec": 0.5
            },
            # "kalman_filter_cv_params": { ... }
        },
        "planning_config": {
            "path_planner": {
                "active_strategy": "simple_waypoint_planner", # e.g., "a_star", "rrt_star"
                "simple_waypoint_planner_params": {"num_waypoints": 5, "waypoint_spacing_m": 1.0}
            },
            "decision_maker": {
                "active_strategy": "default_lane_keep", # e.g., "rule_based_traffic_logic"
                "default_lane_keep_params": {"target_speed_kph": 10.0}, # 기본 주행 속도
            },
            "action_planner": {
                # 기존 Canny 기반 차선 인식용 파라미터
                "steering_kp": 0.006, 
                "max_steer_rad": 0.4, 
                "single_lane_steer_rad": 0.1,
                # steering_balancing.py에서 가져온 파라미터
                "initial_straight_frames": 50,
                "initial_speed_xycar_units": 60,
                "steering_gain": 0.6,
                "steering_offset_gain_deg": 15,
                "max_angle_deg": 30,
                "yellow_fallback_gain": 0.005,
                "yellow_max_angle_deg": 25,
                "no_line_escape_angle_deg": -15,
                "max_delta_steering_deg": 10,
                # Xycar의 물리적 최대 속도 유닛이 50이라고 가정하고, 그 범위 내에서 속도 설정
                "speed_tiers_xycar_units": {"straight": 45, "gentle_turn": 35, "sharp_turn": 25, "fallback": 20, "no_line": 20},
                "xycar_speed_to_mps_factor": 0.028, # 예: 50 유닛 = 1.4 m/s (1.4 / 50.0)
                "white_ratio_diff_threshold_for_offset": 0.05,
            },
        },
        "control_config": {
            "active_control_law": "basic_pid", # or "mpc_control"
            "basic_pid_params": {
                "max_control_speed_mps": 1.4, # 50 (Xycar units) * 0.028 (factor) = 1.4 m/s
                "log_velocity_threshold_mps": 0.05,
                "log_angle_threshold_rad": 0.005
            }
        },
        "vehicle_interface_config": { 
            "max_xycar_speed": 50.0, # Xycar의 최대 속도 유닛
            "min_xycar_speed": 0.0
            # ros_motor_publisher 등은 track_drive.py에서 채워짐
        }
    }

class MainSystem:
    def __init__(self, config: dict):
        self.config = config
        self._initialize_queues()
        self._initialize_modules()
        self._threads = []
        logging.info("MainSystem: Initialized.")

    def _initialize_queues(self):
        logging.info("MainSystem: Initializing queues...")
        self.sensor_to_perception_queue = queue.Queue(maxsize=10)

        # Perception outputs to multiple modules
        self.perception_to_localization_queue = queue.Queue(maxsize=5)
        self.perception_to_prediction_queue = queue.Queue(maxsize=5)
        self.perception_to_planning_queue = queue.Queue(maxsize=5) # For direct scene info to planning

        # Localization outputs to multiple modules
        self.localization_to_prediction_queue = queue.Queue(maxsize=5)
        self.localization_to_planning_queue = queue.Queue(maxsize=5)

        self.prediction_to_planning_queue = queue.Queue(maxsize=5)
        self.planning_to_control_queue = queue.Queue(maxsize=5)

        # Optional direct sensor input to localization (e.g., GNSS/IMU if not through perception)
        self.direct_sensor_to_localization_queue = queue.Queue(maxsize=10) # Example

    def _initialize_modules(self):
        logging.info("MainSystem: Initializing modules...")
        # 1. Sensor Input
        # track_drive.py에서 CvBridge 객체를 config 통해 전달받는다고 가정
        ros_bridge_instance = self.config.get("ros_bridge")
        self.sensor_manager = SensorInputManager(
            self.config.get("sensor_input_config", {}),
            self.sensor_to_perception_queue, # Sensor manager directly outputs to perception
            ros_bridge=ros_bridge_instance
            # If direct GNSS/IMU to localization: self.direct_sensor_to_localization_queue (needs sensor manager logic change)
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

        # 3. Localization Module
        localization_output_queues = {
            "prediction": self.localization_to_prediction_queue,
            "planning": self.localization_to_planning_queue
        }
        self.localization_module = LocalizationModule(
            self.config.get("localization_config", {}),
            self.config.get("hd_map_path", "dummy_map.hd"),
            input_queue_perception=self.perception_to_localization_queue,
            input_queue_sensor=None, # Assuming GNSS/IMU goes through perception or is handled internally by perception for features
            output_queues=localization_output_queues
        )

        # 4. Prediction Module (Behavioral)
        self.prediction_module = PredictionModule(
            self.config.get("prediction_config", {}),
            input_queue_perception=self.perception_to_prediction_queue,
            input_queue_localization=self.localization_to_prediction_queue,
            output_queue=self.prediction_to_planning_queue
        )

        # Planning Module
        planning_input_queues = {"localization": self.localization_to_planning_queue, "prediction": self.prediction_to_planning_queue, "perception": self.perception_to_planning_queue}
        self.planning_module = PlanningModule(
            planning_specific_config=self.config.get("planning_config", {}),
            hd_map_path=self.config.get("hd_map_path", "dummy_map.hd"),
            overall_system_config=self.config,  # Pass the main system's entire config
            input_queues=planning_input_queues,
            output_queue_control=self.planning_to_control_queue
         )
 
        # 6. Control Module
        # track_drive.py에서 motor 퍼블리셔와 메시지 템플릿을 config 통해 전달받는다고 가정
        vehicle_if_config = self.config.get("vehicle_interface_config", {}).copy() # 복사해서 사용
        vehicle_if_config["ros_motor_publisher"] = self.config.get("ros_motor_publisher")
        vehicle_if_config["ros_motor_msg_template"] = self.config.get("ros_motor_msg_template")
        self.control_module = ControlModule(
            self.config.get("control_config", {}),
            input_queue_planning=self.planning_to_control_queue,
            vehicle_interface_config=vehicle_if_config
        )

        self.modules = [
            self.sensor_manager, # Sensor manager has start/stop methods, not run in a list of threads here
            self.perception_module,
            self.localization_module,
            self.prediction_module,
            self.planning_module,
            self.control_module
        ]


    def start(self):
        logging.info("MainSystem: Starting all modules...")
        # Start sensor manager separately as it might manage its own thread(s) differently
        self.sensor_manager.start_sensors()
        time.sleep(0.5) # Give sensors a moment

        # Start other modules
        for module in self.modules:
            if hasattr(module, 'start') and module != self.sensor_manager : # sensor_manager already started
                module.start()
                self._threads.append(module._thread) # Assuming module._thread is the worker thread
        logging.info("MainSystem: All modules started.")


    def stop(self):
        logging.info("MainSystem: Stopping all modules...")

        # Stop modules in reverse order of data flow or based on dependencies
        # Control first, then planning etc.
        # Or, signal all to stop and then join
        for module in reversed(self.modules): # sensor_manager will be last
            if hasattr(module, 'stop'):
                logging.info(f"MainSystem: Stopping {module.__class__.__name__}...")
                module.stop()

        # Join threads (if module.stop() doesn't join already)
        # This might be redundant if module.stop() already calls join.
        # For robustness, ensure threads are joined.
        for thread in self._threads:
            if thread and thread.is_alive():
                logging.info(f"MainSystem: Joining thread {thread.name}...")
                thread.join(timeout=5.0) # Add timeout to join
                if thread.is_alive():
                    logging.warning(f"MainSystem: WARNING - Thread {thread.name} did not terminate.")

        logging.info("MainSystem: All modules stopped.")

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    logger = logging.getLogger(__name__)
    logger.info("=============== Autonomous Driving System Simulation ===============")
    # Create a dummy HD map file if it doesn't exist for HDMapInterface to load
    dummy_map_path = "path/to/dummy_map.hd"
    os.makedirs(os.path.dirname(dummy_map_path), exist_ok=True)
    if not os.path.exists(dummy_map_path):
        with open(dummy_map_path, 'w') as f:
            f.write("This is a dummy HD map file.\n")
        logger.info(f"Created dummy HD map file: {dummy_map_path}")


    config = load_dummy_config()
    config["hd_map_path"] = dummy_map_path # Ensure config uses the created path

    system = MainSystem(config=config)

    try:
        system.start()
        # Let the system run for a short duration for demonstration
        logger.info("\nMainSystem: Running for 10 seconds...\n")
        time.sleep(10)

    except KeyboardInterrupt:
        logger.info("\nMainSystem: KeyboardInterrupt received. Shutting down...")
    finally:
        logger.info("\nMainSystem: Initiating shutdown sequence...")
        system.stop()
        logger.info("=============== System Simulation Ended ===============")