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

    # --- Perception Module Parameters ---
    hsv_lane_detection_params = {
        "debug_cv_show": True, # HSV 알고리즘 전용 디버그 뷰 활성화
        "roi_y_start_ratio": 0.2, # HSV ROI용 (기존 0.8에서 수정)
        "lower_white_hsv": [0, 0, 180],
        "upper_white_hsv": [180, 30, 255],
        "lower_yellow_hsv": [20, 100, 100],
        "upper_yellow_hsv": [30, 255, 255],
        "white_pixel_threshold": 300, # 흰색 픽셀 감지 임계값 (주로 HSV 결과에 사용)
        "yellow_area_threshold": 100, # 노란색 영역 감지 임계값 (주로 HSV 결과에 사용)
    }
    canny_hough_lane_detection_params = {
        "debug_cv_show": False, # Canny/Hough 알고리즘 전용 디버그 뷰 비활성화
        "canny_low_threshold": 50,
        "canny_high_threshold": 150,
        "hough_threshold": 20,
        "hough_min_line_length": 10,
        "hough_max_line_gap": 5,
        "roi_y_start_ratio": 0.5, # Canny/Hough용 ROI
    }
    custom_block_example_params = {
        "debug_cv_show": False, # 사용자 정의 알고리즘 디버그 뷰 비활성화
        "custom_param_1": 123,
        "custom_param_2": "test_value"
    }

    # --- Localization Module Parameters ---
    placeholder_localization_params = {
        "update_rate_hz": 10,
        "sim_step_x": 0.05
    }
    gps_imu_fusion_params = { # 새로운 Localization 전략 파라미터
        "gps_weight": 0.7,
        "imu_weight": 0.3,
        "initial_covariance": [0.1, 0.1, 0.1]
    }

    # --- Prediction Module Parameters ---
    simple_extrapolation_params = {
        "prediction_horizon_sec": 2.0,
        "time_step_sec": 0.5
    }
    kalman_cv_prediction_params = { # 새로운 Prediction 전략 파라미터
        "process_noise_covariance": 0.01,
        "measurement_noise_covariance": 0.1,
        "prediction_steps": 5
    }

    # --- Planning Module Parameters ---
    # Path Planner
    simple_waypoint_planner_params = {"num_waypoints": 5, "waypoint_spacing_m": 1.0}
    a_star_planner_params = { # 새로운 PathPlanner 전략 파라미터
        "heuristic_weight": 1.0,
        "grid_resolution_m": 0.5
    }
    # Decision Maker
    default_lane_keep_params = {"target_speed_kph": 10.0} # 기본 주행 속도
    # Action Planner
    hsv_lane_following_params = {
        "initial_straight_frames": 50,
        "initial_speed_xycar_units": 60,
        "white_steering_gain": 0.6,
        "white_max_angle_deg": 30,
        "white_offset_ratio_threshold": 0.05,
        "white_offset_angle_deg": 15,
        "yellow_fallback_steering_gain": 0.005,
        "yellow_fallback_max_angle_deg": 25,
        "no_line_escape_angle_deg": -15,
        "max_steering_delta_deg": 10,
        "speed_tiers_xycar_units": {
            "straight": 45, "gentle_turn": 35, "sharp_turn": 25,
            "no_line_or_fallback": 20
        },
        "xycar_speed_to_mps_factor": 0.028,
    }
    pid_path_tracking_params = {
        "kp_steer": 0.5,
        "ki_steer": 0.01,
        "kd_steer": 0.1,
        "target_lookahead_distance_m": 2.0,
        "log_cte_threshold": 0.01
    }
    rule_based_logic_params = { # 새로운 DecisionMaker 전략 파라미터
        "stop_line_distance_threshold_m": 2.0,
        "traffic_light_response_time_sec": 1.0
    }

    # --- Control Module Parameters ---
    basic_pid_control_params = {
        "max_control_speed_mps": 1.4, # 50 (Xycar units) * 0.028 (factor) = 1.4 m/s
        "log_velocity_threshold_mps": 0.05,
        "log_angle_threshold_rad": 0.005
    }
    vehicle_model_pid_params = { # 새로운 Control 전략 파라미터
        "kp_speed": 0.8, "ki_speed": 0.05, "kd_speed": 0.1,
        "kp_steer_lat": 0.7, "kd_steer_lat": 0.05, # Lateral error based
        "kp_steer_yaw": 0.5, "kd_steer_yaw": 0.02  # Yaw error based
    }

    return {
        "image_width": 640, # 카메라 이미지 너비 (PlanningModule에서 사용)
        "image_height": 480, # 카메라 이미지 높이
        "sensor_input_config": {
            "publish_rate_hz": 20 # 센서 데이터 발행 빈도
        },
        "perception_config": {
            "detection": {
                # track_drive.py에서 이 값을 오버라이드할 수 있습니다.
                # None으로 설정 시 PerceptionModule은 "작업을 수행하기 위한 모듈이 선택되지 않았습니다" 메시지를 출력합니다.
                "active_perception_algorithm": "canny_hough_lane_detection", # 기본 인식 알고리즘
                "hsv_lane_detection_params": hsv_lane_detection_params,
                "canny_hough_lane_detection_params": canny_hough_lane_detection_params,
                "custom_block_example_params": custom_block_example_params,
            },
            "scene_understanding": {}, "tracking": {}, "perception_prediction": {}
            },
        "localization_config": {
            "active_localization_strategy": "gps_imu_fusion", # 기본 측위 전략 변경
            "placeholder_localization_params": placeholder_localization_params,
            "gps_imu_fusion_params": gps_imu_fusion_params,
        },
        "prediction_config": {
            "active_prediction_strategy": "kalman_cv_prediction", # 기본 예측 전략 변경
            "simple_extrapolation_params": simple_extrapolation_params,
            "kalman_cv_prediction_params": kalman_cv_prediction_params,
        },
        "planning_config": {
            "path_planner": {
                "active_strategy": "a_star_planner", # 기본 경로 계획 전략 변경
                "simple_waypoint_planner_params": simple_waypoint_planner_params,
                "a_star_planner_params": a_star_planner_params,
            },
            "decision_maker": {
                "active_strategy": "rule_based_logic", # 기본 의사 결정 전략 변경
                "default_lane_keep_params": default_lane_keep_params,
                "rule_based_logic_params": rule_based_logic_params,
            },
            "action_planner": {
                "active_strategy": "pid_path_tracking", # 기본 행동 계획 전략 변경
                "hsv_lane_following_params": hsv_lane_following_params,
                "pid_path_tracking_params": pid_path_tracking_params
            },
        },
        "control_config": {
            "active_control_law": "vehicle_model_pid", # 기본 제어 법칙 변경
            "basic_pid_params": basic_pid_control_params,
            "vehicle_model_pid_params": vehicle_model_pid_params,
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