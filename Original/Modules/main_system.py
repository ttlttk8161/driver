# 모듈 import
import threading
import queue
import time
import os
import logging

# 시스템 모듈 import
from .optimized_sensor_input_module import OptimizedSensorInputManager
from .perception_module import PerceptionModule
from .localization_module import LocalizationModule
from .prediction_module import PredictionModule
from .planning_module import PlanningModule
from .control_module import ControlModule
from .optimized_data_structures import (
    OptimizedSensorInput, PriorityQueue, RingBufferQueue, 
    TimestampSynchronizer, DataPriority
)
from .performance_monitor import PerformanceMonitor
from .adaptive_optimization import AdaptiveOptimizer

# 기본 설정 함수
def load_dummy_config() -> dict:
    # Perception 모듈 파라미터
    hsv_lane_detection_params = {
        "debug_cv_show": True,
        "roi_y_start_ratio": 0.2,
        "lower_white_hsv": [0, 0, 180],
        "upper_white_hsv": [180, 30, 255],
        "lower_yellow_hsv": [20, 100, 100],
        "upper_yellow_hsv": [30, 255, 255],
        "white_pixel_threshold": 300,
        "yellow_area_threshold": 100,
    }
    canny_hough_lane_detection_params = {
        "debug_cv_show": False,
        "canny_low_threshold": 50,
        "canny_high_threshold": 150,
        "hough_threshold": 20,
        "hough_min_line_length": 10,
        "hough_max_line_gap": 5,
        "roi_y_start_ratio": 0.5,
    }
    custom_block_example_params = {
        "debug_cv_show": False,
        "custom_param_1": 123,
        "custom_param_2": "test_value"
    }

    # Localization 모듈 파라미터
    placeholder_localization_params = {
        "update_rate_hz": 10,
        "sim_step_x": 0.05
    }
    gps_imu_fusion_params = {
        "gps_weight": 0.7,
        "imu_weight": 0.3,
        "initial_covariance": [0.1, 0.1, 0.1],
        "sim_step_x_fallback": 0.05,
        "update_rate_hz_fallback": 10
    }

    # Prediction 모듈 파라미터
    simple_extrapolation_params = {
        "prediction_horizon_sec": 2.0,
        "time_step_sec": 0.5
    }
    kalman_cv_prediction_params = {
        "process_noise_std_dev_acc": 0.5,
        "measurement_noise_std_dev_pos": 0.1,
        "prediction_steps": 5,
        "prediction_time_step_sec": 0.2,
        "initial_velocity_if_none": 0.1
    }

    # Planning 모듈 파라미터
    simple_waypoint_planner_params = {"num_waypoints": 5, "waypoint_spacing_m": 1.0}
    a_star_planner_params = {
        "heuristic_weight": 1.0,
        "grid_resolution_m": 0.5
    }
    default_lane_keep_params = {"target_speed_kph": 10.0}
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
    rule_based_logic_params = {
        "stop_line_distance_threshold_m": 2.0,
        "traffic_light_response_time_sec": 1.0
    }

    # Control 모듈 파라미터
    basic_pid_control_params = {
        "max_control_speed_mps": 1.4,
        "log_velocity_threshold_mps": 0.05,
        "log_angle_threshold_rad": 0.005
    }
    vehicle_model_pid_params = {
        "kp_speed": 0.8, "ki_speed": 0.05, "kd_speed": 0.1,
        "kp_steer_lat": 0.7, "kd_steer_lat": 0.05,
        "kp_steer_yaw": 0.5, "kd_steer_yaw": 0.02,
        "max_throttle_speed_mps": 1.5
    }

    return {
        "image_width": 640,
        "image_height": 480,
        "sensor_input_config": {
            "publish_rate_hz": 20
        },
        "perception_config": {
            "detection": {
                "active_perception_algorithm": "hsv_lane_detection",
                "hsv_lane_detection_params": hsv_lane_detection_params,
                "canny_hough_lane_detection_params": canny_hough_lane_detection_params,
                "custom_block_example_params": custom_block_example_params,
            },
            "scene_understanding": {}, "tracking": {}, "perception_prediction": {}
            },
        "localization_config": {
            "active_localization_strategy": "gps_imu_fusion",
            "placeholder_localization_params": placeholder_localization_params,
            "gps_imu_fusion_params": gps_imu_fusion_params,
        },
        "prediction_config": {
            "active_prediction_strategy": "kalman_cv_prediction",
            "simple_extrapolation_params": simple_extrapolation_params,
            "kalman_cv_prediction_params": kalman_cv_prediction_params,
        },
        "planning_config": {
            "path_planner": {
                "active_strategy": "a_star_planner",
                "simple_waypoint_planner_params": simple_waypoint_planner_params,
                "a_star_planner_params": a_star_planner_params,
            },
            "decision_maker": {
                "active_strategy": "rule_based_logic",
                "default_lane_keep_params": default_lane_keep_params,
                "rule_based_logic_params": rule_based_logic_params,
            },
            "action_planner": {
                "active_strategy": "hsv_lane_following",
                "hsv_lane_following_params": hsv_lane_following_params,
                "pid_path_tracking_params": pid_path_tracking_params
            },
        },
        "control_config": {
            "active_control_law": "basic_pid",
            "basic_pid_params": basic_pid_control_params,
            "vehicle_model_pid_params": vehicle_model_pid_params,
        },
        "vehicle_interface_config": { 
            "max_xycar_speed": 50.0,
            "min_xycar_speed": 0.0
        }
    }

class MainSystem:
    def __init__(self, config: dict):
        self.config = config
        self._initialize_performance_monitor()
        self._initialize_queues()
        self._initialize_modules()
        self._threads = []
        logging.info("MainSystem: Initialized with performance monitoring.")

    def _initialize_performance_monitor(self):
        """성능 모니터 초기화"""
        monitor_config = self.config.get("performance_monitor_config", {
            "enable_monitoring": True,
            "monitoring_interval_sec": 1.0,
            "alert_thresholds": {
                "cpu_percent_high": 75.0,
                "memory_percent_high": 80.0,
                "queue_size_high": 8,
                "fps_low": 15.0,
                "latency_high_ms": 80.0
            }
        })
        
        self.performance_monitor = PerformanceMonitor(monitor_config)
        logging.info("MainSystem: Performance monitor initialized")
        
        # 적응적 최적화 시스템 초기화
        optimizer_config = self.config.get("adaptive_optimizer_config", {
            "optimization_interval_sec": 5.0,
            "cpu_high_threshold": 85.0,
            "memory_high_threshold": 80.0,
            "queue_full_threshold": 0.8,
            "latency_high_threshold_ms": 50.0
        })
        self.adaptive_optimizer = AdaptiveOptimizer(self.performance_monitor, optimizer_config)
        logging.info("MainSystem: Adaptive optimizer initialized")

    def _initialize_queues(self):
        logging.info("MainSystem: Initializing optimized queues...")
        
        # 센서 데이터용 고속 링 버퍼 큐
        self.sensor_to_perception_queue = RingBufferQueue(maxsize=8)

        # 인지 결과용 우선순위 큐
        self.perception_to_localization_queue = PriorityQueue(maxsize=5, enable_metrics=True)
        self.perception_to_prediction_queue = PriorityQueue(maxsize=5, enable_metrics=True)
        self.perception_to_planning_queue = PriorityQueue(maxsize=5, enable_metrics=True)

        # 위치 정보용 우선순위 큐
        self.localization_to_prediction_queue = PriorityQueue(maxsize=5, enable_metrics=True)
        self.localization_to_planning_queue = PriorityQueue(maxsize=5, enable_metrics=True)

        # 예측 및 제어 명령용 큐
        self.prediction_to_planning_queue = PriorityQueue(maxsize=3, enable_metrics=True)
        self.planning_to_control_queue = PriorityQueue(maxsize=2, enable_metrics=True)

        # 직접 센서 입력용
        self.direct_sensor_to_localization_queue = RingBufferQueue(maxsize=10)
        
        # 타임스탬프 동기화 관리자
        self.timestamp_synchronizer = TimestampSynchronizer(tolerance_ms=50.0)
        
        logging.info("MainSystem: Optimized queues initialized successfully")

    def _initialize_modules(self):
        logging.info("MainSystem: Initializing modules...")
        
        # 1. 최적화된 센서 입력 관리자
        ros_bridge_instance = self.config.get("ros_bridge")
        if ros_bridge_instance is None:
            from cv_bridge import CvBridge
            ros_bridge_instance = CvBridge()
        sensor_config = self.config.get("sensor_input_config", {})
        
        # 성능 모니터링 설정 추가
        sensor_config.update({
            "enable_performance_monitoring": True,
            "memory_pool_size": 15,
            "sync_tolerance_ms": 30.0,
            "publish_rate_hz": 25
        })
        
        self.sensor_manager = OptimizedSensorInputManager(
            sensor_config,
            self.sensor_to_perception_queue,
            ros_bridge=ros_bridge_instance
        )

        # 2. 인지 모듈
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
            input_queue_perception=self.perception_to_localization_queue,
            input_queue_sensor=None,
            output_queues=localization_output_queues
        )

        # 4. Prediction Module
        self.prediction_module = PredictionModule(
            self.config.get("prediction_config", {}),
            input_queue_perception=self.perception_to_prediction_queue,
            input_queue_localization=self.localization_to_prediction_queue,
            output_queue=self.prediction_to_planning_queue
        )

        # 5. Planning Module
        planning_input_queues = {"localization": self.localization_to_planning_queue, "prediction": self.prediction_to_planning_queue, "perception": self.perception_to_planning_queue}
        self.planning_module = PlanningModule(
            planning_specific_config=self.config.get("planning_config", {}),
            overall_system_config=self.config,
            input_queues=planning_input_queues,
            output_queue_control=self.planning_to_control_queue
         )
 
        # 6. Control Module
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
            self.localization_module,
            self.prediction_module,
            self.planning_module,
            self.control_module
        ]
        
        # 성능 모니터에 모듈들과 큐들을 등록
        self._register_with_performance_monitor()
        
        # 적응적 최적화 타이머 설정
        self._optimization_interval = 5.0
        self._last_optimization_time = time.time()
        self._optimization_enabled = True


    def start(self):
        print("MainSystem: Starting all modules...", flush=True)
        logging.info("MainSystem: Starting all modules...")
        
        # 성능 모니터링 시작
        print("MainSystem: Starting performance monitoring...", flush=True)
        self.performance_monitor.start_monitoring()
        print("MainSystem: Performance monitoring started", flush=True)
        logging.info("MainSystem: Performance monitoring started")
        
        # 적응적 최적화 시작
        print("MainSystem: Starting adaptive optimization...", flush=True)
        self.adaptive_optimizer.start()
        print("MainSystem: Adaptive optimization started", flush=True)
        logging.info("MainSystem: Adaptive optimization started")
        
        # 센서 관리자 시작
        print("MainSystem: Starting sensor manager...", flush=True)
        self.sensor_manager.start_sensors()
        print("MainSystem: Sensor manager started, waiting 0.5 seconds...", flush=True)
        time.sleep(0.5)

        # 다른 모듈들 시작
        print("MainSystem: Starting individual modules...", flush=True)
        for module in self.modules:
            if hasattr(module, 'start') and module != self.sensor_manager:
                module_name = module.__class__.__name__
                print(f"MainSystem: About to call {module_name}.start()", flush=True)
                try:
                    module.start()
                    print(f"MainSystem: {module_name}.start() returned. _thread: {getattr(module, '_thread', None)}", flush=True)
                except Exception as e:
                    print(f"MainSystem: Exception during {module_name}.start(): {e}", flush=True)
                if hasattr(module, '_thread'):
                    self._threads.append(module._thread)
                print(f"MainSystem: {module_name} started successfully", flush=True)
        print("MainSystem: All modules started.", flush=True)
        logging.info("MainSystem: All modules started.")

    def _register_with_performance_monitor(self):
        """성능 모니터에 모듈들과 큐들을 등록"""
        logging.info("MainSystem: Registering modules and queues with performance monitor...")
        
        # 큐들 등록
        queue_info = {
            "sensor_to_perception": self.sensor_to_perception_queue,
            "perception_to_localization": self.perception_to_localization_queue,
            "perception_to_prediction": self.perception_to_prediction_queue,
            "perception_to_planning": self.perception_to_planning_queue,
            "localization_to_prediction": self.localization_to_prediction_queue,
            "localization_to_planning": self.localization_to_planning_queue,
            "prediction_to_planning": self.prediction_to_planning_queue,
            "planning_to_control": self.planning_to_control_queue,
            "direct_sensor_to_localization": self.direct_sensor_to_localization_queue
        }
        
        for queue_name, queue_obj in queue_info.items():
            self.performance_monitor.register_queue(queue_name, queue_obj)
            self.adaptive_optimizer.register_queue(queue_name, queue_obj)
        
        # 모듈들 등록
        module_info = {
            "sensor_manager": self.sensor_manager,
            "perception_module": self.perception_module,
            "localization_module": self.localization_module,
            "prediction_module": self.prediction_module,
            "planning_module": self.planning_module,
            "control_module": self.control_module
        }
        
        for module_name, module_obj in module_info.items():
            self.performance_monitor.register_module(module_name, module_obj)
            self.adaptive_optimizer.register_module(module_name, module_obj)
        
        logging.info("MainSystem: All components registered with performance monitor and adaptive optimizer")
        
    def get_performance_report(self):
        """현재 성능 보고서를 반환"""
        return self.performance_monitor.generate_report()


    def stop(self):
        logging.info("MainSystem: Stopping all modules...")

        # 성능 모니터링 및 적응적 최적화 정지
        if hasattr(self, 'adaptive_optimizer'):
            self.adaptive_optimizer.stop()
            optimization_report = self.adaptive_optimizer.get_optimization_report()
            logging.info(f"MainSystem: Adaptive Optimizer Report:\n{optimization_report}")
            
        if hasattr(self, 'performance_monitor'):
            self.performance_monitor.stop_monitoring()
            final_report = self.performance_monitor.generate_report()
            logging.info(f"MainSystem: Final Performance Report:\n{final_report}")

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

    def _adaptive_optimization(self):
        """적응적 성능 최적화 실행"""
        if not self._optimization_enabled:
            return
            
        current_time = time.time()
        if current_time - self._last_optimization_time < self._optimization_interval:
            return
            
        logging.info("MainSystem: Running adaptive optimization...")
        
        try:
            # 성능 메트릭 수집
            metrics = self.performance_monitor.get_current_metrics()
            
            # 큐 사용률 기반 크기 조정
            for queue_name, queue_obj in [
                ("sensor_to_perception", self.sensor_to_perception_queue),
                ("perception_to_localization", self.perception_to_localization_queue),
                ("perception_to_prediction", self.perception_to_prediction_queue),
                ("perception_to_planning", self.perception_to_planning_queue),
                ("planning_to_control", self.planning_to_control_queue)
            ]:
                if hasattr(queue_obj, 'utilization'):
                    utilization = queue_obj.utilization
                    
                    # 높은 사용률(>80%)이면 큐 크기 확장
                    if utilization > 0.8 and hasattr(queue_obj, 'resize'):
                        new_size = min(queue_obj.maxsize * 2, 50)  # 최대 50으로 제한
                        logging.info(f"MainSystem: Expanding {queue_name} queue size to {new_size}")
                        queue_obj.resize(new_size)
                    
                    # 낮은 사용률(<20%)이면 큐 크기 축소
                    elif utilization < 0.2 and hasattr(queue_obj, 'resize'):
                        new_size = max(queue_obj.maxsize // 2, 2)  # 최소 2로 제한
                        logging.info(f"MainSystem: Reducing {queue_name} queue size to {new_size}")
                        queue_obj.resize(new_size)
            
            # 메모리 사용량 기반 최적화
            if 'memory_usage_mb' in metrics and metrics['memory_usage_mb'] > 500:
                logging.warning(f"MainSystem: High memory usage detected: {metrics['memory_usage_mb']:.1f}MB")
                # 메모리 정리 트리거
                if hasattr(self.sensor_manager, 'cleanup_memory'):
                    self.sensor_manager.cleanup_memory()
            
            # CPU 사용률 기반 처리 속도 조정
            if 'cpu_usage_percent' in metrics and metrics['cpu_usage_percent'] > 85:
                logging.warning(f"MainSystem: High CPU usage detected: {metrics['cpu_usage_percent']:.1f}%")
                # 센서 데이터 발행 빈도 감소
                if hasattr(self.sensor_manager, 'reduce_publish_rate'):
                    self.sensor_manager.reduce_publish_rate()
            
            self._last_optimization_time = current_time
            logging.info("MainSystem: Adaptive optimization completed")
            
        except Exception as e:
            logging.error(f"MainSystem: Error during adaptive optimization: {e}")
    
    def enable_adaptive_optimization(self, enabled: bool):
        """적응적 최적화 활성화/비활성화"""
        self._optimization_enabled = enabled
        logging.info(f"MainSystem: Adaptive optimization {'enabled' if enabled else 'disabled'}")
    
    def set_optimization_interval(self, interval_seconds: float):
        """최적화 실행 간격 설정"""
        self._optimization_interval = max(1.0, interval_seconds)
        logging.info(f"MainSystem: Optimization interval set to {self._optimization_interval}s")

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    logger = logging.getLogger(__name__)
    logger.info("=============== Autonomous Driving System Simulation ===============")

    config = load_dummy_config()

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