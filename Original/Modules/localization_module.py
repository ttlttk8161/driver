# 위치인식 모듈
import queue
import _queue
import threading
import time
from typing import Optional, Dict, Tuple, List, Any
from .optimized_data_structures import SensorData, LocalizationInfo
from .optimized_data_structures import OptimizedSensorInput, OptimizedPerceptionOutput
import rospy
import logging

logger = logging.getLogger(__name__)


class LocalizationModule:
    def __init__(self, config: dict,
                 input_queue_perception: queue.Queue,
                 input_queue_sensor: Optional[queue.Queue], # For direct GNSS/IMU
                 output_queues: Dict[str, queue.Queue]):
        self.config = config
        self.input_queue_perception = input_queue_perception
        self.input_queue_sensor = input_queue_sensor # Can be None if sensors go via perception
        self.output_queues = output_queues # e.g., {"prediction": q_pred, "planning": q_plan}
        self.active_strategy_name = self.config.get("active_localization_strategy", "placeholder_localization")
        self.strategy_params = self.config.get(f"{self.active_strategy_name}_params", {})
        logger.info(f"LocalizationModule: Active strategy: {self.active_strategy_name} with params: {self.strategy_params}")

        self.strategy_map = {
            "placeholder_localization": self._execute_placeholder_localization,
            "gps_imu_fusion": self._execute_gps_imu_fusion, # 새로운 전략 추가
        }

        self._current_localization = LocalizationInfo(
            timestamp=0.0, position=(0,0,0), orientation_quaternion=(1,0,0,0),
            velocity_vector=(0,0,0), covariance_matrix=None
        )
        self._running = False
        self._thread = None
        logger.info("LocalizationModule: Initialized.")

    def _execute_placeholder_localization(self, perception_data, 
                                          sensor_data_direct: Optional[SensorData], 
                                          params: dict) -> LocalizationInfo:
        # 입력 데이터 타입 확인 및 호환성 처리
        current_event_timestamp = 0.0
        
        # OptimizedPerceptionOutput 처리
        if perception_data:
            if hasattr(perception_data, 'timestamp'):
                current_event_timestamp = perception_data.timestamp
                
        if sensor_data_direct and sensor_data_direct.timestamp > 0:
            # If both are present, prefer the one that makes more sense or is newer
            current_event_timestamp = max(current_event_timestamp, sensor_data_direct.timestamp)
        
        if current_event_timestamp == 0.0: # Fallback if no valid input timestamp
            current_event_timestamp = rospy.Time.now().to_sec() if rospy.Time.now().to_sec() > 0 else time.time()

        sim_step_x = params.get("sim_step_x", 0.01)

        # logger.debug(f"PlaceholderLocalization: Using data at {current_event_timestamp}")

        # Simulate position update
        new_pos = (self._current_localization.position[0] + sim_step_x, 
                   self._current_localization.position[1],
                   self._current_localization.position[2])
        
        time_delta = current_event_timestamp - self._current_localization.timestamp # 'timestamp' -> 'current_event_timestamp'
        velocity_x = sim_step_x / time_delta if time_delta > 1e-6 else 0 # Avoid division by zero or tiny dt

        self._current_localization = LocalizationInfo(
            timestamp=current_event_timestamp,
            position=new_pos,
            orientation_quaternion=self._current_localization.orientation_quaternion, # Keep orientation same
            velocity_vector=(velocity_x, 0, 0),
            covariance_matrix="Sample Covariance"
        )
        # logger.debug(f"PlaceholderLocalization: New pose: {self._current_localization.position}")
        return self._current_localization

    def _execute_gps_imu_fusion(self, perception_data,
                                sensor_data_direct: Optional[SensorData],
                                params: dict) -> LocalizationInfo:
        # 입력 데이터 타입 확인 및 호환성 처리
        current_event_timestamp = 0.0
        
        if sensor_data_direct and sensor_data_direct.timestamp > 0:
            current_event_timestamp = sensor_data_direct.timestamp
        elif perception_data and hasattr(perception_data, 'timestamp'):
            current_event_timestamp = perception_data.timestamp
            
        if current_event_timestamp == 0.0: # Fallback if no valid input timestamp
            current_event_timestamp = rospy.Time.now().to_sec() if rospy.Time.now().to_sec() > 0 else time.time()
        
        time_delta = current_event_timestamp - self._current_localization.timestamp
        # Handle first run or cases where timestamps might be problematic
        if self._current_localization.timestamp == 0.0 or time_delta <= 1e-6: # 첫 실행이거나 time_delta가 매우 작거나 음수일 경우
            time_delta = 1.0 / params.get("update_rate_hz_fallback", 10)

        new_pos_x, new_pos_y, new_pos_z = self._current_localization.position
        new_orient_w, new_orient_x, new_orient_y, new_orient_z = self._current_localization.orientation_quaternion
        vel_x, vel_y, vel_z = (0.0, 0.0, 0.0) # Initialize velocities

        gnss_available = False
        if sensor_data_direct and sensor_data_direct.gnss_data:
            gnss_pos_data = sensor_data_direct.gnss_data.get("position")
            if gnss_pos_data and len(gnss_pos_data) == 3:
                gnss_available = True
                
                gps_weight = params.get("gps_weight", 0.7)
                prev_weight = 1.0 - gps_weight
                prev_x, prev_y, prev_z = self._current_localization.position

                # 단순 가중 평균으로 위치 업데이트
                new_pos_x = gps_weight * gnss_pos_data[0] + prev_weight * prev_x
                new_pos_y = gps_weight * gnss_pos_data[1] + prev_weight * prev_y
                new_pos_z = gps_weight * gnss_pos_data[2] + prev_weight * prev_z
                
                # 속도 계산
                vel_x = (new_pos_x - prev_x) / time_delta
                vel_y = (new_pos_y - prev_y) / time_delta
                vel_z = (new_pos_z - prev_z) / time_delta
                
                logger.debug(f"GPSIMUFusion: Used GNSS. NewPos:({new_pos_x:.2f},{new_pos_y:.2f}), Vel:({vel_x:.2f},{vel_y:.2f}) @ {current_event_timestamp:.2f}")

        if not gnss_available:
            # GNSS 데이터가 없으면 시뮬레이션된 전진 운동
            sim_step_x = params.get("sim_step_x_fallback", 0.05)
            prev_x = new_pos_x # 이전 x 위치는 현재 new_pos_x (업데이트 전 값)
            new_pos_x += sim_step_x # x축으로만 이동 가정
            
            vel_x = sim_step_x / time_delta
            vel_y = 0.0 # y, z 방향 속도는 0으로 가정
            vel_z = 0.0
            logger.debug(f"GPSIMUFusion: No GNSS. Simulated step. NewPosX:{new_pos_x:.2f}, VelX:{vel_x:.2f} @ {current_event_timestamp:.2f}")

        # IMU 데이터로 방향 업데이트 (플레이스홀더)
        if sensor_data_direct and sensor_data_direct.imu_data:
            imu_orient_data = sensor_data_direct.imu_data.get("orientation_quaternion")
            if imu_orient_data and len(imu_orient_data) == 4:
                new_orient_w, new_orient_x, new_orient_y, new_orient_z = imu_orient_data # IMU 방향 직접 사용 (융합 필요)
                logger.debug(f"GPSIMUFusion: Used IMU orientation.")
            
        self._current_localization = LocalizationInfo(current_event_timestamp, (new_pos_x, new_pos_y, new_pos_z), 
                                                      (new_orient_w, new_orient_x, new_orient_y, new_orient_z), 
                                                      (vel_x, vel_y, vel_z), "GPS/IMU Fusion Cov (Updated)")
        return self._current_localization

    def run(self):
        print("[LOCALIZATION] run() 진입", flush=True)
        logger.info(f"LocalizationModule: Thread started. Active strategy: {self.active_strategy_name}")
        while self._running:
            print("[LOCALIZATION] run() 루프 진입, 큐 get 시도", flush=True)
            try:
                perception_data = self.input_queue_perception.get(timeout=1.0)
                print(f"LocalizationModule: Got perception data from queue (timestamp={getattr(perception_data, 'timestamp', 'unknown')})", flush=True)
                logger.info(f"LocalizationModule: Got perception data from queue (timestamp={getattr(perception_data, 'timestamp', 'unknown')})")
                # 센서 직접 입력은 None일 수 있음
                sensor_data_direct = None
                if self.input_queue_sensor:
                    try:
                        sensor_data_direct = self.input_queue_sensor.get_nowait()
                    except Exception:
                        pass
                if self.active_strategy_name in self.strategy_map:
                    loc = self.strategy_map[self.active_strategy_name](perception_data, sensor_data_direct, self.strategy_params)
                else:
                    logger.warning(f"LocalizationModule: Unknown strategy '{self.active_strategy_name}', using placeholder.")
                    loc = self._execute_placeholder_localization(perception_data, sensor_data_direct, self.strategy_params)
                for key, q in self.output_queues.items():
                    try:
                        q.put(loc, timeout=0.1)
                        print(f"LocalizationModule: Put localization output to '{key}' queue (timestamp={getattr(loc, 'timestamp', 'unknown')})", flush=True)
                        logger.info(f"LocalizationModule: Put localization output to '{key}' queue (timestamp={getattr(loc, 'timestamp', 'unknown')})")
                    except queue.Full:
                        logger.warning(f"LocalizationModule: Output queue '{key}' is full. Discarding data.")
                self.input_queue_perception.task_done()
            except (queue.Empty, _queue.Empty):
                if not self._running:
                    break
                logger.info("LocalizationModule: Waiting for perception data in queue...")
            except Exception as e:
                logger.error(f"LocalizationModule: Error processing data: {e}", exc_info=True)
        logger.info("LocalizationModule: Thread stopped.")

    def start(self):
        if not self._running:
            self._running = True
            self._thread = threading.Thread(target=self.run, name="LocalizationThread")
            self._thread.start()
            logger.info("LocalizationModule: Started.")

    def stop(self):
        if self._running:
            self._running = False
            if self._thread:
                self._thread.join(timeout=2.0)
            logger.info("LocalizationModule: Stopped.")