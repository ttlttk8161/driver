import queue
import threading
import time
from typing import Optional, Dict, Tuple, List, Any
from .data_structures import SensorData, PerceptionOutput, LocalizationInfo # HDMapInterface는 이 파일에 정의됨
import logging

# Get a logger for this module
logger = logging.getLogger(__name__)

# Dummy HDMapInterface for now
class HDMapInterface:
    def __init__(self, map_path: str):
        self.map_path = map_path
        logger.info(f"HDMapInterface: Initialized with map {map_path}")

    def get_local_map_data(self, position: tuple, extent: float) -> Any:
        # logger.debug(f"HDMapInterface: Queried local map at {position} with extent {extent}") # Debug level for frequent calls
        return {"lanes": "sample_lane_data", "intersections": "sample_intersection_data"}


class LocalizationModule:
    def __init__(self, config: dict, hd_map_path: str,
                 input_queue_perception: queue.Queue,
                 input_queue_sensor: Optional[queue.Queue], # For direct GNSS/IMU
                 output_queues: Dict[str, queue.Queue]):
        self.module_config = config if config is not None else {} # 전체 localization_config 저장
        self.hd_map = HDMapInterface(hd_map_path)
        self.input_queue_perception = input_queue_perception
        self.input_queue_sensor = input_queue_sensor 
        self.output_queues = output_queues

        # main_system.py의 load_dummy_config()에 정의된 구조를 따름
        # self.module_config = {"active_strategy": "...", "strategies": {"placeholder_localization_params": {...}, ...}}
        self.active_strategy_name = self.module_config.get("active_strategy", "placeholder_localization")
        
        # 활성화된 전략의 파라미터를 가져옵니다.
        # 파라미터는 "strategies" 딕셔너리 내에 각 전략 이름 + "_params" 키로 저장되어 있습니다.
        all_strategies_params = self.module_config.get("strategies", {})
        self.current_strategy_params = all_strategies_params.get(f"{self.active_strategy_name}_params", {})
        
        logger.info(f"LocalizationModule: Initialized. Active strategy: {self.active_strategy_name} with params: {self.current_strategy_params}")

        self.strategy_map = {
            "placeholder_localization": self._execute_placeholder_localization,
            # "ekf_slam": self._execute_ekf_slam, # 예시: 새로운 전략 추가 시
        }

        self._current_localization = LocalizationInfo(
            timestamp=0.0, position=(0,0,0), orientation_quaternion=(1,0,0,0),
            velocity_vector=(0,0,0), covariance_matrix=None
        )
        self._running = False
        self._thread = None
        logger.info("LocalizationModule: Initialized.")

    def _execute_placeholder_localization(self, perception_data: Optional[PerceptionOutput], 
                                          sensor_data_direct: Optional[SensorData], 
                                          params: dict) -> LocalizationInfo:
        # Placeholder for actual localization logic (Fig 3: GNSS/IMU, SLAM variants, Fusion)
        # This method would use data from perception (features, object lists) and/or direct sensor data (GNSS, IMU)
        # along with the HD Map.
        timestamp = time.time()
        sim_step_x = params.get("sim_step_x", 0.01)

        if perception_data:
            timestamp = perception_data.timestamp
            # logger.debug(f"PlaceholderLocalization: Using perception data at {timestamp}")
            # Use perception_data.raw_features_for_localization, detected_objects, etc.
        if sensor_data_direct:
            timestamp = sensor_data_direct.timestamp
            # logger.debug(f"PlaceholderLocalization: Using direct sensor data (GNSS/IMU) at {timestamp}")
            # Use sensor_data_direct.gnss_data, sensor_data_direct.imu_data

        # Simulate position update
        new_pos = (self._current_localization.position[0] + sim_step_x, 
                   self._current_localization.position[1],
                   self._current_localization.position[2])
        
        time_delta = timestamp - self._current_localization.timestamp
        velocity_x = sim_step_x / time_delta if time_delta > 0 else 0

        self._current_localization = LocalizationInfo(
            timestamp=timestamp,
            position=new_pos,
            orientation_quaternion=self._current_localization.orientation_quaternion, # Keep orientation same
            velocity_vector=(velocity_x, 0, 0),
            covariance_matrix="Sample Covariance"
        )
        # logger.debug(f"PlaceholderLocalization: New pose: {self._current_localization.position}")
        return self._current_localization

    def run(self):
        logger.info(f"LocalizationModule: Thread started. Strategy: {self.active_strategy_name}")
        selected_strategy_method = self.strategy_map.get(self.active_strategy_name)

        if not selected_strategy_method and self.active_strategy_name is not None:
            logger.error(f"LocalizationModule: Active strategy '{self.active_strategy_name}' has no corresponding method in strategy_map!")
            # 선택적: 여기서 스레드를 안전하게 종료하거나, 기본 동작을 수행
            self._running = False # 예시: 스레드 종료

        while self._running:
            perception_data = None
            sensor_data_direct = None
            processed_input = False # 입력 처리 여부 플래그

            # 입력 큐에서 데이터 가져오기 (논블로킹)
            try:
                perception_data = self.input_queue_perception.get(block=False)
                processed_input = True
            except queue.Empty:
                pass 

            if self.input_queue_sensor:
                try:
                    sensor_data_direct = self.input_queue_sensor.get(block=False)
                    processed_input = True
                except queue.Empty:
                    pass

            if processed_input:
                if self.active_strategy_name is None:
                    logger.info("LocalizationModule: No active strategy selected. Skipping localization.")
                    # 아무것도 안하거나, 기본 LocalizationInfo를 발행할 수 있음
                    # 예: self._publish_default_localization_info()
                elif selected_strategy_method:
                    localization_output = selected_strategy_method(
                        perception_data, 
                        sensor_data_direct, 
                        self.current_strategy_params # 현재 활성화된 전략의 파라미터 전달
                    )
                    for key, q_out in self.output_queues.items(): # 변수명 변경 q -> q_out
                        try:
                            q_out.put(localization_output, timeout=0.1)
                        except queue.Full:
                             logging.warning(f"LocalizationModule: Output queue '{key}' is full. Discarding data.")
                else:
                    # 이 경우는 시작 시점에 이미 로그가 남았어야 하지만, 안전을 위해 추가
                    logger.warning(f"LocalizationModule: Strategy '{self.active_strategy_name}' method not found, though it was selected. Skipping.")
                
                # 작업 완료 알림
                if perception_data: self.input_queue_perception.task_done()
                if sensor_data_direct and self.input_queue_sensor: self.input_queue_sensor.task_done()
            else:
                # 입력이 없으면 잠시 대기하여 CPU 사용 방지
                time.sleep(0.01) 
            
            # 루프 종료 조건 (스레드 중지 요청 시)
            if not self._running:
                break

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