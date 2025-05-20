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
        self.config = config
        self.hd_map = HDMapInterface(hd_map_path)
        self.input_queue_perception = input_queue_perception
        self.input_queue_sensor = input_queue_sensor # Can be None if sensors go via perception
        self.output_queues = output_queues # e.g., {"prediction": q_pred, "planning": q_plan}

        self.active_strategy_name = self.config.get("active_localization_strategy", "placeholder_localization")
        self.strategy_params = self.config.get(f"{self.active_strategy_name}_params", {})
        logger.info(f"LocalizationModule: Active strategy: {self.active_strategy_name} with params: {self.strategy_params}")

        self.strategy_map = {
            "placeholder_localization": self._execute_placeholder_localization,
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

        while self._running:
            perception_data = None
            sensor_data_direct = None
            processed_something = False

            # Prioritize perception data if available
            try:
                perception_data = self.input_queue_perception.get(block=False)
                processed_something = True
            except queue.Empty:
                pass # No perception data this cycle

            if self.input_queue_sensor:
                try:
                    sensor_data_direct = self.input_queue_sensor.get(block=False)
                    processed_something = True
                except queue.Empty:
                    pass # No direct sensor data this cycle

            if processed_something:
                if selected_strategy_method:
                    localization_output = selected_strategy_method(perception_data, sensor_data_direct, self.strategy_params)
                    for key, q in self.output_queues.items():
                        try:
                            q.put(localization_output, timeout=0.1)
                        except queue.Full:
                             logging.warning(f"LocalizationModule: Output queue '{key}' is full.")
                else:
                    logger.warning(f"LocalizationModule: Strategy '{self.active_strategy_name}' not found in strategy_map.")
                    # Potentially sleep or handle error
                if perception_data: self.input_queue_perception.task_done()
                if sensor_data_direct and self.input_queue_sensor: self.input_queue_sensor.task_done()

            if not processed_something:
                time.sleep(0.01) # Avoid busy waiting if no data
            if not self._running and self.input_queue_perception.empty() and (not self.input_queue_sensor or self.input_queue_sensor.empty()):
                break # Exit condition

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