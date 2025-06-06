import logging
from .thread_queue_manager import ThreadQueueManager
from queue import Empty
import threading
import time
from typing import Optional, Dict, Tuple, List, Any
from .data_structures import SensorData, PerceptionOutput, LocalizationInfo
from .error_manager import error_manager, ErrorCode

# Get a logger for this module
logger = logging.getLogger(__name__)

class LocalizationModule:
    def __init__(self, config: dict,
                 input_queue_perception: ThreadQueueManager,
                 input_queue_sensor: Optional[ThreadQueueManager], # For direct GNSS/IMU
                 output_queues: Dict[str, ThreadQueueManager]):
        self.config = config
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
        logger.info("LocalizationModule: Thread started. Strategy: %s", self.active_strategy_name)
        self._running = True
        try:
            while self._running:
                try:
                    perception_data = self.input_queue_perception.get(block=False)
                except Empty:
                    perception_data = None
                try:
                    sensor_data = self.input_queue_sensor.get(block=False) if self.input_queue_sensor else None
                except Empty:
                    sensor_data = None

                if perception_data or sensor_data:
                    selected_strategy_method = self.strategy_map.get(self.active_strategy_name)
                    if selected_strategy_method:
                        localization_output = selected_strategy_method(perception_data, sensor_data, self.strategy_params)
                        for key, q in self.output_queues.items():
                            try:
                                q.put(localization_output, timeout=0.1)
                            except queue.Full:
                                 logging.warning(f"LocalizationModule: Output queue '{key}' is full.")
                    else:
                        logger.warning(f"LocalizationModule: Strategy '{self.active_strategy_name}' not found in strategy_map.")
                        # Potentially sleep or handle error
                    if perception_data: self.input_queue_perception.task_done()
                    if sensor_data and self.input_queue_sensor: self.input_queue_sensor.task_done()

                if not perception_data and not sensor_data:
                    time.sleep(0.001) # Avoid busy waiting if no data
                if not self._running and self.input_queue_perception.empty() and (not self.input_queue_sensor or self.input_queue_sensor.empty()):
                    break # Exit condition

            logger.info("LocalizationModule: Thread stopped.")
        except Exception as e:
            error_manager.handle(ErrorCode.MODULE_RUNTIME_EXCEPTION, str(e))

    def start(self):
        if not self._running:
            self._running = True
            try:
                self._thread = threading.Thread(target=self.run, name="LocalizationThread")
                self._thread.start()
                logger.info("LocalizationModule: Started.")
            except Exception as e:
                error_manager.handle(ErrorCode.MODULE_START_FAIL, str(e))
                raise

    def stop(self):
        if self._running:
            self._running = False
            try:
                if self._thread:
                    self._thread.join(timeout=2.0)
                logger.info("LocalizationModule: Stopped.")
            except Exception as e:
                error_manager.handle(ErrorCode.MODULE_RUNTIME_EXCEPTION, str(e))