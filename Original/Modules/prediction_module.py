import logging
from .thread_queue_manager import ThreadQueueManager
from queue import Empty
import threading
import time
from typing import Dict, List
from .data_structures import (
    PerceptionOutput, LocalizationInfo, BehavioralPredictionOutput, PredictedTrajectory, DetectedObject
)

logger = logging.getLogger(__name__)

class PredictionModule:
    def __init__(self, config: dict,
                 input_queue_perception: ThreadQueueManager,
                 input_queue_localization: ThreadQueueManager,
                 output_queue: ThreadQueueManager):
        self.config = config
        self.input_queue_perception = input_queue_perception
        self.input_queue_localization = input_queue_localization
        self.output_queue = output_queue

        self.active_strategy_name = self.config.get("active_prediction_strategy", "simple_extrapolation")
        self.strategy_params = self.config.get(f"{self.active_strategy_name}_params", {})
        logger.info(f"PredictionModule: Active strategy: {self.active_strategy_name} with params: {self.strategy_params}")

        self.strategy_map = {
            "simple_extrapolation": self._execute_simple_extrapolation,
        }

        self._latest_localization: LocalizationInfo = None
        self._running = False
        self._thread = None
        self.perception_buffer = {}  # 버퍼 선언
        logger.info("PredictionModule (Behavioral): Initialized.")

    def _execute_simple_extrapolation(self, perception_data: PerceptionOutput, 
                                      ego_localization: LocalizationInfo, 
                                      params: dict) -> BehavioralPredictionOutput:
        # Placeholder for behavioral prediction logic (Fig. 2 Prediction block)
        # Uses perceived objects and ego state to predict future trajectories/intentions of other agents
        # logger.debug(f"SimpleExtrapolation: Predicting behavior based on perception at {perception_data.timestamp} and localization at {ego_localization.timestamp}")

        predicted_trajectories: List[PredictedTrajectory] = []
        prediction_horizon_sec = params.get("prediction_horizon_sec", 2.0)
        time_step_sec = params.get("time_step_sec", 0.5)

        for obj in perception_data.detected_objects:
            # Example: Simple extrapolation or more complex model (RNN, etc.)
            if obj.id == 1: # Example: predict for object with ID 1
                path = []
                current_pos = obj.position_3d
                current_vel = obj.velocity if obj.velocity else (0,0,0)
                for t_offset in [0.5, 1.0, 1.5, 2.0]: # Predict up to 2 seconds ahead
                    pred_x = current_pos[0] + current_vel[0] * t_offset
                    pred_y = current_pos[1] + current_vel[1] * t_offset
                    pred_z = current_pos[2] # Assuming 2D movement for simplicity
                    path.append((pred_x, pred_y, pred_z)) # Storing (x,y,z) at time t_offset
                
                # Generate time offsets based on horizon and step
                # num_steps = int(prediction_horizon_sec / time_step_sec)
                # for i in range(1, num_steps + 1):
                #    t_offset = i * time_step_sec ... (more detailed implementation)
                predicted_trajectories.append(
                    PredictedTrajectory(object_id=obj.id, probability=0.8, path_points=path)
                )

        return BehavioralPredictionOutput(
            timestamp=perception_data.timestamp,
            predicted_trajectories=predicted_trajectories
        )

    def run(self):
        logger.info("PredictionModule: Thread started. Strategy: %s", self.active_strategy_name)
        self._running = True
        selected_strategy_method = self.strategy_map.get(self.active_strategy_name)
        try:
            while self._running:
                latest_perception = None
                latest_localization = None
                # perception 큐에서 최신 데이터만 남기고 모두 소비
                while True:
                    try:
                        latest_perception = self.input_queue_perception.get(block=False)
                        self.input_queue_perception.task_done()
                    except Empty:
                        break
                # localization 큐에서 최신 데이터만 남기고 모두 소비
                while True:
                    try:
                        latest_localization = self.input_queue_localization.get(block=False)
                        self.input_queue_localization.task_done()
                    except Empty:
                        break
                # 둘 다 최신 데이터가 있을 때만 예측 수행
                if latest_perception is not None and latest_localization is not None:
                    try:
                        if selected_strategy_method:
                            prediction_result = selected_strategy_method(latest_perception, latest_localization, self.strategy_params)
                            try:
                                self.output_queue.put(prediction_result, timeout=0.1)
                            except Exception:
                                logger.warning("PredictionModule: Output queue full.")
                        else:
                            logger.warning(f"PredictionModule: Strategy '{self.active_strategy_name}' not found.")
                    except Exception as e:
                        logger.error(f"PredictionModule: Error during prediction: {e}", exc_info=True)
                else:
                    # 데이터가 부족하면 짧게 sleep
                    time.sleep(0.001)
        except Exception as e:
            logger.error(f"PredictionModule: Error processing data: {e}", exc_info=True)
        logger.info("PredictionModule: Thread stopped.")

    def start(self):
        if not self._running:
            self._running = True
            self._thread = threading.Thread(target=self.run, name="PredictionThread")
            self._thread.start()
            logger.info("PredictionModule: Started.")

    def stop(self):
        if self._running:
            self._running = False
            if self._thread:
                self._thread.join(timeout=2.0)
            logger.info("PredictionModule: Stopped.")