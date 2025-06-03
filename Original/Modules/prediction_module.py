import queue
import threading
import time
from typing import Dict, List
from .data_structures import (
    PerceptionOutput, LocalizationInfo, BehavioralPredictionOutput, PredictedTrajectory, DetectedObject
)
import logging

logger = logging.getLogger(__name__)

class PredictionModule:
    def __init__(self, config: dict,
                 input_queue_perception: queue.Queue,
                 input_queue_localization: queue.Queue,
                 output_queue: queue.Queue):
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
        logger.info("PredictionModule (Behavioral): Initialized.")

    def _execute_simple_extrapolation(self, perception_data: PerceptionOutput, 
                                      ego_localization: LocalizationInfo, 
                                      params: dict) -> BehavioralPredictionOutput:
        # Placeholder for behavioral prediction logic (Fig. 2 Prediction block)
        # Uses perceived objects and ego state to predict future trajectories/intentions of other agents
        # logger.debug(f"SimpleExtrapolation: Predicting behavior based on perception at {perception_data.timestamp} and localization at {ego_localization.timestamp}")

        output_timestamp = perception_data.timestamp if perception_data else time.time()
        predicted_trajectories: List[PredictedTrajectory] = []
        
        prediction_horizon_sec = params.get("prediction_horizon_sec", 2.0)
        time_step_sec = params.get("time_step_sec", 0.5)
        default_pred_vel_x = params.get("default_pred_vel_x", 0.5) # m/s
        default_pred_vel_y = params.get("default_pred_vel_y", 0.0) # m/s

        if perception_data and perception_data.detected_objects:
            for obj in perception_data.detected_objects:
                path_points = []
                current_pos = obj.position_3d
                # 객체의 현재 속도가 없으면 기본 예측 속도 사용
                current_vel = obj.velocity if obj.velocity else (default_pred_vel_x, default_pred_vel_y, 0.0)
                
                num_steps = 0
                if time_step_sec > 1e-3: # 0으로 나누기 방지
                    num_steps = int(prediction_horizon_sec / time_step_sec)
                
                if num_steps == 0 and prediction_horizon_sec > 0: # horizon이 있지만 step이 너무 크면 최소 1 step
                    num_steps = 1
                    
                for i in range(1, num_steps + 1):
                    t_offset = i * time_step_sec
                    pred_x = current_pos[0] + current_vel[0] * t_offset
                    pred_y = current_pos[1] + current_vel[1] * t_offset
                    pred_z = current_pos[2] + current_vel[2] * t_offset # Z축 예측도 포함
                    path_points.append((pred_x, pred_y, pred_z))
                
                if path_points: # 경로 포인트가 생성된 경우에만 추가
                    predicted_trajectories.append(
                        PredictedTrajectory(object_id=obj.id, probability=0.7, path_points=path_points)
                    )
        # else:
            # logger.debug("SimpleExtrapolation: No detected objects or perception data.")

        return BehavioralPredictionOutput(timestamp=output_timestamp, predicted_trajectories=predicted_trajectories)

    def run(self):
        logger.info(f"PredictionModule: Thread started. Strategy: {self.active_strategy_name}")
        perception_buffer = {} # Buffer perception data by timestamp
        selected_strategy_method = self.strategy_map.get(self.active_strategy_name)

        while self._running:
            # Update latest localization
            try:
                self._latest_localization = self.input_queue_localization.get(block=False)
                self.input_queue_localization.task_done()
            except queue.Empty:
                pass # No new localization, use the latest one

            # Process perception data
            try:
                perception_data: PerceptionOutput = self.input_queue_perception.get(timeout=0.1) # Timeout to allow checking _running
                perception_buffer[perception_data.timestamp] = perception_data # Store in buffer

                # Try to match with localization (simple timestamp matching or nearest)
                if self._latest_localization:
                    # Find closest perception data to latest localization, or use latest perception data
                    if selected_strategy_method:
                        prediction_result = selected_strategy_method(perception_data, self._latest_localization, self.strategy_params)
                        try:
                            self.output_queue.put(prediction_result, timeout=0.1)
                        except queue.Full:
                            logger.warning("PredictionModule: Output queue full.")
                    else:
                        logger.warning(f"PredictionModule: Strategy '{self.active_strategy_name}' not found.")
                else:
                    # Wait for localization data if not available yet
                    logger.info("PredictionModule: Waiting for initial localization data.")

                self.input_queue_perception.task_done()

            except queue.Empty:
                if not self._running:
                    break
                time.sleep(0.01) # Avoid busy wait
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