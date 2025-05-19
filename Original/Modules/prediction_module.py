import queue
import threading
import time
from typing import Dict, List
from .data_structures import (
    PerceptionOutput, LocalizationInfo, BehavioralPredictionOutput, PredictedTrajectory, DetectedObject
)

class PredictionModule:
    def __init__(self, config: dict,
                 input_queue_perception: queue.Queue,
                 input_queue_localization: queue.Queue,
                 output_queue: queue.Queue):
        self.config = config
        self.input_queue_perception = input_queue_perception
        self.input_queue_localization = input_queue_localization
        self.output_queue = output_queue

        self._latest_localization: LocalizationInfo = None
        self._running = False
        self._thread = None
        print("PredictionModule (Behavioral): Initialized.")

    def _predict_behavior(self, perception_data: PerceptionOutput, ego_localization: LocalizationInfo) -> BehavioralPredictionOutput:
        # Placeholder for behavioral prediction logic (Fig. 2 Prediction block)
        # Uses perceived objects and ego state to predict future trajectories/intentions of other agents
        print(f"PredictionModule: Predicting behavior based on perception at {perception_data.timestamp} and localization at {ego_localization.timestamp}")

        predicted_trajectories: List[PredictedTrajectory] = []
        for obj in perception_data.detected_objects:
            # Example: Simple extrapolation or more complex model (RNN, etc.)
            if obj.id == 1: # Let's predict for object with ID 1
                path = []
                current_pos = obj.position_3d
                current_vel = obj.velocity if obj.velocity else (0,0,0)
                for t_offset in [0.5, 1.0, 1.5, 2.0]: # Predict up to 2 seconds ahead
                    pred_x = current_pos[0] + current_vel[0] * t_offset
                    pred_y = current_pos[1] + current_vel[1] * t_offset
                    pred_z = current_pos[2] # Assuming 2D movement for simplicity
                    path.append((pred_x, pred_y, t_offset))
                predicted_trajectories.append(
                    PredictedTrajectory(object_id=obj.id, probability=0.8, path_points=path)
                )

        return BehavioralPredictionOutput(
            timestamp=perception_data.timestamp,
            predicted_trajectories=predicted_trajectories
        )

    def run(self):
        print("PredictionModule: Thread started.")
        perception_buffer = {} # Buffer perception data by timestamp

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
                    # For simplicity, we use the just received perception_data if localization is available
                    prediction_result = self._predict_behavior(perception_data, self._latest_localization)
                    try:
                        self.output_queue.put(prediction_result, timeout=0.1)
                    except queue.Full:
                        print("PredictionModule: Output queue full.")
                else:
                    # Wait for localization data if not available yet
                    print("PredictionModule: Waiting for initial localization data.")

                self.input_queue_perception.task_done()

            except queue.Empty:
                if not self._running:
                    break
                continue # No perception data, loop again

        print("PredictionModule: Thread stopped.")


    def start(self):
        if not self._running:
            self._running = True
            self._thread = threading.Thread(target=self.run, name="PredictionThread")
            self._thread.start()
            print("PredictionModule: Started.")

    def stop(self):
        if self._running:
            self._running = False
            if self._thread:
                self._thread.join(timeout=2.0)
            print("PredictionModule: Stopped.")