# 예측 모듈
import queue
import _queue
import threading
import time
from typing import Dict, List, Tuple
from .optimized_data_structures import (
    LocalizationInfo, BehavioralPredictionOutput, PredictedTrajectory
)
from .optimized_data_structures import OptimizedPerceptionOutput, DetectedObject
import numpy as np
import logging

logger = logging.getLogger(__name__)

class KalmanFilterCV:
    """2D 등속 모델용 칼만 필터"""
    def __init__(self, initial_pos: Tuple[float, float],
                 initial_vel: Tuple[float, float] = (0.0, 0.0),
                 process_noise_std_acc: float = 0.5,
                 measurement_noise_std_pos: float = 0.1,
                 initial_covariance_scale: float = 1.0):
        # State: [x, y, vx, vy]
        self.x = np.array([initial_pos[0], initial_pos[1], initial_vel[0], initial_vel[1]]).reshape(4, 1)
        # State Covariance Matrix P
        self.P = np.eye(4) * initial_covariance_scale
        # Measurement Matrix H
        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]])
        # Measurement Noise Covariance R
        self.R = np.eye(2) * (measurement_noise_std_pos ** 2)
        # Process Noise standard deviation for acceleration (used to build Q)
        self.process_noise_std_acc = process_noise_std_acc
        # State Transition Matrix F (dt will be set in predict)
        # self.F = np.eye(4) # F is calculated in predict based on dt
        # Process Noise Covariance Q (dt will be set in predict)
        # self.Q = np.eye(4) # Q is calculated in predict based on dt

    def predict(self, dt: float):
        F = np.array([[1, 0, dt, 0],
                      [0, 1, 0, dt],
                      [0, 0, 1,  0],
                      [0, 0, 0,  1]])
        q_pos_component = (dt**2) / 2.0
        q_vel_component = dt
        Q = np.diag([q_pos_component**2, q_pos_component**2, q_vel_component**2, q_vel_component**2]) * (self.process_noise_std_acc ** 2)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update(self, z_measurement: np.ndarray): # z_measurement is [x_meas, y_meas]
        y_residual = z_measurement.reshape(2,1) - self.H @ self.x
        S_innovation_cov = self.H @ self.P @ self.H.T + self.R
        K_kalman_gain = self.P @ self.H.T @ np.linalg.inv(S_innovation_cov)
        self.x = self.x + K_kalman_gain @ y_residual
        self.P = (np.eye(4) - K_kalman_gain @ self.H) @ self.P

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
            "kalman_cv_prediction": self._execute_kalman_cv_prediction, # 새로운 전략 추가
        }

        self._latest_localization: LocalizationInfo = None
        self._running = False
        self._thread = None
        self.perception_buffer_max_size = self.config.get("perception_buffer_max_size", 20) # Use self.config
        
        # Store KalmanFilterCV instances per object ID
        self.kalman_filters: Dict[int, KalmanFilterCV] = {}
        self.last_kf_update_time: Dict[int, float] = {} # object_id -> last update timestamp


        self.timestamp_match_threshold = self.config.get("timestamp_match_threshold_sec", 0.2) # Use self.config
        logger.info("PredictionModule (Behavioral): Initialized.")

    def _execute_simple_extrapolation(self, perception_data, 
                                      ego_localization: LocalizationInfo, 
                                      params: dict) -> BehavioralPredictionOutput:
        # 입력 데이터 타입 확인 및 호환성 처리
        predicted_trajectories: List[PredictedTrajectory] = []
        prediction_horizon_sec = params.get("prediction_horizon_sec", 2.0)
        time_step_sec = params.get("time_step_sec", 0.5)
        
        # OptimizedPerceptionOutput 처리
        detected_objects = []
        timestamp = 0.0
        
        if hasattr(perception_data, 'detected_objects'):
            detected_objects = perception_data.detected_objects
            timestamp = perception_data.timestamp
        elif hasattr(perception_data, 'timestamp'):
            # OptimizedPerceptionOutput의 경우 detected_objects가 없을 수 있음
            timestamp = perception_data.timestamp
            # 기본 빈 리스트 사용

        for obj in detected_objects:
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
            timestamp=timestamp,
            predicted_trajectories=predicted_trajectories
        )

    def _execute_kalman_cv_prediction(self, perception_data,
                                        ego_localization: LocalizationInfo,
                                        params: dict) -> BehavioralPredictionOutput:
        # 입력 데이터 타입 확인 및 호환성 처리
        predicted_trajectories: List[PredictedTrajectory] = []
        
        # OptimizedPerceptionOutput 처리
        detected_objects = []
        current_time = 0.0
        
        if hasattr(perception_data, 'detected_objects'):
            detected_objects = perception_data.detected_objects
            current_time = perception_data.timestamp
        elif hasattr(perception_data, 'timestamp'):
            # OptimizedPerceptionOutput의 경우 detected_objects가 없을 수 있음
            current_time = perception_data.timestamp
            # 기본 빈 리스트 사용
            
        process_noise_std_acc = params.get("process_noise_std_dev_acc", 0.5)
        measurement_noise_std_pos = params.get("measurement_noise_std_pos", 0.1)
        prediction_steps = params.get("prediction_steps", 5)
        initial_vel_fallback = params.get("initial_velocity_if_none", 0.1)

        for obj in detected_objects:
            obj_id = obj.id
            measured_pos = np.array(obj.position_3d[:2]) # Use (x, y)

            dt = current_time - self.last_kf_update_time.get(obj_id, current_time) # dt for predict step
            if dt <= 1e-6 : # Avoid zero or too small dt, especially for the first update
                dt = 1.0 / self.config.get("perception_update_rate_hz", 10) # Assume a default rate

            if obj_id not in self.kalman_filters:
                initial_vel = (0.0, 0.0)
                if obj.velocity and len(obj.velocity) >= 2:
                    initial_vel = obj.velocity[:2]
                else: # Estimate initial velocity if possible or use fallback
                    # For simplicity, if no velocity, assume a small initial velocity or zero
                    # A better approach would be to use the first two measurements to estimate velocity.
                    # Here, we use a fallback or zero.
                    # If object is moving towards ego, vx might be negative.
                    # This part needs more sophisticated handling for robust initialization.
                    # For now, let's assume a small forward velocity if type is 'car' etc.
                    if obj.type in ["car", "vehicle"]: # Example
                         initial_vel = (initial_vel_fallback, 0.0) # Small forward velocity

                self.kalman_filters[obj_id] = KalmanFilterCV(
                    initial_pos=measured_pos,
                    initial_vel=initial_vel,
                    process_noise_std_acc=process_noise_std_acc,
                    measurement_noise_std_pos=measurement_noise_std_pos
                )
                logger.debug(f"KalmanCVPrediction: Initialized KF for obj {obj_id} at pos {measured_pos}, vel {initial_vel}")
            
            kf = self.kalman_filters[obj_id]
            kf.predict(dt=dt)
            kf.update(measured_pos)
            self.last_kf_update_time[obj_id] = current_time

            # Generate future trajectory
            path_points = []
            temp_kf_state = np.copy(kf.x) # Use a copy of the state for multi-step prediction
            prediction_dt = params.get("prediction_time_step_sec", 0.2) # Get prediction dt from params

            for _ in range(prediction_steps):
                # Predict one step ahead using the fixed prediction_dt
                # For multi-step prediction, we only apply the predict step of KF
                F_pred = np.array([[1,0,prediction_dt,0],[0,1,0,prediction_dt],[0,0,1,0],[0,0,0,1]])
                temp_kf_state = F_pred @ temp_kf_state
                # Q_pred can be calculated similarly to kf.predict if needed for uncertainty propagation
                path_points.append((temp_kf_state[0,0], temp_kf_state[1,0], obj.position_3d[2])) # Keep original Z
            
            predicted_trajectories.append(
                PredictedTrajectory(object_id=obj_id, probability=0.8, path_points=path_points) # Placeholder probability
            )
            logger.debug(f"KalmanCVPrediction: Obj {obj_id} predicted path: {path_points}")

        return BehavioralPredictionOutput(timestamp=current_time, predicted_trajectories=predicted_trajectories)

    def run(self):
        print("[PREDICTION] run() 진입", flush=True)
        logger.info(f"PredictionModule: Thread started. Active strategy: {self.active_strategy_name}")
        while self._running:
            print("[PREDICTION] run() 루프 진입, 큐 get 시도", flush=True)
            try:
                perception_data = self.input_queue_perception.get(timeout=1.0)
                print(f"PredictionModule: Got perception data from queue (timestamp={getattr(perception_data, 'timestamp', 'unknown')})", flush=True)
                logger.info(f"PredictionModule: Got perception data from queue (timestamp={getattr(perception_data, 'timestamp', 'unknown')})")
                try:
                    localization_data = self.input_queue_localization.get_nowait()
                except Exception:
                    localization_data = None
                if self.active_strategy_name in self.strategy_map:
                    pred = self.strategy_map[self.active_strategy_name](perception_data, localization_data, self.strategy_params)
                else:
                    logger.warning(f"PredictionModule: Unknown strategy '{self.active_strategy_name}', using simple_extrapolation.")
                    pred = self._execute_simple_extrapolation(perception_data, localization_data, self.strategy_params)
                self.output_queue.put(pred, timeout=0.1)
                logger.info(f"PredictionModule: Put prediction output to output queue (timestamp={pred.timestamp})")
                print(f"PredictionModule: Put prediction output to output queue (timestamp={pred.timestamp})", flush=True)
                self.input_queue_perception.task_done()
            except (queue.Empty, _queue.Empty):
                if not self._running:
                    break
                logger.info("PredictionModule: Waiting for perception data in queue...")
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