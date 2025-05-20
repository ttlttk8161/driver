import queue
import threading
import time
from typing import Dict, Optional, Tuple
from .data_structures import (
    LocalizationInfo, BehavioralPredictionOutput, PerceptionOutput,
    PlannedPath, ManeuverDecision, ActionCommand
)
from .localization_module import HDMapInterface # HDMapInterface from localization_module
import numpy as np
import math # math 모듈 추가
import logging

logger = logging.getLogger(__name__)

class PathPlannerComponent:
    def __init__(self, config: dict, hd_map_interface: HDMapInterface):
        self.config = config
        self.hd_map = hd_map_interface
        self.active_strategy_name = self.config.get("active_strategy", "simple_waypoint_planner")
        self.strategy_params = self.config.get(f"{self.active_strategy_name}_params", {})
        self.strategy_map = {
            "simple_waypoint_planner": self._execute_simple_waypoint_planner,
        }
        logger.info(f"PathPlannerComponent: Initialized. Strategy: {self.active_strategy_name} with params: {self.strategy_params}")

    def _execute_simple_waypoint_planner(self, current_pose: LocalizationInfo, scene_info: PerceptionOutput,
                                         behavioral_predictions: BehavioralPredictionOutput, params: dict, 
                                         image_width: int = 640) -> PlannedPath:
        # logger.debug(f"SimpleWaypointPlanner: Planning path from {current_pose.position}")
        waypoints = []
        num_waypoints = params.get("num_waypoints", 5)
        waypoint_spacing_m = params.get("waypoint_spacing_m", 1.0)
        
        # 차선 정보를 기반으로 간단한 목표 지점 설정 (차선 중앙)
        left_lane, right_lane = None, None
        for lm in scene_info.lane_markings:
            # This logic might be too simple if lane_markings are not in vehicle frame or not easily usable
            # For a true waypoint planner, you'd use HD Map data relative to current_pose
            # or project perceived lanes into a vehicle-local frame.
            if lm.type == "left_lane":
                left_lane = lm
            elif lm.type == "right_lane":
                right_lane = lm
        
        # 차선 중앙을 따라가는 간단한 경로 생성 (예: 5개의 웨이포인트)
        # 이 부분은 더 정교한 경로 계획 로직으로 대체될 수 있습니다.
        # 여기서는 ActionPlanner가 직접 차선 정보를 사용하도록 하고, PathPlanner는 단순화합니다.
        if left_lane and right_lane:
            # 예시: 이미지 y축 중간 지점에서의 차선 중앙 x좌표를 목표로 설정
            # 실제로는 차량 좌표계로 변환 후 경로 계획이 필요
            # 여기서는 ActionPlanner가 차선 정보를 직접 사용하므로, 경로는 단순 직진으로 가정
            pass # ActionPlanner가 차선 정보를 직접 사용

        # 기본적으로 직진 경로 (ActionPlanner가 차선 기반으로 조향)
        start_x, start_y, _ = current_pose.position # 글로벌 좌표계 (더미)
        for i in range(1, num_waypoints + 1): 
            waypoints.append((start_x + i * waypoint_spacing_m, start_y)) # 직진 웨이포인트
        return PlannedPath(timestamp=current_pose.timestamp, waypoints=waypoints)

    def plan_path(self, current_pose: LocalizationInfo, scene_info: PerceptionOutput,
                  behavioral_predictions: BehavioralPredictionOutput, image_width: int = 640) -> PlannedPath:
        selected_method = self.strategy_map.get(self.active_strategy_name)
        if selected_method:
            return selected_method(current_pose, scene_info, behavioral_predictions, self.strategy_params, image_width)
        else:
            logger.warning(f"PathPlannerComponent: Strategy '{self.active_strategy_name}' not found. Returning empty path.")
            return PlannedPath(timestamp=time.time(), waypoints=[])

class DecisionMakerComponent:
    def __init__(self, config: dict):
        self.config = config
        self.active_strategy_name = self.config.get("active_strategy", "default_lane_keep")
        self.strategy_params = self.config.get(f"{self.active_strategy_name}_params", {})
        self.strategy_map = {
            "default_lane_keep": self._execute_default_lane_keep,
        }
        logger.info(f"DecisionMakerComponent: Initialized. Strategy: {self.active_strategy_name} with params: {self.strategy_params}")

    def _execute_default_lane_keep(self, current_pose: LocalizationInfo, planned_path: PlannedPath,
                                   behavioral_predictions: BehavioralPredictionOutput, 
                                   scene_info: PerceptionOutput, params: dict) -> ManeuverDecision:
        # Placeholder for Decision-making (Fig 2)
        # Uses path, predictions, localization, (and potentially direct perception like traffic lights)
        # logger.debug(f"DefaultLaneKeep: Making decision based on path with {len(planned_path.waypoints)} waypoints.")
        # Example: Default to lane keep
        target_speed_kph = params.get("target_speed_kph", 10.0) # 설정에서 기본 속도 가져오기
        return ManeuverDecision(
            timestamp=current_pose.timestamp,
            chosen_maneuver="LANE_KEEP",
            target_speed_kph=target_speed_kph, # 고정 속도 또는 설정값 사용
            lead_vehicle_id=None
        )

    def make_decision(self, current_pose: LocalizationInfo, planned_path: PlannedPath,
                      behavioral_predictions: BehavioralPredictionOutput, scene_info: PerceptionOutput) -> ManeuverDecision:
        selected_method = self.strategy_map.get(self.active_strategy_name)
        if selected_method:
            return selected_method(current_pose, planned_path, behavioral_predictions, scene_info, self.strategy_params)
        else:
            logger.warning(f"DecisionMakerComponent: Strategy '{self.active_strategy_name}' not found. Returning default decision.")
            return ManeuverDecision(timestamp=time.time(), chosen_maneuver="EMERGENCY_STOP", target_speed_kph=0.0, lead_vehicle_id=None)

class ActionPlannerComponent:
    def __init__(self, config: dict):
        self.config = config
        # steering_balancing.py의 상태 변수들
        self.prev_steering_angle_rad = 0.0 # 이전 조향각 (라디안)
        self.white_lost_count = 0 # 흰색 선 연속 손실 횟수
        self.frame_counter = 0 # 초기 직진 주행을 위한 프레임 카운터

        # steering_balancing.py의 파라미터들 (설정 파일에서 가져옴)
        # 이 값들은 main_system.py의 load_dummy_config() 내 planning_config -> action_planner 에 정의되어야 함
        self.initial_straight_frames = self.config.get("initial_straight_frames", 50)
        self.initial_speed_xycar_units = self.config.get("initial_speed_xycar_units", 60)
        
        self.white_steering_gain = self.config.get("white_steering_gain", 0.6) # steering_balancing.py: error * 0.6
        self.white_max_angle_deg = self.config.get("white_max_angle_deg", 30)   # steering_balancing.py: np.clip(..., -30, 30)
        self.white_offset_ratio_threshold = self.config.get("white_offset_ratio_threshold", 0.05) # steering_balancing.py: left_ratio - right_ratio > 0.05
        self.white_offset_angle_deg = self.config.get("white_offset_angle_deg", 15) # steering_balancing.py: angle += 15

        self.yellow_fallback_steering_gain = self.config.get("yellow_fallback_steering_gain", 0.005) # steering_balancing.py: error * 0.005
        self.yellow_fallback_max_angle_deg = self.config.get("yellow_fallback_max_angle_deg", 25) # steering_balancing.py: np.clip(..., -25, 25)
        
        self.no_line_escape_angle_deg = self.config.get("no_line_escape_angle_deg", -15) # steering_balancing.py: angle = -15
        
        self.max_steering_delta_deg = self.config.get("max_steering_delta_deg", 10) # steering_balancing.py: max_delta = 10

        # 속도 설정 (Xycar 속도 단위) - steering_balancing.py 기준
        self.speed_config_xycar_units = self.config.get("speed_tiers_xycar_units", {
            "straight": 80,       # abs_angle < 5
            "gentle_turn": 60,    # abs_angle < 10
            "sharp_turn": 45,     # else
            "no_line_or_fallback": 30 # "HOLD" or "NO LINE" in steering_balancing.py
        })
        # Xycar 속도 단위를 m/s로 변환하는 계수
        self.xycar_speed_to_mps_factor = self.config.get("xycar_speed_to_mps_factor", 0.028) # 예: 50유닛 = 1.4m/s => 1.4/50 = 0.028

        logger.info(f"ActionPlannerComponent: Initialized with params: {self.config}")

    def _calculate_hsv_based_steering_and_speed(self, perception_info: PerceptionOutput, image_roi_width: int) -> Tuple[float, float, str]:
        """
        steering_balancing.py 로직에 따라 HSV 차선 정보를 사용하여 조향각(도)과 속도(Xycar 단위)를 계산합니다.
        image_roi_width: Perception 모듈에서 HSV 처리에 사용된 ROI의 너비입니다.
        """
        angle_deg = 0.0
        current_log = "START"

        white_metrics = perception_info.white_line_hsv_metrics
        yellow_metrics = perception_info.yellow_line_hsv_metrics

        # steering_balancing.py: if total_white > 300 (white_pixel_threshold는 Perception에서 처리)
        if white_metrics and white_metrics.is_detected:
            self.white_lost_count = 0
            # steering_balancing.py: error = (left_ratio - right_ratio) * 100
            error = (white_metrics.left_ratio - white_metrics.right_ratio) * 100 
            angle_deg = np.clip(error * self.white_steering_gain, -self.white_max_angle_deg, self.white_max_angle_deg)

            # steering_balancing.py: if left_ratio - right_ratio > 0.05: angle += 15
            if (white_metrics.left_ratio - white_metrics.right_ratio) > self.white_offset_ratio_threshold:
                angle_deg += self.white_offset_angle_deg
            elif (white_metrics.right_ratio - white_metrics.left_ratio) > self.white_offset_ratio_threshold:
                angle_deg -= self.white_offset_angle_deg
            
            angle_deg = np.clip(angle_deg, -self.white_max_angle_deg, self.white_max_angle_deg) # 오프셋 적용 후 다시 클리핑
            current_log = "WHITE_TRACK"
        else: # 흰색 선 미감지 또는 부족
            self.white_lost_count += 1
            # steering_balancing.py: if white_lost_count >= 1 (즉시 폴백)
            # steering_balancing.py: if M['m00'] > 0 (yellow_area_threshold는 Perception에서 처리)
            if yellow_metrics and yellow_metrics.is_detected and yellow_metrics.center_x is not None:
                # yellow_metrics.center_x는 Perception에서 사용된 ROI 내부의 x좌표.
                # image_roi_width는 해당 ROI의 너비.
                roi_center_x = image_roi_width / 2.0
                error = yellow_metrics.center_x - roi_center_x
                angle_deg = np.clip(error * self.yellow_fallback_steering_gain, -self.yellow_fallback_max_angle_deg, self.yellow_fallback_max_angle_deg)
                current_log = "YELLOW_FALLBACK"
            else: # 노란색 선도 미감지
                angle_deg = self.no_line_escape_angle_deg
                current_log = "NO_LINE_ESCAPE"
        
        # 조향각 변화 제한 (스무딩) - steering_balancing.py: max_delta = 10
        prev_angle_deg = math.degrees(self.prev_steering_angle_rad)
        delta_angle = angle_deg - prev_angle_deg
        if abs(delta_angle) > self.max_steering_delta_deg:
            angle_deg = prev_angle_deg + np.sign(delta_angle) * self.max_steering_delta_deg
        
        # 속도 결정 (Xycar 단위) - steering_balancing.py 기준
        abs_angle_deg = abs(angle_deg)
        if "NO_LINE" in current_log or "FALLBACK" in current_log: # steering_balancing.py: "HOLD" or "NO LINE"
            speed_xycar = self.speed_config_xycar_units["no_line_or_fallback"]
        elif abs_angle_deg < 5:
            speed_xycar = self.speed_config_xycar_units["straight"]
        elif abs_angle_deg < 10:
            speed_xycar = self.speed_config_xycar_units["gentle_turn"]
        else:
            speed_xycar = self.speed_config_xycar_units["sharp_turn"]
            
        return angle_deg, speed_xycar, current_log

    def plan_action(self, current_pose: LocalizationInfo, decision: ManeuverDecision, 
                      planned_path: PlannedPath, perception_info: PerceptionOutput, 
                      image_width: int = 640) -> ActionCommand:
        
        self.frame_counter += 1
        target_steering_deg = 0.0
        target_speed_xycar_units = self.initial_speed_xycar_units
        log_info = "[INIT_DEFAULT]"

        # PerceptionModule의 _detect_hsv_lines에서 ROI는 image[roi_y_start:, :] 이므로,
        # 해당 ROI의 너비는 전체 이미지 너비(image_width)와 동일합니다.
        # 따라서 _calculate_hsv_based_steering_and_speed에 image_width를 image_roi_width로 전달합니다.
        image_roi_width_for_hsv = image_width 

        if self.frame_counter <= self.initial_straight_frames:
            target_steering_deg = 0.0 # 초기 직진
            target_speed_xycar_units = self.initial_speed_xycar_units
            log_info = f"[INIT_STRAIGHT] Frame {self.frame_counter}"
        elif perception_info.white_line_hsv_metrics is not None or perception_info.yellow_line_hsv_metrics is not None:
            # HSV 메트릭이 있는 경우에만 HSV 기반 로직 사용
            target_steering_deg, target_speed_xycar_units, log_info = \
                self._calculate_hsv_based_steering_and_speed(perception_info, image_roi_width_for_hsv)
        else:
            # HSV 메트릭이 없는 경우 (예: Perception 모듈에서 아직 준비되지 않음) -> 이전 Canny 로직 또는 기본값 사용
            # 여기서는 steering_balancing.py 통합에 집중하므로, 기본값(직진 또는 이전 값 유지)으로 설정
            target_steering_deg = math.degrees(self.prev_steering_angle_rad) # 이전 각도 유지 시도
            target_speed_xycar_units = self.speed_config_xycar_units["no_line_or_fallback"] # 안전 속도
            log_info = "[NO_HSV_METRICS_FALLBACK]"

        # 최종 조향각(도)을 라디안으로 변환
        target_steering_rad = math.radians(target_steering_deg)
        self.prev_steering_angle_rad = target_steering_rad # 다음 프레임을 위해 현재 조향각(라디안) 저장

        # Xycar 속도 단위를 m/s로 변환
        target_velocity_mps = target_speed_xycar_units * self.xycar_speed_to_mps_factor

        # 로그 추가 (필요시) 이 로그를 통해 `[INIT_STRAIGHT]`, `WHITE_TRACK`, `YELLOW_FALLBACK`, `NO_LINE_ESCAPE`, `[NO_HSV_METRICS_FALLBACK]` 중 
        # 어떤 상태인지 파악할 수 있습니다. 만약 계속 `[NO_HSV_METRICS_FALLBACK]`가 출력된다면, HSV 차선 정보가 `ActionPlannerComponent`에 제대로 전달되지 
        # 않거나, `perception_info.white_line_hsv_metrics`와 `perception_info.yellow_line_hsv_metrics`가 `None`으로 전달되고 있다는 의미입니다.
        logger.debug(f"ActionPlanner: Mode: {log_info}, Angle(deg): {target_steering_deg:.2f}, Speed(xycar): {target_speed_xycar_units}, Vel(mps): {target_velocity_mps:.2f}")

        return ActionCommand(
            timestamp=current_pose.timestamp,
            target_velocity_mps=target_velocity_mps,
            target_steering_angle_rad=target_steering_rad
        )

class PlanningModule:
    def __init__(self, planning_specific_config: dict, # Renamed from 'config'
                 hd_map_path: str,
                 overall_system_config: dict, # Renamed from 'overall_config'
                 input_queues: Dict[str, queue.Queue], # {"localization": q_loc, "prediction": q_pred, "perception": q_perc}
                 output_queue_control: queue.Queue):
        self.planning_config = planning_specific_config # Store planning specific settings
        self.hd_map = HDMapInterface(hd_map_path) # Re-use or pass instance

        # Use planning_specific_config for sub-components
        self.path_planner = PathPlannerComponent(self.planning_config.get("path_planner", {}), self.hd_map)
        self.decision_maker = DecisionMakerComponent(self.planning_config.get("decision_maker", {}))
        self.action_planner = ActionPlannerComponent(self.planning_config.get("action_planner", {}))
        
        # Use overall_system_config for global settings like image_width
        self.image_width = overall_system_config.get("image_width", 640)

        self.input_queue_localization = input_queues["localization"]
        self.input_queue_prediction = input_queues["prediction"]
        self.input_queue_perception = input_queues["perception"] # For static scene info, traffic lights etc.
        self.output_queue_control = output_queue_control

        self._latest_localization: Optional[LocalizationInfo] = None
        self._latest_prediction: Optional[BehavioralPredictionOutput] = None
        self._latest_perception: Optional[PerceptionOutput] = None

        self._running = False
        self._thread = None
        logger.info("PlanningModule: Initialized.")

    def run(self):
        logger.info("PlanningModule: Thread started.")
        while self._running:
            # Fetch latest data from all input queues (non-blocking)
            try:
                self._latest_localization = self.input_queue_localization.get(block=False)
                self.input_queue_localization.task_done()
            except queue.Empty: pass

            try:
                self._latest_prediction = self.input_queue_prediction.get(block=False)
                self.input_queue_prediction.task_done()
            except queue.Empty: pass

            try:
                self._latest_perception = self.input_queue_perception.get(block=False)
                self.input_queue_perception.task_done()
            except queue.Empty: pass

            # Only proceed if we have essential data (at least localization)
            if self._latest_localization and self._latest_prediction and self._latest_perception :
                current_time = time.time()
                # Check data freshness (optional, for simplicity not implemented here)
                # if abs(current_time - self._latest_localization.timestamp) > STALE_THRESHOLD: continue etc.

                # logger.debug(f"PlanningModule: Processing with Loc_ts={self._latest_localization.timestamp}, Pred_ts={self._latest_prediction.timestamp}, Perc_ts={self._latest_perception.timestamp}")

                # 1. Path Planning
                planned_path = self.path_planner.plan_path(
                    self._latest_localization, self._latest_perception, self._latest_prediction, self.image_width
                )

                # 2. Decision Making
                maneuver_decision = self.decision_maker.make_decision(
                    self._latest_localization, planned_path, self._latest_prediction, self._latest_perception
                )
                # 3. Action Planning
                action_command = self.action_planner.plan_action(
                    self._latest_localization, maneuver_decision, planned_path, self._latest_perception, self.image_width
                )

                try:
                    self.output_queue_control.put(action_command, timeout=0.1)
                except queue.Full:
                    logging.warning("PlanningModule: Control output queue full.")

                # Clear latest data to ensure new data is used next cycle, or manage timestamps carefully
                # self._latest_localization = None # Or rely on overwriting by new queue items
                # self._latest_prediction = None
                # self._latest_perception = None
            else:
                # Wait if essential data is missing
                time.sleep(0.02) # Avoid busy-wait
            
            if not self._running: # Check running flag again before sleeping
                break
            # Add a small sleep if no data was processed to avoid tight loop if all queues are empty
            if not (self._latest_localization and self._latest_prediction and self._latest_perception):
                time.sleep(0.01) # Small sleep if waiting for data

        logger.info("PlanningModule: Thread stopped.")

    def start(self):
        if not self._running:
            self._running = True
            self._thread = threading.Thread(target=self.run, name="PlanningThread")
            self._thread.start()
            logger.info("PlanningModule: Started.")

    def stop(self):
        if self._running:
            self._running = False
            if self._thread:
                self._thread.join(timeout=2.0)
            logger.info("PlanningModule: Stopped.")