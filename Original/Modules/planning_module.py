import queue
import threading
import time
from typing import Dict, Optional
from .data_structures import (
    LocalizationInfo, BehavioralPredictionOutput, PerceptionOutput,
    PlannedPath, ManeuverDecision, ActionCommand
)
from .localization_module import HDMapInterface # HDMapInterface from localization_module
import numpy as np

class PathPlannerComponent:
    def __init__(self, config: dict, hd_map_interface: HDMapInterface):
        self.config = config
        self.hd_map = hd_map_interface
        print("PathPlannerComponent: Initialized.")

    def plan_path(self, current_pose: LocalizationInfo, scene_info: PerceptionOutput,
                  behavioral_predictions: BehavioralPredictionOutput, image_width: int = 640) -> PlannedPath:
        # print(f"PathPlannerComponent: Planning path from {current_pose.position}")
        waypoints = []
        
        # 차선 정보를 기반으로 간단한 목표 지점 설정 (차선 중앙)
        left_lane, right_lane = None, None
        for lm in scene_info.lane_markings:
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
        for i in range(1, 6): 
            waypoints.append((start_x + i * 1.0, start_y)) # 1미터 간격으로 5개
        return PlannedPath(timestamp=current_pose.timestamp, waypoints=waypoints)

class DecisionMakerComponent:
    def __init__(self, config: dict):
        self.config = config
        print("DecisionMakerComponent: Initialized.")

    def make_decision(self, current_pose: LocalizationInfo, planned_path: PlannedPath,
                      behavioral_predictions: BehavioralPredictionOutput, scene_info: PerceptionOutput) -> ManeuverDecision:
        # Placeholder for Decision-making (Fig 2)
        # Uses path, predictions, localization, (and potentially direct perception like traffic lights)
        # print(f"DecisionMakerComponent: Making decision based on path with {len(planned_path.waypoints)} waypoints.")
        # Example: Default to lane keep
        target_speed_kph = self.config.get("target_speed_kph", 10.0) # 설정에서 기본 속도 가져오기
        return ManeuverDecision(
            timestamp=current_pose.timestamp,
            chosen_maneuver="LANE_KEEP",
            target_speed_kph=target_speed_kph, # 고정 속도 또는 설정값 사용
            lead_vehicle_id=None
        )

class ActionPlannerComponent:
    def __init__(self, config: dict):
        self.config = config
        print("ActionPlannerComponent: Initialized.")

    def plan_action(self, current_pose: LocalizationInfo, decision: ManeuverDecision, 
                      planned_path: PlannedPath, perception_info: PerceptionOutput, 
                      image_width: int = 640) -> ActionCommand:
        # print(f"ActionPlannerComponent: Planning action for maneuver '{decision.chosen_maneuver}'")
        target_velocity_mps = decision.target_speed_kph / 3.6
        target_steering_rad = 0.0

        left_lane, right_lane = None, None
        for lm in perception_info.lane_markings:
            if lm.type == "left_lane" and lm.points:
                left_lane = lm.points # [(x1,y1), (x2,y2)]
            elif lm.type == "right_lane" and lm.points:
                right_lane = lm.points

        # 차선 중앙 계산 및 조향각 결정 (P 제어)
        if left_lane and right_lane:
            # 이미지의 특정 y 지점 (예: 이미지 상단에서 70% 지점)에서의 차선 x 좌표 사용
            # left_lane[1] = (x_top_left, y_top_left), right_lane[1] = (x_top_right, y_top_right)
            # 여기서는 간단히 각 차선의 두 번째 점(상단 점)의 x좌표 평균을 사용
            # 실제로는 y좌표가 동일한 지점에서의 x좌표를 찾아야 함.
            # 여기서는 average_lines에서 y2_new를 동일하게 설정했으므로, 그 x좌표를 사용 가능
            
            # y_horizon = image_height * 0.7 (DetectionComponent와 동일한 y_top 가정)
            # left_x_at_horizon = interpolate_x(left_lane, y_horizon)
            # right_x_at_horizon = interpolate_x(right_lane, y_horizon)
            
            # 간단하게 각 차선의 상단 x 좌표 사용
            left_x_top = left_lane[1][0]
            right_x_top = right_lane[1][0]
            
            lane_center_x = (left_x_top + right_x_top) / 2.0
            
            # 차량의 현재 위치 (이미지 중앙 x 좌표)
            vehicle_center_x = image_width / 2.0
            
            # 오차 계산 (차량 중심과 차선 중심 간의 x좌표 차이)
            error = lane_center_x - vehicle_center_x
            
            # P 제어기 (Kp 값은 튜닝 필요)
            # TODO: config에서 Kp 값 가져오기
            kp = self.config.get("steering_kp", 0.005) # 예시 Kp 값
            target_steering_rad = -kp * error # 오차가 양수(오른쪽)이면 음수(왼쪽) 조향

            # 최대 조향각 제한 (라디안 단위)
            max_steer_rad = self.config.get("max_steer_rad", 0.5) # 약 28도
            target_steering_rad = np.clip(target_steering_rad, -max_steer_rad, max_steer_rad)
            
        elif left_lane: # 왼쪽 차선만 보일 때 (오른쪽으로 붙도록)
            target_steering_rad = self.config.get("single_lane_steer_rad", 0.15) 
        elif right_lane: # 오른쪽 차선만 보일 때 (왼쪽으로 붙도록)
            target_steering_rad = -self.config.get("single_lane_steer_rad", 0.15)
        
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
        print("PlanningModule: Initialized.")

    def run(self):
        print("PlanningModule: Thread started.")
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

                # print(f"PlanningModule: Processing with Loc_ts={self._latest_localization.timestamp}, Pred_ts={self._latest_prediction.timestamp}, Perc_ts={self._latest_perception.timestamp}")

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
                    print("PlanningModule: Control output queue full.")

                # Clear latest data to ensure new data is used next cycle, or manage timestamps carefully
                # self._latest_localization = None # Or rely on overwriting by new queue items
                # self._latest_prediction = None
                # self._latest_perception = None
            else:
                # Wait if essential data is missing
                time.sleep(0.02) # Avoid busy-wait
            
            if not self._running: # Check running flag again before sleeping
                break

        print("PlanningModule: Thread stopped.")

    def start(self):
        if not self._running:
            self._running = True
            self._thread = threading.Thread(target=self.run, name="PlanningThread")
            self._thread.start()
            print("PlanningModule: Started.")

    def stop(self):
        if self._running:
            self._running = False
            if self._thread:
                self._thread.join(timeout=2.0)
            print("PlanningModule: Stopped.")