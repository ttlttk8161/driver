# 계획 모듈
import queue
import _queue
import threading
import time
from typing import Dict, Optional, Tuple
from .optimized_data_structures import (
    LocalizationInfo, BehavioralPredictionOutput,
    PlannedPath, ManeuverDecision, ActionCommand
)
from .optimized_data_structures import DataPriority, PriorityQueueItem, OptimizedPerceptionOutput
from .performance_monitor import performance_timing, PerformanceTracker
import numpy as np
import math
import rospy
import logging

logger = logging.getLogger(__name__)

# 성능 추적기 초기화
performance_tracker = PerformanceTracker("PlanningModule")

class PathPlannerComponent:
    def __init__(self, config: dict):
        self.config = config
        self.active_strategy_name = self.config.get("active_strategy", "simple_waypoint_planner")
        self.strategy_params = self.config.get(f"{self.active_strategy_name}_params", {})
        self.strategy_map = {
            "simple_waypoint_planner": self._execute_simple_waypoint_planner,
            "a_star_planner": self._execute_a_star_planner,
        }
        logger.info(f"PathPlannerComponent: Initialized. Strategy: {self.active_strategy_name} with params: {self.strategy_params}")

    def _execute_simple_waypoint_planner(self, current_pose: LocalizationInfo, scene_info: OptimizedPerceptionOutput,
                                         behavioral_predictions: BehavioralPredictionOutput, params: dict, 
                                         image_width: int = 640) -> PlannedPath:
        waypoints = []
        num_waypoints = params.get("num_waypoints", 5)
        waypoint_spacing_m = params.get("waypoint_spacing_m", 1.0)
        
        # 차선 정보 추출
        left_lane, right_lane = None, None
        for lm in scene_info.lane_markings:
            if lm.type == "left_lane":
                left_lane = lm
            elif lm.type == "right_lane":
                right_lane = lm
        
        if left_lane and right_lane:
            # For a mapless system, a simple path planner might generate waypoints
            # based on the perceived lane center or a desired offset from a lane.
            # Here, we assume ActionPlanner will handle detailed lane following,
            # so PathPlanner generates a generic forward path.
            pass # ActionPlanner가 차선 정보를 직접 사용

        # 기본적으로 직진 경로 (ActionPlanner가 차선 기반으로 조향)
        start_x, start_y, _ = current_pose.position # 글로벌 좌표계 (더미)
        for i in range(1, num_waypoints + 1): 
            waypoints.append((start_x + i * waypoint_spacing_m, start_y)) # 직진 웨이포인트
        return PlannedPath(timestamp=current_pose.timestamp, waypoints=waypoints)

    def _execute_a_star_planner(self, current_pose: LocalizationInfo, scene_info: OptimizedPerceptionOutput,
                                behavioral_predictions: BehavioralPredictionOutput, params: dict,
                                image_width: int = 640) -> PlannedPath:
        logger.debug(f"AStarPlanner (Basic): Generating waypoints with params: {params}")
        # This is a placeholder for a mapless path generation strategy.
        # A true A* requires a map/grid. For mapless, this might be a behavior-based planner
        # or one that projects a path based on current lane perception.
        # It generates a simple path ahead, slightly adjusted.
        
        waypoints = []
        num_waypoints = params.get("num_waypoints", 5) # Reuse from simple_waypoint_planner_params if not in a_star_params
        waypoint_spacing_m = params.get("waypoint_spacing_m", 1.0) # Reuse
        
        start_x, start_y, _ = current_pose.position
        # Assuming current_pose.orientation_quaternion gives vehicle's heading.
        # For simplicity, we'll assume heading is along the x-axis in the current dummy localization if orientation is not used.
        # A real implementation would use the orientation to project waypoints.

        for i in range(1, num_waypoints + 1):
            # Simple straight path for now, as orientation handling is complex for a placeholder
            wp_x = start_x + i * waypoint_spacing_m 
            wp_y = start_y # + lateral_offset_correction * (i / num_waypoints) # Gradually apply correction
            waypoints.append((wp_x, wp_y))
        logger.info(f"AStarPlanner (Basic): Generated {len(waypoints)} waypoints ahead.")
        return PlannedPath(timestamp=current_pose.timestamp, waypoints=waypoints)

    @performance_timing
    def plan_path(self, current_pose: LocalizationInfo, scene_info: OptimizedPerceptionOutput,
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
            "rule_based_logic": self._execute_rule_based_logic, # 새로운 전략 추가
        }
        logger.info(f"DecisionMakerComponent: Initialized. Strategy: {self.active_strategy_name} with params: {self.strategy_params}")

    def _execute_default_lane_keep(self, current_pose: LocalizationInfo, planned_path: PlannedPath,
                                   behavioral_predictions: BehavioralPredictionOutput, 
                                   scene_info: OptimizedPerceptionOutput, params: dict) -> ManeuverDecision:
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

    def _execute_rule_based_logic(self, current_pose: LocalizationInfo, planned_path: PlannedPath,
                                  behavioral_predictions: BehavioralPredictionOutput,
                                  scene_info: OptimizedPerceptionOutput, params: dict) -> ManeuverDecision:
        logger.debug(f"RuleBasedLogic: Making decision with params: {params}")
        
        stop_line_threshold = params.get("stop_line_distance_threshold_m", 3.0)
        default_target_speed_kph = params.get("target_speed_kph", 10.0) # From rule_based_logic_params

        if scene_info and scene_info.traffic_signs:
            for sign in scene_info.traffic_signs:
                if sign.type == "stop_sign": # Assuming 'stop_sign' is a defined type
                    # Calculate distance (simplified 2D distance)
                    dist_to_sign = math.sqrt(
                        (current_pose.position[0] - sign.position_3d[0])**2 +
                        (current_pose.position[1] - sign.position_3d[1])**2
                    )
                    if dist_to_sign < stop_line_threshold:
                        logger.info(f"RuleBasedLogic: Detected stop sign at {dist_to_sign:.2f}m. Commanding STOP.")
                        return ManeuverDecision(current_pose.timestamp, "STOP_AT_SIGN", 0.0, None)
        
        # Default to lane keeping if no specific rules are met
        logger.debug("RuleBasedLogic: No specific rules met. Defaulting to LANE_KEEP.")
        return ManeuverDecision( # 기본적으로 차선 유지
            timestamp=current_pose.timestamp,
            chosen_maneuver="LANE_KEEP_RULE_BASED",
            target_speed_kph=default_target_speed_kph,
            lead_vehicle_id=None
        )

    @performance_timing
    def make_decision(self, current_pose: LocalizationInfo, planned_path: PlannedPath,
                      behavioral_predictions: BehavioralPredictionOutput, scene_info: OptimizedPerceptionOutput) -> ManeuverDecision:
        selected_method = self.strategy_map.get(self.active_strategy_name)
        if selected_method:
            return selected_method(current_pose, planned_path, behavioral_predictions, scene_info, self.strategy_params)
        else:
            logger.warning(f"DecisionMakerComponent: Strategy '{self.active_strategy_name}' not found. Returning default decision.")
            return ManeuverDecision(timestamp=time.time(), chosen_maneuver="EMERGENCY_STOP", target_speed_kph=0.0, lead_vehicle_id=None)

class ActionPlannerComponent:
    def __init__(self, config: dict):
        self.config = config
        self.active_strategy_name = self.config.get("active_strategy", "hsv_lane_following")
        self.strategy_params = self.config.get(f"{self.active_strategy_name}_params", {})

        self.strategy_map = {
            "hsv_lane_following": self._execute_hsv_lane_following,
            "pid_path_tracking": self._execute_pid_path_tracking,
        }

        # steering_balancing.py의 상태 변수들
        self.prev_steering_angle_rad = 0.0 # 이전 조향각 (라디안)
        self.white_lost_count = 0 # 흰색 선 연속 손실 횟수
        self.frame_counter = 0 # 초기 직진 주행을 위한 프레임 카운터

        # 파라미터들은 self.strategy_params 에서 가져와 사용 (아래 _execute_hsv_lane_following 내부에서)
        logger.info(f"ActionPlannerComponent: Initialized. Strategy: {self.active_strategy_name} with params: {self.strategy_params}")

    def _execute_hsv_lane_following(self, current_pose: LocalizationInfo, decision: ManeuverDecision,
                                     planned_path: PlannedPath, perception_info,
                                     image_width: int, params: dict) -> ActionCommand:
        # HSV 차선 정보를 사용하여 조향각과 속도를 계산하는 전략
        self.frame_counter += 1
        target_steering_deg = 0.0
        
        # 파라미터 로드
        initial_straight_frames = params.get("initial_straight_frames", 50)
        initial_speed_xycar_units = params.get("initial_speed_xycar_units", 60)
        white_steering_gain = params.get("white_steering_gain", 0.6)
        white_max_angle_deg = params.get("white_max_angle_deg", 30)
        white_offset_ratio_threshold = params.get("white_offset_ratio_threshold", 0.05)
        white_offset_angle_deg = params.get("white_offset_angle_deg", 15)
        yellow_fallback_steering_gain = params.get("yellow_fallback_steering_gain", 0.005)
        yellow_fallback_max_angle_deg = params.get("yellow_fallback_max_angle_deg", 25)
        no_line_escape_angle_deg = params.get("no_line_escape_angle_deg", -15)
        max_steering_delta_deg = params.get("max_steering_delta_deg", 10)
        speed_config_xycar_units = params.get("speed_tiers_xycar_units", {})
        xycar_speed_to_mps_factor = params.get("xycar_speed_to_mps_factor", 0.028)

        target_speed_xycar_units = initial_speed_xycar_units # 기본값
        log_info = "[INIT_DEFAULT_HSV_STRATEGY]"
        
        # PerceptionModule의 _detect_hsv_lines에서 ROI는 image[roi_y_start:, :] 이므로,
        # 해당 ROI의 너비는 전체 이미지 너비(image_width)와 동일합니다.
        image_roi_width_for_hsv = image_width

        # 입력 데이터 타입 확인 및 호환성 처리
        white_metrics = None
        yellow_metrics = None
        
        # OptimizedPerceptionOutput 처리
        white_metrics = perception_info.white_line_metrics
        yellow_metrics = perception_info.yellow_line_metrics

        # steering_balancing.py 로직에 따라 HSV 차선 정보를 사용하여 조향각(도)과 속도(Xycar 단위)를 계산합니다.
        angle_deg = 0.0
        current_log = "START"

        # steering_balancing.py: if total_white > 300 (white_pixel_threshold는 Perception에서 처리)
        if white_metrics and white_metrics.is_detected:
            self.white_lost_count = 0
            # steering_balancing.py: error = (left_ratio - right_ratio) * 100
            error = (white_metrics.left_ratio - white_metrics.right_ratio) * 100 
            angle_deg = np.clip(error * white_steering_gain, -white_max_angle_deg, white_max_angle_deg)

            # steering_balancing.py: if left_ratio - right_ratio > 0.05: angle += 15
            if (white_metrics.left_ratio - white_metrics.right_ratio) > white_offset_ratio_threshold:
                angle_deg += white_offset_angle_deg
            elif (white_metrics.right_ratio - white_metrics.left_ratio) > white_offset_ratio_threshold:
                angle_deg -= white_offset_angle_deg
            
            angle_deg = np.clip(angle_deg, -white_max_angle_deg, white_max_angle_deg) # 오프셋 적용 후 다시 클리핑
            current_log = "WHITE_TRACK"
        else: # 흰색 선 미감지 또는 부족
            self.white_lost_count += 1
            # steering_balancing.py: if white_lost_count >= 1 (즉시 폴백)
            # steering_balancing.py: if M['m00'] > 0 (yellow_area_threshold는 Perception에서 처리)
            if yellow_metrics and yellow_metrics.is_detected and yellow_metrics.center_x is not None:
                # yellow_metrics.center_x는 Perception에서 사용된 ROI 내부의 x좌표.
                # image_roi_width_for_hsv는 해당 ROI의 너비.
                roi_center_x = image_roi_width_for_hsv / 2.0
                error = yellow_metrics.center_x - roi_center_x
                angle_deg = np.clip(error * yellow_fallback_steering_gain, -yellow_fallback_max_angle_deg, yellow_fallback_max_angle_deg)
                current_log = "YELLOW_FALLBACK"
            else: # 노란색 선도 미감지
                angle_deg = no_line_escape_angle_deg
                current_log = "NO_LINE_ESCAPE"
        
        # 조향각 변화 제한 (스무딩) - steering_balancing.py: max_delta = 10
        prev_angle_deg = math.degrees(self.prev_steering_angle_rad)
        delta_angle = angle_deg - prev_angle_deg
        if abs(delta_angle) > max_steering_delta_deg:
            angle_deg = prev_angle_deg + np.sign(delta_angle) * max_steering_delta_deg
        
        # 속도 결정 (Xycar 단위) - steering_balancing.py 기준
        abs_angle_deg = abs(angle_deg)
        if "NO_LINE" in current_log or "FALLBACK" in current_log: # steering_balancing.py: "HOLD" or "NO LINE"
            speed_xycar = speed_config_xycar_units.get("no_line_or_fallback", 20)
        elif abs_angle_deg < 5:
            speed_xycar = speed_config_xycar_units.get("straight", 45)
        elif abs_angle_deg < 10:
            speed_xycar = speed_config_xycar_units.get("gentle_turn", 35)
        else:
            speed_xycar = speed_config_xycar_units.get("sharp_turn", 25)
        
        target_steering_deg = angle_deg
        target_speed_xycar_units = speed_xycar
        log_info = current_log

        # 초기 직진 로직
        if self.frame_counter <= initial_straight_frames:
            target_steering_deg = 0.0 # 초기 직진
            target_speed_xycar_units = initial_speed_xycar_units
            log_info = f"[INIT_STRAIGHT] Frame {self.frame_counter}"
        elif not (perception_info.white_line_hsv_metrics or perception_info.yellow_line_hsv_metrics):
            # HSV 메트릭이 없는 경우 (예: Perception 모듈에서 아직 준비되지 않음) -> 이전 Canny 로직 또는 기본값 사용
            # 여기서는 steering_balancing.py 통합에 집중하므로, 기본값(직진 또는 이전 값 유지)으로 설정
            target_steering_deg = math.degrees(self.prev_steering_angle_rad) # 이전 각도 유지 시도
            target_speed_xycar_units = speed_config_xycar_units.get("no_line_or_fallback", 20) # 안전 속도
            log_info = "[NO_HSV_METRICS_FALLBACK]"

        # 최종 조향각(도)을 라디안으로 변환
        target_steering_rad = math.radians(target_steering_deg)
        self.prev_steering_angle_rad = target_steering_rad # 다음 프레임을 위해 현재 조향각(라디안) 저장

        # Xycar 속도 단위를 m/s로 변환
        target_velocity_mps = target_speed_xycar_units * xycar_speed_to_mps_factor

        # 로그 추가 (필요시) 이 로그를 통해 `[INIT_STRAIGHT]`, `WHITE_TRACK`, `YELLOW_FALLBACK`, `NO_LINE_ESCAPE`, `[NO_HSV_METRICS_FALLBACK]` 중 
        # 어떤 상태인지 파악할 수 있습니다. 만약 계속 `[NO_HSV_METRICS_FALLBACK]`가 출력된다면, HSV 차선 정보가 `ActionPlannerComponent`에 제대로 전달되지 
        # 않거나, `perception_info.white_line_hsv_metrics`와 `perception_info.yellow_line_hsv_metrics`가 `None`으로 전달되고 있다는 의미입니다.
        logger.debug(f"ActionPlanner: Mode: {log_info}, Angle(deg): {target_steering_deg:.2f}, Speed(xycar): {target_speed_xycar_units}, Vel(mps): {target_velocity_mps:.2f}")

        return ActionCommand(
            timestamp=current_pose.timestamp,
            target_velocity_mps=target_velocity_mps,
            target_steering_angle_rad=target_steering_rad
        )

    def _execute_pid_path_tracking(self, current_pose: LocalizationInfo, decision: ManeuverDecision,
                                     planned_path: PlannedPath, perception_info,
                                     image_width: int, params: dict) -> ActionCommand:
        """
        PID 제어를 사용하여 계획된 경로를 추종하는 전략입니다.
        (현재는 플레이스홀더, 실제 PID 로직은 추가 구현 필요)
        """
        kp = params.get("kp_steer", 0.5)
        ki = params.get("ki_steer", 0.01) # 현재 미사용
        kd = params.get("kd_steer", 0.1) # 현재 미사용
        log_cte_threshold = params.get("log_cte_threshold", 0.01)

        target_velocity_mps = decision.target_speed_kph / 3.6  # kph to mps
        target_steering_rad = 0.0

        if not planned_path.waypoints:
            logger.warning("PIDPathTracking: No waypoints in planned_path. Commanding zero steering.")
            return ActionCommand(timestamp=current_pose.timestamp,
                                 target_velocity_mps=target_velocity_mps, # 속도는 유지
                                 target_steering_angle_rad=0.0)

        # 가장 가까운 웨이포인트 또는 lookahead 지점을 찾아 CTE(Cross-Track Error) 계산 (단순화된 버전)
        # 실제 구현에서는 차량의 현재 위치/방향과 경로 세그먼트 간의 관계를 정확히 계산해야 함.
        # 여기서는 첫 번째 웨이포인트와의 y 오차를 CTE로 가정 (매우 단순한 예시)
        # current_pose.position = (x, y, z)
        # planned_path.waypoints = [(x1,y1), (x2,y2), ...]
        
        # 예시: 차량의 현재 y 위치와 첫 번째 웨이포인트의 y 위치 차이를 CTE로 사용
        # 이는 차량이 x축을 따라 이동하고 y축 오차를 수정한다고 가정할 때 매우 단순화된 접근 방식입니다.
        # 실제로는 차량의 방향과 경로의 방향을 모두 고려해야 합니다.
        vehicle_x, vehicle_y, _ = current_pose.position
        
        # 가장 가까운 경로 지점 찾기 (또는 lookahead 지점) - 여기서는 첫번째 웨이포인트를 단순 목표로 가정
        target_wp_x, target_wp_y = planned_path.waypoints[0]

        # 단순 CTE: y방향 오차 (차량이 x축을 따라 주행한다고 가정)
        # 이 CTE 계산은 current_pose.orientation_quaternion을 사용하여 차량 좌표계 기준으로 변환해야 더 정확합니다.
        # 현재 LocalizationModule의 orientation은 더미 값이므로, 글로벌 y 오차를 사용합니다.
        cross_track_error = target_wp_y - vehicle_y # 목표 y - 현재 y

        if abs(cross_track_error) > log_cte_threshold: # 로깅 임계값 추가
            logger.debug(f"PIDPathTracking: CTE: {cross_track_error:.3f} (VehicleY: {vehicle_y:.2f}, TargetWP_Y: {target_wp_y:.2f})")

        # P 제어만 사용한 단순 조향각 계산 (실제로는 I, D 항 및 차량 모델 고려 필요)
        target_steering_rad = -kp * cross_track_error # 오차에 비례하여 반대 방향으로 조향

        # 조향각 제한 (필요시)
        # max_steer_rad = math.radians(params.get("max_angle_deg_pid", 30)) # 예시
        # target_steering_rad = np.clip(target_steering_rad, -max_steer_rad, max_steer_rad)

        self.prev_steering_angle_rad = target_steering_rad # 이전 조향각 업데이트 (HSV 전략과 공유하지 않도록 주의)

        return ActionCommand(timestamp=current_pose.timestamp,
                             target_velocity_mps=target_velocity_mps,
                             target_steering_angle_rad=target_steering_rad)

    @performance_timing
    def plan_action(self, current_pose: LocalizationInfo, decision: ManeuverDecision, 
                      planned_path: PlannedPath, perception_info, 
                      image_width: int = 640) -> ActionCommand:
        selected_method = self.strategy_map.get(self.active_strategy_name)
        if selected_method:
            return selected_method(current_pose, decision, planned_path, perception_info, image_width, self.strategy_params)
        else:
            logger.warning(f"ActionPlannerComponent: Strategy '{self.active_strategy_name}' not found. Returning default action (stop).")
            return ActionCommand(timestamp=time.time(), target_velocity_mps=0.0, target_steering_angle_rad=0.0)

class PlanningModule:
    def __init__(self, planning_specific_config: dict, overall_system_config: dict,
                 input_queues: dict, output_queue_control: queue.Queue):
        self.config = planning_specific_config
        self.input_queues = input_queues
        self.output_queue_control = output_queue_control
        self._running = False
        self._thread = None
        # 각 컴포넌트 초기화
        self.path_planner = PathPlannerComponent(self.config.get("path_planner", {}))
        self.decision_maker = DecisionMakerComponent(self.config.get("decision_maker", {}))
        self.action_planner = ActionPlannerComponent(self.config.get("action_planner", {}))
        
        logger.info("PlanningModule: Initialized.")

    @performance_timing
    def run(self):
        print("[PLANNING] run() 진입", flush=True)
        logger.info("PlanningModule: Thread started.")
        
        while self._running:
            print("[PLANNING] run() 루프 진입, 데이터 수집 시도", flush=True)
            try:
                # 입력 큐들에서 최신 데이터 수집
                self._collect_latest_data()
                
                # 모든 필수 데이터가 있는지 확인
                if self._latest_localization and self._latest_perception:
                    print(f"PlanningModule: Processing data - localization: {self._latest_localization.timestamp:.3f}, perception: {self._latest_perception.timestamp:.3f}", flush=True)
                    
                    # 경로 계획
                    planned_path = self.path_planner.plan_path(
                        current_pose=self._latest_localization,
                        scene_info=self._latest_perception,
                        behavioral_predictions=self._latest_prediction,
                        image_width=self.overall_config.get("image_width", 640)
                    )
                    
                    # 의사 결정
                    decision = self.decision_maker.make_decision(
                        current_pose=self._latest_localization,
                        planned_path=planned_path,
                        behavioral_predictions=self._latest_prediction,
                        scene_info=self._latest_perception
                    )
                    
                    # 행동 계획
                    action_command = self.action_planner.plan_action(
                        current_pose=self._latest_localization,
                        decision=decision,
                        planned_path=planned_path,
                        perception_info=self._latest_perception,
                        image_width=self.overall_config.get("image_width", 640)
                    )
                    
                    # 제어 모듈로 전달
                    try:
                        # 우선순위 큐 사용
                        from .optimized_data_structures import PriorityQueueItem, DataPriority
                        priority_item = PriorityQueueItem(
                            priority=DataPriority.CONTROL_COMMAND,
                            timestamp=action_command.timestamp,
                            data=action_command
                        )
                        self.output_queue_control.put(priority_item, timeout=0.1)
                        print(f"PlanningModule: Sent action command to control - steering: {action_command.steering_angle:.2f}, speed: {action_command.target_speed_kph:.2f}", flush=True)
                        logger.info(f"PlanningModule: Sent action command - steering: {action_command.steering_angle:.2f}, speed: {action_command.target_speed_kph:.2f}")
                        performance_tracker.record_metric("action_commands_sent", 1)
                    except queue.Full:
                        logger.warning("PlanningModule: Control queue is full, discarding action command")
                        performance_tracker.record_metric("action_commands_dropped", 1)
                else:
                    # 필수 데이터 부족 시 대기
                    time.sleep(0.01)
                    
            except Exception as e:
                logger.error(f"PlanningModule: Error in main loop: {e}", exc_info=True)
                performance_tracker.record_metric("error_events", 1)
                time.sleep(0.01)
        
        logger.info("PlanningModule: Thread stopped.")

    def _collect_latest_data(self):
        """입력 큐들에서 최신 데이터를 수집"""
        # Localization 데이터
        try:
            while True:
                queue_item = self.input_queues["localization"].get_nowait()
                if hasattr(queue_item, 'data'):
                    self._latest_localization = queue_item.data
                else:
                    self._latest_localization = queue_item
                self.input_queues["localization"].task_done()
        except (queue.Empty, _queue.Empty):
            pass
        
        # Prediction 데이터
        try:
            while True:
                queue_item = self.input_queues["prediction"].get_nowait()
                if hasattr(queue_item, 'data'):
                    self._latest_prediction = queue_item.data
                else:
                    self._latest_prediction = queue_item
                self.input_queues["prediction"].task_done()
        except (queue.Empty, _queue.Empty):
            pass
        
        # Perception 데이터
        try:
            while True:
                queue_item = self.input_queues["perception"].get_nowait()
                if hasattr(queue_item, 'data'):
                    self._latest_perception = queue_item.data
                else:
                    self._latest_perception = queue_item
                self.input_queues["perception"].task_done()
        except (queue.Empty, _queue.Empty):
            pass

    def start(self):
        """Planning 모듈 시작"""
        if not self._running:
            self._running = True
            self._thread = threading.Thread(target=self.run, name="PlanningThread")
            self._thread.start()
            logger.info("PlanningModule: Started")

    def stop(self):
        """Planning 모듈 정지"""
        if self._running:
            self._running = False
            if self._thread:
                self._thread.join(timeout=2.0)
            logger.info("PlanningModule: Stopped")