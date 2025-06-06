import queue
import threading
import time
from typing import Dict, Optional, Tuple, NamedTuple
from .data_structures import (
    PerceptionOutput,
    PlannedPath, ManeuverDecision, ActionCommand
) #LocalizationInfo, BehavioralPredictionOutput
import numpy as np
import math # math 모듈 추가
import logging
from .error_manager import error_manager, ErrorCode
from .thread_queue_manager import ThreadQueueManager

logger = logging.getLogger(__name__)

class PathPlannerComponent:
    def __init__(self, config: dict):
        # config: {"active_strategy": ..., "simple_waypoint_planner_params": {...}}
        self.active_strategy = config.get("active_strategy", "_spline_based_path_planning")
        logger.info(f"PathPlannerComponent: Initialized with {self.active_strategy}")

    def plan_path(self, scene_info: PerceptionOutput, image_width: int) -> PlannedPath:        # plan_path(self, current_pose: LocalizationInfo, scene_info: PerceptionOutput, behavioral_predictions: BehavioralPredictionOutput, image_width: int) -> PlannedPath:        
        if self.active_strategy == "_spline_based_path_planning":
            return self._spline_based_path_planning(current_pose, scene_info)
        else:
            logger.warning(f"PathPlannerComponent: Unknown active strategy '{self.active_strategy}', falling back to example planner.")
            return self.example_plan_path(current_pose, scene_info, behavioral_predictions)


    # Spline-Based Path Planning Algorithm
    def _spline_based_path_planning(self, scene_info: PerceptionOutput) -> PlannedPath:
        logger.info(f"PathPlannerComponent: Starting spline-based path planning")

        white_mask = getattr(scene_info, 'white_mask', None)
        if white_mask is None:
            logger.warning("plan_path: white_mask가 PerceptionOutput에 없습니다.")
            return PlannedPath(timestamp=current_pose.timestamp, waypoints=[])

        logger.debug(f"PathPlannerComponent: white_mask shape: {white_mask.shape}")
        roi_height, roi_width = white_mask.shape
        logger.debug(f"PathPlannerComponent: ROI dimensions: {roi_width}x{roi_height}")

        # 1. 모든 차선 경계 추출 (각 y에서 연속된 흰색 픽셀 구간)
        def extract_all_lane_boundaries(white_mask, min_lane_width_px=5):  # 10에서 5로 감소
            boundaries_list = []
            for y in range(roi_height-1, -1, -5):
                row = white_mask[y, :]
                boundaries = []
                in_lane = False
                start_x = None
                for x in range(roi_width):
                    if row[x] > 0 and not in_lane:
                        in_lane = True
                        start_x = x
                    elif row[x] == 0 and in_lane:
                        end_x = x - 1
                        if end_x - start_x + 1 >= min_lane_width_px:
                            boundaries.append((start_x, end_x))
                        in_lane = False
                if in_lane and start_x is not None:
                    end_x = roi_width - 1
                    if end_x - start_x + 1 >= min_lane_width_px:
                        boundaries.append((start_x, end_x))
                boundaries_list.append((y, boundaries))
            return boundaries_list

        all_lane_boundaries = extract_all_lane_boundaries(white_mask)
        logger.info(f"PathPlannerComponent: Extracted lane boundaries for {len(all_lane_boundaries)} y-levels")
        
        # 실제로 경계가 발견된 y 레벨만 카운트
        boundaries_with_data = [(y, boundaries) for y, boundaries in all_lane_boundaries if boundaries]
        logger.info(f"PathPlannerComponent: Found actual boundaries in {len(boundaries_with_data)} y-levels out of {len(all_lane_boundaries)}")
        
        for y, boundaries in all_lane_boundaries[:5]:  # 처음 5개만 로그로 출력
            logger.info(f"PathPlannerComponent: y={y}, boundaries={boundaries}")

        # 2. 수평선 y좌표 리스트 (하단에 집중)
        num_lines = 5
        # 하단 영역에 더 많은 웨이포인트 생성 (차선 감지가 더 안정적)
        horizontal_line_ys = [roi_height-1, roi_height//2, roi_height//3, roi_height//4, roi_height//6]

        # 3. 차량 중심점 (ROI 하단 중앙)
        car_center_point = (roi_width // 2, roi_height - 1)

        # 4. ego가 위치한 차선 구간의 중심을 따라 웨이포인트 생성
        waypoints = []
        logger.info(f"PathPlannerComponent: Generating waypoints for {len(horizontal_line_ys)} horizontal lines: {horizontal_line_ys}")
        
        # 하단에서 검출된 차선들을 기반으로 예상 차선 패턴 추정
        bottom_boundaries = next((b for yy, b in all_lane_boundaries if yy == horizontal_line_ys[0]), [])
        estimated_lane_width = None
        if len(bottom_boundaries) >= 2:
            # 인접한 차선들 간의 간격을 계산하여 평균 차선폭 추정
            gaps = []
            for i in range(len(bottom_boundaries) - 1):
                gap = bottom_boundaries[i+1][0] - bottom_boundaries[i][1]
                if gap > 0:  # 양수인 간격만 고려 (겹치지 않는 경우)
                    gaps.append(gap)
            if gaps:
                estimated_lane_width = sum(gaps) / len(gaps)
                logger.info(f"PathPlannerComponent: Estimated lane width from bottom: {estimated_lane_width:.1f} pixels")
        
        for i, y in enumerate(horizontal_line_ys):
            # 해당 y에서 모든 차선 구간 추출
            boundaries = next((b for yy, b in all_lane_boundaries if yy == y), [])
            logger.info(f"PathPlannerComponent: Line {i} at y={y}: {len(boundaries)} lane boundaries found: {boundaries}")
            
            if boundaries:
                # ego_x가 포함된 구간 찾기
                ego_x = roi_width // 2
                found = False
                for j, seg in enumerate(boundaries):
                    logger.debug(f"PathPlannerComponent: Checking segment {j}: [{seg[0]}-{seg[1]}] vs ego_x={ego_x}")
                    if seg[0] <= ego_x <= seg[1]:
                        center_x = (seg[0] + seg[1]) / 2
                        waypoints.append((center_x, y))
                        logger.info(f"PathPlannerComponent: Line {i}: ego in segment {j} [{seg[0]}-{seg[1]}], center_x={center_x:.1f}")
                        found = True
                        break
                
                if not found:
                    # ego_x가 어떤 구간에도 속하지 않으면 가장 가까운 차선의 중심 사용
                    logger.info(f"PathPlannerComponent: Line {i}: ego_x={ego_x} not in any segment, searching for closest...")
                    closest_seg = None
                    min_distance = float('inf')
                    
                    # 모든 차선 구간에 대해 거리 계산
                    for j, seg in enumerate(boundaries):
                        seg_center = (seg[0] + seg[1]) / 2
                        distance = abs(ego_x - seg_center)
                        logger.info(f"PathPlannerComponent: Line {i}, Segment {j}: [{seg[0]}-{seg[1]}], center={seg_center:.1f}, distance_to_ego={distance:.1f}")
                        if distance < min_distance:
                            min_distance = distance
                            closest_seg = seg
                    
                    # 가장 가까운 차선이 발견되면 사용
                    if closest_seg is not None:
                        center_x = (closest_seg[0] + closest_seg[1]) / 2
                        waypoints.append((center_x, y))
                        logger.info(f"PathPlannerComponent: Line {i}: Using CLOSEST segment [{closest_seg[0]}-{closest_seg[1]}], center_x={center_x:.1f}, distance={min_distance:.1f}")
                    else:
                        # 예외적 상황: ROI 중앙 fallback
                        waypoints.append((car_center_point[0], y))
                        logger.warning(f"PathPlannerComponent: Line {i}: NO closest segment found (boundaries={boundaries}), using fallback center_x={car_center_point[0]}")
            else:
                # 차선이 검출되지 않은 경우, 하단에서 추정된 패턴 사용
                if estimated_lane_width and bottom_boundaries:
                    # 하단 차선 패턴을 기반으로 ego 위치 추정
                    ego_x = roi_width // 2
                    estimated_center = None
                    
                    # 하단 차선들 중 ego에 가장 가까운 것을 기준으로 추정
                    for seg in bottom_boundaries:
                        seg_center = (seg[0] + seg[1]) / 2
                        if abs(ego_x - seg_center) < estimated_lane_width:
                            estimated_center = seg_center
                            break
                    
                    if estimated_center:
                        waypoints.append((estimated_center, y))
                        logger.info(f"PathPlannerComponent: Line {i}: no boundaries, using estimated center from bottom pattern: {estimated_center:.1f}")
                    else:
                        waypoints.append((car_center_point[0], y))
                        logger.warning(f"PathPlannerComponent: Line {i}: no boundaries, estimation failed, using fallback center_x={car_center_point[0]}")
                else:
                    waypoints.append((car_center_point[0], y))
                    logger.warning(f"PathPlannerComponent: Line {i}: no boundaries found, using fallback center_x={car_center_point[0]}")

        logger.info(f"PathPlannerComponent: Generated {len(waypoints)} waypoints: {waypoints}")
        return PlannedPath(timestamp=current_pose.timestamp, waypoints=waypoints)

    def _get_ego_lane_index(self, left_lane_boundary, right_lane_boundary, roi_width):
        """
        현재 ego(차량)가 어느 차로(차선 경계 사이)에 위치하는지 인덱스를 반환.
        - left_lane_boundary, right_lane_boundary: (x, y) 점들의 리스트
        - roi_width: ROI 이미지의 가로 픽셀 수
        반환값: (ego_lane_index, lane_regions)
          - ego_lane_index: ego가 위치한 차로 인덱스 (0부터 시작, 못 찾으면 None)
          - lane_regions: [(left_x, right_x), ...] 각 차로의 x좌표 구간 리스트
        """
        # y=roi_height-1(ROI 하단)에서 좌/우 경계의 x좌표 추출
        y_query = left_lane_boundary[0][1] if left_lane_boundary else None
        if left_lane_boundary and right_lane_boundary and y_query is not None:
            left_x = left_lane_boundary[0][0]
            right_x = right_lane_boundary[0][0]
            # 여러 차로가 있을 경우, 중간 경계도 추가 가능(확장)
            lane_regions = [(left_x, right_x)]
            ego_x = roi_width // 2  # 차량 중심(ROI 하단 중앙)
            ego_lane_index = None
            for idx, (lx, rx) in enumerate(lane_regions):
                if lx <= ego_x <= rx:
                    ego_lane_index = idx
                    break
            logger.debug(f"Ego 차량은 {ego_lane_index}번 차로에 위치 (lane_regions={lane_regions})")
            return ego_lane_index, lane_regions
        return None, []

# 사용 예시 (def _spline_based_path_planning 내부에서)


class ManeuverDecision(NamedTuple):
    timestamp: float
    chosen_maneuver: str
    speed_up: bool = False
    slow_down: bool = False
    turn_left: bool = False
    turn_right: bool = False
    lead_vehicle_id: Optional[int] = None

class DecisionMakerComponent:
    def __init__(self, config: dict):
        # config: {"active_strategy": ..., "default_lane_keep_params": {...}}
        self.active_strategy = config.get("active_strategy", "default_lane_keep")
        self.params = config.get("default_lane_keep_params", {})
        self.target_speed_kph = self.params.get("target_speed_kph", 10.0)
        logger.info(f"DecisionMakerComponent: Initialized with {self.active_strategy}, params={self.params}")

    def make_decision(self, planned_path: PlannedPath, scene_info: PerceptionOutput) -> ManeuverDecision:
        # 예시: 단순히 직진, 가속/감속/좌우회전 의도만 결정
        speed_up = False
        slow_down = False
        turn_left = False
        turn_right = False
        # 예: 장애물, 신호등, 차로 변경 등 상황에 따라 의도 결정
        # (여기서는 단순 예시)
        return ManeuverDecision(
            timestamp=current_pose.timestamp,
            chosen_maneuver="LANE_KEEP",
            speed_up=speed_up,
            slow_down=slow_down,
            turn_left=turn_left,
            turn_right=turn_right,
            lead_vehicle_id=None
        )

class ActionPlannerComponent:
    def __init__(self, config: dict):
        self.config = config
        logger.info(f"ActionPlannerComponent: Initialized with params: {self.config}")
        logger.info(f"ActionPlannerComponent: Waypoint steering enabled: {self.use_waypoint_steering}")


    def _calculate_waypoint_based_steering(self, planned_path: PlannedPath, image_roi_width: int, image_roi_height: int) -> Tuple[float, str]:
        """
        웨이포인트 기반 조향각 계산 알고리즘
        Pure Pursuit과 유사한 방식으로 가장 가까운 웨이포인트를 추적
        """
        if not planned_path.waypoints or len(planned_path.waypoints) == 0:
            logger.warning("ActionPlanner: No waypoints available for steering calculation")
            return 0.0, "WAYPOINT_NO_DATA"
        
        # 차량 현재 위치 (ROI 하단 중앙)
        ego_x = image_roi_width // 2
        ego_y = image_roi_height - 1
        
        # Look-ahead distance를 기반으로 목표 웨이포인트 선택
        # ROI 좌표계에서는 y가 작을수록 더 멀리 있음
        look_ahead_distance_px = self.waypoint_look_ahead_distance_px
        
        target_waypoint = None
        min_distance = float('inf')
        
        # 웨이포인트들 중에서 look-ahead distance에 가장 가까운 점 찾기
        for wp_x, wp_y in planned_path.waypoints:
            # ego 위치로부터 웨이포인트까지의 거리
            distance = math.sqrt((wp_x - ego_x)**2 + (wp_y - ego_y)**2)
            
            # look-ahead distance 근처의 웨이포인트를 선호
            distance_to_lookahead = abs(distance - look_ahead_distance_px)
            
            if distance_to_lookahead < min_distance and wp_y < ego_y:  # 앞쪽 웨이포인트만 고려
                min_distance = distance_to_lookahead
                target_waypoint = (wp_x, wp_y)
        
        # 적절한 웨이포인트를 찾지 못한 경우, 가장 가까운 웨이포인트 사용
        if target_waypoint is None and planned_path.waypoints:
            # 가장 앞쪽(y가 가장 작은) 웨이포인트 선택
            target_waypoint = min(planned_path.waypoints, key=lambda wp: wp[1])
            logger.info(f"ActionPlanner: Using closest waypoint as fallback: {target_waypoint}")
        
        if target_waypoint is None:
            logger.warning("ActionPlanner: No suitable waypoint found")
            return 0.0, "WAYPOINT_NO_TARGET"
        
        target_x, target_y = target_waypoint
        
        # 조향 오차 계산 (픽셀 단위)
        lateral_error = target_x - ego_x
        
        # 거리 기반 조정 (멀리 있는 웨이포인트일수록 조향 민감도 감소)
        distance_to_target = math.sqrt((target_x - ego_x)**2 + (target_y - ego_y)**2)
        distance_factor = max(0.3, min(1.0, 30.0 / max(distance_to_target, 10.0))) * self.waypoint_distance_factor_gain
        
        # 조향각 계산
        angle_deg = lateral_error * self.waypoint_steering_gain * distance_factor
        
        # 조향각 제한
        angle_deg = np.clip(angle_deg, -self.waypoint_max_angle_deg, self.waypoint_max_angle_deg)
        
        logger.debug(f"ActionPlanner: Waypoint steering - Target: ({target_x:.1f}, {target_y:.1f}), "
                    f"Error: {lateral_error:.1f}px, Distance: {distance_to_target:.1f}px, "
                    f"Factor: {distance_factor:.2f}, Angle: {angle_deg:.2f}°")
        
        return angle_deg, f"WAYPOINT_BASED (target: {target_x:.0f},{target_y:.0f})"

    def plan_action(self, planned_path, perception_info, image_width=640): # def plan_action(self, current_pose, decision, planned_path, perception_info, image_width=640):

        if self.use_waypoint_steering and planned_path and planned_path.waypoints and len(planned_path.waypoints) > 0:  # 웨이포인트 기반 조향각 계산
            image_height = perception_info.image.shape[0] if hasattr(perception_info, 'image') and perception_info.image is not None else 480
            angle_deg, log_info = self._calculate_waypoint_based_steering(planned_path, image_width, image_height)
            logger.info(f"ActionPlanner: Using waypoint-based steering - {log_info}")

            target_steering_rad = math.radians(angle_deg)
            self.prev_steering_angle_rad = target_steering_rad

        # DecisionMaker의 의도에 따라 속도/조향 보정
        speed_xycar = self.base_speed_xycar
        if getattr(decision, "speed_up", False):
            speed_xycar += 10
        if getattr(decision, "slow_down", False):
            speed_xycar -= 10
        speed_xycar = np.clip(speed_xycar, 0, self.speed_config_xycar_units.get("straight", 80))

        # 조향각 보정(예: turn_left/right)
        if getattr(decision, "turn_left", False):
            angle_deg += 5
        if getattr(decision, "turn_right", False):
            angle_deg -= 5
        target_steering_rad = math.radians(angle_deg)

        target_velocity_mps = speed_xycar * self.xycar_speed_to_mps_factor

        logger.debug(f"ActionPlanner: Mode: {log_info}, Angle(deg): {angle_deg:.2f}, Speed(xycar): {speed_xycar}, Vel(mps): {target_velocity_mps:.2f}")
        planning_results = {
            "perception_info": perception_info,
            # "localization_info": current_pose,
            "planned_path": planned_path
            # "maneuver_decision": decision
        }
        action_cmd = ActionCommand(
            # timestamp=current_pose.timestamp,
            target_velocity_mps=target_velocity_mps,
            target_steering_angle_rad=target_steering_rad,
            planning_results=planning_results
        )
        return action_cmd

class PlanningModule:
    def __init__(self, planning_specific_config: dict,
                 overall_system_config: dict,
                 input_queues: Dict[str, ThreadQueueManager],
                 output_queue_control: ThreadQueueManager):
        self.planning_config = planning_specific_config
        self.path_planner = PathPlannerComponent(self.planning_config.get("path_planner", {}))
        self.decision_maker = DecisionMakerComponent(self.planning_config.get("decision_maker", {}))
        self.action_planner = ActionPlannerComponent(self.planning_config.get("action_planner", {}))
        self.image_width = overall_system_config.get("image_width", 640)
        self.input_queues = input_queues
        self.output_queue_control = output_queue_control
        self._running = False
        self._thread = None
        logger.info("PlanningModule: Initialized.")

    def run(self):
        logger.info("PlanningModule: Thread started.")
        try:
            while self._running:
                try:
                    localization_info = self.input_queues["localization"].get(block=False)
                except queue.Empty:
                    localization_info = None
                try:
                    prediction_info = self.input_queues["prediction"].get(block=False)
                except queue.Empty:
                    prediction_info = None
                try:
                    perception_info = self.input_queues["perception"].get(block=False)
                except queue.Empty:
                    perception_info = None
                
                # 로그 추가: 큐에서 받은 데이터 상태 확인
                logger.info(f"PlanningModule: Got data - "
                            f"Loc: {localization_info is not None}, "
                            f"Pred: {prediction_info is not None}, "
                            f"Perc: {perception_info is not None} from queues.")
                
                if localization_info and perception_info:
                    logger.info(f"PlanningModule: Processing with Loc_ts={localization_info.timestamp}, "
                               f"Pred_ts={prediction_info.timestamp if prediction_info else 'None'}, "
                               f"Perc_ts={perception_info.timestamp}")
                    
                    planned_path = self.path_planner.plan_path(perception_info, self.image_width) #planned_path = self.path_planner.plan_path(localization_info, perception_info, prediction_info, self.image_width)
                    logger.info(f"PlanningModule: Generated path with {len(planned_path.waypoints)} waypoints")
                    
                    maneuver_decision = self.decision_maker.make_decision(localization_info, planned_path, prediction_info, perception_info) #self.decision_maker.make_decision(localization_info, planned_path, prediction_info, perception_info)
                    logger.info(f"PlanningModule: Decision: {maneuver_decision.chosen_maneuver}")

                    action_command = self.action_planner.plan_action(localization_info, maneuver_decision, planned_path, perception_info, self.image_width)
                    logger.info(f"PlanningModule: Action command generated")

                    try:
                        self.output_queue_control.put(action_command, timeout=0.1)
                        logger.info(f"PlanningModule: Successfully sent action command")
                    except queue.Full:
                        logger.warning("PlanningModule: Output queue is full.")
                time.sleep(0.001)
            logger.info("PlanningModule: Thread stopped.")
        except Exception as e:
            error_manager.handle(ErrorCode.MODULE_RUNTIME_EXCEPTION, str(e))

    def start(self):
        if not self._running:
            self._running = True
            try:
                self._thread = threading.Thread(target=self.run, name="PlanningThread")
                self._thread.start()
                logger.info("PlanningModule: Started.")
            except Exception as e:
                error_manager.handle(ErrorCode.MODULE_START_FAIL, str(e))
                raise

    def stop(self):
        if self._running:
            self._running = False
            try:
                if self._thread:
                    self._thread.join(timeout=2.0)
                logger.info("PlanningModule: Stopped.")
            except Exception as e:
                error_manager.handle(ErrorCode.MODULE_RUNTIME_EXCEPTION, str(e))