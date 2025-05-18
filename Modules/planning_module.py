import queue
import threading
import time
from typing import Dict, Optional
from data_structures import (
    LocalizationInfo, BehavioralPredictionOutput, PerceptionOutput,
    PlannedPath, ManeuverDecision, ActionCommand, HDMapInterface # HDMapInterface from localization
)

class PathPlannerComponent:
    def __init__(self, config: dict, hd_map_interface: HDMapInterface):
        self.config = config
        self.hd_map = hd_map_interface
        print("PathPlannerComponent: Initialized.")

    def plan_path(self, current_pose: LocalizationInfo, scene_info: PerceptionOutput,
                  behavioral_predictions: BehavioralPredictionOutput) -> PlannedPath:
        # Placeholder for Path Planning (Fig 2)
        print(f"PathPlannerComponent: Planning path from {current_pose.position}")
        # Uses HD map, current pose, scene (static obstacles from perception), dynamic agent predictions
        # Example: Simple path straight ahead
        waypoints = []
        start_x, start_y, _ = current_pose.position
        for i in range(1, 11): # 10 waypoints ahead
            waypoints.append((start_x + i * 2.0, start_y)) # 2 meters apart
        return PlannedPath(timestamp=current_pose.timestamp, waypoints=waypoints)

class DecisionMakerComponent:
    def __init__(self, config: dict):
        self.config = config
        print("DecisionMakerComponent: Initialized.")

    def make_decision(self, current_pose: LocalizationInfo, planned_path: PlannedPath,
                      behavioral_predictions: BehavioralPredictionOutput, scene_info: PerceptionOutput) -> ManeuverDecision:
        # Placeholder for Decision-making (Fig 2)
        # Uses path, predictions, localization, (and potentially direct perception like traffic lights)
        print(f"DecisionMakerComponent: Making decision based on path with {len(planned_path.waypoints)} waypoints.")
        # Example: Default to lane keep
        return ManeuverDecision(
            timestamp=current_pose.timestamp,
            chosen_maneuver="LANE_KEEP",
            target_speed_kph=50.0,
            lead_vehicle_id=None
        )

class ActionPlannerComponent:
    def __init__(self, config: dict):
        self.config = config
        print("ActionPlannerComponent: Initialized.")

    def plan_action(self, current_pose: LocalizationInfo, decision: ManeuverDecision, planned_path: PlannedPath) -> ActionCommand:
        # Placeholder for Action Planning (Fig 2) - generates velocity/angle
        print(f"ActionPlannerComponent: Planning action for maneuver '{decision.chosen_maneuver}'")
        # Example: Simple action based on decision
        target_velocity_mps = decision.target_speed_kph / 3.6
        # Basic steering towards the first waypoint if path is available
        target_steering_rad = 0.0
        if planned_path.waypoints:
            # Simplified: Aim for first waypoint (needs proper controller)
            dx = planned_path.waypoints[0][0] - current_pose.position[0]
            dy = planned_path.waypoints[0][1] - current_pose.position[1]
            # Angle based on arctan - very basic, needs vehicle orientation
            # target_steering_rad = math.atan2(dy, dx) - current_pose.orientation_quaternion (yaw component)
            # For now, keep it simple
            if dy > 0.1 : target_steering_rad = 0.05 # Slight left
            elif dy < -0.1: target_steering_rad = -0.05 # Slight right

        return ActionCommand(
            timestamp=current_pose.timestamp,
            target_velocity_mps=target_velocity_mps,
            target_steering_angle_rad=target_steering_rad
        )

class PlanningModule:
    def __init__(self, config: dict, hd_map_path: str,
                 input_queues: Dict[str, queue.Queue], # {"localization": q_loc, "prediction": q_pred, "perception": q_perc}
                 output_queue_control: queue.Queue):
        self.config = config
        self.hd_map = HDMapInterface(hd_map_path) # Re-use or pass instance

        self.path_planner = PathPlannerComponent(config.get("path_planner", {}), self.hd_map)
        self.decision_maker = DecisionMakerComponent(config.get("decision_maker", {}))
        self.action_planner = ActionPlannerComponent(config.get("action_planner", {}))

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

                print(f"PlanningModule: Processing with Loc_ts={self._latest_localization.timestamp}, Pred_ts={self._latest_prediction.timestamp}, Perc_ts={self._latest_perception.timestamp}")

                # 1. Path Planning
                planned_path = self.path_planner.plan_path(
                    self._latest_localization, self._latest_perception, self._latest_prediction
                )

                # 2. Decision Making
                maneuver_decision = self.decision_maker.make_decision(
                    self._latest_localization, planned_path, self._latest_prediction, self._latest_perception
                )

                # 3. Action Planning
                action_command = self.action_planner.plan_action(
                    self._latest_localization, maneuver_decision, planned_path
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