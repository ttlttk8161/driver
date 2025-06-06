from typing import NamedTuple, List, Any, Tuple, Optional, Dict
from dataclasses import dataclass, field

class SensorData(NamedTuple):
    timestamp: float
    lidar_data: Optional[Any]  # Raw or pre-processed LiDAR point cloud
    vision_data: Optional[Any] # Raw or pre-processed image/video frames

class DetectedObject(NamedTuple):
    id: int
    type: str
    position_3d: Tuple[float, float, float] # In a common coordinate frame
    bounding_box_2d: Optional[Tuple[int, int, int, int]] # x_min, y_min, x_max, y_max
    velocity: Optional[Tuple[float, float, float]]
    confidence: float
    tracked_history: Optional[List[Tuple[float, float, float]]] # Historical positions
    # predicted_trajectory_short_term: Optional[List[Tuple[float, float, float]]] # For PerceptionPredictionComponent

class LaneMarking(NamedTuple):
    points: List[Tuple[float, float]] # 2D points defining the lane
    type: str # e.g., solid, dashed, center
    confidence: float

class TrafficSignInfo(NamedTuple):
    type: str # e.g., stop_sign, speed_limit_60
    position_3d: Tuple[float, float, float]
    confidence: float

class PerceptionOutput(NamedTuple):
    timestamp: float
    detected_objects: List[DetectedObject]
    lane_markings: List[LaneMarking]
    drivable_area_mask: Optional[Any] # e.g., a binary mask image
    traffic_signs: List[TrafficSignInfo]
    semantic_segmentation_map: Optional[Any]
    instance_segmentation_map: Optional[Any]
    depth_map: Optional[Any]
    optical_flow_map: Optional[Any]
    scene_flow_map: Optional[Any]
    # Data that might be useful for a separate SLAM/Localization module if not fully handled within perception
    raw_features_for_localization: Optional[Any]
    white_mask: Optional[Any] = None  # HSV 차선 감지에서 생성된 흰색 마스크
    yellow_mask: Optional[Any] = None  # HSV 차선 감지에서 생성된 노란색 마스크

   
# class LocalizationInfo(NamedTuple):
#     timestamp: float
#     position: Tuple[float, float, float]  # x, y, z in global frame
#     orientation_quaternion: Tuple[float, float, float, float]  # w, x, y, z
#     velocity_vector: Tuple[float, float, float] # vx, vy, vz in global frame
#     covariance_matrix: Optional[Any] # Uncertainty

# class PredictedTrajectory(NamedTuple):
#     object_id: int
#     probability: float
#     path_points: List[Tuple[float, float, float]]  # Sequence of (x, y, time_offset)

# class BehavioralPredictionOutput(NamedTuple):
#     timestamp: float
#     predicted_trajectories: List[PredictedTrajectory] # For various objects in the scene

class PlannedPath(NamedTuple):
    timestamp: float
    waypoints: List[Tuple[float, float]] # Sequence of (x,y) waypoints

class ManeuverDecision(NamedTuple):
    timestamp: float
    chosen_maneuver: str  # e.g., "LANE_KEEP", "LANE_CHANGE_LEFT", "OVERTAKE"
    target_speed_kph: float
    lead_vehicle_id: Optional[int]

@dataclass
class ActionCommand:
    timestamp: float
    target_velocity_mps: float
    target_steering_angle_rad: float # Or curvature
    planning_results: Optional[Dict] = field(default=None)

class ControlActuatorCommands(NamedTuple):
    timestamp: float
    steering_command: float # e.g., angle, torque
    throttle_command: float # e.g., percentage, target acceleration
    brake_command: float    # e.g., percentage, target deceleration