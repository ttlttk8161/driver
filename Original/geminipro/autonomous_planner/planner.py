from typing import List, Tuple, Optional

from .utils.data_structures import VehicleState, Obstacle, MovingObstacle, Waypoint, PathCandidate, CenterLine
from .centerline import construct_center_line
from .path_generation import generate_path_candidates
from .path_selection import select_optimal_path
from .config import PLANNING_DT

class DynamicPathPlanner:
    def __init__(self):
        self.previous_optimal_path: Optional[PathCandidate] = None
        self.center_line_cache: Optional[CenterLine] = None # Cache for center line if waypoints don't change often

    def plan_path(
        self,
        predefined_waypoints: List[Tuple[Waypoint, Waypoint]], # For center line
        current_vehicle_state: VehicleState,
        static_obstacles: List[Obstacle],
        moving_obstacles: List[MovingObstacle],
        road_edges: List[List[Waypoint]], # For collision check
        lane_lines: List[List[Waypoint]]  # For collision check
    ) -> Tuple[Optional[PathCandidate], float, float]:
        """
        Executes one cycle of the dynamic path planning algorithm.
        Returns: (selected_path, target_acceleration, target_speed)
        """

        # 1. Construct Center Line (or use cached if appropriate)
        # For simplicity, reconstruct each time. Caching can be added.
        print("Step 1: Constructing Center Line...")
        center_line = construct_center_line(predefined_waypoints)
        if not center_line.segments:
            print("Failed to construct center line. Aborting planning cycle.")
            # Fallback: e.g., maintain current state or emergency stop
            return None, 0.0, current_vehicle_state.speed 
            
        self.center_line_cache = center_line
        print(f"Center line constructed with {len(center_line.segments)} segments.")
        if center_line.segments:
             print(f"Total length: {center_line.segments[-1].start_s_global + center_line.segments[-1].segment_arc_length:.2f}m")


        # 2. Generate Path Candidates
        print("Step 2: Generating Path Candidates...")
        path_candidates = generate_path_candidates(
            current_vehicle_state,
            center_line
            # Configs are used by default from config.py
        )
        print(f"Generated {len(path_candidates)} path candidates.")
        if not path_candidates:
            print("No path candidates generated. Aborting planning cycle.")
            return None, 0.0, current_vehicle_state.speed


        # 3. Select Optimal Path
        print("Step 3: Selecting Optimal Path...")
        optimal_path, optimal_acceleration, optimal_speed = select_optimal_path(
            path_candidates,
            current_vehicle_state,
            static_obstacles,
            moving_obstacles,
            road_edges,
            lane_lines,
            self.previous_optimal_path
        )

        if optimal_path:
            print(f"Optimal path selected with total cost: {optimal_path.total_cost:.4f}")
            print(f"  Static Safety Cost (raw): {optimal_path.cost_static_safety:.4f}")
            print(f"  Comfortability Cost (raw): {optimal_path.cost_comfortability:.4f}")
            print(f"  Dynamic Safety Cost (raw): {optimal_path.cost_dynamic_safety:.4f}")
            print(f"  Target Acceleration: {optimal_acceleration:.2f} m/s^2, Target Speed: {optimal_speed:.2f} m/s")
            self.previous_optimal_path = optimal_path
        else:
            print("No optimal path could be selected.")
            # Handle fallback behavior, e.g., use previous path with more caution or stop.
            # For now, planner returns None, accel=0, speed=current_speed or decelerate.
            # The select_optimal_path already returns a fallback accel/speed.
            self.previous_optimal_path = None # Clear previous path if current planning failed

        return optimal_path, optimal_acceleration, optimal_speed