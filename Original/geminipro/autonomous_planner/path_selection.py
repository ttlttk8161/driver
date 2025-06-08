import math
import numpy as np
from typing import List, Tuple

from .utils.data_structures import PathCandidate, VehicleState, Obstacle, MovingObstacle, CenterLine, Waypoint
from .cost_functions import (
    calculate_static_safety_costs,
    calculate_comfortability_cost,
    calculate_dynamic_safety_cost_and_accel
)
from .config import WS, WC, WD, PLANNING_DT #, VSIGN (already in cost_functions)

def normalize_costs(costs: List[float], fill_nan_with=1.0) -> List[float]:
    """Normalizes a list of costs to the [0, 1] range."""
    if not costs:
        return []
    
    # Handle cases where all costs are the same (e.g., all zero)
    min_cost = min(costs)
    max_cost = max(costs)
    
    if abs(max_cost - min_cost) < 1e-6: # All costs are effectively the same
        # If all are zero, normalized is zero. If all non-zero same, could be 0.5 or 0.
        # Let's make them 0 if they are all same and close to zero, else 0.5.
        return [0.0 if abs(c) < 1e-6 else 0.5 for c in costs]

    normalized = [(c - min_cost) / (max_cost - min_cost) for c in costs]
    
    # Replace NaN if any (e.g. if max_cost == min_cost was not handled perfectly, though it should be)
    return [fill_nan_with if math.isnan(c) else c for c in normalized]


def select_optimal_path(
    path_candidates: List[PathCandidate],
    vehicle_state: VehicleState,
    static_obstacles: List[Obstacle],
    moving_obstacles: List[MovingObstacle],
    road_edges: List[List[Waypoint]],
    lane_lines: List[List[Waypoint]],
    previous_path: PathCandidate = None,
    # center_line: CenterLine # Not directly used here, but cost functions might need it implicitly
) -> Tuple[PathCandidate | None, float, float]:
    """
    Selects the optimal path from candidates. (Section 2.3.4)
    Returns (optimal_path, optimal_acceleration, optimal_speed)
    """
    if not path_candidates:
        return None, 0.0, vehicle_state.speed

    num_candidates = len(path_candidates)

    # 1. Calculate all raw costs
    # Static safety costs are calculated for all paths together due to convolution
    raw_static_safety_costs = calculate_static_safety_costs(
        path_candidates, static_obstacles, road_edges, lane_lines
    )
    # Other costs are per-path
    raw_comfortability_costs = []
    raw_dynamic_safety_costs = []
    associated_accelerations = []

    # Normalize static_safety_costs for use in vr_ri calculation (Eq 20)
    # This creates a slight circular dependency if fs_ri itself is used raw then normalized later.
    # For Eq 20, fs_ri is used. Let's assume it's the raw, unnormalized fs(ri) from convolution.
    # The final total cost function uses normalized fs, fc, fd.
    
    # For vr_ri, we need fs_ri (raw). For total cost, we need normalized fs_ri.
    # Let's assume calculate_static_safety_costs populates path_candidate.cost_static_safety with raw values.
    
    # Need a version of normalized static costs just for vr_ri calculation
    # Or, assume fs(ri) in Eq.20 is already somewhat scaled by its nature.
    # The paper states: "cost functions are normalized into values between 0 and 1 before summation" for Eq.23.
    # For Eq.20: "fs(ri) is static safety cost of ri". It's ambiguous if this is normalized.
    # Let's assume for Eq.20, we use a temporarily normalized fs_ri if its range is large.
    # For simplicity, use raw fs_ri from path_candidate.cost_static_safety for vr_ri,
    # and then normalize all costs for the final sum.

    # Pre-normalize static costs if they are used as input to dynamic safety calculation
    # (e.g. for vr_ri calculation). A bit of a look-ahead.
    temp_normalized_static_costs = normalize_costs([pc.cost_static_safety for pc in path_candidates])


    for i, r_i in enumerate(path_candidates):
        # Static safety cost is already calculated and stored in r_i.cost_static_safety by calculate_static_safety_costs
        
        fc_i = calculate_comfortability_cost(r_i, previous_path) # Stores in r_i
        raw_comfortability_costs.append(fc_i)
        
        # For dynamic safety, it needs the normalized static safety cost of *this* path for vr_ri
        fs_ri_normalized_for_vr = temp_normalized_static_costs[i]
        
        fd_i, accel_i = calculate_dynamic_safety_cost_and_accel(
            r_i, vehicle_state, moving_obstacles, fs_ri_normalized_for_vr
        ) # Stores in r_i
        raw_dynamic_safety_costs.append(fd_i)
        associated_accelerations.append(accel_i)

    # 2. Normalize all costs
    # Static costs were already computed and stored in path_candidates by calculate_static_safety_costs
    final_norm_static_costs = normalize_costs([pc.cost_static_safety for pc in path_candidates])
    final_norm_comfort_costs = normalize_costs([pc.cost_comfortability for pc in path_candidates])
    final_norm_dynamic_costs = normalize_costs([pc.cost_dynamic_safety for pc in path_candidates])

    min_total_cost = float('inf')
    optimal_path_candidate: PathCandidate | None = None
    optimal_final_acceleration: float = 0.0
    
    for i in range(num_candidates):
        r_i = path_candidates[i]
        
        # Total cost (Eq. 23)
        total_cost = (
            WS * final_norm_static_costs[i] +
            WC * final_norm_comfort_costs[i] +
            WD * final_norm_dynamic_costs[i]
        )
        r_i.total_cost = total_cost
        
        if total_cost < min_total_cost:
            # Add checks here: e.g. if static cost is too high, reject path
            # For instance, if raw_static_safety_costs[i] > some_threshold (e.g. >0.8 after its own internal scaling)
            # This prevents selecting a path that is "relatively" good among bad options but still unsafe.
            # The paper implies this is handled by R values and convolution naturally.
            
            min_total_cost = total_cost
            optimal_path_candidate = r_i
            optimal_final_acceleration = r_i.associated_acceleration # Already calculated and stored

    if optimal_path_candidate is None: # No valid path found
        # Fallback: e.g. emergency stop or follow previous path with deceleration
        print("Warning: No optimal path candidate found.")
        return None, MAX_DECELERATION, max(0, vehicle_state.speed + MAX_DECELERATION * PLANNING_DT)

    # 4. Determine optimal speed for the chosen path
    # The optimal_acceleration is already constrained by v_limit_ri from dynamic cost fn.
    # Optimal speed can be the target speed at the end of the planning horizon,
    # or simply v_limit_ri of the chosen path.
    # Let's use: v_final = v0 + a_opt * dt (capped by v_limit_ri)
    
    # Re-calculate v_limit for the chosen optimal path
    # (as it was done inside calculate_dynamic_safety_cost_and_accel)
    max_abs_curv_opt = 0.0
    if optimal_path_candidate.curvature_values:
        max_abs_curv_opt = max(abs(j) for j in optimal_path_candidate.curvature_values)
    
    vj_opt = float('inf')
    if max_abs_curv_opt > 1e-6:
        vj_opt = sqrt(abs(JALJMAX) / max_abs_curv_opt)
    
    # Find the index of the optimal path to get its normalized static cost
    opt_idx = path_candidates.index(optimal_path_candidate)
    fs_opt_normalized = final_norm_static_costs[opt_idx] # Use the finally normalized one
    vr_opt = (1 - KSAFE * fs_opt_normalized**2) * VCURVE
    
    v_limit_optimal = min(vj_opt, vr_opt, VSIGN)

    # Optimal speed is the speed limit of this path, or speed after acceleration
    # The goal is to *reach* v_limit_optimal if possible, or maintain current speed if already optimal.
    # The associated_acceleration is calculated to respect this v_limit_optimal.
    # So, the speed at the end of the planning horizon is v0 + a*dt,
    # and this should be <= v_limit_optimal.
    
    # For "appropriate speed", it could be the v_limit itself, or the speed profile.
    # Let's use v_limit_optimal as the target speed for this path.
    optimal_target_speed = v_limit_optimal
    
    # And ensure current acceleration doesn't overshoot it too quickly
    # This is already handled by Eq 21 logic in dynamic_safety_cost.
    # So optimal_final_acceleration is already "appropriate".

    return optimal_path_candidate, optimal_final_acceleration, optimal_target_speed