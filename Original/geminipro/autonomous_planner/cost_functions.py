import math
import numpy as np
from typing import List, Tuple

from .utils.data_structures import PathCandidate, VehicleState, Obstacle, MovingObstacle, CenterLine, Waypoint
from .utils.math_utils import sqrt, abs, normalize_angle
from .config import (
    SIGMA_R, CONVOLUTION_WINDOW_HALF_SIZE,
    COLLISION_VALUE_NONE, COLLISION_VALUE_LANE_LINE_DRIVING,
    COLLISION_VALUE_LANE_LINE_OPPOSITE, COLLISION_VALUE_OBSTACLE_OR_EDGE,
    ALPHA, BETA, LC, L0, VSIGN, JALJMAX, KSAFE, VCURVE, PLANNING_DT,
    MAX_ACCELERATION, MAX_DECELERATION
)

# --- Static Safety Cost (Section 2.3.1) ---

def perform_collision_check_for_path(
    path_candidate: PathCandidate,
    static_obstacles: List[Obstacle],
    road_edges: List[List[Waypoint]], # List of polylines representing edges
    lane_lines: List[List[Waypoint]] # List of polylines for lane lines (with type info needed)
) -> List[float]:
    """
    Performs collision check for a single path candidate against static elements.
    Returns a list of risk values (R_values) for segments of the path, or for the path as a whole.
    For simplicity, let's return a single risk value for the entire path.
    A more detailed implementation would discretize the path and check each point/segment.
    """
    # TODO: Implement detailed geometric collision checking.
    # This is a complex part involving geometry (e.g., shapely library or custom math).
    # For now, a placeholder that checks if any point is too close to an obstacle.
    # The paper implies R[k] is a single value per *path candidate rk*, not per point on path.
    
    max_risk_on_path = COLLISION_VALUE_NONE
    path_points = path_candidate.points_cartesian
    
    if not path_points:
        return [COLLISION_VALUE_OBSTACLE_OR_EDGE] # Invalid path

    for point in path_points:
        # Check against static obstacles
        for obs in static_obstacles:
            dist_sq = (point.x - obs.position.x)**2 + (point.y - obs.position.y)**2
            # Assuming vehicle radius is also part of LC or a fixed value.
            # Let's say vehicle radius is 1.0m for this check.
            vehicle_radius = 1.0 
            if sqrt(dist_sq) < obs.radius + vehicle_radius:
                max_risk_on_path = max(max_risk_on_path, COLLISION_VALUE_OBSTACLE_OR_EDGE)
                break # Found highest risk, no need to check other obstacles for this point
        if max_risk_on_path == COLLISION_VALUE_OBSTACLE_OR_EDGE:
            break # Path already has max risk

        # TODO: Check against road_edges (COLLISION_VALUE_OBSTACLE_OR_EDGE)
        # TODO: Check against lane_lines (COLLISION_VALUE_LANE_LINE_DRIVING / OPPOSITE)
        # This requires point-to-polyline distance and line type classification.

    # Paper states R[k+i] is a collision check value (0, 0.2, 0.5, 1).
    # This suggests one value per path candidate.
    return [max_risk_on_path] # Returning a list of one for consistency if R is an array of candidate risks


def calculate_static_safety_costs(
    all_path_candidates: List[PathCandidate],
    static_obstacles: List[Obstacle],
    road_edges: List[List[Waypoint]],
    lane_lines: List[List[Waypoint]]
) -> List[float]:
    """Calculates static safety cost for all candidates using Gaussian convolution."""
    
    num_candidates = len(all_path_candidates)
    if num_candidates == 0:
        return []

    # R_values: list of collision check results, one for each path candidate
    R_values = []
    for r_i in all_path_candidates:
        # perform_collision_check_for_path returns a list, take the first (and only) element.
        R_values.append(perform_collision_check_for_path(r_i, static_obstacles, road_edges, lane_lines)[0])

    static_safety_costs = [0.0] * num_candidates
    
    # Gaussian kernel g_i[k] from Eq (11) in overallmath.md.
    # The paper's Eq 11: g_i[k] = (1/sqrt(2pi)sigma) * exp(-(k-i)^2 / (2sigma^2))
    # The sum in Eq 10 is sum_{k=-N}^{N} g_i[k] * R[k+i]
    # This is non-standard. A standard convolution would be sum g[j]R[i-j] or g[i-j]R[j].
    # The g_i[k] * R[k+i] means the kernel g_i shifts with i (center of kernel is at i),
    # and it multiplies R values relative to i.
    # If k ranges from -N to N:
    # k=-N: g_i[-N] * R[i-N]
    # k=0:  g_i[0]  * R[i]   (kernel centered at i, multiplies R[i])
    # k=N:  g_i[N]  * R[i+N]

    # Let's try to implement Eq (10) and (11) as literally as possible.
    # N = CONVOLUTION_WINDOW_HALF_SIZE
    # k ranges from -N to N.
    
    # Precompute Gaussian values for g_i[k] style kernel (kernel values depend on distance from center `i`)
    # Kernel g[delta_idx] where delta_idx = k-i.
    # So, g_kernel[k_prime] where k_prime = k-i is the index relative to center of kernel.
    # The sum is over k, and g_i[k] means kernel centered at 'i' evaluated at 'k'.
    # It seems the paper meant a standard convolution kernel g[j] (independent of i)
    # and fs(ri) = sum_{j=-N}^{N} g[j] * R[i-j] (if R is indexed appropriately)
    # OR fs(ri) = sum_{j=-N}^{N} g[j] * R[i+j] (if kernel is centered at 0, R indexed relative to i)

    # The pseudocode in tech.md (Optimal Path Selection) suggests:
    # gaussian_kernel[k_offset + N_conv] * collision_val where collision_val = R[i + k_offset]
    # k_offset ranges from -N_conv to N_conv. N_conv = CONVOLUTION_WINDOW_HALF_SIZE.
    # This is sum_{k'=-N}^{N} G[k'] * R[i+k'], where G is a fixed kernel. This makes more sense.

    N_conv = CONVOLUTION_WINDOW_HALF_SIZE
    gaussian_kernel_vals = []
    kernel_sum = 0.0
    for k_offset in range(-N_conv, N_conv + 1):
        val = (1.0 / (sqrt(2 * math.pi) * SIGMA_R)) * math.exp(-(k_offset**2) / (2 * SIGMA_R**2))
        gaussian_kernel_vals.append(val)
        kernel_sum += val
    
    # Normalize kernel (optional, but good practice for weighted average)
    if kernel_sum > 1e-6:
        gaussian_kernel_vals = [val / kernel_sum for val in gaussian_kernel_vals]

    for i in range(num_candidates):
        fs_ri = 0.0
        for k_idx, k_offset in enumerate(range(-N_conv, N_conv + 1)):
            R_index = i + k_offset
            collision_val_at_R_index: float
            if 0 <= R_index < num_candidates:
                collision_val_at_R_index = R_values[R_index]
            else:
                # Out of bounds for R index (k+i+N) means collision check is 1.0 (paper desc)
                collision_val_at_R_index = COLLISION_VALUE_OBSTACLE_OR_EDGE 
            
            fs_ri += gaussian_kernel_vals[k_idx] * collision_val_at_R_index
        static_safety_costs[i] = fs_ri
        all_path_candidates[i].cost_static_safety = fs_ri # Store it

    return static_safety_costs


# --- Comfortability Cost (Section 2.3.2) ---

def calculate_smoothness_cost(path_candidate: PathCandidate) -> float:
    """Calculates smoothness cost f_sm (Eq. 12)."""
    # Integral of j_i^2(s) ds
    # Approximate by summing j_i^2 * delta_s_path for each segment
    f_sm = 0.0
    if len(path_candidate.points_cartesian) < 2:
        return 0.0 # Or a large penalty for invalid path

    for k in range(len(path_candidate.points_cartesian) - 1):
        p1 = path_candidate.points_cartesian[k]
        p2 = path_candidate.points_cartesian[k+1]
        delta_s_path_segment = sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)
        
        # Use average curvature or curvature at start of segment
        curvature_at_k = path_candidate.curvature_values[k]
        f_sm += curvature_at_k**2 * delta_s_path_segment
    return f_sm

def calculate_consistency_cost(
    current_path_candidate: PathCandidate,
    previous_path: PathCandidate = None # Optimal path from previous step
) -> float:
    """Calculates consistency cost f_co (Eq. 13)."""
    if previous_path is None or not previous_path.points_cartesian or not current_path_candidate.points_cartesian:
        return 0.0 # No previous path, or one of them is empty

    # Find overlapping s-interval.
    # Assume s_values are global s from center line.
    # This requires s_values to be populated in PathCandidate.
    
    # For simplicity, let's assume paths are roughly aligned and compare point-wise up to min length.
    # A more robust method uses the s-values to find genuine overlap.
    # Here, we'll just use the Cartesian points and headings.

    f_co = 0.0
    num_overlap_points = 0
    total_delta_s_overlap = 0.0

    # Determine overlap based on s_values (longitudinal progression along centerline)
    # This is a simplified overlap check. A more robust method would interpolate.
    
    # Find s_start and s_end for both paths
    s_start_current = current_path_candidate.s_values[0]
    s_end_current = current_path_candidate.s_values[-1]
    s_start_prev = previous_path.s_values[0]
    s_end_prev = previous_path.s_values[-1]

    overlap_s_start = max(s_start_current, s_start_prev)
    overlap_s_end = min(s_end_current, s_end_prev)

    if overlap_s_end <= overlap_s_start: # No overlap or single point overlap
        return 1.0 # High cost if no overlap, or could be 0 if consistency not critical

    sum_abs_delta_h_ds = 0.0
    
    # Iterate through the current path's points that fall within the s-overlap region
    for i in range(len(current_path_candidate.s_values) -1):
        s_curr_pt = current_path_candidate.s_values[i]
        s_curr_next_pt = current_path_candidate.s_values[i+1]
        
        # Consider the segment [s_curr_pt, s_curr_next_pt]
        # If this segment (or part of it) is in the overlap region
        seg_overlap_start = max(s_curr_pt, overlap_s_start)
        seg_overlap_end = min(s_curr_next_pt, overlap_s_end)

        if seg_overlap_end > seg_overlap_start:
            delta_s_seg_overlap = seg_overlap_end - seg_overlap_start
            total_delta_s_overlap += delta_s_seg_overlap

            # Get heading_current for this segment (e.g., at midpoint of seg_overlap)
            s_mid_overlap = (seg_overlap_start + seg_overlap_end) / 2.0
            
            # Interpolate heading for current path at s_mid_overlap
            # For simplicity, use heading_values[i]
            h_current = current_path_candidate.heading_values[i] 
            
            # Interpolate heading for previous path at s_mid_overlap
            h_prev = 0.0
            found_prev_s = False
            for j in range(len(previous_path.s_values) -1):
                if previous_path.s_values[j] <= s_mid_overlap < previous_path.s_values[j+1]:
                    # Linear interpolation of heading
                    s1, s2 = previous_path.s_values[j], previous_path.s_values[j+1]
                    h1, h2 = previous_path.heading_values[j], previous_path.heading_values[j+1]
                    if abs(s2-s1) > 1e-6:
                        h_prev = h1 + (h2-h1)*(s_mid_overlap-s1)/(s2-s1)
                    else:
                        h_prev = h1
                    found_prev_s = True
                    break
            if not found_prev_s and previous_path.s_values: # s_mid_overlap might be at the very end
                if abs(s_mid_overlap - previous_path.s_values[-1]) < 1e-3:
                     h_prev = previous_path.heading_values[-1]
                     found_prev_s = True
            
            if found_prev_s:
                abs_delta_h = abs(normalize_angle(h_current - h_prev))
                sum_abs_delta_h_ds += abs_delta_h * delta_s_seg_overlap
            else:
                # If s_mid_overlap not found in previous_path s_values range (should not happen if overlap_s is correct)
                # Add a penalty or skip. For now, skip.
                pass
                
    if total_delta_s_overlap > 1e-3: # Avoid division by zero
        f_co = sum_abs_delta_h_ds / total_delta_s_overlap
    else:
        # No significant overlap, or paths are very short.
        # Assign a high cost, or 0 if consistency is not critical in this case.
        # Let's assume high cost if current path has length but no overlap.
        f_co = 1.0 if current_path_candidate.get_length() > 1.0 else 0.0

    return f_co


def calculate_comfortability_cost(
    path_candidate: PathCandidate,
    previous_path: PathCandidate = None
) -> float:
    """Calculates total comfortability cost f_c (Eq. 14)."""
    f_sm = calculate_smoothness_cost(path_candidate)
    f_co = calculate_consistency_cost(path_candidate, previous_path)
    
    fc = ALPHA * f_sm + BETA * f_co
    path_candidate.cost_comfortability = fc # Store it
    return fc


# --- Dynamic Safety Cost (Section 2.3.3) ---

def calculate_dynamic_safety_cost_and_accel(
    path_candidate: PathCandidate,
    vehicle_state: VehicleState,
    moving_obstacles: List[MovingObstacle],
    static_safety_cost_normalized: float # Used for v_r
) -> Tuple[float, float]: # Returns (f_d, a_ri)
    """
    Calculates dynamic safety cost f_d and required/adjusted acceleration a_ri. (Eq. 15-22)
    """
    # TODO: Implement prediction of moving obstacles' trajectories.
    # For now, assume obstacles move straight with current velocity.
    
    # Check if path is "available" (not colliding with static elements).
    # This check should ideally be done before calling, or use path_candidate.cost_static_safety
    # Assume if static_safety_cost is very high, path is unavailable.
    # The paper says "for those available paths r_i".
    # For simplicity, let's assume this check is done by the caller.

    min_Ds_ri = float('inf') # Shortest distance to a potential collision point on path_candidate
    closest_colliding_obstacle = None

    for m_obs in moving_obstacles:
        # Predict m_obs trajectory for PLANNING_DT or slightly longer
        # For each point on path_candidate, calculate time to reach it for ego vehicle
        # Calculate m_obs position at that time. Check for collision.
        
        # Simplified: Find if any m_obs is "ahead" on the path and might collide.
        # This requires projecting m_obs onto the path, or more complex TTC calculation.
        # For a very basic placeholder:
        for i, p_point in enumerate(path_candidate.points_cartesian):
            dist_to_m_obs_sq = (p_point.x - m_obs.position.x)**2 + (p_point.y - m_obs.position.y)**2
            # This is a very crude check, doesn't consider timing or trajectory.
            # Let's assume we have a function that gives us Ds(ri) if collision is predicted.
            # For now, if any obstacle is within a certain raw distance, consider it for Ds.
            # This needs a proper spatio-temporal collision check.

            # Placeholder for Ds(ri): distance along path to point nearest to m_obs's current pos
            if sqrt(dist_to_m_obs_sq) < 10.0 : # If obstacle is somewhat close
                # Calculate arc length to p_point along path_candidate
                current_Ds = 0.0
                for k in range(i):
                    seg_p1 = path_candidate.points_cartesian[k]
                    seg_p2 = path_candidate.points_cartesian[k+1]
                    current_Ds += sqrt((seg_p2.x-seg_p1.x)**2 + (seg_p2.y-seg_p1.y)**2)
                
                if current_Ds < min_Ds_ri:
                    min_Ds_ri = current_Ds
                    closest_colliding_obstacle = m_obs # Simplistic: just first one found "close"
                break # Considered this obstacle for now


    calculated_a_ri = 0.0 # Default: cruise (0 acceleration)
    
    if closest_colliding_obstacle and min_Ds_ri < float('inf'):
        Ds_ri = min_Ds_ri
        if Ds_ri > LC: # Collision is beyond critical range LC
            # Eq (17) Lf
            Lf = L0 if L0 < Ds_ri else Ds_ri # Paper: L0 >= Ds(ri) => Lf=L0. So L0 < Ds(ri) => Lf=L0
                                           # others => Lf = Ds(ri). This means if L0 >= Ds(ri), Lf = L0.
                                           # If L0 < Ds(ri), Lf = L0.
                                           # This means Lf = L0, unless L0 is too large, then Lf = Ds(ri)?
                                           # The "others" case implies Lf = Ds(ri) when L0 < Ds(ri)
                                           # This doesn't make sense. Lf should be <= Ds(ri).
                                           # Let's interpret Eq 17 as: Lf = min(L0, Ds(ri))
                                           # No, the pseudocode for overallmath.md has:
                                           # Lf = L0 if L0 >= Ds(ri) (this means Lf can be > Ds(ri), problematic for following)
                                           # Lf = Ds(ri) if L0 < Ds(ri) (this means Lf = Ds(ri) if L0 is small)
                                           # This is confusing. Let's use a more standard interpretation: Lf = min(L0, Ds_ri - safety_margin_Lf)
                                           # Or, as per a common understanding of following: Lf is the desired following distance, L0.
                                           # But it should not exceed Ds_ri.
                                           # Let's use Lf = L0, but ensure Lc > Lf for meaningful acceleration.
            Lf = L0

            if Lc <= Lf: # Cannot achieve following distance if collision range is smaller/equal
                # This implies we should try to maintain Lc or more.
                # If this happens, a_ri calculation with (Lc-Lf) might be problematic.
                # Consider this an edge case or parameter tuning issue.
                # For now, proceed, a_ri might become positive (accelerate to create Lf gap from Lc).
                pass


            # Eq (16) - Intended version: a(ri) = 2*(Lc - Lf)*v0^2 / (Ds(ri) - Lc)^2
            # v0 is current vehicle speed
            v0 = vehicle_state.speed
            denominator_a_ri = (Ds_ri - LC)**2
            if abs(denominator_a_ri) > 1e-6 and v0 > 0.1: # Avoid division by zero and handle v0=0
                # (Lc - Lf) negative means Lf > Lc. We want to be further than collision range.
                # If Lc - Lf > 0, it means we are trying to get closer than Lf from Lc (problematic).
                # This formula is for creating/maintaining a gap. If Lc-Lf is negative, a_ri is negative (decelerate).
                calculated_a_ri = (2 * (LC - Lf) * v0**2) / denominator_a_ri
            else: # Cannot calculate a_ri meaningfully
                calculated_a_ri = MAX_DECELERATION # Emergency brake if too close or v0 is zero but collision imminent
        
        else: # Ds_ri <= LC, collision is imminent or already happening
            calculated_a_ri = MAX_DECELERATION # Emergency brake
    else:
        # No dynamic obstacles of concern, aim for cruise or target speed
        # calculated_a_ri remains 0 (or could be small positive to reach target speed)
        pass

    # TODO: Gaussian smoothing of calculated_a_ri over time (requires history)
    smoothed_a_ri = calculated_a_ri # Placeholder

    # Speed limit calculation (Eq 18-20)
    max_abs_curvature_on_path = 0.0
    if path_candidate.curvature_values:
        max_abs_curvature_on_path = max(abs(j) for j in path_candidate.curvature_values)

    vj_ri = float('inf')
    if max_abs_curvature_on_path > 1e-6:
        vj_ri = sqrt(abs(JALJMAX) / max_abs_curvature_on_path) # Eq 19
    
    # fs_ri (static_safety_cost) should be normalized [0,1] for Eq 20
    # Assuming static_safety_cost_normalized is passed in.
    vr_ri = (1 - KSAFE * static_safety_cost_normalized**2) * VCURVE # Eq 20
    
    v_limit_ri = min(vj_ri, vr_ri, VSIGN) # Eq 18

    # Adjust acceleration for speed limit (Eq 21)
    # a(ri) <= (v_limit_ri^2 - v0^2) / (2 * (Ds_path - Lf_eff))
    # Ds_path is the total length of the candidate path.
    # Lf_eff is an effective distance, could be 0 if not following.
    # This limits acceleration to not exceed v_limit_ri at end of path.
    adjusted_a_ri = smoothed_a_ri
    path_total_length = path_candidate.get_length()
    
    # If Lf is not applicable (no leading vehicle for this path candidate)
    # the denominator (Ds(ri)-Lf) for Eq 21 should be just path length.
    # The Ds(ri) in Eq 21 is the distance over which acceleration is applied.
    # Let's take it as path_total_length.
    accel_dist_for_limit = path_total_length 
    if accel_dist_for_limit > 1e-3:
        max_accel_by_speed_limit = (v_limit_ri**2 - vehicle_state.speed**2) / (2 * accel_dist_for_limit)
        
        if adjusted_a_ri > max_accel_by_speed_limit:
            adjusted_a_ri = max_accel_by_speed_limit
    
    # Ensure accel is within vehicle's physical limits
    adjusted_a_ri = max(MAX_DECELERATION, min(MAX_ACCELERATION, adjusted_a_ri))


    # Dynamic safety cost (Eq 22)
    # fd(ri, a(ri)) = |a(ri)| * (Ds(ri) - Lf)
    # Ds(ri) here is distance to collision point if one exists, otherwise path length?
    # The paper is a bit ambiguous. If no collision, Ds(ri)-Lf could be path_length - 0.
    # If there was a collision point:
    fd_ri = 0.0
    if closest_colliding_obstacle and min_Ds_ri < float('inf'):
        # Lf again for Eq 22. Use L0 as per typical following.
        Lf_for_cost = L0 
        effective_dist_for_cost = min_Ds_ri - Lf_for_cost
        if effective_dist_for_cost > 0: # Only apply cost if there's a positive distance to manage
            fd_ri = abs(adjusted_a_ri) * effective_dist_for_cost
        # If effective_dist_for_cost is negative (Lf is large, or Ds_ri is small),
        # it means we are already too close or Lf is too ambitious. Cost could be high.
        # For now, only positive contribution.
    else:
        # No specific dynamic threat, cost relates to acceleration effort over path length.
        # fd_ri = abs(adjusted_a_ri) * path_total_length # Alternative if no specific Ds(ri)
        fd_ri = 0.0 # Or small cost for any non-zero acceleration.


    path_candidate.cost_dynamic_safety = fd_ri # Store it
    path_candidate.associated_acceleration = adjusted_a_ri
    return fd_ri, adjusted_a_ri