import math
import numpy as np
from typing import List, Tuple

from .utils.data_structures import VehicleState, CenterLine, PathCandidate, Waypoint
from .utils.math_utils import cos, sin, tan, atan2, sqrt, sign, solve_linear_system, normalize_angle
from .config import S_END_ARC_LENGTH, TARGET_LATERAL_OFFSETS_Q_END, NUM_POINTS_PER_PATH

def localize_on_center_line(
    vehicle_pos: Waypoint, 
    center_line: CenterLine
) -> Tuple[float, float, float]:
    """
    Localizes the vehicle on the center line. (Section 2.2.1)
    Finds s_start, q_start, and center_line_tangent_at_s_start.
    This is an optimization problem: find s that minimizes distance |vehicle_pos - center_line.get_cartesian_coords(s)|.
    Output: (s_start_global, q_start, center_line_tangent_at_s_start)
    """
    # TODO: Implement a robust localization algorithm.
    # This typically involves iterating along the center line or using an optimization method.
    # For now, a simplified search:
    min_dist_sq = float('inf')
    s_start_global = 0.0
    
    # Search along the entire length of the center line
    # Total length can be found from the last segment's start_s_global and its arc_length
    total_length = 0
    if center_line.segments:
        last_seg = center_line.segments[-1]
        total_length = last_seg.start_s_global + last_seg.segment_arc_length
    
    if total_length == 0: # No center line or zero length
        # Fallback if center line is empty or zero length
        cl_tangent = 0.0 # Or vehicle's current heading if no reference
        q_start = 0.0 # Assuming on the "line"
        # Check if vehicle_pos matches the single point of the center line
        if center_line.segments and len(center_line.segments) ==1 and center_line.segments[0].segment_arc_length == 0:
            cl_point = center_line.segments[0].get_coords(0)
            q_start = math.sqrt((vehicle_pos.x - cl_point.x)**2 + (vehicle_pos.y - cl_point.y)**2)
            # Tangent is undefined, use a default or vehicle heading
        return 0.0, q_start, cl_tangent


    num_search_points = 100 # Discretization for search
    for s_test in np.linspace(0, total_length, num_search_points):
        cl_point = center_line.get_cartesian_coords(s_test)
        dist_sq = (vehicle_pos.x - cl_point.x)**2 + (vehicle_pos.y - cl_point.y)**2
        if dist_sq < min_dist_sq:
            min_dist_sq = dist_sq
            s_start_global = s_test

    # Calculate q_start (signed lateral offset)
    cl_point_at_s_start = center_line.get_cartesian_coords(s_start_global)
    center_line_tangent_at_s_start = center_line.get_tangent_angle(s_start_global)
    
    dx = vehicle_pos.x - cl_point_at_s_start.x
    dy = vehicle_pos.y - cl_point_at_s_start.y
    
    # q_start is the perpendicular distance.
    # Project (dx, dy) onto the normal vector of the tangent.
    # Normal vector: (-sin(tangent), cos(tangent))
    q_start = dx * (-sin(center_line_tangent_at_s_start)) + dy * cos(center_line_tangent_at_s_start)
    
    return s_start_global, q_start, center_line_tangent_at_s_start


def solve_cubic_coefficients_for_q_s(
    s_start: float, q_start: float, 
    s_end: float, q_end: float, 
    delta_h_start_rad: float
) -> Tuple[float, float, float]:
    """
    Solves for cubic coefficients a, b, c for q(s) (Eq. 5 based).
    q(s) = a*(s-s_start)^3 + b*(s-s_start)^2 + c*(s-s_start) + q_start
    Boundary conditions:
    q(s_start) = q_start (inherent)
    q(s_end) = q_end
    q'(s_start) = tan(delta_h_start)
    q'(s_end) = 0
    """
    # Let ds = s_curr - s_start. Then q(ds) = a*ds^3 + b*ds^2 + c*ds + q_start
    # q'(ds) = 3*a*ds^2 + 2*b*ds + c
    
    c_coeff = tan(delta_h_start)
    
    ds_end = s_end - s_start

    if abs(ds_end) < 1e-6: # s_start and s_end are too close
        # Cannot form a meaningful cubic. Assume straight or default.
        # This case should ideally be handled by ensuring s_end_arc_length is sufficiently large.
        # If ds_end is 0, q_end must be q_start, and c_coeff must be 0 for q'(s_end)=0.
        # If these conflict, it's an ill-posed problem.
        # For now, return coefficients that result in q(s) = q_start if ds_end is zero.
        # (meaning a=0, b=0, and c must be 0 if q'(s_end)=0 which might conflict with tan(delta_h_start))
        # This is a tricky edge case. A robust solution might try a lower-order polynomial or fail.
        # For now, if ds_end is very small, let's assume no change in q beyond initial slope.
        # For q'(s_end)=0, if ds_end is tiny, then c must be tiny.
        # Let's return a=0, b=0, c=c_coeff, implying a linear extrapolation,
        # acknowledging this might violate q'(s_end)=0.
        # A more correct handling if ds_end is ~0:
        if abs(q_end - q_start) > 1e-3 : # q_start and q_end are different but s_start and s_end are same
            print(f"Warning: s_start ~ s_end ({s_start}, {s_end}) but q_start != q_end ({q_start}, {q_end}). Ill-posed.")
            # Force a straight line in q for the tiny ds interval
            # This may not satisfy all boundary conditions perfectly.
            return 0.0, 0.0, (q_end - q_start) / (ds_end + 1e-6) # Approximate c

        # if q_start == q_end and ds_end == 0
        # We need c = tan(delta_h_start) and 3ads^2+2bds+c=0.
        # So c=0 implies tan(delta_h_start)=0.
        # If tan(delta_h_start) is not 0, it's unsolvable for a cubic with these constraints.
        # For simplicity, assume a=0, b=0 if ds_end is near zero.
        return 0.0, 0.0, c_coeff


    # System of equations for a and b:
    # a*ds_end^3 + b*ds_end^2 = q_end - q_start - c_coeff*ds_end
    # 3*a*ds_end^2 + 2*b*ds_end = -c_coeff
    
    matrix_A = np.array([
        [ds_end**3, ds_end**2],
        [3*ds_end**2, 2*ds_end]
    ])
    vector_b = np.array([
        q_end - q_start - c_coeff*ds_end,
        -c_coeff
    ])
    
    try:
        solution = solve_linear_system(matrix_A, vector_b)
        a_coeff, b_coeff = solution[0], solution[1]
    except Exception as e: # Catch potential errors from solve_linear_system if singular
        print(f"Error solving for cubic coefficients: {e}. ds_end={ds_end}. Using a=0, b=0.")
        # Fallback if solving fails (e.g., ds_end is too small leading to singularity)
        a_coeff = 0.0
        # If a=0, then b*ds_end^2 = q_end - q_start - c_coeff*ds_end
        # and 2*b*ds_end = -c_coeff
        if abs(ds_end) > 1e-6:
            b_coeff_from_eq2 = -c_coeff / (2 * ds_end)
            # Check consistency:
            # b_coeff_from_eq1 = (q_end - q_start - c_coeff*ds_end) / (ds_end**2)
            # If inconsistent, this fallback is problematic. For now, use one.
            b_coeff = b_coeff_from_eq2
        else:
            b_coeff = 0.0 # ds_end is zero, implies c must be 0 if b is finite.
                          # This means tan(delta_h_start) = 0. Highly constrained.
    
    return a_coeff, b_coeff, c_coeff


def generate_path_candidates(
    vehicle_state: VehicleState,
    center_line: CenterLine,
    s_end_arc_length_config: float = S_END_ARC_LENGTH,
    target_lateral_offsets_q_end_config: List[float] = TARGET_LATERAL_OFFSETS_Q_END,
    num_points_per_path_config: int = NUM_POINTS_PER_PATH
) -> List[PathCandidate]:
    """
    Generates path candidates. (Section 2.2)
    """
    list_of_path_candidates: List[PathCandidate] = []

    if not center_line.segments:
        print("Warning: Center line is empty. Cannot generate paths.")
        return []

    # 2.2.1 Localization on the center line
    s_start_global, q_start, cl_tangent_at_s_start = localize_on_center_line(
        vehicle_state.position, center_line
    )

    delta_h_start = normalize_angle(vehicle_state.heading_angle - cl_tangent_at_s_start)
    s_end_global = s_start_global + s_end_arc_length_config

    # 2.2.2 Path candidates generation in s-q
    for q_end in target_lateral_offsets_q_end_config:
        current_path_candidate = PathCandidate(
            s_start_global=s_start_global,
            q_start=q_start
        )

        a, b, c = solve_cubic_coefficients_for_q_s(
            s_start_global, q_start, s_end_global, q_end, delta_h_start
        )
        current_path_candidate.coeff_a = a
        current_path_candidate.coeff_b = b
        current_path_candidate.coeff_c = c

        # 2.2.3 Coordinates conversion
        # Start integration from current vehicle state, not from projected center line point + offset
        current_x = vehicle_state.position.x
        current_y = vehicle_state.position.y
        current_h_path = vehicle_state.heading_angle # Path heading starts at vehicle heading

        # Store the very first point
        current_path_candidate.points_cartesian.append(Waypoint(current_x, current_y))
        current_path_candidate.s_values.append(s_start_global)
        current_path_candidate.q_values.append(q_start) # q at s_start_global
        current_path_candidate.heading_values.append(current_h_path)
        # Curvature at start can be tricky; it depends on the generated path itself.
        # For now, let's calculate it based on the formulas for the first step.

        actual_s_end_arc_length = s_end_global - s_start_global
        if actual_s_end_arc_length <= 1e-3: # Path length is too small
            print(f"Warning: Path candidate from s={s_start_global} to s={s_end_global} is too short. Skipping conversion for q_end={q_end}.")
            # Potentially add just the start point if needed by selection logic
            # For now, skip adding this candidate if it can't be properly discretized.
            if not current_path_candidate.points_cartesian: # If list is empty
                current_path_candidate.points_cartesian.append(Waypoint(current_x, current_y))
                current_path_candidate.s_values.append(s_start_global)
                current_path_candidate.q_values.append(q_start)
                current_path_candidate.heading_values.append(current_h_path)
                current_path_candidate.curvature_values.append(0.0) # Placeholder
            # list_of_path_candidates.append(current_path_candidate) # Add single point path?
            continue


        delta_s_centerline = actual_s_end_arc_length / (num_points_per_path_config -1) if num_points_per_path_config > 1 else actual_s_end_arc_length


        for i in range(num_points_per_path_config): # Generate num_points, including start
            s_current_cl = s_start_global + delta_s_centerline * i # s along center line
            
            # For the very first point (i=0), values are already set or taken from vehicle_state
            if i == 0:
                # Calculate initial curvature based on the path's properties at s_start_global
                s_offset = s_current_cl - s_start_global # Should be 0 for i=0
                # q_current is q_start
                # dqds_current = c (from solve_cubic_coefficients)
                # d2qds2_current = 2*b (from q''(s) = 6a(s-s_start) + 2b)
                dqds_current = 3*a*s_offset**2 + 2*b*s_offset + c
                d2qds2_current = 6*a*s_offset + 2*b
                j0_current = center_line.get_curvature(s_current_cl)
                
                term_1_minus_qj0 = (1 - q_start * j0_current)
                A_val = sqrt(dqds_current**2 + term_1_minus_qj0**2)
                # B_val = sign(term_1_minus_qj0) # Not used in curvature formula Eq 7 version in overallmath
                
                if abs(A_val) < 1e-6:
                    j_current = 0.0 # Avoid division by zero if A is ~0
                else:
                    # Eq (7) for path curvature j
                    # j = (B/A)j0 + ( (1-qj0)q'' + j0(q')^2 ) / A^2
                    # The B/A * j0 term in Eq 7 seems to differ from some standard Frenet-Serret transformations.
                    # The version in overallmath.md is: j = (B/A)j0 + ( (1-qj0)q'' + j0(q')^2 ) / A^2
                    # Let's use the one from overallmath.md which is from the paper:
                    # j = (B/A) * j0 + ( (1-qj0)d2qds2 + j0*(dqds)^2 ) / A^2
                    # If B = sgn(1-qj0), then (B/A)j0 = sgn(1-qj0)/A * j0
                    # The pseudocode Eq 7 looks like: j = ( (B/A) * ( (1-qj0)d2qds2 + j0(dqds)^2 ) / A^2 ) + ??? NO, it's more complex.
                    # Let's use the formula from overallmath.md PDF:
                    # j = (B/A) * j_0 + ( (1-q*j_0)*q_s_s + j_0 * q_s^2 ) / A^2
                    # where q_s = dq/ds, q_s_s = d^2q/ds^2
                    # B = sgn(1-q*j_0)
                    # A = sqrt(q_s^2 + (1-q*j_0)^2)
                    # This looks like: j = sgn(1-qj0)/A * j0 + ( (1-qj0)q_ss + j0*q_s^2 ) / A^2
                    # This could be a typo in the paper and should be j = (d(theta_centerline + atan(dq/ds / (1-qj0))))/ds_path
                    # Using the provided Eq (7) and (8) from overallmath.md literally:
                    # A = sqrt((dq/ds)^2 + (1 - qj0)^2)
                    # B = sgn(1 - q j0)
                    # j = (B/A) * j0 + ( (1 - qj0) * d2q/ds2 + j0 * (dq/ds)^2 ) / A^2
                    # Note: The pseudocode in tech.md for 2.2 (Path Candidates Generation) uses a different form for j_current:
                    # j_current = (B/A) * ( (1 - q_current * j0_current) * d2qds2_current + j0_current * dqds_current^2 ) / A^2
                    # This form ( (B/A) * STUFF / A^2 ) means B * STUFF / A^3. This seems more plausible.
                    # Let's use the pseudocode's interpretation of Eq 7.

                    numerator_j = (term_1_minus_qj0 * d2qds2_current + j0_current * dqds_current**2)
                    B_val = sign(term_1_minus_qj0)
                    j_current = (B_val / A_val) * numerator_j / (A_val**2) if abs(A_val)>1e-6 else 0.0

                current_path_candidate.curvature_values.append(j_current)
                continue # Skip to next iteration, point 0 is vehicle_state.position

            # --- For i > 0 ---
            s_offset = s_current_cl - s_start_global # (s_current - s_start) for polynomial q(s)
            
            q_current = a*s_offset**3 + b*s_offset**2 + c*s_offset + q_start
            dqds_current = 3*a*s_offset**2 + 2*b*s_offset + c
            d2qds2_current = 6*a*s_offset + 2*b

            j0_current = center_line.get_curvature(s_current_cl) # Curvature of center line at s_current_cl
            
            term_1_minus_qj0 = (1 - q_current * j0_current)
            A_val = sqrt(dqds_current**2 + term_1_minus_qj0**2)
            B_val = sign(term_1_minus_qj0)

            if abs(A_val) < 1e-6: # A is close to zero, path may be ill-defined (e.g. 1-qj0=0 and dqds=0)
                j_current = 0.0 # Avoid division by zero
                # If A is zero, ds_path/ds_centerline is zero. This implies no advancement.
                # This indicates an issue with path parameters or extreme curvature.
                # For now, just set curvature to 0 and don't advance x,y,h.
                # A robust solution might mark this path as invalid.
                dx_ds_path = 0 
                dy_ds_path = 0
                dh_ds_path = 0
            else:
                # Path curvature j, using pseudocode version of Eq (7) which is (B/A) * (num) / A^2 = B*num / A^3
                numerator_j = (term_1_minus_qj0 * d2qds2_current + j0_current * dqds_current**2)
                j_current = (B_val / A_val) * numerator_j / (A_val**2)

                # Eq (6) gives dx/ds_path, dy/ds_path, dh/ds_path
                # where ds is ds_path (arc length along the candidate path)
                # dx/ds_path = cos(h_path)
                # dy/ds_path = sin(h_path)
                # dh/ds_path = j_current (curvature of path candidate)
                # We need to relate ds_path to ds_centerline. ds_path = A * ds_centerline
                # So, dx/ds_centerline = (dx/ds_path) * (ds_path/ds_centerline) = cos(h_path) * A
                dx_d_s_centerline = A_val * cos(current_h_path)
                dy_d_s_centerline = A_val * sin(current_h_path)
                dh_d_s_centerline = A_val * j_current


            # Euler integration using ds_centerline step
            current_x += dx_d_s_centerline * delta_s_centerline
            current_y += dy_d_s_centerline * delta_s_centerline
            current_h_path = normalize_angle(current_h_path + dh_d_s_centerline * delta_s_centerline)
            
            current_path_candidate.points_cartesian.append(Waypoint(current_x, current_y))
            current_path_candidate.s_values.append(s_current_cl)
            current_path_candidate.q_values.append(q_current)
            current_path_candidate.heading_values.append(current_h_path)
            current_path_candidate.curvature_values.append(j_current)

        list_of_path_candidates.append(current_path_candidate)
        
    return list_of_path_candidates