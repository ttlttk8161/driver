import math
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.integrate import quad
from typing import List, Tuple

from .utils.data_structures import Waypoint, SplineSegment, CenterLine
from .config import NUM_INTEGRATION_STEPS_ARC_LENGTH

# Helper function to define the integrand for arc length calculation
def _spline_derivative_norm(t, cs_x, cs_y):
    dx_dt = cs_x(t, 1)
    dy_dt = cs_y(t, 1)
    return np.sqrt(dx_dt**2 + dy_dt**2)

def construct_center_line(predefined_waypoints: List[Tuple[Waypoint, Waypoint]]) -> CenterLine:
    """
    Constructs the center line from predefined waypoints.
    Input: predefined_waypoints (list of (waypoint_left, waypoint_right) tuples)
    Output: CenterLine object
    """
    if not predefined_waypoints:
        return CenterLine([], [])

    center_waypoints_xy: List[Waypoint] = []
    for wp_left, wp_right in predefined_waypoints:
        center_x = (wp_left.x + wp_right.x) / 2
        center_y = (wp_left.y + wp_right.y) / 2
        center_waypoints_xy.append(Waypoint(center_x, center_y))

    if len(center_waypoints_xy) < 2:
        # Cannot form a spline with less than 2 points
        # Handle this case, perhaps by returning a simple line segment if exactly 2, or error
        if len(center_waypoints_xy) == 1: # Single point, effectively no path
             return CenterLine([], [])
        # If 2 points, can form a linear spline (or a degenerate cubic)
        # For simplicity, let's assume we have at least 2 for now.
        # Or, one could create a dummy second point very close to the first.
        # This part needs robust handling for edge cases.
        print("Warning: Less than 2 center waypoints, cannot form a proper spline.")
        return CenterLine([], [])


    # Parametrize by a dummy variable 't' (0 to N-1) for initial fitting
    t_params = np.arange(len(center_waypoints_xy))
    x_coords = np.array([wp.x for wp in center_waypoints_xy])
    y_coords = np.array([wp.y for wp in center_waypoints_xy])

    # Fit cubic splines (x(t), y(t))
    # bc_type='natural' sets second derivatives at endpoints to zero, common for paths.
    # or 'clamped' if derivatives are known/estimated.
    try:
        cs_x = CubicSpline(t_params, x_coords, bc_type='natural')
        cs_y = CubicSpline(t_params, y_coords, bc_type='natural')
    except ValueError as e:
        print(f"Error fitting spline: {e}. Ensure enough unique points.")
        # Fallback: create a simple linear representation or return empty
        return CenterLine([], [])


    center_line_segments: List[SplineSegment] = []
    cumulative_arc_lengths_at_nodes: List[float] = [0.0]
    current_global_s = 0.0

    # Iterate through segments defined by the original center_waypoints_xy
    for i in range(len(center_waypoints_xy) - 1):
        t_start_segment = t_params[i]
        t_end_segment = t_params[i+1]

        # Numerically calculate arc length of this segment (t_start_segment to t_end_segment)
        segment_arc_length, _ = quad(
            _spline_derivative_norm, 
            t_start_segment, 
            t_end_segment, 
            args=(cs_x, cs_y),
            limit=NUM_INTEGRATION_STEPS_ARC_LENGTH  # Increased limit for better accuracy
        )
        
        if segment_arc_length < 1e-6: # Effectively zero length segment
            # Potentially skip or handle duplicate points
            if i == len(center_waypoints_xy) - 2: # If it's the last segment and zero length
                 if not cumulative_arc_lengths_at_nodes: # if list is empty due to all zero length segments
                    cumulative_arc_lengths_at_nodes = [0.0] * len(center_waypoints_xy)
                 else: # Fill remaining with the last valid cumulative length
                    last_s = cumulative_arc_lengths_at_nodes[-1]
                    cumulative_arc_lengths_at_nodes.extend([last_s] * (len(center_waypoints_xy) - len(cumulative_arc_lengths_at_nodes)))

            elif not cumulative_arc_lengths_at_nodes : # list empty and not last segment
                cumulative_arc_lengths_at_nodes.append(0.0)
            else : # Not last segment and list not empty
                cumulative_arc_lengths_at_nodes.append(cumulative_arc_lengths_at_nodes[-1])

            continue # Skip creating a SplineSegment for zero-length part


        # To get coefficients ax, bx, cx, dx for s_local parameterization:
        # We need to sample points (x,y) along this arc-length parameterized segment
        # and then fit a new cubic spline to *these* points vs s_local.
        # This is the "ReparameterizeSegmentByArcLength" step.
        
        # For simplicity in this first pass, we'll use the t-parameterized spline
        # and assume its parameter 't' over [t_start_segment, t_end_segment]
        # can be *linearly scaled* to s_local over [0, segment_arc_length].
        # This is an approximation. A more accurate way is to numerically invert s(t).
        # Equation (1) implies ax, bx, ... are for s_local.
        # Let's sample points (s_local, x_val) and (s_local, y_val) and fit.
        num_samples_for_reparam = 10 # More samples for better fit
        s_local_samples = np.linspace(0, segment_arc_length, num_samples_for_reparam)
        
        # We need t values corresponding to these s_local samples.
        # This requires solving s_local = integral(_spline_derivative_norm, t_start_segment, t_current, ...) for t_current
        # This is non-trivial.
        # Simplified approach: Assume t progresses linearly with s_local for this segment for coefficient extraction
        # This is a major simplification for now.
        
        x_samples_for_reparam = []
        y_samples_for_reparam = []

        # Generate samples (s_local_k, x(s_local_k)), (s_local_k, y(s_local_k))
        # by finding t_k such that arc_length(t_start_segment to t_k) = s_local_k
        # and then evaluating x(t_k), y(t_k).
        # For now, let's assume a direct fit to the t-parameterized segment is a placeholder.
        # The coefficients from cs_x.c and cs_y.c are for the 't' parameter.
        # We need to transform them or re-fit.
        # For this initial implementation, we'll store a reference to the t-splines
        # and the t_start/t_end for this segment, and s_local mapping.
        # This is NOT what Eq (1) implies directly. Eq (1) is ALREADY arc-length parameterized.

        # Placeholder: Ideally, re-fit to get coefficients for s_local.
        # For now, we will create a SplineSegment that *evaluates* correctly using the t-spline
        # but the ax, bx, ... coefficients will be placeholders or derived under simplifying assumptions.

        # To get an approximate cubic fit for x(s_local) and y(s_local) for THIS segment:
        # Sample points (x_i, y_i) at various t values within [t_params[i], t_params[i+1]]
        # Calculate their corresponding s_local values.
        # Then fit CubicSpline(s_local_values, x_values) and CubicSpline(s_local_values, y_values).
        
        # This is a complex part. For now, let's assume we obtain these coefficients.
        # Let's define a simplified SplineSegment that uses the original cs_x, cs_y
        # and maps s_local back to t.
        
        # For a proper implementation of Eq (1) structure:
        # 1. Sample points along the t-spline segment: (t_sample, x_sample, y_sample)
        # 2. For each t_sample, calculate its s_local from t_start_segment.
        # 3. We now have (s_local_sample, x_sample) and (s_local_sample, y_sample).
        # 4. Fit new cubic splines: x_reparam = CubicSpline(s_local_samples, x_samples)
        #                           y_reparam = CubicSpline(s_local_samples, y_samples)
        # 5. Extract coefficients from x_reparam.c and y_reparam.c (these are for a single segment).
        #    CubicSpline returns coefficients for polynomials relative to knot points.
        #    So, if s_local_samples are knots, cs.c[k,j] is coeff for (s-s_k)^j.
        #    For a single segment [0, L], knots are [0,L].
        #    ax = c[3], bx = c[2], cx = c[1], dx = c[0] relative to the segment start.
        #    So if s_local_samples are [0, s1, s2, ..., segment_arc_length],
        #    and we fit a spline. The coefficients for the first piece (0 to s1) would be used.
        #    This is tricky with scipy.CubicSpline directly for *one* segment's explicit ax..dx form.
        #    Alternative for a single segment: polyfit of degree 3.

        temp_t_samples = np.linspace(t_start_segment, t_end_segment, num_samples_for_reparam)
        temp_x_samples = cs_x(temp_t_samples)
        temp_y_samples = cs_y(temp_t_samples)
        
        temp_s_local_samples = [0.0]
        for k in range(1, len(temp_t_samples)):
            len_k, _ = quad(_spline_derivative_norm, temp_t_samples[0], temp_t_samples[k], args=(cs_x, cs_y), limit=50)
            temp_s_local_samples.append(len_k)
        temp_s_local_samples = np.array(temp_s_local_samples)
        
        # Ensure s_local samples are unique and sorted for CubicSpline fitting
        unique_s_indices = np.unique(temp_s_local_samples, return_index=True)[1]
        if len(unique_s_indices) < 2: # Need at least 2 unique points to fit
            print(f"Warning: Segment {i} has insufficient unique s_local samples for reparameterization. Skipping.")
            if not cumulative_arc_lengths_at_nodes : cumulative_arc_lengths_at_nodes.append(0.0)
            else : cumulative_arc_lengths_at_nodes.append(cumulative_arc_lengths_at_nodes[-1])
            continue

        s_fit = temp_s_local_samples[unique_s_indices]
        x_fit = temp_x_samples[unique_s_indices]
        y_fit = temp_y_samples[unique_s_indices]

        if len(s_fit) < 4: # Polyfit deg 3 needs at least 4 points
            # Fallback to linear if not enough points for cubic
            deg = len(s_fit) - 1
            if deg < 1 : # Not enough points even for linear
                print(f"Warning: Segment {i} has < 2 unique s_local points. Cannot fit. Skipping.")
                if not cumulative_arc_lengths_at_nodes : cumulative_arc_lengths_at_nodes.append(0.0)
                else : cumulative_arc_lengths_at_nodes.append(cumulative_arc_lengths_at_nodes[-1])
                continue
        else:
            deg = 3
            
        coeffs_x = np.polyfit(s_fit, x_fit, deg)
        coeffs_y = np.polyfit(s_fit, y_fit, deg)

        ax, bx, cx, dx = (coeffs_x if deg==3 else ([0,0] + list(coeffs_x)))[-4:] # Pad with 0 for lower degrees
        ay, by, cy, dy = (coeffs_y if deg==3 else ([0,0] + list(coeffs_y)))[-4:]

        seg = SplineSegment(
            ax=ax, bx=bx, cx=cx, dx=dx,
            ay=ay, by=by, cy=cy, dy=dy,
            segment_arc_length=segment_arc_length,
            start_s_global=current_global_s
        )
        center_line_segments.append(seg)
        current_global_s += segment_arc_length
        cumulative_arc_lengths_at_nodes.append(current_global_s)

    # Fix cumulative_arc_lengths_at_nodes if some segments were skipped
    if len(cumulative_arc_lengths_at_nodes) != len(center_waypoints_xy):
        # This logic might need refinement based on how many points are expected
        # For now, if it's short, pad with the last value.
        # This means the "nodes" don't perfectly match original center_waypoints_xy if segments are skipped.
        # This part is tricky and depends on how downstream code uses cumulative_arc_lengths_at_nodes.
        # It's intended to map original waypoint indices to s-values.
        # For now, we'll ensure it has the right length, even if values are approximate due to skips.
        if cumulative_arc_lengths_at_nodes:
            last_s = cumulative_arc_lengths_at_nodes[-1]
            needed = len(center_waypoints_xy) - len(cumulative_arc_lengths_at_nodes)
            cumulative_arc_lengths_at_nodes.extend([last_s] * needed)
        else: # All segments skipped, e.g. only one waypoint
            cumulative_arc_lengths_at_nodes = [0.0] * len(center_waypoints_xy)


    return CenterLine(center_line_segments, cumulative_arc_lengths_at_nodes)