#!/usr/bin/env python3
"""
Path Planning Module
Implements the s-q coordinate system path planning algorithm from the paper
"""

import numpy as np
from typing import List, Tuple, Dict, Optional
from scipy import interpolate
from scipy.optimize import minimize_scalar
import math

class PathPlanner:
    def __init__(self):
        # Path planning parameters from paper
        self.s_end_arc_length = 25.0  # Target planning horizon (meters)
        self.target_lateral_offsets = np.linspace(-2.5, 2.5, 9)  # q_end values
        self.num_points_per_path = 40
        
        # Center line parameters
        self.center_line_params = None
        self.center_line_total_length = 0.0
        
        # Cost function weights (from paper Section 2.3.4)
        self.w_s = 0.4  # Static safety weight
        self.w_c = 0.3  # Comfort weight  
        self.w_d = 0.3  # Dynamic safety weight
        
        # Comfort sub-weights (Equation 14)
        self.alpha = 0.7  # Smoothness weight
        self.beta = 0.3   # Consistency weight
        
        # Safety parameters
        self.max_lateral_acceleration = 3.0  # m/s^2
        self.safety_gain = 0.5
        self.collision_radius = 1.2  # Vehicle collision radius
        
        # Gaussian convolution parameters (Equations 10, 11)
        self.gaussian_sigma = 2.0
        self.convolution_half_width = 5
        
        # Previous path for consistency calculation
        self.previous_path = None
        
    def initialize_center_line(self, waypoints_left: np.ndarray, waypoints_right: np.ndarray):
        """
        Initialize center line from lane waypoints
        Implements Section 2.1 from the paper
        """
        # Calculate center waypoints (Step 1 from paper)
        center_waypoints = (waypoints_left + waypoints_right) / 2.0
        
        if len(center_waypoints) < 4:
            raise ValueError("Need at least 4 waypoints for cubic spline")
        
        # Fit cubic spline with arc length parameterization
        self.center_line_params = self._construct_arc_length_spline(center_waypoints)
        self.center_line_total_length = self.center_line_params['total_length']
        
    def _construct_arc_length_spline(self, waypoints: np.ndarray) -> Dict:
        """
        Construct cubic spline parameterized by arc length
        Implements the center line construction algorithm from Section 2.1
        """
        # Fit initial parametric cubic spline
        distances = np.cumsum(np.sqrt(np.sum(np.diff(waypoints, axis=0)**2, axis=1)))
        distances = np.insert(distances, 0, 0) / distances[-1]
        
        tck, u = interpolate.splprep([waypoints[:, 0], waypoints[:, 1]], 
                                   u=distances, s=0, k=3)
        
        # Calculate arc length parameterization
        num_points = 1000
        u_fine = np.linspace(0, 1, num_points)
        x_fine, y_fine = interpolate.splev(u_fine, tck)
        
        # Calculate cumulative arc lengths
        dx = np.diff(x_fine)
        dy = np.diff(y_fine)
        ds = np.sqrt(dx**2 + dy**2)
        arc_lengths = np.concatenate([[0], np.cumsum(ds)])
        
        return {
            'tck': tck,
            'u_fine': u_fine,
            'x_fine': x_fine,
            'y_fine': y_fine,
            'arc_lengths': arc_lengths,
            'total_length': arc_lengths[-1]
        }
    
    def generate_path_candidates(self, vehicle_state: Dict) -> List[Dict]:
        """
        Generate path candidates using s-q coordinate system
        Implements Section 2.2 from the paper
        """
        if self.center_line_params is None:
            return []
        
        # Extract vehicle state
        vehicle_pos = np.array([vehicle_state['x'], vehicle_state['y']])
        vehicle_heading = vehicle_state['heading']
        
        # Localize on center line (Section 2.2.1)
        s_start, q_start = self._localize_on_center_line(vehicle_pos)
        
        # Calculate heading difference (delta_h_start)
        center_line_tangent = self._get_center_line_tangent_angle(s_start)
        delta_h_start = vehicle_heading - center_line_tangent
        
        # Normalize angle difference
        while delta_h_start > math.pi:
            delta_h_start -= 2 * math.pi
        while delta_h_start < -math.pi:
            delta_h_start += 2 * math.pi
        
        path_candidates = []
        
        # Generate path for each target lateral offset (Section 2.2.2)
        for q_end in self.target_lateral_offsets:
            # Solve cubic coefficients (Equation 5)
            a, b, c = self._solve_cubic_coefficients(
                s_start, q_start, s_start + self.s_end_arc_length, 
                q_end, delta_h_start
            )
            
            # Convert to cartesian coordinates (Section 2.2.3)
            path_cartesian = self._convert_sq_to_cartesian(
                s_start, a, b, c, q_start
            )
            
            if len(path_cartesian) > 0:
                path_candidates.append({
                    'path': path_cartesian,
                    'coefficients': (a, b, c),
                    's_start': s_start,
                    'q_start': q_start,
                    'q_end': q_end,
                    'delta_h_start': delta_h_start
                })
        
        return path_candidates
    
    def _localize_on_center_line(self, vehicle_pos: np.ndarray) -> Tuple[float, float]:
        """
        Localize vehicle position on center line using quadratic minimization + Newton's method
        Implements reference [41] algorithm as mentioned in Section 2.2.1
        Returns s (arc length) and q (lateral offset)
        """
        center_line = self.center_line_params
        
        # Initial approximation: find closest point
        min_dist = float('inf')
        closest_idx = 0
        
        for i, (x, y) in enumerate(zip(center_line['x_fine'], center_line['y_fine'])):
            dist = np.linalg.norm(vehicle_pos - np.array([x, y]))
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # Initial s estimate
        s_initial = center_line['arc_lengths'][closest_idx]
        
        # Newton's method refinement for accurate localization
        s_current = s_initial
        tolerance = 1e-6
        max_iterations = 10
        
        for iteration in range(max_iterations):
            # Get center line point and derivatives at current s
            point, tangent = self._get_center_line_point_and_tangent(s_current)
            
            # Vector from center line point to vehicle
            delta_pos = vehicle_pos - point
            
            # Objective: minimize squared distance to center line
            # f(s) = ||P_vehicle - P_centerline(s)||^2
            # f'(s) = -2 * (P_vehicle - P_centerline(s)) · P'_centerline(s)
            f_prime = -2.0 * np.dot(delta_pos, tangent)
            
            # Check convergence
            if abs(f_prime) < tolerance:
                break
            
            # Get second derivative for Newton's method
            # Approximate f''(s) using finite differences
            eps = 1e-4
            s_plus = min(s_current + eps, center_line['total_length'])
            s_minus = max(s_current - eps, 0.0)
            
            point_plus, tangent_plus = self._get_center_line_point_and_tangent(s_plus)
            point_minus, tangent_minus = self._get_center_line_point_and_tangent(s_minus)
            
            delta_pos_plus = vehicle_pos - point_plus
            delta_pos_minus = vehicle_pos - point_minus
            
            f_prime_plus = -2.0 * np.dot(delta_pos_plus, tangent_plus)
            f_prime_minus = -2.0 * np.dot(delta_pos_minus, tangent_minus)
            
            f_double_prime = (f_prime_plus - f_prime_minus) / (2 * eps)
            
            # Newton's method update
            if abs(f_double_prime) > 1e-8:
                s_new = s_current - f_prime / f_double_prime
                s_new = max(0.0, min(s_new, center_line['total_length']))
            else:
                # Fallback to gradient descent
                s_new = s_current - 0.1 * f_prime
                s_new = max(0.0, min(s_new, center_line['total_length']))
            
            # Check for convergence
            if abs(s_new - s_current) < tolerance:
                break
                
            s_current = s_new
        
        # Final s value
        s_start = s_current
        
        # Calculate lateral offset q with proper sign
        final_point, tangent = self._get_center_line_point_and_tangent(s_start)
        to_vehicle = vehicle_pos - final_point
        
        # Normal vector pointing left (perpendicular to tangent)
        normal = np.array([-tangent[1], tangent[0]])
        q_start = np.dot(to_vehicle, normal)
        
        return s_start, q_start
    
    def _solve_cubic_coefficients(self, s_start: float, q_start: float, 
                                s_end: float, q_end: float, 
                                delta_h_start: float) -> Tuple[float, float, float]:
        """
        Solve cubic polynomial coefficients for lateral offset function
        Implements boundary conditions from Equation (5) in tech.md
        
        Boundary conditions:
        q(s_start) = q_start
        q(s_end) = q_end  
        q'(s_start) = tan(delta_h_start)
        q'(s_end) = 0
        
        Cubic form: q(s) = a*(s-s_start)^3 + b*(s-s_start)^2 + c*(s-s_start) + q_start
        """
        ds = s_end - s_start
        
        if abs(ds) < 1e-6:
            return 0.0, 0.0, 0.0
        
        # From boundary conditions:
        # c = q'(s_start) = tan(delta_h_start)
        c = math.tan(delta_h_start)
        
        # Set up 2x2 linear system for a and b:
        # From q(s_end) = q_end:
        #   a*ds^3 + b*ds^2 + c*ds + q_start = q_end
        #   => a*ds^3 + b*ds^2 = q_end - q_start - c*ds
        #
        # From q'(s_end) = 0:
        #   3*a*ds^2 + 2*b*ds + c = 0
        #   => 3*a*ds^2 + 2*b*ds = -c
        
        try:
            # Matrix form: [3*ds^2  2*ds ] [a] = [-c                    ]
            #              [ds^3    ds^2] [b]   [q_end - q_start - c*ds]
            A = np.array([
                [3*ds**2, 2*ds],
                [ds**3, ds**2]
            ])
            
            B = np.array([
                -c,
                q_end - q_start - c*ds
            ])
            
            # Check condition number for numerical stability
            cond_num = np.linalg.cond(A)
            if cond_num > 1e12:
                # Matrix is ill-conditioned, use simplified approach
                print(f"Warning: Ill-conditioned matrix (cond={cond_num:.2e}), using fallback")
                # Fallback: simple polynomial fitting
                a = (q_end - q_start - c*ds) / (ds**3) - (2*c) / (3*ds**2)
                b = -1.5*a*ds - c/ds
            else:
                a, b = np.linalg.solve(A, B)
                
        except np.linalg.LinAlgError as e:
            print(f"Linear algebra error in cubic coefficient solve: {e}")
            # Fallback: use simple quadratic approximation
            a = 0.0
            # Quadratic: q(s) = b*(s-s_start)^2 + c*(s-s_start) + q_start
            # From q(s_end) = q_end: b*ds^2 + c*ds + q_start = q_end
            b = (q_end - q_start - c*ds) / (ds**2) if abs(ds) > 1e-6 else 0.0
        
        return a, b, c
    
    def _convert_sq_to_cartesian(self, s_start: float, a: float, b: float, c: float, 
                               q_start: float) -> np.ndarray:
        """
        Convert s-q path to cartesian coordinates using proper integration
        Implements coordinate transformation from Section 2.2.3 using Equations (6), (7), (8) from tech.md
        
        Equations:
        - Equation (6): dx/ds = A*cos(h), dy/ds = A*sin(h), dh/ds = A*j
        - Equation (7): j = (B/A) * [(1-q*j0)*d²q/ds² + j0*(dq/ds)²] / A²
        - Equation (8): A = sqrt((dq/ds)² + (1-q*j0)²), B = sgn(1-q*j0)
        """
        path_points = []
        
        # Initial conditions from vehicle state
        s_current = s_start
        ds = self.s_end_arc_length / self.num_points_per_path
        
        # Get initial cartesian position and heading from center line
        initial_center_point, initial_tangent = self._get_center_line_point_and_tangent(s_start)
        initial_heading = math.atan2(initial_tangent[1], initial_tangent[0])
        
        # Initialize integration variables
        current_x = initial_center_point[0] + q_start * (-initial_tangent[1])  # Add initial lateral offset
        current_y = initial_center_point[1] + q_start * initial_tangent[0]
        current_heading = initial_heading
        
        # Add initial point
        path_points.append(np.array([current_x, current_y]))
        
        # Integrate along the path using Euler integration (Equation 6)
        for i in range(1, self.num_points_per_path + 1):
            s_current = s_start + i * ds
            
            # Ensure we don't exceed center line bounds
            if s_current > self.center_line_total_length:
                s_current = self.center_line_total_length
            
            # Calculate q(s) and its derivatives at current s
            s_rel = s_current - s_start
            q = a * s_rel**3 + b * s_rel**2 + c * s_rel + q_start
            dq_ds = 3*a * s_rel**2 + 2*b * s_rel + c
            d2q_ds2 = 6*a * s_rel + 2*b
            
            # Get center line properties at s_current
            center_point, tangent_vector = self._get_center_line_point_and_tangent(s_current)
            j0 = self._get_center_line_curvature(s_current)
            
            # Calculate A and B according to Equation (8)
            term = 1 - q * j0
            A = math.sqrt(dq_ds**2 + term**2)
            B = 1.0 if term >= 0 else -1.0
            
            # Calculate path curvature j according to Equation (7)
            if A > 1e-6:
                # j = (B/A) * [(1-q*j0)*d²q/ds² + j0*(dq/ds)²] / A²
                numerator = term * d2q_ds2 + j0 * dq_ds**2
                j = (B/A) * numerator / (A**2)
            else:
                j = 0.0
            
            # Integration step using Equation (6): dx/ds = A*cos(h), dy/ds = A*sin(h), dh/ds = A*j
            dx_ds = A * math.cos(current_heading)
            dy_ds = A * math.sin(current_heading)
            dh_ds = A * j
            
            # Euler integration step
            current_x += dx_ds * ds
            current_y += dy_ds * ds
            current_heading += dh_ds * ds
            
            # Normalize heading angle
            while current_heading > math.pi:
                current_heading -= 2 * math.pi
            while current_heading < -math.pi:
                current_heading += 2 * math.pi
            
            # Add integrated point to path
            path_points.append(np.array([current_x, current_y]))
            
            # Break if we've reached the end of center line
            if s_current >= self.center_line_total_length:
                break
        
        return np.array(path_points)
    
    def _get_center_line_point_and_tangent(self, s: float) -> Tuple[np.ndarray, np.ndarray]:
        """Get center line point and tangent vector at arc length s"""
        center_line = self.center_line_params
        
        # Find corresponding u parameter
        if s <= 0:
            idx = 0
        elif s >= center_line['total_length']:
            idx = len(center_line['arc_lengths']) - 1
        else:
            idx = np.searchsorted(center_line['arc_lengths'], s)
            if idx >= len(center_line['u_fine']):
                idx = len(center_line['u_fine']) - 1
        
        u = center_line['u_fine'][idx]
        
        # Get point coordinates
        x, y = interpolate.splev(u, center_line['tck'])
        point = np.array([x, y])
        
        # Get tangent vector
        dx_du, dy_du = interpolate.splev(u, center_line['tck'], der=1)
        magnitude = math.sqrt(dx_du**2 + dy_du**2)
        
        if magnitude > 1e-6:
            tangent = np.array([dx_du / magnitude, dy_du / magnitude])
        else:
            tangent = np.array([1.0, 0.0])
        
        return point, tangent
    
    def _get_center_line_tangent_angle(self, s: float) -> float:
        """Get tangent angle at arc length s"""
        _, tangent_vector = self._get_center_line_point_and_tangent(s)
        return math.atan2(tangent_vector[1], tangent_vector[0])
    
    def _get_center_line_tangent_vector(self, s: float) -> np.ndarray:
        """Get tangent vector at arc length s"""
        _, tangent_vector = self._get_center_line_point_and_tangent(s)
        return tangent_vector
    
    def _get_center_line_curvature(self, s: float) -> float:
        """Get center line curvature at arc length s (simplified)"""
        center_line = self.center_line_params
        
        # Find corresponding u parameter
        if s <= 0:
            idx = 0
        elif s >= center_line['total_length']:
            idx = len(center_line['arc_lengths']) - 1
        else:
            idx = np.searchsorted(center_line['arc_lengths'], s)
            if idx >= len(center_line['u_fine']):
                idx = len(center_line['u_fine']) - 1
        
        u = center_line['u_fine'][idx]
        
        # Calculate curvature using first and second derivatives
        try:
            dx_du, dy_du = interpolate.splev(u, center_line['tck'], der=1)
            d2x_du2, d2y_du2 = interpolate.splev(u, center_line['tck'], der=2)
            
            # Curvature formula: k = |x'y'' - y'x''| / (x'^2 + y'^2)^(3/2)
            numerator = abs(dx_du * d2y_du2 - dy_du * d2x_du2)
            denominator = (dx_du**2 + dy_du**2)**(3/2)
            
            if denominator > 1e-6:
                curvature = numerator / denominator
            else:
                curvature = 0.0
                
        except:
            curvature = 0.0
        
        return curvature
    
    def select_optimal_path(self, path_candidates: List[Dict], 
                          obstacles: List, lane_boundaries: Optional[List] = None) -> Optional[Dict]:
        """
        Select optimal path by minimizing total cost function
        Implements Section 2.3 from the paper
        """
        if not path_candidates:
            return None
        
        min_cost = float('inf')
        optimal_path = None
        
        for candidate in path_candidates:
            # Calculate total cost (Equation 23)
            total_cost = self._calculate_total_cost(candidate, obstacles, lane_boundaries)
            
            if total_cost < min_cost:
                min_cost = total_cost
                optimal_path = candidate
        
        # Update previous path for next iteration
        if optimal_path is not None:
            self.previous_path = optimal_path
        
        return optimal_path
    
    def _calculate_total_cost(self, path_candidate: Dict, obstacles: List, 
                           lane_boundaries: Optional[List] = None) -> float:
        """
        Calculate total cost function (Equation 23)
        f(ri, a(ri)) = ws*fs(ri) + wc*fc(ri) + wd*fd(ri, a(ri))
        """
        # Static safety cost (Section 2.3.1)
        f_s = self._calculate_static_safety_cost(path_candidate, lane_boundaries)
        
        # Comfort cost (Section 2.3.2)
        f_c = self._calculate_comfort_cost(path_candidate)
        
        # Dynamic safety cost (Section 2.3.3)
        f_d = self._calculate_dynamic_safety_cost(path_candidate, obstacles)
        
        # Normalize costs to [0, 1] range
        f_s_norm = min(1.0, f_s)
        f_c_norm = min(1.0, f_c)
        f_d_norm = min(1.0, f_d)
        
        # Total cost (Equation 23)
        total_cost = self.w_s * f_s_norm + self.w_c * f_c_norm + self.w_d * f_d_norm
        
        return total_cost
    
    def _calculate_static_safety_cost(self, path_candidate: Dict, 
                                    lane_boundaries: Optional[List] = None) -> float:
        """
        Calculate static safety cost using discrete Gaussian convolution
        Implements Equations (10) and (11)
        """
        path = path_candidate['path']
        
        # Create collision check results R[i]
        collision_results = []
        
        for point in path:
            collision_risk = 0.0
            
            # Check collision with lane boundaries
            q_end = path_candidate['q_end']
            if abs(q_end) > 1.5:  # Outside normal lane (assuming 3m lane width)
                if abs(q_end) > 2.0:  # Near lane boundary
                    collision_risk = 0.5  # Solid line crossing
                if abs(q_end) > 2.5:  # Very close to boundary
                    collision_risk = 1.0  # Road edge collision
            
            # Check collision with static obstacles (simplified)
            # In real implementation, use actual obstacle positions
            
            collision_results.append(collision_risk)
        
        if not collision_results:
            return 0.0
        
        # Apply Gaussian convolution (Equations 10, 11)
        N = self.convolution_half_width
        sigma = self.gaussian_sigma
        static_cost = 0.0
        
        for i in range(len(collision_results)):
            gaussian_sum = 0.0
            for k in range(-N, N+1):
                if 0 <= i+k < len(collision_results):
                    # Discrete inverse Gaussian function (Equation 11)
                    gaussian_weight = (1.0 / (math.sqrt(2*math.pi) * sigma)) * \
                                    math.exp(-(k-i)**2 / (2 * sigma**2))
                    gaussian_sum += gaussian_weight * collision_results[i+k]
            static_cost += gaussian_sum
        
        return static_cost / len(collision_results) if collision_results else 0.0
    
    def _calculate_comfort_cost(self, path_candidate: Dict) -> float:
        """
        Calculate comfort cost (smoothness + consistency)
        Implements Equations (12), (13), and (14)
        """
        path = path_candidate['path']
        
        # Smoothness cost (Equation 12) - integral of curvature squared
        smoothness_cost = self._calculate_smoothness_cost(path)
        
        # Consistency cost (Equation 13) - heading difference with previous path
        consistency_cost = self._calculate_consistency_cost(path)
        
        # Total comfort cost (Equation 14)
        comfort_cost = self.alpha * smoothness_cost + self.beta * consistency_cost
        
        return comfort_cost
    
    def _calculate_smoothness_cost(self, path: np.ndarray) -> float:
        """
        Calculate smoothness cost (Equation 12) - integral of curvature squared
        fsm(ri) = ∫ j²(s) ds along the path
        """
        if len(path) < 3:
            return 0.0
        
        total_smoothness = 0.0
        total_length = 0.0
        
        for i in range(1, len(path) - 1):
            # Calculate curvature at each point using three consecutive points
            p1 = path[i-1]
            p2 = path[i]
            p3 = path[i+1]
            
            # Vectors
            v1 = p2 - p1
            v2 = p3 - p2
            
            # Lengths
            len1 = np.linalg.norm(v1)
            len2 = np.linalg.norm(v2)
            
            if len1 > 1e-6 and len2 > 1e-6:
                # Curvature approximation using cross product
                cross_product = np.cross(v1, v2)
                curvature = abs(cross_product) / (len1 * len2 * (len1 + len2))
                
                # Differential arc length
                ds = (len1 + len2) / 2.0
                
                # Add curvature squared times differential length
                total_smoothness += curvature**2 * ds
                total_length += ds
        
        return total_smoothness / total_length if total_length > 0 else 0.0
    
    def _calculate_consistency_cost(self, path: np.ndarray) -> float:
        """
        Calculate consistency cost (Equation 13) - heading difference with previous path
        fco(ri) = (1/Loverlap) ∫ |hpre(s) - hi(s)| ds
        """
        if self.previous_path is None or len(path) < 2:
            return 1.0  # High consistency cost if no previous path
        
        prev_path = self.previous_path.get('path', [])
        if len(prev_path) < 2:
            return 1.0
        
        # Calculate overlap region (simplified: use minimum length)
        min_length = min(len(path), len(prev_path))
        if min_length < 2:
            return 1.0
        
        total_heading_diff = 0.0
        total_length = 0.0
        
        for i in range(min_length - 1):
            # Current path heading
            if i < len(path) - 1:
                current_vec = path[i+1] - path[i]
                current_heading = math.atan2(current_vec[1], current_vec[0])
            else:
                current_heading = 0.0
            
            # Previous path heading  
            if i < len(prev_path) - 1:
                prev_vec = prev_path[i+1] - prev_path[i]
                prev_heading = math.atan2(prev_vec[1], prev_vec[0])
            else:
                prev_heading = 0.0
            
            # Heading difference (normalize to [-π, π])
            heading_diff = current_heading - prev_heading
            while heading_diff > math.pi:
                heading_diff -= 2 * math.pi
            while heading_diff < -math.pi:
                heading_diff += 2 * math.pi
            
            # Differential length
            ds = np.linalg.norm(current_vec) if i < len(path) - 1 else 0.0
            
            total_heading_diff += abs(heading_diff) * ds
            total_length += ds
        
        return total_heading_diff / total_length if total_length > 0 else 0.0
    
    def _calculate_dynamic_safety_cost(self, path_candidate: Dict, obstacles: List) -> float:
        """
        Calculate dynamic safety cost for moving obstacles
        Implements Equations (15), (16), and (22)
        fd(ri, a(ri)) = |a(ri)| * (Ds(ri) - Lf)
        """
        path = path_candidate['path']
        
        if not obstacles or len(path) < 2:
            return 0.0
        
        # Find potential collision points with moving obstacles
        collision_distance = self._find_closest_collision_distance(path, obstacles)
        
        if collision_distance is None or collision_distance <= 0:
            return 0.0  # No collision or already passed collision point
        
        # Calculate required acceleration (Equations 15, 16)
        Ds = collision_distance  # Distance to collision point
        Lc = self.collision_radius  # Collision radius
        Lf = 3.0  # Following distance (simplified)
        
        if Ds <= Lc:
            return 1000.0  # Very high cost for immediate collision
        
        # Time to collision (Equation 15)
        # t(ri) = (Ds(ri) - Lc) / v0
        current_speed = 10.0  # Assume 10 m/s current speed (should come from vehicle state)
        if current_speed <= 0:
            current_speed = 1.0  # Avoid division by zero
        
        time_to_collision = (Ds - Lc) / current_speed
        
        # Required acceleration (Equation 16)
        # a(ri) = 2(Lc - Lf) * v0² / (Ds(ri) - Lc)²
        if abs(Ds - Lc) < 1e-6:
            required_acceleration = 1000.0  # Very high acceleration needed
        else:
            required_acceleration = 2 * (Lc - Lf) * current_speed**2 / (Ds - Lc)**2
        
        # Dynamic safety cost (Equation 22)
        # fd(ri, a(ri)) = |a(ri)| * (Ds(ri) - Lf)
        dynamic_cost = abs(required_acceleration) * max(0, Ds - Lf)
        
        return dynamic_cost
    
    def _find_closest_collision_distance(self, path: np.ndarray, obstacles: List) -> Optional[float]:
        """Find distance to closest potential collision with moving obstacles"""
        if not obstacles or len(path) < 2:
            return None
        
        min_distance = float('inf')
        found_collision = False
        
        for obstacle in obstacles:
            # Extract obstacle information (simplified)
            obs_pos = obstacle.get('position', [0, 0])
            obs_radius = obstacle.get('radius', 1.0)
            
            # Check each path segment for collision
            path_distance = 0.0
            for i in range(len(path) - 1):
                # Distance from path start to current segment
                segment_start_dist = path_distance
                
                # Check collision with obstacle
                p1 = path[i]
                p2 = path[i+1]
                
                # Distance from obstacle to line segment
                segment_vec = p2 - p1
                segment_len = np.linalg.norm(segment_vec)
                
                if segment_len > 1e-6:
                    # Point to line distance calculation
                    obs_to_p1 = np.array(obs_pos) - p1
                    proj_length = np.dot(obs_to_p1, segment_vec) / segment_len
                    proj_length = max(0, min(segment_len, proj_length))
                    
                    closest_point = p1 + (proj_length / segment_len) * segment_vec
                    distance_to_obs = np.linalg.norm(np.array(obs_pos) - closest_point)
                    
                    # Check if collision occurs
                    if distance_to_obs < (obs_radius + self.collision_radius):
                        collision_distance = segment_start_dist + proj_length
                        min_distance = min(min_distance, collision_distance)
                        found_collision = True
                
                path_distance += segment_len
        
        return min_distance if found_collision else None
        
        curvature_squared_sum = 0.0
        
        for i in range(1, len(path)-1):
            # Calculate curvature at point i using three consecutive points
            p1, p2, p3 = path[i-1], path[i], path[i+1]
            
            # Vectors
            v1 = p2 - p1
            v2 = p3 - p2
            
            # Calculate curvature
            if np.linalg.norm(v1) > 1e-6 and np.linalg.norm(v2) > 1e-6:
                # Cross product magnitude
                cross_product = v1[0]*v2[1] - v1[1]*v2[0]
                # Curvature approximation
                curvature = abs(cross_product) / (np.linalg.norm(v1) * np.linalg.norm(v2))
                curvature_squared_sum += curvature**2
        
        # Approximate integral (multiply by path segment length)
        path_length = self.s_end_arc_length
        smoothness_cost = curvature_squared_sum * (path_length / max(1, len(path) - 2))
        
        return smoothness_cost
    
    def _calculate_consistency_cost(self, path: np.ndarray) -> float:
        """Calculate consistency cost (Equation 13)"""
        if self.previous_path is None or len(path) < 2:
            return 0.0
        
        previous_path_points = self.previous_path.get('path', [])
        if len(previous_path_points) < 2:
            return 0.0
        
        # Calculate heading differences between current and previous path
        heading_diff_sum = 0.0
        overlap_points = min(len(path), len(previous_path_points)) - 1
        
        for i in range(overlap_points):
            # Current path heading
            if i + 1 < len(path):
                curr_heading = math.atan2(path[i+1][1] - path[i][1], 
                                        path[i+1][0] - path[i][0])
            else:
                continue
            
            # Previous path heading
            if i + 1 < len(previous_path_points):
                prev_heading = math.atan2(previous_path_points[i+1][1] - previous_path_points[i][1],
                                        previous_path_points[i+1][0] - previous_path_points[i][0])
            else:
                continue
            
            # Calculate heading difference
            heading_diff = abs(curr_heading - prev_heading)
            while heading_diff > math.pi:
                heading_diff -= 2 * math.pi
            heading_diff = abs(heading_diff)
            
            heading_diff_sum += heading_diff
        
        # Average heading difference over overlap region
        consistency_cost = heading_diff_sum / max(1, overlap_points)
        
        return consistency_cost
    
    def _calculate_dynamic_safety_cost(self, path_candidate: Dict, obstacles: List) -> float:
        """
        Calculate dynamic safety cost for moving obstacles
        Implements Equation (22)
        """
        if not obstacles:
            return 0.0
        
        path = path_candidate['path']
        
        # Find potential collision with moving obstacles
        collision_distance = None
        min_collision_time = float('inf')
        
        for obstacle in obstacles:
            if obstacle.get('type') != 'dynamic':
                continue
            
            obstacle_pos = obstacle.get('position', np.array([0, 0]))
            obstacle_vel = obstacle.get('velocity', np.array([0, 0]))
            obstacle_size = obstacle.get('size', 0.5)
            
            # Check collision along path
            for i, point in enumerate(path):
                # Predict obstacle position when vehicle reaches this point
                time_to_reach = i * 0.1  # Assume 0.1s per path point
                predicted_obstacle_pos = obstacle_pos + obstacle_vel * time_to_reach
                
                # Check collision
                distance = np.linalg.norm(point - predicted_obstacle_pos)
                collision_threshold = self.collision_radius + obstacle_size + 0.5  # 0.5m safety margin
                
                if distance < collision_threshold:
                    path_distance = i * (self.s_end_arc_length / len(path))
                    if time_to_reach < min_collision_time:
                        min_collision_time = time_to_reach
                        collision_distance = path_distance
                    break
        
        if collision_distance is None:
            return 0.0
        
        # Calculate required acceleration and dynamic cost (Equation 22)
        # Simplified implementation
        L_f = min(5.0, collision_distance)  # Following distance
        required_acceleration = max(0.1, collision_distance / 10.0)  # Simplified
        
        dynamic_cost = required_acceleration * (collision_distance - L_f)
        
        return min(dynamic_cost / 50.0, 1.0)  # Normalize to [0, 1]
