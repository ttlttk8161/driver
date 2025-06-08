#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Standalone Autonomous Driving Test - No ROS Required
This version can run without ROS for testing the path planning algorithms
"""

import numpy as np
import cv2
import time
import threading
from scipy import interpolate

# Mock ROS messages for standalone testing
class MockLaserScan:
    def __init__(self):
        self.ranges = np.random.uniform(0.5, 10.0, 360)
        self.angle_min = -np.pi
        self.angle_max = np.pi
        self.range_min = 0.1
        self.range_max = 12.0

class MockXycarMotor:
    def __init__(self):
        self.angle = 0
        self.speed = 0

class AutonomousDrivingStandalone:
    def __init__(self):
        print("Initializing Standalone Autonomous Driving System...")
        
        # Vehicle state (simulated position and heading)
        self.current_position = np.array([0.0, 0.0])  # Estimated position [x, y]
        self.current_velocity = 10.0  # Estimated velocity (m/s)
        self.current_heading = 0.0  # Current heading in radians
        self.current_pose = None
        self.last_time = time.time()
        
        # Control parameters for Xycar
        self.fix_speed = 10  # Base speed for XycarMotor
        self.max_angle = 50  # Maximum steering angle (degrees)
        self.look_ahead_distance = 3.0  # Look-ahead distance for path following
        
        # Path planning parameters
        self.target_lateral_offsets = np.linspace(-2.0, 2.0, 5)  # Target q values
        self.s_end_arc_length = 20.0  # Path planning horizon
        self.num_points_per_path = 50  # Points per path candidate
        
        # Cost function weights
        self.w_s = 0.4  # Static safety weight
        self.w_c = 0.3  # Comfort weight  
        self.w_d = 0.3  # Dynamic safety weight
        
        # Safety parameters
        self.max_lateral_acceleration = 3.0  # m/s^2
        self.safety_gain = 0.5
        self.following_distance_base = 5.0  # L_0
        
        # Initialize center line
        self.center_line_params = None
        self.previous_path = None
        self.detected_obstacles = []
        
        self._initialize_example_center_line()
        
        # Create mock pose
        class MockPosition:
            def __init__(self, x, y):
                self.x = x
                self.y = y
        
        class MockPose:
            def __init__(self, x, y):
                self.position = MockPosition(x, y)
        
        self.current_pose = MockPose(self.current_position[0], self.current_position[1])
        
        print("Standalone system initialized successfully!")

    def _initialize_example_center_line(self):
        """Initialize center line with example waypoints"""
        # Example waypoints - in real scenario, load from HD map
        waypoints_left = np.array([
            [0, -1], [10, -1], [20, 0], [30, 2], [40, 2], [50, 0]
        ])
        waypoints_right = np.array([
            [0, 1], [10, 1], [20, 2], [30, 4], [40, 4], [50, 2]
        ])
        
        # Construct center line
        self.center_line_params = self.construct_center_line(waypoints_left, waypoints_right)
        print("Center line initialized with example waypoints")

    def construct_center_line(self, waypoints_left, waypoints_right):
        """Construct center line using cubic splines parameterized by arc length"""
        # Calculate center waypoints
        center_waypoints = (waypoints_left + waypoints_right) / 2.0
        
        # Fit cubic spline
        tck, u = interpolate.splprep([center_waypoints[:, 0], center_waypoints[:, 1]], 
                                   s=0, k=3)
        
        # Calculate arc lengths
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
            'arc_lengths': arc_lengths,
            'total_length': arc_lengths[-1],
            'u_fine': u_fine,
            'x_fine': x_fine,
            'y_fine': y_fine
        }

    def generate_path_candidates(self):
        """Generate path candidates using s-q coordinate system"""
        if self.current_pose is None or self.center_line_params is None:
            return []
        
        # Current vehicle position
        vehicle_x = self.current_pose.position.x
        vehicle_y = self.current_pose.position.y
        vehicle_pos = np.array([vehicle_x, vehicle_y])
        
        # Localize on center line
        s_start, q_start = self.localize_on_center_line(vehicle_pos)
        
        # Calculate delta_h_start (angle difference)
        center_line_tangent = self.get_center_line_tangent(s_start)
        delta_h_start = self.current_heading - center_line_tangent
        
        path_candidates = []
        
        # Generate path for each target lateral offset
        for q_end in self.target_lateral_offsets:
            # Solve cubic coefficients
            a, b, c = self.solve_cubic_coefficients(
                s_start, q_start, s_start + self.s_end_arc_length, 
                q_end, delta_h_start
            )
            
            # Convert to cartesian coordinates
            path_cartesian = self.convert_sq_to_cartesian(
                s_start, a, b, c, q_start
            )
            
            path_candidates.append({
                'path': path_cartesian,
                'coefficients': (a, b, c),
                's_start': s_start,
                'q_start': q_start,
                'q_end': q_end
            })
        
        return path_candidates

    def localize_on_center_line(self, vehicle_pos):
        """Localize vehicle position on center line"""
        center_line = self.center_line_params
        
        # Find closest point on center line
        min_dist = float('inf')
        closest_idx = 0
        
        for i, (x, y) in enumerate(zip(center_line['x_fine'], center_line['y_fine'])):
            dist = np.linalg.norm(vehicle_pos - np.array([x, y]))
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # Get s value (arc length)
        s_start = center_line['arc_lengths'][closest_idx]
        
        # Calculate lateral offset q
        closest_point = np.array([center_line['x_fine'][closest_idx], 
                                center_line['y_fine'][closest_idx]])
        
        # Get tangent vector at closest point
        tangent = self.get_center_line_tangent_vector(s_start)
        
        # Calculate lateral offset (signed distance)
        to_vehicle = vehicle_pos - closest_point
        normal = np.array([-tangent[1], tangent[0]])  # Perpendicular to tangent
        q_start = np.dot(to_vehicle, normal)
        
        return s_start, q_start

    def get_center_line_tangent(self, s):
        """Get tangent angle at arc length s"""
        tangent_vector = self.get_center_line_tangent_vector(s)
        return np.arctan2(tangent_vector[1], tangent_vector[0])

    def get_center_line_tangent_vector(self, s):
        """Get tangent vector at arc length s"""
        center_line = self.center_line_params
        
        # Find corresponding u parameter
        idx = np.searchsorted(center_line['arc_lengths'], s)
        if idx >= len(center_line['u_fine']):
            idx = len(center_line['u_fine']) - 1
        
        u = center_line['u_fine'][idx]
        
        # Calculate derivative
        dx_du, dy_du = interpolate.splev(u, center_line['tck'], der=1)
        
        # Normalize
        magnitude = np.sqrt(dx_du**2 + dy_du**2)
        if magnitude > 0:
            return np.array([dx_du / magnitude, dy_du / magnitude])
        else:
            return np.array([1.0, 0.0])

    def solve_cubic_coefficients(self, s_start, q_start, s_end, q_end, delta_h_start):
        """Solve cubic polynomial coefficients for lateral offset function"""
        ds = s_end - s_start
        
        c = np.tan(delta_h_start)
        
        # Solve 2x2 system for a and b
        A = np.array([
            [3*ds**2, 2*ds],
            [ds**3, ds**2]
        ])
        
        B = np.array([
            -c,
            q_end - q_start - c*ds
        ])
        
        try:
            a, b = np.linalg.solve(A, B)
        except np.linalg.LinAlgError:
            # Fallback if matrix is singular
            a, b = 0.0, 0.0
        
        return a, b, c

    def convert_sq_to_cartesian(self, s_start, a, b, c, q_start):
        """Convert s-q path to cartesian coordinates"""
        path_points = []
        
        for i in range(self.num_points_per_path):
            s = s_start + (self.s_end_arc_length / self.num_points_per_path) * i
            
            # Calculate q(s) using cubic polynomial
            ds = s - s_start
            q = a * ds**3 + b * ds**2 + c * ds + q_start
            
            # Get center line point at s
            center_point, tangent_vector = self.get_center_line_point_and_tangent(s)
            
            # Calculate normal vector
            normal_vector = np.array([-tangent_vector[1], tangent_vector[0]])
            
            # Calculate cartesian position
            cartesian_point = center_point + q * normal_vector
            
            path_points.append(cartesian_point)
        
        return np.array(path_points)

    def get_center_line_point_and_tangent(self, s):
        """Get center line point and tangent vector at arc length s"""
        center_line = self.center_line_params
        
        # Find corresponding u parameter
        idx = np.searchsorted(center_line['arc_lengths'], s)
        if idx >= len(center_line['u_fine']):
            idx = len(center_line['u_fine']) - 1
        
        u = center_line['u_fine'][idx]
        
        # Get point coordinates
        x, y = interpolate.splev(u, center_line['tck'])
        point = np.array([x, y])
        
        # Get tangent vector
        tangent = self.get_center_line_tangent_vector(s)
        
        return point, tangent

    def test_path_generation(self):
        """Test path generation functionality"""
        print("\n" + "="*50)
        print("Testing Path Generation")
        print("="*50)
        
        # Test at different vehicle positions
        test_positions = [
            [0, 0],    # Start position
            [5, 0.5],  # Slightly offset
            [10, -0.5], # Other side
            [15, 1.0]   # Further along
        ]
        
        for i, pos in enumerate(test_positions):
            print(f"\nTest {i+1}: Vehicle at position {pos}")
            
            # Update vehicle position
            self.current_pose.position.x = pos[0]
            self.current_pose.position.y = pos[1]
            
            # Generate path candidates
            candidates = self.generate_path_candidates()
            
            print(f"Generated {len(candidates)} path candidates")
            
            if candidates:
                for j, candidate in enumerate(candidates):
                    q_end = candidate['q_end']
                    path_length = len(candidate['path'])
                    print(f"  Candidate {j+1}: target_q={q_end:.2f}, path_points={path_length}")
                
                # Calculate costs for demonstration
                print("  Calculating costs...")
                for j, candidate in enumerate(candidates[:3]):  # Show first 3
                    static_cost = self.calculate_static_safety_cost(candidate)
                    comfort_cost = self.calculate_comfort_cost(candidate)
                    print(f"    Candidate {j+1}: static_cost={static_cost:.3f}, comfort_cost={comfort_cost:.3f}")

    def calculate_static_safety_cost(self, path_candidate):
        """Calculate static safety cost (simplified version)"""
        # Simplified static cost based on lateral offset
        q_end = path_candidate['q_end']
        if abs(q_end) > 2.0:
            return 0.8
        elif abs(q_end) > 1.0:
            return 0.3
        else:
            return 0.1

    def calculate_comfort_cost(self, path_candidate):
        """Calculate comfort cost (simplified version)"""
        path = path_candidate['path']
        
        # Calculate curvature-based cost
        if len(path) < 3:
            return 0.0
        
        curvature_sum = 0.0
        for i in range(1, len(path)-1):
            p1, p2, p3 = path[i-1], path[i], path[i+1]
            v1 = p2 - p1
            v2 = p3 - p2
            
            if np.linalg.norm(v1) > 0 and np.linalg.norm(v2) > 0:
                cross_product = v1[0]*v2[1] - v1[1]*v2[0]
                curvature = abs(cross_product) / (np.linalg.norm(v1) * np.linalg.norm(v2))
                curvature_sum += curvature**2
        
        return curvature_sum / max(1, len(path) - 2)

    def run_test(self):
        """Run comprehensive test"""
        print("Autonomous Driving Standalone Test")
        print("=" * 50)
        
        # Test path generation
        self.test_path_generation()
        
        # Test with mock obstacles
        print("\n" + "="*50)
        print("Testing with Mock Obstacles")
        print("="*50)
        
        # Create some mock obstacles
        mock_obstacles = [
            {'position': np.array([8.0, 0.5]), 'size': 0.5},
            {'position': np.array([15.0, -1.0]), 'size': 0.3},
        ]
        
        self.detected_obstacles = mock_obstacles
        print(f"Added {len(mock_obstacles)} mock obstacles")
        
        # Test path selection with obstacles
        self.current_pose.position.x = 5.0
        self.current_pose.position.y = 0.0
        
        candidates = self.generate_path_candidates()
        if candidates:
            print(f"Generated {len(candidates)} candidates with obstacles present")
            
            # Simple path selection (choose middle path for safety)
            optimal_idx = len(candidates) // 2
            optimal_path = candidates[optimal_idx]
            print(f"Selected path {optimal_idx + 1} as optimal (q_end={optimal_path['q_end']:.2f})")
            
            # Test control command calculation
            motor_cmd = self.calculate_xycar_control_commands(optimal_path)
            print(f"Control command: angle={motor_cmd.angle}, speed={motor_cmd.speed}")
        
        print("\n" + "="*50)
        print("Test completed successfully!")
        print("="*50)

    def calculate_xycar_control_commands(self, optimal_path):
        """Calculate steering and speed commands for Xycar"""
        motor_msg = MockXycarMotor()
        
        if optimal_path is None or len(optimal_path['path']) < 2:
            motor_msg.angle = 0
            motor_msg.speed = 0
            return motor_msg
        
        # Get target point (look-ahead)
        look_ahead_distance = 2.0
        target_point = None
        
        vehicle_pos = np.array([self.current_pose.position.x, self.current_pose.position.y])
        
        for point in optimal_path['path']:
            if np.linalg.norm(point - vehicle_pos) >= look_ahead_distance:
                target_point = point
                break
        
        if target_point is None:
            target_point = optimal_path['path'][-1]
        
        # Calculate steering angle
        dx = target_point[0] - vehicle_pos[0]
        dy = target_point[1] - vehicle_pos[1]
        target_heading = np.arctan2(dy, dx)
        
        heading_error = target_heading - self.current_heading
        
        # Normalize angle
        while heading_error > np.pi:
            heading_error -= 2 * np.pi
        while heading_error < -np.pi:
            heading_error += 2 * np.pi
        
        # Control gains
        K_angular = 30.0
        
        # Calculate steering angle in degrees
        steering_angle = K_angular * heading_error
        
        # Speed control
        base_speed = self.fix_speed
        target_speed = base_speed
        
        # Reduce speed for high curvature
        if abs(steering_angle) > 20:
            target_speed *= 0.7
        elif abs(steering_angle) > 30:
            target_speed *= 0.5
        
        # Apply limits
        steering_angle = np.clip(steering_angle, -self.max_angle, self.max_angle)
        target_speed = np.clip(target_speed, 5, 20)
        
        motor_msg.angle = int(steering_angle)
        motor_msg.speed = int(target_speed)
        
        return motor_msg

def main():
    """Main function"""
    try:
        # Create and run test
        system = AutonomousDrivingStandalone()
        system.run_test()
        
        return 0
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == "__main__":
    exit(main())
