#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Autonomous Driving Node for Xycar Simulator
Based on the paper's path planning algorithm using s-q coordinate system
Processes camera image and lidar scan data for autonomous navigation
Adapted for 2025 Kookmin University Autonomous Driving Competition
"""

import rospy
import numpy as np
import cv2
import math
import time
import threading
from scipy import interpolate
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float32MultiArray
from xycar_msgs.msg import XycarMotor
from cv_bridge import CvBridge

# Import our custom modules
from lane_detector import LaneDetector
from obstacle_detector import ObstacleDetector  
from path_planner import PathPlanner

class AutonomousDrivingNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('autonomous_driving_node', anonymous=True)
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Vehicle state (simulated position and heading)
        self.current_position = np.array([0.0, 0.0])  # Estimated position [x, y]
        self.current_velocity = 0.0
        self.current_heading = 0.0  # Current heading in radians
        self.current_pose = None
        self.last_time = time.time()
        
        # Sensor data
        self.latest_image = np.empty(shape=[0])
        self.latest_lidar = None
        self.image_lock = threading.Lock()
        self.lidar_lock = threading.Lock()
        
        # Initialize detection and planning modules
        self.lane_detector = LaneDetector()
        self.obstacle_detector = ObstacleDetector()
        self.path_planner = PathPlanner()
        
        # Path planning data
        self.optimal_path = None
        self.detected_obstacles = []
        self.lane_boundaries = []
        self.previous_path = None
        
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
        self._initialize_example_center_line()
        
        # Subscribers for Xycar
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        
        # Publishers for Xycar
        self.motor_pub = rospy.Publisher('/xycar_motor', XycarMotor, queue_size=1)
        
        # Optional publishers for visualization (if needed)
        try:
            self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=1)
            self.candidates_pub = rospy.Publisher('/path_candidates', Float32MultiArray, queue_size=1)
        except:
            self.path_pub = None
            self.candidates_pub = None
        
        # Main control loop timer
        self.control_timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)
        
        rospy.loginfo("Autonomous Driving Node initialized for Xycar")

    def image_callback(self, msg):
        """Process camera image for lane detection and obstacle recognition"""
        try:
            with self.image_lock:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                self.latest_image = cv_image
                self.process_image(cv_image)
        except Exception as e:
            rospy.logerr(f"Image processing error: {e}")

    def lidar_callback(self, msg):
        """Process lidar scan for obstacle detection"""
        try:
            with self.lidar_lock:
                self.latest_lidar = msg
                self.process_lidar(msg)
        except Exception as e:
            rospy.logerr(f"Lidar processing error: {e}")

    def odom_callback(self, msg):
        """Update vehicle state from odometry"""
        self.current_pose = msg.pose.pose
        self.current_velocity = np.sqrt(
            msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2
        )
        
        # Extract heading from quaternion
        from tf.transformations import euler_from_quaternion
        orientation = msg.pose.pose.orientation
        _, _, self.current_heading = euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])

    def initialize_center_line(self):
        """Initialize center line with example waypoints (Section 2.1 from paper)"""
        # Example waypoints - in real scenario, load from HD map
        waypoints_left = np.array([
            [0, -1], [10, -1], [20, 0], [30, 2], [40, 2], [50, 0]
        ])
        waypoints_right = np.array([
            [0, 1], [10, 1], [20, 2], [30, 4], [40, 4], [50, 2]
        ])
        
        # Construct center line as per equation (1) in paper
        self.center_line_params = self.construct_center_line(waypoints_left, waypoints_right)

    def construct_center_line(self, waypoints_left, waypoints_right):
        """
        Construct center line using cubic splines parameterized by arc length
        Implements Section 2.1 from the paper
        """
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

    def process_image(self, image):
        """Process camera image for lane detection and static obstacle recognition"""
        # Convert to HSV for better lane detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Lane detection (simplified - detect white/yellow lanes)
        # White lane detection
        white_lower = np.array([0, 0, 200])
        white_upper = np.array([180, 30, 255])
        white_mask = cv2.inRange(hsv, white_lower, white_upper)
        
        # Yellow lane detection  
        yellow_lower = np.array([15, 80, 80])
        yellow_upper = np.array([35, 255, 255])
        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
        
        # Combine masks
        lane_mask = cv2.bitwise_or(white_mask, yellow_mask)
        
        # Find lane boundaries (simplified approach)
        # In real implementation, use more sophisticated lane detection
        # This is used for static safety cost calculation
        
        return lane_mask

    def process_lidar(self, scan_msg):
        """
        Process lidar data for dynamic obstacle detection
        Convert scan to cartesian coordinates and detect obstacles
        """
        ranges = np.array(scan_msg.ranges)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))
        
        # Filter out invalid readings
        valid_indices = np.isfinite(ranges) & (ranges > scan_msg.range_min) & (ranges < scan_msg.range_max)
        valid_ranges = ranges[valid_indices]
        valid_angles = angles[valid_indices]
        
        # Convert to cartesian coordinates
        x = valid_ranges * np.cos(valid_angles)
        y = valid_ranges * np.sin(valid_angles)
        
        # Simple obstacle detection - cluster nearby points
        obstacles = self.detect_obstacles(x, y)
        
        return obstacles

    def detect_obstacles(self, x, y):
        """Detect obstacles from lidar point cloud"""
        obstacles = []
        
        # Simple clustering based on distance threshold
        points = np.column_stack([x, y])
        if len(points) < 3:
            return obstacles
            
        # Use DBSCAN-like clustering
        visited = np.zeros(len(points), dtype=bool)
        
        for i in range(len(points)):
            if visited[i]:
                continue
                
            cluster = [i]
            visited[i] = True
            
            for j in range(i+1, len(points)):
                if not visited[j]:
                    dist = np.linalg.norm(points[i] - points[j])
                    if dist < 0.5:  # 0.5m clustering threshold
                        cluster.append(j)
                        visited[j] = True
            
            if len(cluster) > 3:  # Minimum points for obstacle
                cluster_points = points[cluster]
                centroid = np.mean(cluster_points, axis=0)
                size = np.max(np.linalg.norm(cluster_points - centroid, axis=1))
                
                obstacles.append({
                    'position': centroid,
                    'size': size,
                    'velocity': np.array([0.0, 0.0])  # Simplified - no velocity estimation
                })
        
        return obstacles

    def generate_path_candidates(self):
        """
        Generate path candidates using s-q coordinate system
        Implements Section 2.2 from the paper
        """
        if self.current_pose is None or self.center_line_params is None:
            return []
        
        # Current vehicle position
        vehicle_x = self.current_pose.position.x
        vehicle_y = self.current_pose.position.y
        vehicle_pos = np.array([vehicle_x, vehicle_y])
        
        # Localize on center line (Section 2.2.1)
        s_start, q_start = self.localize_on_center_line(vehicle_pos)
        
        # Calculate delta_h_start (angle difference)
        center_line_tangent = self.get_center_line_tangent(s_start)
        delta_h_start = self.current_heading - center_line_tangent
        
        path_candidates = []
        
        # Generate path for each target lateral offset (Section 2.2.2)
        for q_end in self.target_lateral_offsets:
            # Solve cubic coefficients (Equation 5)
            a, b, c = self.solve_cubic_coefficients(
                s_start, q_start, s_start + self.s_end_arc_length, 
                q_end, delta_h_start
            )
            
            # Convert to cartesian coordinates (Section 2.2.3)
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
        """
        Localize vehicle position on center line
        Returns s (arc length) and q (lateral offset)
        """
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
        """
        Solve cubic polynomial coefficients for lateral offset function
        Implements boundary conditions from Equation (5)
        """
        ds = s_end - s_start
        
        # From boundary conditions:
        # q(s_start) = q_start
        # q(s_end) = q_end  
        # q'(s_start) = tan(delta_h_start)
        # q'(s_end) = 0
        
        c = np.tan(delta_h_start)
        
        # Solve 2x2 system for a and b
        # 3*a*ds^2 + 2*b*ds + c = 0  (q'(s_end) = 0)
        # a*ds^3 + b*ds^2 + c*ds + q_start = q_end  (q(s_end) = q_end)
        
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
        """
        Convert s-q path to cartesian coordinates
        Implements coordinate transformation from Section 2.2.3
        """
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

    def calculate_total_cost(self, path_candidate, obstacles):
        """
        Calculate total cost function
        Implements Equation (23) from the paper
        """
        # Static safety cost (Equation 10)
        f_s = self.calculate_static_safety_cost(path_candidate)
        
        # Comfort cost (Equation 14) 
        f_c = self.calculate_comfort_cost(path_candidate)
        
        # Dynamic safety cost (Equation 22)
        f_d = self.calculate_dynamic_safety_cost(path_candidate, obstacles)
        
        # Total cost (Equation 23)
        total_cost = self.w_s * f_s + self.w_c * f_c + self.w_d * f_d
        
        return total_cost, (f_s, f_c, f_d)

    def calculate_static_safety_cost(self, path_candidate):
        """
        Calculate static safety cost using discrete Gaussian convolution
        Implements Equations (10) and (11)
        """
        path = path_candidate['path']
        
        # Simplified collision checking
        # In real implementation, check against lane boundaries and static obstacles
        collision_results = []
        
        for point in path:
            # Check if point is within safe boundaries
            # This is a simplified check - replace with actual map-based collision detection
            collision_risk = 0.0
            
            # Example: penalize if too far from center line
            if abs(path_candidate['q_end']) > 2.0:  # Outside normal lane
                collision_risk = 0.5
            elif abs(path_candidate['q_end']) > 3.0:  # Near lane boundary
                collision_risk = 1.0
            
            collision_results.append(collision_risk)
        
        # Apply Gaussian convolution (simplified)
        sigma = 2.0
        N = 5
        static_cost = 0.0
        
        for i, risk in enumerate(collision_results):
            gaussian_sum = 0.0
            for k in range(-N, N+1):
                if 0 <= i+k < len(collision_results):
                    gaussian_weight = np.exp(-(k**2) / (2 * sigma**2))
                    gaussian_sum += gaussian_weight * collision_results[i+k]
            static_cost += gaussian_sum
        
        return static_cost / len(collision_results) if collision_results else 0.0

    def calculate_comfort_cost(self, path_candidate):
        """
        Calculate comfort cost (smoothness + consistency)
        Implements Equations (12), (13), and (14)
        """
        path = path_candidate['path']
        
        # Smoothness cost (Equation 12) - integral of curvature squared
        smoothness_cost = 0.0
        if len(path) > 2:
            for i in range(1, len(path)-1):
                # Calculate curvature at point i
                p1, p2, p3 = path[i-1], path[i], path[i+1]
                
                # Simple curvature calculation
                v1 = p2 - p1
                v2 = p3 - p2
                
                if np.linalg.norm(v1) > 0 and np.linalg.norm(v2) > 0:
                    cross_product = v1[0]*v2[1] - v1[1]*v2[0]
                    curvature = abs(cross_product) / (np.linalg.norm(v1) * np.linalg.norm(v2))
                    smoothness_cost += curvature**2
        
        smoothness_cost /= max(1, len(path) - 2)
        
        # Consistency cost (Equation 13)
        consistency_cost = 0.0
        if self.previous_path is not None:
            # Calculate heading difference with previous path
            # Simplified implementation
            consistency_cost = 0.1  # Placeholder
        
        # Total comfort cost (Equation 14)
        alpha, beta = 0.7, 0.3
        comfort_cost = alpha * smoothness_cost + beta * consistency_cost
        
        return comfort_cost

    def calculate_dynamic_safety_cost(self, path_candidate, obstacles):
        """
        Calculate dynamic safety cost for moving obstacles
        Implements Equation (22)
        """
        if not obstacles:
            return 0.0
        
        path = path_candidate['path']
        
        # Find closest obstacle and collision point
        min_distance = float('inf')
        collision_distance = None
        
        for obstacle in obstacles:
            for i, point in enumerate(path):
                dist_to_obstacle = np.linalg.norm(point - obstacle['position'])
                collision_radius = 1.0 + obstacle['size']  # Vehicle radius + obstacle
                
                if dist_to_obstacle < collision_radius:
                    path_distance = i * (self.s_end_arc_length / len(path))
                    if path_distance < min_distance:
                        min_distance = path_distance
                        collision_distance = path_distance
        
        if collision_distance is None:
            return 0.0
        
        # Calculate required acceleration (Equation 16)
        L_c = 2.0  # Collision range
        L_f = min(self.following_distance_base, collision_distance)
        
        if collision_distance <= L_c:
            return 1.0  # Maximum cost for immediate collision
        
        # Calculate required acceleration
        try:
            required_acceleration = abs(2 * (L_c - L_f) * self.current_velocity**2 / 
                                      (collision_distance - L_c)**2)
        except ZeroDivisionError:
            required_acceleration = 10.0  # High acceleration for near collision
        
        # Dynamic safety cost (Equation 22)
        dynamic_cost = required_acceleration * (collision_distance - L_f)
        
        return min(dynamic_cost / 100.0, 1.0)  # Normalize

    def select_optimal_path(self, path_candidates, obstacles):
        """Select optimal path by minimizing total cost function"""
        if not path_candidates:
            return None
        
        min_cost = float('inf')
        optimal_path = None
        
        for candidate in path_candidates:
            total_cost, cost_components = self.calculate_total_cost(candidate, obstacles)
            
            if total_cost < min_cost:
                min_cost = total_cost
                optimal_path = candidate
        
        return optimal_path

    def calculate_control_commands(self, optimal_path):
        """
        Calculate steering and speed commands from optimal path
        """
        if optimal_path is None or len(optimal_path['path']) < 2:
            return Twist()  # Stop
        
        cmd = Twist()
        
        # Get target point (look-ahead)
        look_ahead_distance = 2.0  # meters
        target_point = None
        
        vehicle_pos = np.array([self.current_pose.position.x, self.current_pose.position.y])
        
        for point in optimal_path['path']:
            if np.linalg.norm(point - vehicle_pos) >= look_ahead_distance:
                target_point = point
                break
        
        if target_point is None:
            target_point = optimal_path['path'][-1]
        
        # Calculate steering angle (simplified pure pursuit)
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
        K_angular = 2.0
        K_linear = 1.0
        
        # Calculate commands
        cmd.angular.z = K_angular * heading_error
        
        # Speed control based on curvature and safety
        max_speed = 5.0  # m/s
        target_speed = max_speed
        
        # Reduce speed for high curvature
        if abs(cmd.angular.z) > 0.5:
            target_speed *= 0.5
        
        # Simple speed control
        speed_error = target_speed - self.current_velocity
        cmd.linear.x = K_linear * speed_error
        
        # Apply limits
        cmd.linear.x = np.clip(cmd.linear.x, -2.0, 5.0)
        cmd.angular.z = np.clip(cmd.angular.z, -1.0, 1.0)
        
        return cmd

    def publish_path(self, path_candidate):
        """Publish planned path for visualization"""
        if path_candidate is None:
            return
        
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"
        
        for point in path_candidate['path']:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)

    def control_loop(self, event):
        """Main control loop - runs at 10Hz"""
        try:
            # Process sensor data and get obstacles
            obstacles = []
            if self.latest_lidar is not None:
                obstacles = self.process_lidar(self.latest_lidar)
            
            # Generate path candidates
            path_candidates = self.generate_path_candidates()
            
            if not path_candidates:
                rospy.logwarn("No path candidates generated")
                return
            
            # Select optimal path
            optimal_path = self.select_optimal_path(path_candidates, obstacles)
            
            if optimal_path is None:
                rospy.logwarn("No optimal path found")
                return
            
            # Calculate control commands
            cmd = self.calculate_control_commands(optimal_path)
            
            # Publish commands and path
            self.cmd_vel_pub.publish(cmd)
            self.publish_path(optimal_path)
            
            # Update previous path for consistency
            self.previous_path = optimal_path
            
            rospy.logdebug(f"Control: linear={cmd.linear.x:.2f}, angular={cmd.angular.z:.2f}")
            
        except Exception as e:
            rospy.logerr(f"Control loop error: {e}")

    def run(self):
        """Main run loop"""
        rospy.loginfo("Starting autonomous driving...")
        rospy.spin()

if __name__ == '__main__':
    try:
        node = AutonomousDrivingNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Autonomous driving node terminated")
