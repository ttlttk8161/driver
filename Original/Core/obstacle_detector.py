#!/usr/bin/env python3
"""
Obstacle Detection Module
Processes lidar scan data to detect and track obstacles
"""

import numpy as np
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass
from sklearn.cluster import DBSCAN
import time

@dataclass
class Obstacle:
    """Data class for obstacle representation"""
    id: int
    position: np.ndarray  # [x, y] in vehicle frame
    velocity: np.ndarray  # [vx, vy] in vehicle frame
    size: float          # Radius of obstacle
    confidence: float    # Detection confidence (0-1)
    last_seen: float     # Timestamp of last detection
    type: str           # 'static' or 'dynamic'

class ObstacleDetector:
    def __init__(self):
        # Clustering parameters
        self.dbscan_eps = 0.5  # Maximum distance between points in a cluster
        self.dbscan_min_samples = 3  # Minimum points to form a cluster
        
        # Obstacle filtering parameters
        self.min_obstacle_size = 0.1  # Minimum obstacle radius (m)
        self.max_obstacle_size = 3.0  # Maximum obstacle radius (m)
        self.min_points_per_obstacle = 3
        
        # Tracking parameters
        self.max_tracking_distance = 2.0  # Maximum distance to associate obstacles
        self.obstacle_timeout = 2.0  # Remove obstacles not seen for this long
        
        # Detection range limits
        self.max_detection_range = 20.0  # Maximum detection range (m)
        self.min_detection_range = 0.5   # Minimum detection range (m)
        
        # Obstacle tracking
        self.tracked_obstacles: Dict[int, Obstacle] = {}
        self.next_obstacle_id = 0
        
        # Vehicle dimensions for collision checking
        self.vehicle_width = 1.8   # Vehicle width (m)
        self.vehicle_length = 4.5  # Vehicle length (m)
        
    def process_lidar_scan(self, scan_msg) -> List[Obstacle]:
        """
        Process lidar scan message and return detected obstacles
        
        Args:
            scan_msg: sensor_msgs/LaserScan message
            
        Returns:
            List of detected obstacles
        """
        # Convert scan to cartesian coordinates
        points = self._scan_to_cartesian(scan_msg)
        
        if len(points) < self.min_points_per_obstacle:
            return []
        
        # Filter points by range
        points = self._filter_points_by_range(points)
        
        # Remove ground points (simplified ground removal)
        points = self._remove_ground_points(points)
        
        # Cluster points into potential obstacles
        clusters = self._cluster_points(points)
        
        # Extract obstacle features from clusters
        detected_obstacles = self._extract_obstacles_from_clusters(clusters, points)
        
        # Update tracking
        self._update_obstacle_tracking(detected_obstacles)
        
        # Clean up old obstacles
        self._cleanup_old_obstacles()
        
        return list(self.tracked_obstacles.values())
    
    def _scan_to_cartesian(self, scan_msg) -> np.ndarray:
        """Convert laser scan to cartesian coordinates"""
        ranges = np.array(scan_msg.ranges)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))
        
        # Filter out invalid readings
        valid_mask = (ranges >= scan_msg.range_min) & \
                    (ranges <= scan_msg.range_max) & \
                    np.isfinite(ranges)
        
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]
        
        if len(valid_ranges) == 0:
            return np.array([]).reshape(0, 2)
        
        # Convert to cartesian
        x = valid_ranges * np.cos(valid_angles)
        y = valid_ranges * np.sin(valid_angles)
        
        return np.column_stack([x, y])
    
    def _filter_points_by_range(self, points: np.ndarray) -> np.ndarray:
        """Filter points by detection range"""
        if len(points) == 0:
            return points
        
        distances = np.linalg.norm(points, axis=1)
        valid_mask = (distances >= self.min_detection_range) & \
                    (distances <= self.max_detection_range)
        
        return points[valid_mask]
    
    def _remove_ground_points(self, points: np.ndarray) -> np.ndarray:
        """
        Simple ground point removal
        In a real implementation, use more sophisticated methods like RANSAC
        """
        if len(points) == 0:
            return points
        
        # Assume ground points are below a certain height threshold
        # This is simplified - in 2D lidar, we might filter by intensity or use other methods
        
        # For simulation, we'll use a simple distance-based filter
        # Remove points that are very close to the vehicle (likely ground/vehicle parts)
        distances = np.linalg.norm(points, axis=1)
        non_ground_mask = distances > 0.5
        
        return points[non_ground_mask]
    
    def _cluster_points(self, points: np.ndarray) -> List[np.ndarray]:
        """Cluster points using DBSCAN"""
        if len(points) < self.dbscan_min_samples:
            return []
        
        # Apply DBSCAN clustering
        clustering = DBSCAN(eps=self.dbscan_eps, min_samples=self.dbscan_min_samples)
        cluster_labels = clustering.fit_predict(points)
        
        # Extract clusters (ignore noise points with label -1)
        clusters = []
        unique_labels = set(cluster_labels) - {-1}
        
        for label in unique_labels:
            cluster_points = points[cluster_labels == label]
            if len(cluster_points) >= self.min_points_per_obstacle:
                clusters.append(cluster_points)
        
        return clusters
    
    def _extract_obstacles_from_clusters(self, clusters: List[np.ndarray], 
                                       all_points: np.ndarray) -> List[Obstacle]:
        """Extract obstacle features from point clusters"""
        obstacles = []
        
        for cluster_points in clusters:
            # Calculate centroid
            centroid = np.mean(cluster_points, axis=0)
            
            # Calculate size (maximum distance from centroid)
            distances_to_centroid = np.linalg.norm(cluster_points - centroid, axis=1)
            obstacle_size = np.max(distances_to_centroid)
            
            # Filter by size
            if obstacle_size < self.min_obstacle_size or obstacle_size > self.max_obstacle_size:
                continue
            
            # Calculate confidence based on number of points and compactness
            num_points = len(cluster_points)
            compactness = np.std(distances_to_centroid)
            confidence = min(1.0, num_points / 10.0) * max(0.1, 1.0 - compactness)
            
            # Create obstacle
            obstacle = Obstacle(
                id=-1,  # Will be assigned during tracking update
                position=centroid,
                velocity=np.array([0.0, 0.0]),  # Will be estimated during tracking
                size=obstacle_size,
                confidence=confidence,
                last_seen=time.time(),
                type='unknown'  # Will be determined based on motion
            )
            
            obstacles.append(obstacle)
        
        return obstacles
    
    def _update_obstacle_tracking(self, detected_obstacles: List[Obstacle]):
        """Update obstacle tracking with new detections"""
        current_time = time.time()
        
        # Association matrix between detected and tracked obstacles
        if not detected_obstacles:
            return
        
        tracked_list = list(self.tracked_obstacles.values())
        
        # Calculate association costs (distances)
        association_matrix = np.full((len(detected_obstacles), len(tracked_list)), 
                                   self.max_tracking_distance + 1)
        
        for i, detected in enumerate(detected_obstacles):
            for j, tracked in enumerate(tracked_list):
                distance = np.linalg.norm(detected.position - tracked.position)
                if distance <= self.max_tracking_distance:
                    association_matrix[i, j] = distance
        
        # Simple greedy association (in practice, use Hungarian algorithm)
        used_detections = set()
        used_tracks = set()
        
        # Associate detections to tracks
        for _ in range(min(len(detected_obstacles), len(tracked_list))):
            # Find minimum cost association
            min_cost = np.inf
            best_detection = -1
            best_track = -1
            
            for i in range(len(detected_obstacles)):
                if i in used_detections:
                    continue
                for j in range(len(tracked_list)):
                    if j in used_tracks:
                        continue
                    if association_matrix[i, j] < min_cost:
                        min_cost = association_matrix[i, j]
                        best_detection = i
                        best_track = j
            
            if min_cost <= self.max_tracking_distance:
                # Update existing track
                tracked_obstacle = tracked_list[best_track]
                detected_obstacle = detected_obstacles[best_detection]
                
                # Estimate velocity
                dt = current_time - tracked_obstacle.last_seen
                if dt > 0:
                    velocity = (detected_obstacle.position - tracked_obstacle.position) / dt
                    # Simple velocity smoothing
                    tracked_obstacle.velocity = 0.7 * tracked_obstacle.velocity + 0.3 * velocity
                
                # Update position and other attributes
                tracked_obstacle.position = detected_obstacle.position
                tracked_obstacle.size = 0.8 * tracked_obstacle.size + 0.2 * detected_obstacle.size
                tracked_obstacle.confidence = min(1.0, tracked_obstacle.confidence + 0.1)
                tracked_obstacle.last_seen = current_time
                
                # Determine type based on velocity
                speed = np.linalg.norm(tracked_obstacle.velocity)
                if speed > 0.5:  # Moving faster than 0.5 m/s
                    tracked_obstacle.type = 'dynamic'
                else:
                    tracked_obstacle.type = 'static'
                
                used_detections.add(best_detection)
                used_tracks.add(best_track)
            else:
                break
        
        # Create new tracks for unassociated detections
        for i, detected in enumerate(detected_obstacles):
            if i not in used_detections:
                detected.id = self.next_obstacle_id
                detected.last_seen = current_time
                self.tracked_obstacles[self.next_obstacle_id] = detected
                self.next_obstacle_id += 1
    
    def _cleanup_old_obstacles(self):
        """Remove obstacles that haven't been seen recently"""
        current_time = time.time()
        obstacles_to_remove = []
        
        for obstacle_id, obstacle in self.tracked_obstacles.items():
            if current_time - obstacle.last_seen > self.obstacle_timeout:
                obstacles_to_remove.append(obstacle_id)
        
        for obstacle_id in obstacles_to_remove:
            del self.tracked_obstacles[obstacle_id]
    
    def check_collision_risk(self, path_points: np.ndarray, 
                           time_horizon: float = 3.0) -> List[Dict]:
        """
        Check collision risk for a given path
        
        Args:
            path_points: Array of path points [(x, y), ...]
            time_horizon: Time horizon for collision prediction (seconds)
            
        Returns:
            List of collision risks with details
        """
        collision_risks = []
        
        if len(path_points) == 0:
            return collision_risks
        
        for obstacle in self.tracked_obstacles.values():
            # Predict obstacle position over time horizon
            for t in np.linspace(0, time_horizon, 10):
                predicted_pos = obstacle.position + obstacle.velocity * t
                
                # Check collision with path points
                for i, path_point in enumerate(path_points):
                    distance = np.linalg.norm(path_point - predicted_pos)
                    collision_threshold = obstacle.size + self.vehicle_width / 2 + 0.5  # 0.5m safety margin
                    
                    if distance < collision_threshold:
                        risk_info = {
                            'obstacle_id': obstacle.id,
                            'obstacle_type': obstacle.type,
                            'collision_time': t,
                            'collision_distance': i * 0.5,  # Assume 0.5m between path points
                            'risk_level': 1.0 - (distance / collision_threshold),
                            'obstacle_position': predicted_pos,
                            'path_point_index': i
                        }
                        collision_risks.append(risk_info)
                        break  # Only report first collision point for this obstacle
        
        return collision_risks
    
    def get_obstacles_in_region(self, center: np.ndarray, radius: float) -> List[Obstacle]:
        """Get obstacles within a specified region"""
        obstacles_in_region = []
        
        for obstacle in self.tracked_obstacles.values():
            distance = np.linalg.norm(obstacle.position - center)
            if distance <= radius + obstacle.size:
                obstacles_in_region.append(obstacle)
        
        return obstacles_in_region
    
    def visualize_obstacles(self, image: np.ndarray, 
                          camera_matrix: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Visualize obstacles on camera image (if camera calibration is available)
        Simplified version without proper camera projection
        """
        viz_image = image.copy()
        
        # Simple visualization - project obstacles to image plane
        # This is simplified - in practice, need proper camera calibration
        image_height, image_width = image.shape[:2]
        
        for obstacle in self.tracked_obstacles.values():
            # Simple perspective projection (placeholder)
            if obstacle.position[0] > 0:  # Only show obstacles in front
                # Convert world coordinates to image coordinates (simplified)
                scale = 200.0 / max(obstacle.position[0], 1.0)  # Perspective scaling
                img_x = int(image_width / 2 + obstacle.position[1] * scale)
                img_y = int(image_height - 50 - obstacle.position[0] * 10)
                
                if 0 <= img_x < image_width and 0 <= img_y < image_height:
                    # Draw obstacle circle
                    radius = max(5, int(obstacle.size * scale))
                    color = (0, 0, 255) if obstacle.type == 'dynamic' else (255, 0, 0)
                    cv2.circle(viz_image, (img_x, img_y), radius, color, 2)
                    
                    # Draw velocity vector for dynamic obstacles
                    if obstacle.type == 'dynamic':
                        vel_end_x = int(img_x + obstacle.velocity[1] * scale * 10)
                        vel_end_y = int(img_y - obstacle.velocity[0] * scale * 10)
                        cv2.arrowedLine(viz_image, (img_x, img_y), 
                                      (vel_end_x, vel_end_y), color, 2)
                    
                    # Draw obstacle ID
                    cv2.putText(viz_image, f"ID:{obstacle.id}", 
                              (img_x - 10, img_y - radius - 5),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        
        return viz_image
