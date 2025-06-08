#!/usr/bin/env python3
"""
Lane Detection Module
Processes camera images to detect lane markings and road boundaries
"""

import cv2
import numpy as np
from typing import Tuple, List, Optional

class LaneDetector:
    def __init__(self):
        # HSV color ranges for lane detection
        self.white_lower = np.array([0, 0, 200])
        self.white_upper = np.array([180, 30, 255])
        
        self.yellow_lower = np.array([15, 80, 80]) 
        self.yellow_upper = np.array([35, 255, 255])
        
        # Region of interest (ROI) parameters
        self.roi_height_ratio = 0.6  # Only look at bottom 60% of image
        self.roi_width_ratio = 0.8   # Only look at middle 80% of image
        
        # Hough transform parameters
        self.hough_threshold = 30
        self.min_line_length = 40
        self.max_line_gap = 20
        
    def detect_lanes(self, image: np.ndarray) -> Tuple[List, List, np.ndarray]:
        """
        Detect lane markings in the image
        
        Args:
            image: Input BGR image
            
        Returns:
            left_lane_points: Points for left lane boundary
            right_lane_points: Points for right lane boundary  
            processed_image: Visualization image
        """
        # Convert to HSV for better color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Create masks for white and yellow lanes
        white_mask = cv2.inRange(hsv, self.white_lower, self.white_upper)
        yellow_mask = cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)
        
        # Combine masks
        lane_mask = cv2.bitwise_or(white_mask, yellow_mask)
        
        # Apply region of interest
        roi_mask = self._create_roi_mask(image.shape[:2])
        lane_mask = cv2.bitwise_and(lane_mask, roi_mask)
        
        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(lane_mask, (5, 5), 0)
        
        # Edge detection
        edges = cv2.Canny(blurred, 50, 150)
        
        # Detect lines using Hough transform
        lines = cv2.HoughLinesP(
            edges, 
            rho=1, 
            theta=np.pi/180, 
            threshold=self.hough_threshold,
            minLineLength=self.min_line_length, 
            maxLineGap=self.max_line_gap
        )
        
        # Separate left and right lanes
        left_lane_points, right_lane_points = self._separate_lane_lines(lines, image.shape[1])
        
        # Create visualization
        processed_image = self._create_visualization(image, left_lane_points, right_lane_points, lane_mask)
        
        return left_lane_points, right_lane_points, processed_image
    
    def _create_roi_mask(self, image_shape: Tuple[int, int]) -> np.ndarray:
        """Create region of interest mask"""
        height, width = image_shape
        mask = np.zeros((height, width), dtype=np.uint8)
        
        # Define trapezoid ROI
        roi_top = int(height * (1 - self.roi_height_ratio))
        roi_bottom = height
        roi_left = int(width * (1 - self.roi_width_ratio) / 2)
        roi_right = int(width * (1 + self.roi_width_ratio) / 2)
        
        # Create trapezoid vertices
        vertices = np.array([
            [roi_left, roi_bottom],
            [roi_left + int(width * 0.1), roi_top],
            [roi_right - int(width * 0.1), roi_top],
            [roi_right, roi_bottom]
        ], dtype=np.int32)
        
        cv2.fillPoly(mask, [vertices], 255)
        
        return mask
    
    def _separate_lane_lines(self, lines: Optional[np.ndarray], image_width: int) -> Tuple[List, List]:
        """Separate detected lines into left and right lanes based on slope and position"""
        left_lane_points = []
        right_lane_points = []
        
        if lines is None:
            return left_lane_points, right_lane_points
        
        image_center = image_width / 2
        
        for line in lines:
            x1, y1, x2, y2 = line[0]
            
            # Calculate slope
            if x2 - x1 == 0:  # Vertical line
                continue
            
            slope = (y2 - y1) / (x2 - x1)
            
            # Filter out nearly horizontal lines
            if abs(slope) < 0.3:
                continue
            
            # Determine if line is on left or right side
            line_center = (x1 + x2) / 2
            
            if slope < 0 and line_center < image_center:  # Left lane (negative slope)
                left_lane_points.extend([(x1, y1), (x2, y2)])
            elif slope > 0 and line_center > image_center:  # Right lane (positive slope)
                right_lane_points.extend([(x1, y1), (x2, y2)])
        
        return left_lane_points, right_lane_points
    
    def _create_visualization(self, original_image: np.ndarray, 
                            left_points: List, right_points: List, 
                            lane_mask: np.ndarray) -> np.ndarray:
        """Create visualization of detected lanes"""
        # Create overlay image
        overlay = original_image.copy()
        
        # Draw lane mask in green
        mask_colored = cv2.cvtColor(lane_mask, cv2.COLOR_GRAY2BGR)
        mask_colored[:, :, 1] = lane_mask  # Green channel
        overlay = cv2.addWeighted(overlay, 0.8, mask_colored, 0.3, 0)
        
        # Draw left lane points in blue
        for i in range(0, len(left_points), 2):
            if i + 1 < len(left_points):
                pt1 = tuple(map(int, left_points[i]))
                pt2 = tuple(map(int, left_points[i + 1]))
                cv2.line(overlay, pt1, pt2, (255, 0, 0), 3)
        
        # Draw right lane points in red
        for i in range(0, len(right_points), 2):
            if i + 1 < len(right_points):
                pt1 = tuple(map(int, right_points[i]))
                pt2 = tuple(map(int, right_points[i + 1]))
                cv2.line(overlay, pt1, pt2, (0, 0, 255), 3)
        
        return overlay
    
    def get_lane_boundaries_for_path_planning(self, left_points: List, right_points: List, 
                                            image_height: int) -> Tuple[List, List]:
        """
        Convert detected lane points to waypoints for path planning
        
        Returns:
            left_waypoints: List of (x, y) coordinates for left boundary
            right_waypoints: List of (x, y) coordinates for right boundary
        """
        # Simplified conversion - in real application, need proper camera calibration
        # and perspective transformation to world coordinates
        
        left_waypoints = []
        right_waypoints = []
        
        # Convert pixel coordinates to approximate world coordinates
        # This is a simplified transformation - replace with proper camera model
        pixels_per_meter = 50  # Approximate conversion factor
        camera_height = 1.5    # Camera height in meters
        
        if left_points:
            # Fit polynomial to left lane points
            if len(left_points) >= 4:
                left_x = [p[0] for p in left_points]
                left_y = [p[1] for p in left_points]
                
                # Convert to world coordinates (simplified)
                for x_pixel, y_pixel in zip(left_x, left_y):
                    # Distance ahead based on pixel row
                    distance_ahead = (image_height - y_pixel) / pixels_per_meter
                    # Lateral offset from center
                    lateral_offset = (x_pixel - 320) / pixels_per_meter  # Assuming 640px width
                    
                    left_waypoints.append([distance_ahead, lateral_offset - 1.5])  # Assume 1.5m lane offset
        
        if right_points:
            # Fit polynomial to right lane points
            if len(right_points) >= 4:
                right_x = [p[0] for p in right_points]
                right_y = [p[1] for p in right_points]
                
                # Convert to world coordinates (simplified)
                for x_pixel, y_pixel in zip(right_x, right_y):
                    distance_ahead = (image_height - y_pixel) / pixels_per_meter
                    lateral_offset = (x_pixel - 320) / pixels_per_meter
                    
                    right_waypoints.append([distance_ahead, lateral_offset + 1.5])  # Assume 1.5m lane offset
        
        return left_waypoints, right_waypoints
    
    def calculate_lane_deviation(self, left_points: List, right_points: List, 
                               image_width: int) -> float:
        """
        Calculate lateral deviation from lane center
        
        Returns:
            deviation: Lateral deviation in pixels (negative = left, positive = right)
        """
        image_center = image_width / 2
        
        # Find lane center at bottom of image
        left_x_bottom = None
        right_x_bottom = None
        
        # Find leftmost and rightmost points at bottom of image
        if left_points:
            bottom_left_points = [p for p in left_points if p[1] > image_width * 0.8]
            if bottom_left_points:
                left_x_bottom = max(bottom_left_points, key=lambda p: p[1])[0]
        
        if right_points:
            bottom_right_points = [p for p in right_points if p[1] > image_width * 0.8]
            if bottom_right_points:
                right_x_bottom = max(bottom_right_points, key=lambda p: p[1])[0]
        
        # Calculate lane center
        if left_x_bottom is not None and right_x_bottom is not None:
            lane_center = (left_x_bottom + right_x_bottom) / 2
            deviation = lane_center - image_center
        elif left_x_bottom is not None:
            # Only left lane detected, estimate center
            estimated_right = left_x_bottom + 100  # Assume 100px lane width
            lane_center = (left_x_bottom + estimated_right) / 2
            deviation = lane_center - image_center
        elif right_x_bottom is not None:
            # Only right lane detected, estimate center
            estimated_left = right_x_bottom - 100
            lane_center = (estimated_left + right_x_bottom) / 2
            deviation = lane_center - image_center
        else:
            # No lanes detected
            deviation = 0.0
        
        return deviation
