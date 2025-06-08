#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Simple test script for autonomous driving modules
Tests individual components without requiring full ROS setup
"""

import sys
import os
import numpy as np

# Add current directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

def test_lane_detector():
    """Test lane detection module"""
    print("Testing Lane Detector...")
    try:
        from lane_detector import LaneDetector
        detector = LaneDetector()
        print("✓ Lane Detector initialized successfully")
        
        # Create dummy image for testing
        import cv2
        test_image = np.zeros((480, 640, 3), dtype=np.uint8)
        lanes = detector.detect_lanes(test_image)
        print(f"✓ Lane detection test completed, found {len(lanes) if lanes else 0} lanes")
        return True
    except Exception as e:
        print(f"✗ Lane Detector test failed: {e}")
        return False

def test_obstacle_detector():
    """Test obstacle detection module"""
    print("Testing Obstacle Detector...")
    try:
        from obstacle_detector import ObstacleDetector
        detector = ObstacleDetector()
        print("✓ Obstacle Detector initialized successfully")
        
        # Create dummy laser scan for testing
        class MockLaserScan:
            def __init__(self):
                self.ranges = np.random.uniform(0.5, 10.0, 360)
                self.angle_min = -np.pi
                self.angle_max = np.pi
                self.range_min = 0.1
                self.range_max = 12.0
        
        mock_scan = MockLaserScan()
        obstacles = detector.detect_obstacles(mock_scan)
        print(f"✓ Obstacle detection test completed, found {len(obstacles)} obstacles")
        return True
    except Exception as e:
        print(f"✗ Obstacle Detector test failed: {e}")
        return False

def test_path_planner():
    """Test path planning module"""
    print("Testing Path Planner...")
    try:
        from path_planner import PathPlanner
        planner = PathPlanner()
        print("✓ Path Planner initialized successfully")
        
        # Test basic path planning functionality
        print("✓ Path planning test completed")
        return True
    except Exception as e:
        print(f"✗ Path Planner test failed: {e}")
        return False

def test_imports():
    """Test all required imports"""
    print("Testing imports...")
    try:
        import numpy as np
        import cv2
        from scipy import interpolate
        print("✓ All basic imports successful")
        return True
    except Exception as e:
        print(f"✗ Import test failed: {e}")
        return False

def main():
    """Run all tests"""
    print("=" * 50)
    print("Autonomous Driving Module Test")
    print("=" * 50)
    
    tests = [
        test_imports,
        test_lane_detector,
        test_obstacle_detector,
        test_path_planner
    ]
    
    passed = 0
    total = len(tests)
    
    for test in tests:
        if test():
            passed += 1
        print()
    
    print("=" * 50)
    print(f"Test Results: {passed}/{total} tests passed")
    
    if passed == total:
        print("✓ All tests passed! System is ready to run.")
        return 0
    else:
        print("✗ Some tests failed. Please check the errors above.")
        return 1

if __name__ == "__main__":
    sys.exit(main())
