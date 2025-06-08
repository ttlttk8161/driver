#!/bin/bash

# Autonomous Driving Test Script for Xycar
# This script runs the autonomous driving system without building

# Set the ROS environment
source /opt/ros/noetic/setup.bash

# Source the existing Xycar workspace if it exists
if [ -f "/home/xytron/xycar_ws/devel/setup.bash" ]; then
    source /home/xytron/xycar_ws/devel/setup.bash
    echo "Sourced existing Xycar workspace"
else
    echo "Warning: Xycar workspace not found, using system ROS only"
fi

# Set the current directory to the script location
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd "$SCRIPT_DIR"

# Add current directory to PYTHONPATH for custom modules
export PYTHONPATH="$SCRIPT_DIR:$PYTHONPATH"

# Set ROS package path to include our test package
export ROS_PACKAGE_PATH="$SCRIPT_DIR/..:$ROS_PACKAGE_PATH"

echo "Starting Autonomous Driving System..."
echo "Script directory: $SCRIPT_DIR"
echo "PYTHONPATH: $PYTHONPATH"
echo "ROS_PACKAGE_PATH: $ROS_PACKAGE_PATH"

# Check if required topics are available (optional)
echo "Checking for required ROS topics..."
if rostopic list 2>/dev/null | grep -q "/usb_cam/image_raw"; then
    echo "✓ Camera topic found"
else
    echo "⚠ Warning: Camera topic not found"
fi

if rostopic list 2>/dev/null | grep -q "/scan"; then
    echo "✓ LiDAR topic found"
else
    echo "⚠ Warning: LiDAR topic not found"
fi

# Run the autonomous driving node directly
echo "Launching autonomous driving node..."
python autonomous_driving_node.py
