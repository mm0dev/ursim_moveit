#!/bin/bash
# Quick setup script for keyboard teleop

echo "=========================================="
echo "Installing Keyboard Teleop Dependencies"
echo "=========================================="

# Install scipy for quaternion/Euler conversions
echo ""
echo "Installing scipy..."
pip3 install scipy

# Build the package
echo ""
echo "Building my_ur_station package..."
cd /root/ros2_ws
colcon build --packages-select my_ur_station

# Source the workspace
echo ""
echo "Sourcing workspace..."
source install/setup.bash

echo ""
echo "=========================================="
echo "Setup Complete!"
echo "=========================================="
echo ""
echo "To run the keyboard teleop:"
echo "  1. Make sure URSim and MoveIt are running"
echo "  2. Run: ros2 run my_ur_station keyboard_teleop"
echo ""
echo "See KEYBOARD_TELEOP_README.md for detailed instructions"
echo ""
