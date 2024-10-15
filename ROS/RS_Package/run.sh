#!/bin/bash

# This script builds, sources, and launches the RS_Package.

# Set the base directory (modify if your directory structure is different)
BASE_DIR="$(dirname "$(realpath "$0")")"
ROS_WS="$BASE_DIR/.."
echo "Set ROS_WS to $ROS_WS"

# Clean previous build artifacts
echo "Cleaning previous build artifacts..."
rm -rf "$ROS_WS/build" "$ROS_WS/install" "$ROS_WS/log"

# Source ROS 2 environment
echo "Sourcing ROS 2 environment..."
source /opt/ros/humble/setup.bash

# Build the package
echo "Building the RS_Package..."
colcon build --base-paths "$ROS_WS/RS_Package"

# Check if build was successful
if [ $? -ne 0 ]; then
    echo "Build failed. Exiting."
    exit 1
fi

# Source the package
echo "Sourcing the package environment..."
source "$ROS_WS/install/setup.bash"

# Determine if we should include Turtlesim
USE_TURTLESIM="false"

if [ "$1" == "--with-turtlesim" ]; then
    USE_TURTLESIM="true"
fi

# Launch the package with the 'use_turtlesim' argument
echo "Launching the RS_Package with use_turtlesim: $USE_TURTLESIM"
ros2 launch RS_Package rs_launch.py use_turtlesim:=$USE_TURTLESIM
