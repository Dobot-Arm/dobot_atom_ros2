#!/bin/bash
echo "Setup Dobot Atom robot ROS2 environment with default interface"

# Source ROS2 environment (change to your ROS2 version if needed)
source /opt/ros/humble/setup.bash

# Source the atom_ros2 workspace
source $HOME/atom_ros2/install/setup.bash

# Set DDS implementation to CycloneDDS for better performance
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

echo "Dobot Atom ROS2 environment setup completed!"
echo "Using default network interface"
echo "ROS2 Distribution: humble"
echo "DDS Implementation: CycloneDDS"