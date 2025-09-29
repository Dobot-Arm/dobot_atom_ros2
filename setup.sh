#!/bin/bash
echo "Setup Dobot Atom robot ROS2 environment"

# Source ROS2 environment (change to your ROS2 version if needed)
source /opt/ros/humble/setup.bash

# Source the atom_ros2 workspace
source $HOME/atom_ros2_ws/install/setup.bash

# Set DDS implementation to CycloneDDS for better performance
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Configure CycloneDDS network interface
# Replace "eth0" with your actual network interface name
# Use "ifconfig" or "ip addr" to check your network interface
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="eth0" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'

echo "Dobot Atom ROS2 environment setup completed!"
echo "Network interface: eth0 (modify if needed)"
echo "ROS2 Distribution: humble"
echo "DDS Implementation: CycloneDDS"