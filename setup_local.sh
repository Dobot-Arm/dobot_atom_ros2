#!/bin/bash
echo "Setup Dobot Atom robot ROS2 local simulation environment"

# Source ROS2 environment (change to your ROS2 version if needed)
source /opt/ros/humble/setup.bash

# Source the atom_ros2 workspace
source $HOME/atom_ros2/install/setup.bash

# Set DDS implementation to CycloneDDS for better performance
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Configure CycloneDDS to use local loopback interface for simulation
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="lo" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'

echo "Dobot Atom ROS2 local simulation environment setup completed!"
echo "Network interface: lo (loopback)"
echo "ROS2 Distribution: humble"
echo "DDS Implementation: CycloneDDS"
echo "This setup is for local simulation and testing without robot hardware."