# Dobot Atom Robot ROS2 Support Package

This project provides complete ROS2 support for the Dobot Atom humanoid robot, including robot models, simulation environment, control interfaces, and example programs.

## 📋 Table of Contents

- [Environment Configuration](#environment-configuration)
- [Installation Instructions](#installation-instructions)
- [Quick Start](#quick-start)
- [Package Description](#package-description)
- [Usage Examples](#usage-examples)
- [Troubleshooting](#troubleshooting)

## 🚀 Environment Configuration

### System Requirements

Tested systems and ROS2 versions:

| System       | ROS2 Version |
| ------------ | ------------ |
| Ubuntu 20.04 | Foxy         |
| Ubuntu 22.04 | Humble       |

### Dependency Installation

#### 1. Install ROS2

Note: This installation method is for reference only. If installation errors occur, please search for tutorials and install by yourself.

Using ROS2 Humble as an example (recommended):

```bash
# Add ROS2 apt repository
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# Set environment variables
source /opt/ros/humble/setup.bash
echo " source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

#### 2. Install Dependencies

```bash
# ROS2 control related
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt install ros-humble-gazebo-*
sudo apt-get install ros-humble-moveit

# Set environment variables
echo "source /usr/share/gazebo/setup.bash" >> ~/.bashrc

# URDF and visualization tools
sudo apt install ros-humble-urdf ros-humble-xacro
sudo apt install ros-humble-robot-state-publisher ros-humble-joint-state-publisher
sudo apt install ros-humble-joint-state-publisher-gui

# Build tools
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep python3-vcstool

# DDS implementation (optional, for better performance)
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

## 📦 Installation Instructions

### 1. Clone Repository

```bash
# Create workspace
mkdir -p ~/atom_ros2_ws/src
cd ~/atom_ros2_ws/src

# Clone this repository
git clone <your-repository-url> atom_ros2
```

### 2. Install Dependencies

```bash
cd ~/atom_ros2_ws

# Initialize rosdep (if first time using)
sudo rosdep init
rosdep update

# Install package dependencies
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build Workspace

```bash
cd ~/atom_ros2_ws
colcon build

# Set environment variables
source install/setup.bash
```

### 4. Configure Environment

```bash
# Copy configuration files to user directory
cp src/atom_ros2/setup*.sh ~/

# Choose configuration file as needed:
# 1. Connect to real robot
source ~/setup.sh

# 2. Local simulation environment
source ~/setup_local.sh

# 3. Default configuration
source ~/setup_default.sh

# For permanent effect, add to environment variables
sudo gedit ~/.bashrc
```

## 🎯 Quick Start

### 1. Launch Gazebo Simulation

```bash
# Set environment
source ~/atom_ros2_ws/install/setup.bash
source ~/setup_local.sh

# Launch Gazebo simulation
ros2 launch atom_gazebo atom_gazebo.launch.py
```

### 2. Launch RViz Visualization

```bash
# New terminal
source ~/atom_ros2_ws/install/setup.bash
ros2 launch atom_urdf display.launch.py
```

### 3. Test Waving Motion

```bash
# New terminal
source ~/atom_ros2_ws/install/setup.bash
ros2 run atom_gazebo atom_wave_controller.py

# Or run test script
ros2 run atom_gazebo test_wave.py
```

## 📁 Package Description

### atom_urdf

- **Function**: URDF/XACRO model definition for Atom robot
- **Contents**:
  - `urdf/atom.urdf`: Main robot model file in URDF format
  - `urdf/atom.xacro`: Main robot model file in XACRO format
  - `urdf/atom_gazebo.xacro`: Gazebo simulation configuration

### atom_gazebo

- **Function**: Gazebo simulation support
- **Contents**:
  - `launch/atom_gazebo.launch.py`: Gazebo simulation launch file
  - `config/atom_controllers.yaml`: Controller configuration
  - `scripts/atom_wave_controller.py`: Waving motion controller
  - `scripts/test_wave.py`: Waving test script

### dobot_atom

- **Function**: Atom robot message definitions
- **Contents**:
  - `msg/`: Custom message types
  - Robot state and control message definitions

    ![interface](/image/interface.jpg)

### dobot_atom_rviz

- **Function**: RViz visualization configuration
- **Contents**:
  - `launch/dobot_rviz.launch.py`: RViz launch file
  - `rviz/`: RViz configuration files

    ![rviz](/image/rviz.jpg)

### atom_control_examples

- **Function**: Control example programs (detailed description in atom_control_examples/README.md)
- **Contents**:
  - Various control algorithm examples
  - Motion planning examples
  - Sensor data processing examples

## 🎮 Usage Examples

### Simulation Joint Control

```bash
# Control arm joints
ros2 topic pub /arm_position_controller/commands std_msgs/msg/Float64MultiArray \
  "data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.5, 0.0, 1.5, 0.0, 0.0]"

# Control leg joints
ros2 topic pub /leg_position_controller/commands std_msgs/msg/Float64MultiArray \
  "data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"

# Control head joints
ros2 topic pub /head_position_controller/commands std_msgs/msg/Float64MultiArray \
  "data: [0.0, 0.0]"
```

### Status Monitoring

```bash
# View joint states
ros2 topic echo /joint_states

# View available controllers
ros2 control list_controllers

# View topic list
ros2 topic list
```

### Custom Motion

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class CustomMotionController(Node):
    def __init__(self):
        super().__init__('custom_motion_controller')
        self.arm_pub = self.create_publisher(
            Float64MultiArray, 
            '/arm_position_controller/commands', 
            10
        )
  
    def move_arms(self, positions):
        msg = Float64MultiArray()
        msg.data = positions
        self.arm_pub.publish(msg)

def main():
    rclpy.init()
    controller = CustomMotionController()
  
    # Custom arm positions
    arm_positions = [0.0] * 14  # 14 arm joints
    arm_positions[8] = -1.0     # Raise right shoulder
    arm_positions[11] = 1.5     # Bend right elbow
  
    controller.move_arms(arm_positions)
    rclpy.spin_once(controller)
  
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 🔧 Configuration Instructions

### Network Configuration

If connecting to a real robot, network interface configuration is required:

   1. Set network segment to 192.168.8.xx:
   ![rviz](/image/IP.jpg)

2. Check network interface:

```bash
ifconfig
# or
ip addr show
```

3. Modify network interface name in `setup.sh`:

```bash
# Edit configuration file
gedit ~/setup.sh

# Modify "eth0" in this line to the actual network interface name
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
    <NetworkInterface name="eth0" priority="default" multicast="default" />
</Interfaces></General></Domain></CycloneDDS>'
```

### Connection Test

After completing the above configuration, restart the `ros2 daemon`: `ros2 daemon stop` then execute `ros2 daemon start`

```bash
source ~/setup.sh
ros2 topic list
```

You can see the following topics:

![topic](/image/topic.jpg)

Open a terminal and check any topic: ros2 topic echo /xxxx. If there is data, communication is normal. For example:

![topic_info.](/image/topic_info.jpg)

### Virtual Control Configuration

Configuration file is located at `atom_gazebo/config/atom_controllers.yaml`, including:

- `joint_state_broadcaster`: Joint state broadcaster
- `arm_position_controller`: Arm position controller
- `leg_position_controller`: Leg position controller
- `head_position_controller`: Head position controller

## 🐛 Troubleshooting

### Common Issues

#### 1. Gazebo Launch Failure

```bash
# Check if Gazebo is correctly installed
gazebo --version

# Reinstall Gazebo
sudo apt install gazebo
```

#### 2. Controller Loading Failure

```bash
# Check controller status
ros2 control list_controllers

# Manually load controller
ros2 control load_controller joint_state_broadcaster
ros2 control set_controller_state joint_state_broadcaster active
```

#### 3. Network Connection Issues

```bash
# Check DDS discovery
ros2 daemon stop
ros2 daemon start

# Check topics
ros2 topic list
```

#### 4. Compilation Errors

```bash
# Clean and rebuild
cd ~/atom_ros2_ws
rm -rf build install log
colcon build
```

### Performance Optimization

#### 1. Gazebo Performance

- Close unnecessary GUI panels
- Reduce physics engine update frequency
- Use headless mode: `gui:=false`

#### 2. DDS Optimization

- Configure appropriate network interface
- Adjust DDS domain ID

## 📚 References

- [ROS2 Official Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Simulation Tutorial](http://gazebosim.org/tutorials)
- [ros2_control Documentation](https://control.ros.org/)
- [URDF Tutorial](http://wiki.ros.org/urdf/Tutorials)

## 📄 License

This project is licensed under the Apache License 2.0.

## 📞 Support

If you have any questions, please:

1. Check the troubleshooting section of this README
2. Search existing Issues
3. Create a new Issue with detailed information

**Update Date**: 2025-9-2
**Maintainer**: dobot_futingxing

---

**Note**: Please adjust configuration files and example code according to the actual robot hardware interface and communication protocol.