# Dobot Atom Robot ROS2 Support Package

This project provides complete ROS2 support for the Dobot Atom humanoid robot, including robot models, simulation environments, control interfaces, and example programs.

**Update Date**: 2025-9-2
**Maintainer**: dobot_futingxing

## 📋 Table of Contents

- [Environment Configuration](#environment-configuration)
- [Installation Instructions](#installation-instructions)
- [Quick Start](#quick-start)
- [Package Description](#package-description)
- [Troubleshooting](#troubleshooting)

## 🚀 Environment Configuration

### System Requirements

System and ROS2 versions:

| System       | ROS2 Version |
| ------------ | ------------ |
| Ubuntu 20.04 | Foxy         |
| Ubuntu 22.04 | Humble       |

### Dependency Installation

#### **Install ROS2**

##### **1. Set Encoding**

```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

##### **2. Add Sources**

```bash
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

##### **3. Install ROS2**

```bash
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
```

##### **4. Set Environment Variables**

```bash
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

##### **5. Turtlesim Simulation Example**

Start two terminals and run the following commands respectively:

```bash
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key
```

- The first command will start a turtle simulator with a blue background.
- The second command will start a keyboard control node, using the "up, down, left, right" keys on the keyboard to control the turtle's movement.

![turtlesim](/image/image2544.png)

---

#### **Gazebo Installation**

##### **Installation**

```bash
sudo apt install ros-humble-gazebo-*
```

##### **Add Environment Variables**

```bash
echo "source /usr/share/gazebo/setup.bash" >> ~/.bashrc
```

##### **Run**

```bash
ros2 launch gazebo_ros gazebo.launch.py
```

---

#### **MoveIt Installation**

##### **Installation**

```bash
sudo apt-get install ros-humble-moveit
```

#### 2. Install Dependencies

```bash
# ROS2 control related
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
# URDF and visualization tools
sudo apt install ros-humble-urdf ros-humble-xacro
sudo apt install ros-humble-robot-state-publisher ros-humble-joint-state-publisher
sudo apt install ros-humble-joint-state-publisher-gui

# Build tools
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep python3-vcstool

# DDS implementation
sudo apt install ros-humble-rmw-cyclonedx-cpp

```

## 📦 Installation Instructions

### 1. Clone Repository

```bash
# Create workspace
mkdir -p ~/atom_ros2_ws/src
cd ~/atom_ros2_ws/src

# Clone this repository
git clone https://github.com/Dobot-Arm/dobot_atom_ros2.git
```

### 2. Install Dependencies

```bash
cd ~/atom_ros2_ws

# Initialize rosdep (if using for the first time)
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

# For permanent effect, write to environment variables
sudo gedit ~/.bashrc
```

## 🔧 Configuration Instructions

### Network Configuration

If connecting to a real robot, network interface configuration is required:

1. Set network segment to 192.168.8.xx:
   ![rviz](/image/IP.jpg)
2. View network interface:

```bash
ifconfig
# or
ip addr show
```

3. Modify the network interface name in `setup.sh`:

```bash
# Edit configuration file
gedit ~/setup.sh

# Modify "eth0" in this line to the actual network interface name
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
    <NetworkInterface name="eth0" priority="default" multicast="default" />
</Interfaces></General></Domain></CycloneDDS>'
```

### Connection Test

After completing the above configuration, restart `ros2 daemon`: `ros2 daemon stop` then execute `ros2 daemon start`

```bash
source ~/setup.sh
ros2 topic list
```

You can see the following topics:

![topic](/image/topic.jpg)

Open terminal and view any topic: ros2 topic echo /xxxx. If there is data, it means communication is normal, for example:

![topic_info.](/image/topic_info.jpg)

## 🎯 Quick Start

Please refer to README.md in each functional module

## 📁 Package Description

### atom_urdf

- **Function**: URDF/XACRO model definition for Atom robot
- **Contents**:
  - `urdf/atom.urdf`: Main robot model file containing complete robot geometric structure and joint definitions
  - `urdf/atom.xacro`: Parameterized robot model file supporting configurable robot description
  - `urdf/atom_gazebo.xacro`: Gazebo simulation specific configuration including physical properties and sensor definitions
  - `meshes/`: Robot 3D mesh files for visualization and collision detection
  - `config/`: Robot parameter configuration files

### atom_gazebo

- **Function**: Gazebo simulation environment support providing complete robot simulation functionality
- **Contents**:
  - `launch/atom_gazebo.launch.py`: Complete Gazebo simulation launch file including world environment and robot loading
  - `launch/atom_arms_only.launch.py`: Arms-only simulation launch file
  - `config/atom_moveit.rviz`: MoveIt integrated RViz configuration file
  - `scripts/control_arms.py`: Dual arm control script supporting trajectory planning and execution
  - Supports ROS2 Control framework providing standardized robot control interfaces

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

#### 2. DDS Optimization Configure Appropriate Network Interface

- Adjust DDS domain ID

## 📚 References

- [ROS2 Official Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Simulation Tutorial](http://gazebosim.org/tutorials)
- [ros2_control Documentation](https://control.ros.org/)
- [URDF Tutorial](http://wiki.ros.org/urdf/Tutorials)

## 📞 Support

If you have issues, please:

1. Check the troubleshooting section of this README
2. Search existing Issues
3. Create a new Issue with detailed information

---

**Note**: Please adjust configuration files and example code according to actual robot hardware interfaces and communication protocols.
