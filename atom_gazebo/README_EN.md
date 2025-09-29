# Atom Gazebo Simulation Package

This package provides complete Gazebo simulation environment support for the Dobot Atom humanoid robot, including robot model loading, controller configuration, and simulation environment setup.

## 🚀 Features

- **Complete Simulation Environment**: Provides complete robot simulation in Gazebo
- **ROS2 Control Integration**: Supports standardized robot control interfaces
- **Dual Arm Control**: Independent left and right arm trajectory planning and execution
- **Flexible Configuration**: Supports both full-body and arms-only simulation modes

## 📁 File Structure

```
atom_gazebo/
├── CMakeLists.txt              # Build configuration file
├── package.xml                 # Package dependency declaration
├── launch/                     # Launch files directory
│   ├── atom_gazebo.launch.py   # Complete simulation launch file
│   └── atom_arms_only.launch.py # Arms-only simulation launch file
├── config/                     # Configuration files directory
│   └── atom_moveit.rviz        # MoveIt RViz configuration
└── scripts/                    # Control scripts directory
    └── control_arms.py         # Dual arm control script
```

## 🎯 Quick Start

### 1. Launch Simulation

#### Complete Robot Simulation

```bash
ros2 launch atom_gazebo atom_gazebo.launch.py
```

#### Arms-Only Simulation

```bash
ros2 launch atom_gazebo atom_arms_only.launch.py
```

## 🔧 Launch Files Description

### atom_gazebo.launch.py

Complete Gazebo simulation launch file, including:

- **Parameter Configuration**:

  - `use_sim_time`: Use simulation time (default: true)
  - `gui`: Whether to display Gazebo GUI (default: true)
  - `headless`: Run in headless mode (default: false)
- **Functional Modules**:

  - Gazebo world environment loading
  - Robot URDF model loading
  - Robot State Publisher startup
  - ROS2 Control controller management

### atom_arms_only.launch.py

Arms-only simulation launch file, suitable for:

- Arm control algorithm development
- Lightweight simulation testing
- Motion planning verification

## 🎮 Control Scripts

### control_arms.py

Dual arm control script providing the following features:

#### Main Features

- **Independent Dual Arm Control**: Supports independent trajectory planning for left and right arms
- **Trajectory Interpolation**: Automatic smooth trajectory interpolation
- **Status Monitoring**: Real-time monitoring of arm execution status
- **Error Handling**: Comprehensive error detection and recovery mechanisms

#### Usage

```bash
# After launching simulation, run the control script
ros2 run atom_gazebo control_arms.py
```

#### Control Interface

The script controls arms through the following Action interfaces:

- `/left_arm_controller/follow_joint_trajectory`
- `/right_arm_controller/follow_joint_trajectory`

#### Predefined Actions

The script includes the following predefined action sequences:

1. **Rise Action**: Arms move upward
2. **Lower Action**: Arms move downward
3. **Reset Action**: Return to initial position

## ⚙️ Configuration Files

### atom_moveit.rviz

MoveIt integrated RViz configuration file, including:

- **Display Configuration**:

  - Robot model display
  - Motion planning visualization
  - Trajectory display
  - Collision detection visualization
- **Interactive Tools**:

  - Interactive markers
  - Motion planning panel
  - Scene planning tools

## 💡 Usage Examples

### Example 1: Basic Simulation Launch

```bash
# Terminal 1: Launch Gazebo simulation
ros2 launch atom_gazebo atom_gazebo.launch.py

# Terminal 2: Launch RViz visualization
ros2 launch dobot_atom_rviz atom_rviz.launch.py

# Terminal 3: Run control script
ros2 run atom_gazebo control_arms.py
```

### Example 2: Headless Mode Simulation

```bash
# Launch without GUI, suitable for server environments
ros2 launch atom_gazebo atom_gazebo.launch.py gui:=false headless:=true
```

### Example 3: Arm Control Testing

```bash
# Launch arms-only simulation
ros2 launch atom_gazebo atom_arms_only.launch.py

# Check controller status
ros2 control list_controllers

# Manually send trajectory commands
ros2 topic pub /left_arm_controller/follow_joint_trajectory/goal ...
```

## 🔍 Topics and Services

### Main Topics

- `/joint_states`: Joint state information
- `/robot_description`: Robot model description
- `/left_arm_controller/follow_joint_trajectory/goal`: Left arm trajectory goal
- `/right_arm_controller/follow_joint_trajectory/goal`: Right arm trajectory goal

### Controller Services

- `/controller_manager/list_controllers`: List all controllers
- `/controller_manager/load_controller`: Load controller
- `/controller_manager/switch_controller`: Switch controller state

## 🐛 Troubleshooting

### Common Issues

#### 1. Gazebo Launch Failure

```bash
# Check Gazebo version
gazebo --version

# Reinstall Gazebo
sudo apt install --reinstall gazebo
```

#### 2. Controller Loading Failure

```bash
# Check controller status
ros2 control list_controllers

# Manually load controller
ros2 control load_controller joint_state_broadcaster
ros2 control set_controller_state joint_state_broadcaster active
```

#### 3. Robot Model Not Displaying

```bash
# Check robot_description topic
ros2 topic echo /robot_description

# Check TF tree
ros2 run tf2_tools view_frames
```

#### 4. Trajectory Execution Failure

```bash
# Check Action server status
ros2 action list

# View error logs
ros2 topic echo /left_arm_controller/follow_joint_trajectory/feedback
```

### Performance Optimization

#### 1. Gazebo Performance Optimization

- Disable unnecessary plugins
- Reduce physics engine update frequency
- Use simplified collision models

#### 2. Controller Optimization

- Adjust trajectory interpolation parameters
- Optimize controller update frequency
- Use appropriate PID parameters

## 📚 Extension Development

### Adding New Controllers

1. Define controller in `atom_urdf/config/atom_controllers.yaml`
2. Modify launch file to load new controller
3. Write corresponding control scripts

### Custom Simulation Environment

1. Create new world file (.world)
2. Modify world file path in launch file
3. Add environment objects and sensors

**Maintainer**: Dobot_futingxing  
**Last Updated**: 2025-09-02