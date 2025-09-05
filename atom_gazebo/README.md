# Atom Gazebo Package

This package provides Gazebo simulation support for the Atom humanoid robot, including launch files, controllers, and demonstration scripts.

## Package Contents

### Launch Files
- `atom_gazebo.launch.py`: Main launch file for Gazebo simulation with ros2_control

### Configuration Files
- `config/atom_controllers.yaml`: Controller configuration for ros2_control

### Scripts
- `scripts/atom_wave_controller.py`: Python node for controlling robot waving motion
- `scripts/test_wave.py`: Test script to demonstrate waving functionality

### World Files
- `worlds/empty.world`: Basic empty world for simulation

## Dependencies

Make sure you have the following packages installed:
- `ros2_control`
- `ros2_controllers` 
- `gazebo_ros2_control`
- `controller_manager`
- `gazebo_ros`
- `robot_state_publisher`
- `joint_state_publisher`
- `xacro`
- `atom_urdf` (contains the robot model)

## Usage

### 1. Build the Package

```bash
cd ~/your_ros2_workspace
colcon build --packages-select atom_gazebo
source install/setup.bash
```

### 2. Launch Gazebo Simulation

```bash
ros2 launch atom_gazebo atom_gazebo.launch.py
```

This will:
- Start Gazebo with the empty world
- Load the Atom robot model
- Start the robot state publisher
- Load and activate the ros2_control controllers

### 3. Control the Robot

#### Manual Joint Control

You can control individual joints using ros2 topic commands:

```bash
# Control arm joints
ros2 topic pub /arm_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.5, 0.0, 1.5, 0.0, 0.0]"

# Control leg joints
ros2 topic pub /leg_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"

# Control head joints
ros2 topic pub /head_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0]"
```

#### Waving Motion Demo

Run the waving controller:

```bash
ros2 run atom_gazebo atom_wave_controller.py
```

Or test the waving motion:

```bash
ros2 run atom_gazebo test_wave.py
```

### 4. Monitor Robot State

```bash
# View joint states
ros2 topic echo /joint_states

# List available controllers
ros2 control list_controllers

# Check controller status
ros2 control list_hardware_interfaces
```

## Controller Configuration

The robot uses the following controllers:

- **joint_state_broadcaster**: Publishes joint states to `/joint_states`
- **arm_position_controller**: Controls all arm joints (left + right)
- **leg_position_controller**: Controls all leg joints (left + right)
- **head_position_controller**: Controls head yaw and pitch joints

### Joint Order for Controllers

#### Arm Controller (14 joints):
1. uidx_l_arm_joint1 (left shoulder pitch)
2. left_shoulder_roll_joint
3. left_shoulder_yaw_joint
4. uidx_l_arm_joint2 (left elbow pitch)
5. left_elbow_roll_joint
6. left_wrist_pitch_joint
7. left_wrist_yaw_joint
8. uidx_r_arm_joint1 (right shoulder pitch)
9. right_shoulder_roll_joint
10. right_shoulder_yaw_joint
11. uidx_r_arm_joint2 (right elbow pitch)
12. right_elbow_roll_joint
13. right_wrist_pitch_joint
14. right_wrist_yaw_joint

#### Leg Controller (12 joints):
1. leg_l1_joint through leg_l6_joint (left leg)
2. leg_r1_joint through leg_r6_joint (right leg)

#### Head Controller (2 joints):
1. head_yaw_joint
2. head_pitch_joint

## Troubleshooting

### Controllers Not Loading

If controllers fail to load, check:

```bash
# Check if Gazebo is running
ps aux | grep gazebo

# Check controller manager
ros2 node list | grep controller_manager

# Manually load controllers
ros2 control load_controller joint_state_broadcaster
ros2 control set_controller_state joint_state_broadcaster active
```

### Robot Not Moving

1. Verify controllers are active:
   ```bash
   ros2 control list_controllers
   ```

2. Check joint limits in the URDF/XACRO files

3. Verify topic names:
   ```bash
   ros2 topic list | grep controller
   ```

### Gazebo Performance Issues

- Reduce physics update rate in the world file
- Close unnecessary GUI panels
- Use headless mode: `ros2 launch atom_gazebo atom_gazebo.launch.py gui:=false`

## Development

### Adding New Controllers

1. Update `config/atom_controllers.yaml`
2. Add controller loading in `launch/atom_gazebo.launch.py`
3. Update dependencies in `package.xml` and `CMakeLists.txt`

### Creating Custom Motions

Use `atom_wave_controller.py` as a template for creating new motion controllers.

## License

This package is licensed under the Apache License 2.0.