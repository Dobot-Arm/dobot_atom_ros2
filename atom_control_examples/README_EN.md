# Atom Control Examples

This is a ROS2 control example package for Dobot Atom robot, providing state reading and basic control functions.

## Features

### State Reading Programs

This package provides four independent state reading programs for monitoring different parts of the robot:

1. **Lower Body State Reader** (`lower_state_reader`)

   - Subscribed topic: `/lower/state`
   - Function: Read the status of 12 lower body motors, IMU data, BMS battery information
   - Includes: Detailed information such as leg joint position, velocity, torque, temperature, etc.
2. **Upper Body State Reader** (`upper_state_reader`)

   - Subscribed topic: `/upper/state`
   - Function: Read the status of 17 upper body motors (robotic arm + head + torso), BMS battery information
   - Includes: Detailed status information of robotic arm, head and torso joints
3. **Joint CAN Board State Reader** (`main_nodes_state_reader`)

   - Subscribed topic: `/main/nodes/state`
   - Function: Read joint CAN board status and EtherCAT slave information
   - Includes: Servo status, error codes, warning codes for left/right legs, torso, left/right arms, head
4. **Dexterous Hand State Reader** (`hands_state_reader`)

   - Subscribed topic: `/hands/state`
   - Function: Read the status of 12 finger motors of dexterous hands
   - Includes: Position, velocity, torque, temperature information for 6 degrees of freedom of each left and right hand

### Control Programs

6. **Full Body Trajectory Tracking Controller** (`atom_full_body_trajectory_controller`)

   - Subscribed topics: `/lower/state`, `/upper/state`, `/hands/state`
   - Published topics: `/lower/cmd`, `/upper/cmd`, `/hands/cmd`, `/set/fsm/id`
   - Function: Full body joint trajectory tracking control (12 leg joints + 17 arm joints + 12 dexterous hand joints)
   - Features:
     - Built-in sine wave trajectory generation
     - Smooth initialization process (5 seconds)
     - Configurable PID parameters
     - Real-time trajectory tracking (10ms control cycle)
     - Loop trajectory playback
7. **Joint Data Recorder** (`atom_joint_recorder`)

   - Subscribed topics: `/lower/state`, `/upper/state`, `/hands/state`
   - Function: Record all joint angle data of the robot to file
   - Features:
     - Configurable recording parameters (duration, frequency, delay)
     - Complete joint data recording (41 joints)
     - Data format with timestamps
     - Detailed file header information
     - Real-time progress display
8. **Dexterous Hand Controller** (`atom_dexterous_hands_controller`)

   - Published topic: `/hands/cmd`
   - Function: Control robot dexterous hands for grasping and releasing actions
   - Features:
     - Synchronized control of both hands (6 degrees of freedom for each left and right hand)
     - Smooth motion trajectory (easing function optimization)
     - Intelligent timing control (thumb delay/advance action)
     - Automatic state switching (grasp ↔ release, 5-second interval)
     - Multi-threaded safe design
     - Complies with finger angle limit range

## Build and Run

### Dependencies

Ensure the following dependencies are installed:

- ROS2 (Humble/Iron/Rolling)
- rclcpp
- dobot_atom (message package)
- std_msgs
- geometry_msgs

### Build

```bash
colcon build 
source install/setup.bash
```

### Run State Reading Programs

```bash
# Lower body state reader
ros2 run atom_control_examples lower_state_reader

# Upper body state reader
ros2 run atom_control_examples upper_state_reader

# Joint CAN board state reader
ros2 run atom_control_examples main_nodes_state_reader

# Dexterous hand state reader
ros2 run atom_control_examples hands_state_reader
```

### Run Control Programs

```bash
# Dexterous hand controller
ros2 run atom_control_examples atom_dexterous_hands_controller
```

## Topic Description

### State Topics (Subscribed)

- `/lower/state` - Lower body state information
- `/upper/state` - Upper body state information
- `/main/nodes/state` - Joint CAN board status
- `/hands/state` - Dexterous hand status
- `/fsm/status` - State machine status

### Control Topics (Published)

- `/lower/cmd` - Lower body control commands
- `/upper/cmd` - Upper body control commands
- `/hands/cmd` - Dexterous hand control commands
- `/set/fsm/id` - Set FSM state ID
- `/switch/upper/control` - Switch upper body control authority
- `/cmd_vel` - Velocity control commands

**Note**: Topic names in ROS2 do not include the `rt` prefix, which is different from the original DDS interface.

## Safety Precautions

⚠️ **Important Safety Reminders**:

1. Ensure the robot is in a safe environment before running control programs
2. Be ready to press the emergency stop button at any time
3. It is recommended to reduce control parameters when running for the first time
4. Ensure there is sufficient activity space around the robot
5. Check that the robot hardware status is normal before running

## Joint Information

### Robot Joint Layout

- **Lower body joints**: 12 joints (6 for each left and right leg)
- **Upper body joints**: 17 joints (7 for each left and right arm + 2 for head + 1 for torso)
- **Dexterous hand joints**: 12 joints (6 for each left and right hand)

### Joint Limits

For detailed joint limit information, please refer to the `Atom关节顺序名称与关节限位.md` file.

## Program Parameters

### Trajectory Control Parameters

- `trajectory_amplitude`: Trajectory amplitude
- `trajectory_frequency`: Trajectory frequency
- `control_frequency`: Control frequency
- `leg_kp`, `leg_kd`: Leg PID parameters
- `arm_kp`, `arm_kd`: Arm PID parameters

### Recorder Parameters

- `record_duration`: Recording duration
- `record_frequency`: Recording frequency
- `start_delay`: Start delay
- `output_file`: Output file path

### Dexterous Hand Control Parameters

- `STATE_SWITCH_INTERVAL`: State switching interval (seconds)
- `MOTION_STEPS`: Total motion steps
- `STEP_DELAY_MS`: Step delay (milliseconds)
- `THUMB_DELAY_STEPS`: Thumb delay steps during grasping
- `THUMB_LEAD_STEPS`: Thumb advance completion steps during release
- `CONTROL_FREQUENCY`: Control frequency (Hz)

### Information Print Control

Each program has corresponding information print switches:

- `INFO_LOWER_STATE`: Lower body state information print
- `INFO_UPPER_STATE`: Upper body state information print
- `INFO_MAIN_NODES`: Joint CAN board information print
- `INFO_HANDS_STATE`: Dexterous hand information print

## Troubleshooting

### Common Issues

1. **Topic Connection Failure**

   - Check if the `dobot_atom` package is correctly installed
   - Confirm robot hardware connection is normal
   - Check if topic names are correct (do not include `rt` prefix)
2. **Build Errors**

   - Ensure all dependencies are installed
   - Check if ROS2 environment is correctly set
   - Confirm `dobot_atom` message package has been built
3. **Control No Response**

   - Check if the robot is in the correct control mode
   - Confirm upper body control authority has been correctly switched
   - Check if FSM state is correctly set

### Debug Commands

```bash
# View available topics
ros2 topic list

# View topic information
ros2 topic info /upper/state

# Listen to topic data
ros2 topic echo /upper/state

```

## Extended Development

### Adding New Control Modes

1. Add new control logic in the corresponding controller file
2. Modify control callback functions
3. Add corresponding parameter configuration
4. Update CMakeLists.txt file

### Creating New State Reading Programs

1. Refer to existing state reading program structure
2. Subscribe to corresponding state topics
3. Implement parsing and display of state information
4. Add new executable files in `CMakeLists.txt`