# Dobot Atom ROS2 DDS Interface

## Overview

This package provides ROS2 DDS interface message definitions for the Dobot humanoid robot, supporting split upper/lower body control. The interface is based on the DDS communication protocol and provides real-time robot status monitoring and control capabilities.

## Package Information

- **Package**: dobot_atom
- **Version**: 1.1.2.0-beta1
- **Maintainer**: futingxing<futingxing@dobot_robots.com>

## Message Types

### Basic Messages

#### BmsState.msg

Battery Management System status information

```
uint16 bms_state                    # BMS state
uint16 afe_state                    # AFE chip state
uint32 bms_alarms                   # BMS fault codes
uint16 battery_level                # Battery level percentage
uint16 battery_health               # Battery health
uint16 pcb_board_temp               # PCB board temperature
uint16 afe_chip_temp                # AFE chip temperature
uint16 battery_now_current          # Battery pack current
uint16[16] cells_voltage            # 16 cell voltages
uint16 battery_pack_current_voltage # Battery pack voltage
uint16 battery_pack_io_voltage      # Battery pack discharge/charge port voltage
uint32 bms_work_time                # BMS runtime
uint16 bms_hardware_version         # BMS hardware version
uint16 bms_software_version         # BMS software version
uint16 heartbeat                    # Heartbeat
```

#### BmsCmd.msg

Battery Management System control command

```
uint16 clear_errors                 # Clear BMS errors
uint16 spare_battery_charge_request # Spare battery charge request
```

#### BmsInfo.msg

Battery Management System info

```
uint16 latch_state                  # Latch state
uint16 spare_battery_level          # Spare battery level
uint16 spare_battery_exchange_complete  # Spare battery exchange complete flag
BmsLog log                          # BMS log
```

#### BmsLog.msg

Battery Management System log

```
uint16 log_index                    # Log index
uint16 log_capacity                 # Log capacity
uint32 log_bms_runtime              # BMS runtime
uint32 log_bms_alarm                # BMS alarm
uint16 log_bms_status               # BMS status
uint16 log_afe_status               # AFE status
uint16 log_bms_soc                  # Battery SOC
uint16 log_bms_vtop                 # Top voltage
uint16 log_bms_vpack                # Pack voltage
uint16 log_bms_current              # Battery current
uint16[16] log_bms_vol_ceil         # 16 cell voltages
uint16 log_bms_temp_pcb             # PCB temperature
uint16 log_bms_temp_afe             # AFE temperature
uint16 log_bms_temp_ceil            # Cell temperature
uint16 log_bms_reserve_flag         # Reserved flag
uint32 log_bms_rtc_time             # RTC timestamp
```

#### MotorState.msg

Motor state information

```
uint8 mode          # Mode
float32 q           # Joint position
float32 dq          # Joint velocity
float32 ddq         # Joint acceleration
float32 tau_est     # Estimated torque
float32 q_raw       # Raw joint position
float32 dq_raw      # Raw joint velocity
float32 ddq_raw     # Raw joint acceleration
uint8 mcu_temp      # Servo control board temperature
uint8 mos_temp      # MOS tube temperature
uint8 motor_temp    # Motor temperature
uint8 bus_voltage   # Bus voltage
```

#### MotorCmd.msg

Motor control command

```
uint8 mode      # Mode
float32 q       # Position
float32 dq      # Velocity
float32 tau     # Torque
float32 kp      # Proportional gain
float32 kd      # Derivative gain
```

#### IMUState.msg

IMU sensor state

```
float32[4] quaternion    # Quaternion
float32[3] gyroscope     # Gyroscope data
float32[3] accelerometer # Accelerometer data
float32[3] rpy          # Euler angles (roll, pitch, yaw)
uint8 temperature       # Temperature sensor data
```

#### JoystickValue.msg

Joystick value information

```
float32 x   # X-axis value [-1, 1]
float32 y   # Y-axis value [-1, 1]
```

#### LaserScan.msg

LiDAR scan data

```
uint64 timestamp       # Timestamp
int32[1024] lidar      # 1024-point LiDAR data
```

#### Odom.msg

Odometry information

```
uint64 timestamp    # Timestamp
int32 x             # X coordinate
int32 y             # Y coordinate
int32 angle         # Angle
```

#### Velocity.msg

Velocity information

```
float32 linear_vel   # Linear velocity
float32 angular_vel  # Angular velocity
```

### Control Interface Messages

#### SetFsmId.msg

Switch low-level state

```
# WorkingState constants
uint8 HIDLE=0          # Idle state
uint8 HTASKING=1       # Tasking state

uint16 id              # Corresponding algorithm FSM ID
string current_action  # Current action being executed by algorithm
uint8 state            # Working state (HIDLE/HTASKING)
```

#### SwitchUpperControl.msg

Switch upper body control authority

```
bool flag  # true: upper body has control, false: upper body has no control
```

#### EnableMotors.msg

Motor enable control

```
bool[29] flag  # Enable flags for 29 joints, true: enable, false: disable
```

#### PosControlCmd.msg

Position control joint command

```
# PosControlCommand constants
uint8 SWITCH_MODE=0
uint8 MOVE_L=1
uint8 MOVE_J=2
uint8 RUN_TO=3
uint8 CART_JOG=4
uint8 STOP_MOTION=5

uint8 command_type
float64[<=7] left_q
float64[<=7] right_q
float64[<=3] hips_q
float64[<=6] left_pose
float64[<=6] right_pose
float64[<=1] hips_height
int32[<=6] left_cart_jog
int32[<=6] right_cart_jog
int32[<=1] hips_cart_jog
int32 stop
float64 speed
int32 target_mode
uint32 command_id
uint64 timestamp
```

#### CartTargetPose.msg

Position control target Cartesian pose

```
float64[6] left_pose
float64[6] right_pose
float64 hips_height
```

#### PosControlJointQ.msg

Position control joint angles

```
float64[7] left_q
float64[7] right_q
float64[3] hips_q
```

#### PosControlState.msg

Position control application state

```
int32 CONTROL_MODE_MIT=0
int32 CONTROL_MODE_POSITION=1
int32 ERROR_OK=0
int32 ERROR_INIT_FAILED=1
int32 ERROR_SERVO_ENABLE=2
int32 ERROR_IK_FAILED=3
int32 ERROR_PLAN_FAILED=4
int32 ERROR_INVALID_DIMENSION=5
int32 ERROR_REF_SPEED_LIMIT=6
int32 STATUS_READY=0
int32 STATUS_RUNNING=1
int32 STATUS_PLAN_STOP=2
int32 STATUS_ERROR=3

CartTargetPose pose
int32[3] error_code
int32[3] status_code
int32 control_mode
PosControlJointQ joint_q
```

### Upper Body Messages

#### UpperState.msg

Upper body state information

```
char[16] robot_type             # Product-defined robot type
bool is_upper_control           # Upper body control status
uint16 fsm_id                   # Corresponding algorithm FSM ID
MotorState[17] motor_state      # 17 motor states
BmsState bms_state              # Battery state
uint8[40] wireless_remote       # Gamepad key values
uint32 reserve                  # Reserved by Dobot
```

#### UpperCmd.msg

Upper body control command

- Left arm 7 joints (index 0-6)
- Right arm 7 joints (index 7-13)
- Head yaw (index 14)
- Head pitch (index 15)
- Torso twist (index 16)

```
MotorCmd[17] motor_cmd  # 17 motor commands
```

### Lower Body Messages

#### LowerState.msg

Lower body state information

```
uint16 fsm_id                   # Corresponding algorithm FSM ID
IMUState imu_state              # IMU state
MotorState[12] motor_state      # 12 motor states
BmsState bms_state              # Battery state
uint8[40] wireless_remote       # Gamepad key values
uint32 reserve                  # Reserved by Dobot
```

#### LowerCmd.msg

Lower body control command

- Left leg 7 joints (index 0-5)
- Right leg 7 joints (index 6-11)

```
MotorCmd[12] motor_cmd  # 12 motor commands
```

### Full Body Control Messages

#### LowState.msg

Full body low-level state information

```
uint16 algs_mode                # Algorithm mode
IMUState imu_state              # IMU state
MotorState[30] motor_state      # 30 motor states
BmsState bms_state              # Battery state
uint8[40] wireless_remote       # Gamepad key values
uint32 reserve                  # Reserved by Dobot
```

#### LowCmd.msg

Full body low-level control command

```
uint16 algs_mode                # Algorithm mode
MotorCmd[30] motor_cmd          # 30 motor commands
```

### Wheeled Chassis (AMR) Messages

#### AMRState.msg

Chassis state information

```
# NavigationStatus constants
uint8 UNKNOWN=0
uint8 QUEUING=1
uint8 RUNNING=2
uint8 COMPLETED=3
uint8 FAILED=4
uint8 PAUSED=5
uint8 CANCELED=6
uint8 WAITING_CONFIRM=7
uint8 IDLE=8
uint8 STOPPED=9

# DeviceStatus constants
uint8 DEVUNKNOWN=0
uint8 DEVIDLE=1
uint8 TASKING=2
uint8 ERROR=3
uint8 OFFLINE=4
uint8 INIT=5
uint8 CHARGING=6
uint8 UPGRADE=7

uint8 device_status             # Device status
uint8 navigation_status         # Navigation status
AMRBasicStatus basic_status     # Basic status
float32[3] position             # Current position {x, y, yaw}
Velocity velocity               # Velocity information
AMREventStatus amr_event        # AMR events
uint32[32] error_code           # Error codes
uint32 task_id                  # Task ID
uint32 work_mode                # Robot work mode
```

#### AMRCommand.msg

Chassis control command

```
# AMR Command Types
uint8 CANCEL_TASK=0
uint8 PAUSE_TASK=1
uint8 RESUME_TASK=2
uint8 MOVE_TO_TAG=3
uint8 MOVE_TO_CHARGE=4
uint8 REMOTE_CONTROL=5
uint8 ROTATE=6
uint8 START_REMOTE=7
uint8 STOP_REMOTE=8
uint8 SUBSCRIBE_LASER=9
uint8 UNSUBSCRIBE_LASER=10
uint8 START_MAPPING=11
uint8 SAVE_MAP=12
uint8 STOP_MAPPING=13
uint8 SET_VEL=14
uint8 SET_ACCEL=15
uint8 SET_DECEL=16

uint8 command_type              # Command type
uint32 target_id                # Target ID
float32 linear_vel              # Linear velocity
float32 angular_vel             # Angular velocity
uint32 command_id               # Command ID
uint64 timestamp                # Timestamp
float32 theta                   # Angle
```

#### AMRBasicStatus.msg

Chassis basic status

```
float32 battery_level           # Battery level
float32 battery_voltage         # Battery voltage
float32 battery_current         # Battery current
uint16 heartbeat                # Heartbeat value
```

#### AMREventStatus.msg

Chassis event status

```
bool emergency_stop_pressed     # Emergency stop button
bool enable_pressed             # Enable button
bool path_blocked               # Path blocked
bool low_battery                # Low battery
bool obstacle_detected          # Obstacle detected
```

### System Status Messages

#### AxisStateInfo.msg

Detailed joint state information

```
uint8 servo_state           # Servo enable/disable/error state
uint16 error_code           # Error code
uint16 warn_code            # Warning code
int32 pos_err_code          # Position over-limit error code
int32 vel_err_code          # Velocity over-limit error code
int32 torque_err_code       # Torque over-limit error code
uint8 node_state            # Joint online state
uint8 display_op_mode       # Joint status word
bool is_virtual             # Virtual/physical axis flag
uint8 mcu_temp              # MCU temperature
uint8 mos_temp              # MOS temperature
uint8 motor_temp            # Motor temperature
uint8 bus_voltage           # Bus voltage
uint16 software_version     # Software version
```

#### EcatSlaveInfo.msg

EtherCAT slave information

```
bool is_virtual             # Virtual/physical flag
uint8 slave_state           # Slave state
uint16 error_code           # Error code
uint16 software_version     # Software version
```

#### MainNodesState.msg

Main node states

```
AxisStateInfo[6] left_leg   # 6 left leg joint states
AxisStateInfo[6] right_leg  # 6 right leg joint states
AxisStateInfo waist         # Waist state
AxisStateInfo[7] left_arm   # Left arm state
AxisStateInfo[7] right_arm  # Right arm state
AxisStateInfo[2] head       # Head state
EcatSlaveInfo[2] ecat2can   # 2 EtherCAT-to-CAN module states
```

#### ClearErrors.msg

Clear errors command

```
int32 msg_id  # Message ID (any value)
```

#### EmergencyState.msg

Emergency stop state information

```
bool soft_emergency_triggered   # Soft emergency (app triggered)
bool hard_emergency_triggered   # Hard emergency (user board triggered)
bool amr_emergency_triggered    # AMR emergency (chassis triggered)
bool di_emergency_triggered     # DI emergency (sensor triggered)
```

### Dexterous Hand Messages

#### HandsState.msg

Dexterous hand state

```
MotorState[12] hands  # 12 finger motor states
```

#### HandsCmd.msg

Dexterous hand control command

```
MotorCmd[12] hands  # 12 finger motor commands
```

### Remote Control Messages

#### RemoteControl.msg

Joystick control information

```
JoystickValue btn_move    # Left knob value
JoystickValue btn_turn    # Right knob value
```

### Teleoperation Messages

#### TeleopCmd.msg

Teleoperation control command

```
# TeleopCommandType constants
# 0 - START_SYNC        # Start synchronization
# 1 - STOP_SYNC         # Stop synchronization
# 2 - START_TELEOP      # Start teleoperation
# 3 - STOP_TELEOP       # Stop teleoperation
# 4 - START_RECORD      # Start recording
# 5 - STOP_RECORD       # Stop recording

uint8 command_type
```

#### TeleopState.msg

Teleoperation state

```
uint8 teleop_state       # Teleoperation state
uint8 sync_state         # Sync state
uint8 record_state       # Record state
# TeleopState enum:
# 0 - DISABLED
# 1 - ENABLING
# 2 - ENABLED
```

### Cerebrum Messages

#### CerebrumCmd.msg

Cerebrum control command

```
# CerebrumCommandType constants
# 0 - START_REASONING   # Start reasoning
# 1 - STOP_REASONING    # Stop reasoning
# 2 - BACK_TOSTRT       # Back to start

uint8 command_type
```

#### CerebrumState.msg

Cerebrum state

```
uint8 teleop_state       # Teleop state
uint8 sync_state         # Sync state
uint8 record_state       # Record state
uint8 reasoning_state    # Reasoning state
uint8 backing_state      # Backing state
# ComponentState enum:
# 0 - DISABLE
# 1 - ENABLIN
# 2 - ENABLE
# 3 - RESETTING
# BackingState enum:
# 0 - UNHOMED
# 1 - HOMED
# 2 - RESET_FAILED
# 3 - RESET
```

### Inspire Hand Messages

#### InspireCmd.msg

Inspire dexterous hand control command

```
MotorCmd[12] hands  # 12 finger motor commands
```

#### InspireState.msg

Inspire dexterous hand state

```
MotorState[12] hands  # 12 finger motor states
```

### Power Management Messages

#### PowerCmd.msg

Power control command

```
uint16 control_power_supply     # Control power supply
uint16 clear_errors             # Clear errors
uint16 set_led_colour           # Set LED color
uint16 set_led_mode             # Set LED mode
uint16 set_sd_brightness        # Set screen brightness
uint16 set_sd_mode              # Set screen mode
```

#### PowerState.msg

Power state

```
uint16 power_supply_state           # Power supply state
uint16 strong_power_current         # Strong power current
uint16 weak_power_current           # Weak power current
uint16 weak_power_voltage           # Weak power voltage
uint16 replenishment_port_voltage   # Replenishment port voltage
uint16 hardware_version             # Hardware version
uint16 software_version             # Software version
uint16 error_codes                  # Error codes
uint16 heartbeat                    # Heartbeat
uint16 strong_power_voltage         # Strong power voltage
uint16 brk_2v5_voltage              # Brake 2.5V voltage
uint16 brk_state                    # Brake state
```

### Other Messages

#### ChangeMode.msg / GoHome.msg / CalibrateImu.msg / SetCalibrationFlag.msg / ModifyJointOffset.msg / PlayTTS.msg / DHGripperCmd.msg / DHGripperSingleCmd.msg / DHGripperState.msg / DHGripperSingleState.msg

Auxiliary function messages for mode switching, homing, IMU calibration, joint offset correction, and TTS playback.

## Topic Map

| Topic                      | Message Type       | Description                    |
| -------------------------- | ------------------ | ------------------------------ |
| `rt/set/fsm/id`           | SetFsmId           | Switch low-level state         |
| `rt/switch/upper/control` | SwitchUpperControl | Switch upper body control      |
| `rt/upper/state`          | UpperState         | Upper body state               |
| `rt/upper/cmd`            | UpperCmd           | Upper body control command     |
| `rt/lower/state`          | LowerState         | Lower body state               |
| `rt/lower/cmd`            | LowerCmd           | Lower body control command     |
| `rt/low/state`            | LowState           | Full body low-level state      |
| `rt/low/cmd`              | LowCmd             | Full body low-level control    |
| `rt/amr/state`            | AMRState           | Wheeled chassis state          |
| `rt/amr/cmd`              | AMRCommand         | Wheeled chassis control        |
| `rt/posctl/cmd`           | PosControlCmd      | Position control command       |
| `rt/posctl/state`         | PosControlState    | Position control state         |
| `rt/main/nodes/state`     | MainNodesState     | Joint and CAN board states     |
| `rt/clear/errors`         | ClearErrors        | Clear errors                   |
| `rt/hands/state`          | HandsState         | Dexterous hand state           |
| `rt/hands/cmd`            | HandsCmd           | Dexterous hand control command |
| `rt/remote/control`       | RemoteControl      | Joystick control               |
| `rt/emergency/state`      | EmergencyState     | Emergency stop state           |
| `rt/laserscan`            | LaserScan          | LiDAR scan data                |
| `rt/odom`                 | Odom               | Odometry                       |
| `rt/enable/motors`        | EnableMotors       | Motor enable control           |
| `rt/bms/state`            | BmsState           | BMS state                      |
| `rt/bms/cmd`              | BmsCmd             | BMS control                    |
| `rt/bms/info`             | BmsInfo            | BMS info                       |
| `rt/teleop/cmd`           | TeleopCmd          | Teleoperation control          |
| `rt/teleop/state`         | TeleopState        | Teleoperation state            |
| `rt/cerebrum/cmd`         | CerebrumCmd        | Cerebrum control               |
| `rt/cerebrum/state`       | CerebrumState      | Cerebrum state                 |
| `rt/inspire/cmd`          | InspireCmd         | Inspire hand control           |
| `rt/inspire/state`        | InspireState       | Inspire hand state             |
| `rt/power/state`          | PowerState         | Power state                    |
| `rt/power/cmd`            | PowerCmd           | Power control                  |

## Usage Notes

### 1. Safety Notes

- Before controlling joints, the low-level state must be set to non-zero torque mode via the `rt/set/fsm/id` interface.
- If external control of the upper body is required, upper body control authority must be enabled via the `rt/switch/upper/control` interface.

### 2. Gamepad Key Mapping

Key mapping in the `wireless_remote[40]` array:

- `wireless_remote[2]`: high-to-low bits correspond to '', '', 'LT', 'RT', 'SELECT', 'START', 'LB', 'RB'
- `wireless_remote[3]`: high-to-low bits correspond to 'LEFT', 'DOWN', 'RIGHT', 'UP', 'Y', 'X', 'B', 'A'
- `wireless_remote[4-7]`: LX value (floating point in [-1, 0] range)
- `wireless_remote[8-11]`: RX value (floating point in [0, 1] range)
- `wireless_remote[12-15]`: RY value (floating point in [-1, 0] range)
- `wireless_remote[16-19]`: LY value (floating point in [0, 1] range)

### 3. Position Control Interface Usage

- First send `rt/posctl/cmd` with `command_type=SWITCH_MODE`, `target_mode=1`.
- Poll `rt/posctl/state` until `control_mode == CONTROL_MODE_POSITION`.
- `MOVE_J` uses `left_q/right_q/hips_q`, `MOVE_L` and `RUN_TO` use `left_pose/right_pose/hips_height`.
- `RUN_TO` and `CART_JOG`: start with `stop=0`, stop with `stop=1`, send once each.
- `left_*` / `right_*` / `hips_*` with length 0 means that part does not participate.
- Length constraints are guaranteed by message upper bounds and business logic, e.g., `left_q` max 7, `left_pose` max 6.

### 4. Dexterous Hand Array Ordering

**Joint order (starting from index 0):**

- Right hand (index 0-5): pinky, ring, middle, index, thumb flex, thumb rotate
- Left hand (index 6-11): pinky, ring, middle, index, thumb flex, thumb rotate

**Angle ranges:**

- Pinky, ring, middle, index: 19°~176.7° (0.3316rad~3.0815rad)
- Thumb flex angle: -13°~53.6° (-0.2269rad~0.9346rad)
- Thumb rotate angle: 90°~165° (1.5708rad~2.8798rad)

### 5. Wheeled Chassis (AMR) Usage

#### Remote Control
1. Send `START_REMOTE` request.
2. Send `REMOTE_CONTROL` request, setting `linear_vel` and `angular_vel`.
3. Send `STOP_REMOTE` request when finished.

#### Navigation
1. Send `MOVE_TO_TAG` request, setting `target_id` and `theta`.
2. Check task status via `navigation_status` in `AMRState`:
   - `RUNNING (2)`: In progress
   - `COMPLETED (3)`: Completed

#### LiDAR & Mapping
1. `SUBSCRIBE_LASER` / `UNSUBSCRIBE_LASER` controls LiDAR data subscription.
2. `START_MAPPING` / `STOP_MAPPING` / `SAVE_MAP` controls mapping workflow.
3. `SET_VEL` / `SET_ACCEL` / `SET_DECEL` sets chassis motion parameters.

### 6. Real-time Topic Prefix

All topics requiring real-time publishing are prefixed with `rt/`.

### 7. Motor Enable

Send `EnableMotors` message via `rt/enable/motors` topic to control joint enables. The `flag` array has a length of 29, corresponding to all body joints.

## Build Instructions

```bash
# Build in ROS2 workspace
colcon build --packages-select dobot_atom

source install/setup.bash
```

## Dependencies

- ROS2 (Humble/Iron/Rolling)
- geometry_msgs
- rosidl_default_generators
- rosidl_generator_dds_idl
