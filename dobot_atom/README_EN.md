# Dobot Atom ROS2 DDS Interface

## Overview

This package provides ROS2 DDS interface message definitions for the Dobot humanoid robot, supporting separate upper and lower limb control. The interface is built on the DDS communication protocol, providing real-time robot state monitoring and control capabilities.

## Package Information

- **Package**: dobot_atom
- **Version**: 1.2.0.0-beta8
- **Maintainer**: futingxing<futingxing@dobot_robots.com>

## Message Types

### Basic Message Types

#### BmsState.msg

Battery Management System state information

```
uint16 bms_state                    # BMS state
uint16 afe_state                    # AFE chip state
uint32 bms_alarms                   # BMS alarm codes
uint16 battery_level                # Battery level percentage
uint16 battery_health               # Battery health status
uint16 pcb_board_temp               # PCB board temperature
uint16 afe_chip_temp                # AFE chip temperature
uint16 battery_now_current          # Battery pack current
uint16[16] cells_voltage            # 16 cell voltages
uint16 battery_pack_current_voltage # Battery pack voltage
uint16 battery_pack_io_voltage      # Battery pack discharge/charge interface voltage
uint32 bms_work_time                # BMS runtime
uint16 bms_hardware_version         # BMS hardware version
uint16 bms_software_version         # BMS software version
uint16 heartbeat                    # Heartbeat
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
uint8 mcu_temp      # Servo driver MCU temperature
uint8 mos_temp      # MOS temperature
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
float32[4] quaternion    # Quaternion (w, x, y, z)
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

### Control Interface Messages

#### SetFsmId.msg

Switch low-level control state

```
# WorkingState constants
uint8 HIDLE=0           # Idle state
uint8 HTASKING=1        # Tasking state

uint16 id               # Algorithm FSM ID
string current_action   # Current action being executed by the algorithm
uint8 state             # Working state (HIDLE/HTASKING)
```

#### SwitchUpperControl.msg

Switch upper limb control authority

```
bool flag  # true: upper limb has control, false: upper limb has no control
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

Position control joint angles state

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

### Upper Limb Messages

#### UpperState.msg

Upper limb state information

```
char[16] robot_type             # Product type identifier
bool is_upper_control           # Upper limb control status
uint16 fsm_id                   # Algorithm FSM ID
IMUState imu_state              # IMU state
MotorState[17] motor_state      # 17 motor states
BmsState bms_state              # Battery state
uint8[40] wireless_remote       # Wireless remote key values
uint32 reserve                  # Reserved for Dobot
```

#### UpperCmd.msg

Upper limb control command

- Left arm: 7 joints (indices 0-6)
- Right arm: 7 joints (indices 7-13)
- Head yaw (index 14)
- Head pitch (index 15)
- Torso twist (index 16)

```
MotorCmd[17] motor_cmd  # 17 motor commands
```

### Lower Limb Messages

#### LowerState.msg

Lower limb state information

```
uint16 fsm_id                   # Algorithm FSM ID
IMUState imu_state              # IMU state
MotorState[12] motor_state      # 12 motor states
BmsState bms_state              # Battery state
uint8[40] wireless_remote       # Wireless remote key values
uint32 reserve                  # Reserved for Dobot
```

#### LowerCmd.msg

Lower limb control command

- Left leg: 6 joints (indices 0-5)
- Right leg: 6 joints (indices 6-11)

```
MotorCmd[12] motor_cmd  # 12 motor commands
```

### Wheeled Base (AMR) Messages

#### AMRState.msg

AMR state information

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
Velocity velocity               # Real-time velocity
AMREventStatus amr_event        # AMR events
uint32[32] error_code           # Error codes
uint32 task_id                  # Task ID
uint32 work_mode                # Robot work mode
uint32 relocate_state           # Relocation state
uint32 map_switch_state         # Map switching state
```

#### AMRCommand.msg

AMR control command

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
uint8 CANCEL_CHARGE=17
uint8 SWITCH_MAP=18
uint8 RELOCATE=19

uint8 command_type              # Command type
uint32 target_id                # Target ID
float32 linear_vel              # Linear velocity
float32 angular_vel             # Angular velocity
uint32 command_id               # Command ID
uint64 timestamp                # Timestamp
float32 theta                   # Angle
```

#### AMRBasicStatus.msg

AMR basic status

```
float32 battery_level           # Battery level
float32 battery_voltage         # Battery voltage
float32 battery_current         # Battery current
uint16 heartbeat                # Heartbeat value
```

#### AMREventStatus.msg

AMR event status

```
bool emergency_stop_pressed     # Emergency stop button
bool enable_pressed             # Enable button
bool path_blocked               # Path blocked
bool low_battery                # Low battery
bool obstacle_detected          # Obstacle detected
```

#### Velocity.msg

Velocity information

```
float32 linear_vel      # Linear velocity
float32 angular_vel     # Angular velocity
```

### System Status Messages

#### AxisStateInfo.msg

Detailed joint state information

```
uint8 servo_state           # Servo state (enabled, disabled, error)
uint16 error_code           # Error code
int32 pos_err_code          # Position limit error code
int32 vel_err_code          # Velocity limit error code
int32 torque_err_code       # Torque limit error code
uint8 node_state            # Joint online state
uint8 display_op_mode       # Joint status word
bool is_virtual             # Virtual/real axis flag
uint8 mcu_temp              # MCU temperature
uint8 mos_temp              # MOS temperature
uint8 motor_temp            # Motor temperature
uint8 bus_voltage           # Bus voltage
uint16 software_version     # Software version
```

#### EcatSlaveInfo.msg

EtherCAT slave information

```
bool is_virtual             # Virtual/real flag
uint8 slave_state           # Slave state
uint16 error_code           # Error code
uint16 software_version     # Software version
```

#### MainNodesState.msg

Main node states

```
AxisStateInfo[12] leg       # 12 leg joint states
AxisStateInfo[7] left_arm   # Left arm state
AxisStateInfo[7] right_arm  # Right arm state
AxisStateInfo waist         # Waist state
AxisStateInfo[2] head       # Head state
EcatSlaveInfo[2] ecat2can   # 2 EtherCAT-to-CAN module states
```

#### ClearErrors.msg

Clear errors command

```
int32 msgid  # Message ID (arbitrary value)
```

#### EmergencyState.msg

Emergency stop state information

```
bool soft_emergency_triggered   # Soft emergency stop (triggered by app)
bool hard_emergency_triggered   # Hard emergency stop (triggered by user board)
bool amr_emergency_triggered    # AMR emergency stop (triggered by base)
bool di_emergency_triggered     # DI emergency stop triggered
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
JoystickValue btn_move    # Left joystick values
JoystickValue btn_turn    # Right joystick values
```

## Topic Mapping

| Topic Name                  | Message Type       | Description                      |
| --------------------------- | ------------------ | -------------------------------- |
| `rt/set/fsm/id`           | SetFsmId           | Switch low-level state           |
| `rt/switch/upper/control` | SwitchUpperControl | Switch upper limb control        |
| `rt/upper/state`          | UpperState         | Upper limb state                 |
| `rt/upper/cmd`            | UpperCmd           | Upper limb control command       |
| `rt/lower/state`          | LowerState         | Lower limb state                 |
| `rt/lower/cmd`            | LowerCmd           | Lower limb control command       |
| `rt/amr/state`            | AMRState           | Wheeled base state               |
| `rt/amr/cmd`              | AMRCommand         | Wheeled base control             |
| `rt/posctl/cmd`           | PosControlCmd      | Position control command         |
| `rt/posctl/state`         | PosControlState    | Position control state feedback  |
| `rt/main/nodes/state`     | MainNodesState     | Joint and CAN board states       |
| `rt/clear/errors`         | ClearErrors        | Clear errors                     |
| `rt/hands/state`          | HandsState         | Dexterous hand state             |
| `rt/hands/cmd`            | HandsCmd           | Dexterous hand control command   |
| `rt/remote/control`       | RemoteControl      | Joystick control                 |
| `rt/emergency/state`      | EmergencyState     | Emergency stop state             |

## Usage Guide

### 1. Safety Notes

- Before controlling joints, the low-level state must be switched to non-zero torque mode using the `rt/set/fsm/id` interface.
- To externally control the upper limb, upper limb control authority must be enabled via the `rt/switch/upper/control` interface.

### 2. Wireless Remote Key Mapping

Key mapping in the `wireless_remote[40]` array:

- `wireless_remote[2]`: MSB to LSB corresponding to '', '', 'LT', 'RT', 'SELECT', 'START', 'LB', 'RB'
- `wireless_remote[3]`: MSB to LSB corresponding to 'LEFT', 'DOWN', 'RIGHT', 'UP', 'Y', 'X', 'B', 'A'
- `wireless_remote[4-7]`: LX value (floating point in [-1, 0] range)
- `wireless_remote[8-11]`: RX value (floating point in [0, 1] range)
- `wireless_remote[12-15]`: RY value (floating point in [-1, 0] range)
- `wireless_remote[16-19]`: LY value (floating point in [0, 1] range)

### 3. Position Control Interface Usage

- First send `rt/posctl/cmd` with `command_type=SWITCH_MODE` and `target_mode=1`.
- Poll `rt/posctl/state` until `control_mode == CONTROL_MODE_POSITION`.
- `MOVE_J` uses `left_q/right_q/hips_q`; `MOVE_L` and `RUN_TO` use `left_pose/right_pose/hips_height`.
- `RUN_TO` and `CART_JOG` use `stop=0` to start and `stop=1` to stop; send once each.
- A `left_*`, `right_*`, or `hips_*` length of 0 means that body part is not involved.
- Length constraints are enforced by both message upper bounds and business logic (e.g., `left_q` max 7, `left_pose` max 6).

### 4. Dexterous Hand Joint Order

**Joint order (starting from index 0):**

- Right hand (indices 0-5): little finger, ring finger, middle finger, index finger, thumb flexion, thumb rotation
- Left hand (indices 6-11): little finger, ring finger, middle finger, index finger, thumb flexion, thumb rotation

**Angle ranges:**

- Little, ring, middle, index fingers: 19deg~176.7deg (0.3316 rad~3.0815 rad)
- Thumb flexion: -13deg~53.6deg (-0.2269 rad~0.9346 rad)
- Thumb rotation: 90deg~165deg (1.5708 rad~2.8798 rad)

### 5. Wheeled Base (AMR) Usage

#### Remote Control
1. Send a `START_REMOTE` request.
2. Send `REMOTE_CONTROL` requests with `linear_vel` and `angular_vel` set.
3. Send a `STOP_REMOTE` request when finished.

#### Navigation
1. Send a `MOVE_TO_TAG` request with `target_id` and `theta` set.
2. Send a `MOVE_TO_CHARGE` request to navigate the base to the charging station.
3. Send a `CANCEL_CHARGE` request to cancel the charging task.
4. Send a `SWITCH_MAP` request to switch the navigation map.
5. Send a `RELOCATE` request to trigger relocation.
6. Monitor `navigation_status` in `AMRState` for task state:
   - `RUNNING (2)`: In progress
   - `COMPLETED (3)`: Completed
7. Use `CANCEL_TASK` / `PAUSE_TASK` / `RESUME_TASK` to manage the task lifecycle.

### 6. Real-time Topic Prefix

All topics requiring real-time publishing use the `rt/` prefix.

## Build Instructions

```bash
# Build in the ROS2 workspace
colcon build --packages-select dobot_atom

source install/setup.bash
```

## Dependencies

- ROS2 (Humble/Iron/Rolling)
- geometry_msgs
- rosidl_default_generators
- rosidl_generator_dds_idl
