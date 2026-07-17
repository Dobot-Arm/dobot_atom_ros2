# 2. DDS interface arrangement

# 1. Preface

This article is an interface description document for humanoid interaction using DDS. Since the DDS interface is defined and serialized through IDL files, the interface documents in this article are all defined by IDL files.

All topics that need to be published in real time start with \*\*rt/\*\*.

Due to the diversification of product forms, Cross-Border Dolls adopt a separate control method for the upper and lower limbs, that is, the upper and lower limbs use different topic controls. Upper limbs: waist, left arm, right arm, head, lower limbs: left leg, right leg. Upper limbs and lower limbs may not exist at the same time, so battery information and handle information exist in both upper and lower limb status topics. That is, although they are defined in both topics, they are actually the same thing.

The bottom layer mentioned below refers to the program that manages joints, BMS, imu, and handles. If the user is familiar with reinforcement learning, they can use the following interface and underlying exchange to control the cross-border robot.

The algorithm state machine is a program built into PC1. It will also control the upper and lower limbs under certain circumstances. The algorithm state machine is introduced in separate documents and will not be repeated here.

Dexterous hand array sorting: right hand (thumb flexion, index finger, middle finger, ring finger, little finger, thumb rotation), left hand (thumb flexion, index finger, middle finger, ring finger, little finger, thumb rotation).

## 1.1 Description of topic frequency

The Host real-time main loop is aligned with the CODESYS EtherCAT task, with a cycle of approximately **1200 μs (833 Hz)**. The following table shows the typical frequency of each topic in the project; **Upper and lower limb control topic** is recommended to be **833 ms** as the release cycle; Host takes the **latest frame** command in the real-time loop and sends it to the servo.

| Topic | Direction | Typical frequencies | Description |
| ------------------------------------------------ | ---- | ------------------ | ---------------------------------------------------------------- |
| `rt/upper/state` | state | ≈833 Hz | period 1200 μs |
| `rt/lower/state` | state | ≈833 Hz | period 1200 μs |
| `rt/upper/cmd` | Control | ≈833 Hz | Period 1200 μs |
| `rt/lower/cmd` | Control | ≈833 Hz | Period 1200 μs |
| `rt/posctl/state` | state | ≈833 Hz | period 1200 μs |
| `rt/posctl/cmd` | Control | Determined by publisher | Event driven; `CART_JOG`/`RUN_TO` recommended about 10 Hz keep alive |
| `rt/main/nodes/state` | state | 5 Hz | period 200 ms |
| `rt/hands/state` | state | ≈125 Hz | period 8000 μs |
| `rt/hands/cmd` | Controlled | Determined by publisher | Event driven |
| `rt/power/state` | state | 5 Hz | period 200 ms |
| `rt/bms/info` | Status | 10 Hz | Period 100 ms |
| `rt/amr/state` | state | 40 Hz | period 25 ms |
| `rt/amr/laserscan`, `rt/amr/odom` | Status | About 100 Hz after subscription | Requires `SUBSCRIBE_LASER` |

## 1.2 Global joint index and order

The system has a total of **29** joint axes (`MAX_AXIS=29`). The Host internal index and the DDS upper and lower limb arrays are mapped as follows (see `general_tools.h`, `dds_interface.cpp`):

| Global index | Part | Joint name (starting from 0 is the serial number within the part) |
| -------- | ---- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 0–5 | Left leg | 0 hip\_pitch, `<br>`1 hip\_roll, `<br>`2 hip\_yaw, `<br>`3 knee, `<br>`4 ankle\_pitch, `<br>`5 ankle\_roll |
| 6–11 | Right leg | 0 hip\_pitch, `<br>`1 hip\_roll, `<br>`2 hip\_yaw, `<br>`3 knee, `<br>`4 ankle\_pitch, `<br>`5 ankle\_roll |
| 12 | waist | torso |
| 13–19 | Left arm | 0 shoulder\_pitch, `<br>`1 shoulder\_roll, `<br>`2 shoulder\_yaw, `<br>`3 elbow\_pitch, `<br>`4 elbow\_roll, `<br>`5 wrist\_pitch, `<br>`6 wrist\_roll |
| 20–26 | Right arm | Same sequence as left arm |
| 27–28 | head | 0 head\_yaw,`<br>`1 head\_pitch |

**DDS array mapping:**

* `motor_state[12]` / `motor_cmd[12]` of `rt/lower/state` / `rt/lower/cmd`: `[0–5]` for left leg, `[6–11]` for right leg (corresponding to global 0–11).
* `motor_state[17]` / `motor_cmd[17]` of `rt/upper/state` / `rt/upper/cmd`: `[0]` waist, `[1–7]` left arm, `[8–14]` right arm, `[15–16]` head (corresponding to global 12–28).

**Dexterous hands** `**hands[12]**` **Index:** Right hand `[0–5]`, left hand `[6–11]`; the order of each side is thumb\_bend, index, middle, ring, pinky, thumb\_rotation (radians, see §9/§10).

## 1.3 Joint limit (position / speed / torque)

The following limits apply to `q` (rad), `dq` (rad/s), `tau` (N·m) in `**MotorCmd_**` **/** `**MotorState_**`. Global indexes are consistent with §1.2.

**The difference between ATOM\_W and ATOM\_MAX:** Only the **left leg global index 0–2** (hip\_pitch / hip\_roll / hip\_yaw) position limit is different; the other joints of the two models are the same. That is, the lifting joint of ATOM\_W corresponds to the 0-2 joint of the left leg.

### ATOM\_W position limit

| global index | parts | joints | min (rad) | max (rad) |
| -------- | ---- | --------------- | ---------- | ---------- |
| 0 | Left leg | hip\_pitch | \-0.9074 | 1.099 |
| 1 | Left leg | hip\_roll | \-0.71 | 2.356 |
| 2 | Left leg | hip\_yaw | \-2.33 | 0.9074 |
| 3 | Left leg | knee | \-0.174 | 2.53 |
| 4 | Left leg | ankle\_pitch | \-0.85 | 0.8 |
| 5 | Left leg | ankle\_roll | \-0.4 | 0.4 |
| 6 | Right leg | hip\_pitch | \-3.14 | 3.14 |
| 7 | Right leg | hip\_roll | \-3.05 | 0.56 |
| 8 | Right leg | hip\_yaw | \-2.75 | 2.75 |
| 9 | Right leg | knee | \-0.174 | 2.53 |
| 10 | Right leg | ankle\_pitch | \-0.85 | 0.8 |
| 11 | Right leg | ankle\_roll | \-0.4 | 0.4 |
| 12 | waist | torso | \-3.14 | 3.14 |
| 13 | Left arm | shoulder\_pitch | \-2.97 | 2.97 |
| 14 | Left arm | shoulder\_roll | \-0.43 | 3.14 |
| 15 | Left arm | shoulder\_yaw | \-2.97 | 2.97 |
| 16 | Left arm | elbow\_pitch | \-0.87 | 3.14 |
| 17 | Left arm | elbow\_roll | \-2.97 | 2.97 |
| 18 | Left arm | wrist\_pitch | \-1.57 | 1.57 |
| 19 | Left arm | wrist\_roll | \-1.57 | 1.57 |
| 20 | Right arm | shoulder\_pitch | \-2.97 | 2.97 |
| 21 | Right arm | shoulder\_roll | \-3.14 | 0.43 |
| 22 | Right arm | shoulder\_yaw | \-2.97 | 2.97 |
| 23 | Right arm | elbow\_pitch | \-0.87 | 3.14 |
| 24 | Right arm | elbow\_roll | \-2.97 | 2.97 |
| 25 | Right arm | wrist\_pitch | \-1.57 | 1.57 |
| 26 | Right arm | wrist\_roll | \-1.57 | 1.57 |
| 27 | head | head\_yaw | \-3.14 | 3.14 |
| 28 | head | head\_pitch | \-1.57 | 0.698 |

### ATOM\_MAX position limit

| global index | parts | joints | min (rad) | max (rad) |
| -------- | ---- | --------------- | ---------- | ---------- |
| 0 | Left leg | hip\_pitch | \-3.14 | 3.14 |
| 1 | Left leg | hip\_roll | \-0.56 | 3.05 |
| 2 | Left leg | hip\_yaw | \-2.75 | 2.75 |
| 3 | Left leg | knee | \-0.174 | 2.53 |
| 4 | Left leg | ankle\_pitch | \-0.85 | 0.8 |
| 5 | Left leg | ankle\_roll | \-0.4 | 0.4 |
| 6 | Right leg | hip\_pitch | \-3.14 | 3.14 |
| 7 | Right leg | hip\_roll | \-3.05 | 0.56 |
| 8 | Right leg | hip\_yaw | \-2.75 | 2.75 |
| 9 | Right leg | knee | \-0.174 | 2.53 |
| 10 | Right leg | ankle\_pitch | \-0.85 | 0.8 |
| 11 | Right leg | ankle\_roll | \-0.4 | 0.4 |
| 12 | waist | torso | \-3.14 | 3.14 |
| 13 | Left arm | shoulder\_pitch | \-2.97 | 2.97 |
| 14 | Left arm | shoulder\_roll | \-0.43 | 3.14 |
| 15 | Left arm | shoulder\_yaw | \-2.97 | 2.97 |
| 16 | Left arm | elbow\_pitch | \-0.87 | 3.14 |
| 17 | Left arm | elbow\_roll | \-2.97 | 2.97 |
| 18 | Left arm | wrist\_pitch | \-1.57 | 1.57 |
| 19 | Left arm | wrist\_roll | \-1.57 | 1.57 |
| 20 | Right arm | shoulder\_pitch | \-2.97 | 2.97 |
| 21 | Right arm | shoulder\_roll | \-3.14 | 0.43 |
| 22 | Right arm | shoulder\_yaw | \-2.97 | 2.97 |
| 23 | Right arm | elbow\_pitch | \-0.87 | 3.14 |
| 24 | Right arm | elbow\_roll | \-2.97 | 2.97 |
| 25 | Right arm | wrist\_pitch | \-1.57 | 1.57 |
| 26 | Right arm | wrist\_roll | \-1.57 | 1.57 |
| 27 | head | head\_yaw | \-3.14 | 3.14 |
| 28 | head | head\_pitch | \-1.57 | 0.698 |

### Speed/torque rating and maximum

Data source: lower limb joint module specification table; head/arm joint module specification table. Rated speed = motor rated speed ÷ reduction ratio × 2π/60 (head reduction ratio 51, arm 100); maximum speed = motor maximum speed ÷ reduction ratio × 2π/60 (lower limbs directly take the maximum joint speed rad/s from the specification table). The rated/maximum torque corresponds to the joint rated torque and joint peak torque (N·m) in the specification table.

| Global index | Part | Joint | Rated speed (rad/s) | Maximum speed (rad/s) | Rated torque (N·m) | Maximum torque (N·m) |
| -------- | ---- | --------------- | ------------------ | ------------------ | ---------------- | ---------------- |
| 0 | Left leg | hip\_pitch | 11.90 | 13.80 | 56.10 | 207.76 |
| 1 | Left leg | hip\_roll | 8.47 | 11.90 | 69.19 | 241.42 |
| 2 | Left leg | hip\_yaw | 9.05 | 11.36 | 22.44 | 104.16 |
| 3 | Left leg | knee | 11.87 | 16.67 | 83.30 | 213.80 |
| 4 | Left leg | ankle\_pitch | 11.13 | 15.71 | 17.68 | 89.90 |
| 5 | Left leg | ankle\_roll | 11.13 | 15.71 | 17.68 | 89.90 |
| 6 | Right leg | hip\_pitch | 11.90 | 13.80 | 56.10 | 207.76 |
| 7 | Right leg | hip\_roll | 8.47 | 11.90 | 69.19 | 241.42 |
| 8 | Right leg | hip\_yaw | 9.05 | 11.36 | 22.44 | 104.16 |
| 9 | Right leg | knee | 11.87 | 16.67 | 83.30 | 213.80 |
| 10 | Right leg | ankle\_pitch | 11.13 | 15.71 | 17.68 | 89.90 |
| 11 | Right leg | ankle\_roll | 11.13 | 15.71 | 17.68 | 89.90 |
| 12 | waist | torso | 5.17 | 6.49 | 39.27 | 182.28 |
| 13 | Left arm | shoulder\_pitch | 3.14 | 4.61 | 33 | 93 |
| 14 | Left arm | shoulder\_roll | 3.14 | 4.50 | 22.5 | 70 |
| 15 | Left arm | shoulder\_yaw | 3.14 | 4.50 | 22.5 | 70 |
| 16 | Left arm | elbow\_pitch | 3.14 | 4.50 | 22.5 | 70 |
| 17 | Left arm | elbow\_roll | 3.14 | 5.13 | 7.5 | 34 |
| 18 | Left arm | wrist\_pitch | 3.14 | 5.13 | 7.5 | 34 |
| 19 | Left arm | wrist\_roll | 3.14 | 5.13 | 7.5 | 34 |
| 20 | Right arm | shoulder\_pitch | 3.14 | 4.61 | 33 | 93 |
| 21 | Right arm | shoulder\_roll | 3.14 | 4.50 | 22.5 | 70 |
| 22 | Right arm | shoulder\_yaw | 3.14 | 4.50 | 22.5 | 70 |
| 23 | Right arm | elbow\_pitch | 3.14 | 4.50 | 22.5 | 70 |
| 24 | Right arm | elbow\_roll | 3.14 | 5.13 | 7.5 | 34 |
| 25 | Right arm | wrist\_pitch | 3.14 | 5.13 | 7.5 | 34 |
| 26 | Right arm | wrist\_roll | 3.14 | 5.13 | 7.5 | 34 |
| 27 | head | head\_yaw | 6.16 | 11.71 | 4 | 12 |
| 28 | head | head\_pitch | 6.16 | 11.71 | 4 | 12 |

# 2. Interface description

Shared sub-IDL files

1. bms information: bms\_state.idl

```shell
   cat bms_state.idl
   struct BmsState_
   module dobot_atom{
       module msg{
           module dds_{
               @extensibility(FINAL)
               struct BmsState_
               {
                   uint16 bms_state; /*BMS state*/
                   uint16 afe_state; /*AFE chip status*/
                   uint32 bms_alarms; /*BMS fault code*/
                   uint16 battery_level; /*battery power percentage*/
                   uint16 battery_health; /*battery health*/
                   uint16 pcb_board_temp; /*PCB board temperature*/
                   uint16 afe_chip_temp; /*AFE chip temperature*/
                   uint16 battery_now_current; /*Current current of battery pack*/
                   uint16 cells_voltage[16]; /*16 cell voltages*/
                   uint16 battery_pack_current_voltage; /*battery pack voltage*/
                   uint16 battery_pack_io_voltage; /*voltage of battery pack discharge/charging interface*/
                   uint32 bms_work_time; /*BMS running time*/
                   uint16 bms_hardware_version; /*BMS hardware version number*/
                   uint16 bms_software_version; /*BMS software version number*/
                   uint16 heartbeat; /*heartbeat*/
               };
           };
       };
   };
   ```

| Fields | Primitive Types | Physical Units |
   | -------------------------------- | -------- | -------- |
   | `battery_level` | uint16 | % |
   | `battery_health` | uint16 | % |
   | `pcb_board_temp` | uint16 | ℃ |
   | `afe_chip_temp` | uint16 | ℃ |
   | `battery_now_current` | uint16 | mA |
   | `cells_voltage[i]` | uint16 | V |
   | `battery_pack_current_voltage` | uint16 | V |
   | `battery_pack_io_voltage` | uint16 | V |
   | `bms_work_time` | uint32 | s |
2. Joint information: motor\_state.idl

```shell
   cat motor_state.idl
   module dobot_atom{
       module msg{
           module dds_{
               @extensibility(FINAL)
               struct MotorState_ {
                   uint8 mode; // mode
                   float q; // Angular position
                   float dq; // angular velocity
                   float ddq; // Angular acceleration
                   float tau_est; // Estimated moment
                   float q_raw; // Original angular position
                   float dq_raw; // Original angular velocity
                   float ddq_raw; // Raw angular acceleration
                   uint8 mcu_temp; // Servo control board temperature
                   uint8 mos_temp; // mos tube temperature
                   uint8 motor_temp; // motor temperature
                   uint8 bus_voltage; // bus voltage data
               };
           };
       };
   };

   ```

| Field | Unit | Description |
   | -------------------------------------------------- | ----- | ----------------------------------------------------------------------------------------------- |
   | `mode` | — | Servo CiA402 display mode: `11`\=MIT, `8`\=position control, `6`\=zero return, etc. (`byDisplayMode`) |
   | `q`, `q_raw` | rad | joint angle position |
   | `dq`, `dq_raw` | rad/s | Joint angular velocity |
   | `ddq`, `ddq_raw` | — | The current implementation does not fill in the effective angular acceleration; the ankle `ddq_raw` is reused for internal debugging |
   | `tau_est` | N·m | Estimated joint moment |
   | `mcu_temp`, `mos_temp`, `motor_temp` | ℃ | Servo board / MOS / Motor temperature |
   | `bus_voltage` | V | Bus voltage |
3. Joint command: motor\_cmd.idl

```shell
   cat motor_cmd.idl
   module dobot_atom{
       module msg{
           module dds_{
               @extensibility(FINAL)
               struct MotorCmd_ {
                   uint8 mode; // mode
                   float q; // position
                   float dq; // speed
                   float tau; // moment
                   float kp; // proportional gain
                   float kd; // Differential gain
               };
           };
       };
   };

   ```

| Fields | Units | Recommended Range | Description |
   | ------- | ----- | -------------------------- | ---------- |
   | `q` | rad | §1.3 Each global index min~max | Target position |
   | `dq` | rad/s | Legs/Waist ±20, Arms/Head ±99 | Target Speed |
   | `tau` | N·m | ±1000 | Feedforward torque |
   | `kp` | — | \>0 | Position loop gain |
   | `kd` | — | ≥0 | Speed loop gain |

Do not pass in NaN/Inf; position/speed/torque exceeding the limit can be queried through the corresponding `*_err_code` field in `rt/main/nodes/state`.
4. imu information: imu\_state.idl

```shell
   cat imu_state.idl
   module dobot_atom{
       module msg{
           module dds_{
               @extensibility(FINAL)
               struct IMUState_ {
                   float quaternion[4]; // four-dimensional quaternion
                   float gyroscope[3]; // Three-dimensional gyroscope data, unit deg/s
                   float accelerometer[3]; // Three-dimensional accelerometer data, unit m/s2
                   float rpy[3]; // Three-dimensional Euler angle (roll, pitch, yaw), unit deg
                   uint8 temperature; // Temperature sensor data, unit °C
               };
           };
       };
   };
   ```
5. Wireless\_remote description mentioned below:

The corresponding key has a value when pressed, and returns to 0 when released.

```shell
   wireless_remote[2], corresponding keys from high to low: '', '', 'LT', 'RT', 'SELECT', 'START', 'LB', 'RB';
   wireless_remote[3], corresponding keys from high to low: 'LEFT', 'DOWN', 'RIGHT', 'UP', 'Y', 'X', 'B', 'A';
   wireless_remote[4->7] stores the floating point value value between the LX key converted into [-1,0], wireless_remote[4] is the low bit of value;
   wireless_remote[8->11] stores the floating point value value between the RX key values [0,1], wireless_remote[8] is the low bit of value;
   wireless_remote[12->15] stores the floating point value value between the RY key values [-1,0], wireless_remote[12] is the low bit of value;
   wireless_remote[16->19] stores the floating point value value between the LY key values [0,1], wireless_remote[16] is the low bit of value.
   ```
6. Chassis status: amr\_state.idl

1. AMRState\_ is the main state of the entire chassis

```shell
module dobot_atom{
    module msg{
        module dds_{
            enum NavigationStatus_ {
                UNKNOWN, // unknown
                QUEUING, // queuing
                RUNNING, // running
                COMPLETED, // completed
                FAILED, // failed
                PAUSED, // pause
                CANCELED, // Cancel
                WAITING_CONFIRM, // Waiting for confirmation
                IDLE, // idle
                STOPPED // Stop state
            };//The enumeration is only used as a numerical reference for navigation_status

Enum DeviceStatus_ {
                DEVUNKNOWN, // unknown
                DEVIDLE, // The car is idle
                TASKING, // Tasking
                ERROR, // Faulting
                OFFLINE, // offline
                INIT, // device initialization
                CHARGING, // Charging
                UPGRADE // Upgrading
            }; // The enumeration is only used as a numerical reference for device_status
            @extensibility(FINAL)
            struct AMREventStatus_{
                boolean emergency_stop_pressed; // The emergency stop button is pressed
                boolean enable_pressed; // The enable key is pressed
                boolean path_blocked; // The path is blocked
                boolean low_battery; // Battery power is too low
                boolean obstacle_detected; // blocked by obstacles
            
            };
        
            @extensibility(FINAL)
            struct AMRBasicStatus_{
                float battery_level; //Battery power percentage
                float battery_voltage; // battery voltage
                float battery_current; // battery current
                uint16 heartbeat; // heartbeat value
            };

@extensibility(FINAL)
            struct AMRState_ {
                DeviceStatus_ device_status; // device status
                NavigationStatus_ navigation_status; // Navigation status. Refer to the above enum to determine whether the task is completed.
                AMRBasicStatus_ basic_status; // Basic status
                float position[3]; // current position {x,y,yaw}
                AMREventStatus_ amr_event; // Chassis related events
                uint32 error_code[32]; // Error code
                uint32 task_id; // task ID
                uint32 work_mode; // Robot working mode The enumeration is as shown on the right: 0: Mission mode || 3: Remote control mode || 4: Maintenance mode
            };



        };
    };
};
```

| 字段                              | 单位 |
| --------------------------------- | ---- |
| `basic_status.battery_level`    | %    |
| `basic_status.battery_voltage`  | V    |
| `basic_status.battery_current`  | A    |
| `basic_status.heartbeat`        | —   |
| `position[0]`, `position[1]` | m    |
| `position[2]`                   | rad  |

## 1 Switch the underlying state

1. Function description: For safety protection reasons, if you need to control joints, you must use this interface to set the bottom layer state to non-zero torque mode, otherwise the bottom layer will not send control instructions to the joints.
2. Topic name: rt/set/fsm/id
3. **Note, this interface is not open to the public! **
4. IDL file content:

```shell
module dobot_atom{
    module msg{
        module dds_{
            //Algorithm/strategy side execution status
            enum WorkingState {
                HIDLE, // idle
                HTASKING // During the task (if movement control, upper limb control, debugging mode, etc. are turned on, the status is HTASKING)
            }; //The reason for adding the H prefix here (H stands for Human) is that IDLE and TASKIG are defined in amr_state.idl. If it is named IDLE again, it will cause a c++ compilation error.

@extensibility(FINAL)
            struct SetFsmId_ {
                uint16 id; // For algorithm state machine
                string current_action; //The action currently being executed by the algorithm, empty means there is no action
                WorkingState state; // HIDLE | HTASKING
            };
        };
    };
};





```

## 2 Switch upper limb control

1. Function description: To avoid external (user or remote operation) and algorithm state machines from controlling the upper limbs at the same time, this interface needs to be added. If the external device, including PC2, wants to control the upper limbs, this switch must be turned on.
2. Topic name: rt/switch/upper/control
3. IDL file content:

```shell
cat switch_upper_control.idl
module dobot_atom{
    module msg{
        module dds_{
            @extensibility(FINAL)
            struct SwitchUpperControl_ {
                boolean flag; /* true: upper limbs are controlled, false: upper limbs are not controlled*/
            };
        };
    };
};
```

## 3 Upper limb status

1. Function description: underlying information of the upper limbs.
2. Topic name: rt/upper/state
3. **Feedback frequency:** ≈833 Hz (period 1200 μs, see §1.1).
4. **Contains joints:** `motor_state[17]` corresponds to global index 12–28: `[0]` waist, `[1–7]` left arm (shoulder → wrist 7 axis), `[8–14]` right arm, `[15–16]` head. Also includes `bms_state`, `wireless_remote[40]`, `fsm_id`, `is_upper_control`, `robot_type`.
5. IDL file content:

```shell

#include "imu_state.idl"
#include "motor_state.idl"
#include "bms_state.idl"

Module dobot_atom{
    module msg{
        module dds_{
            @extensibility(FINAL)
            struct UpperState_ {
                char robot_type[16]; /*Type corresponding to product definition*/
                boolean is_upper_control; /* true: upper limbs are controlled, false: upper limbs are not controlled*/
                uint16 fsm_id; /*Corresponding algorithm state machine*/
                MotorState_ motor_state[17];
                BmsState_ bms_state;
                octet wireless_remote[40];
                uint32 reserve; /*cross-border reserve*/
            };
        };
    };
};

```

## 4 Upper body control

1. Function description: Lower limb status information.
2. Topic name: rt/upper/cmd
3. **Feedback frequency:** ≈833 Hz (period 1200 μs, see §1.1).
4. **Contains joints:** `motor_state[12]`, the topic subscript `k` is the global index `k` (0–11): 0–5 left leg, 6–11 right leg. Also includes `imu_state`, `bms_state`, `wireless_remote[40]`, `fsm_id`.
5. IDL file content:

```shell
cat upper_cmd.idl
#include "motor_cmd.idl"

Module dobot_atom{
    module msg{
        module dds_{
            @extensibility(FINAL)
            struct UpperCmd_ {
                MotorCmd_ motor_cmd[17]; // Motor command, including 17 MotorCmd_ structures
            };
        };
    };
};

```

## 5 Lower limb status

1. Function description: Lower limb control instructions.
2. Topic name: rt/lower/state
3. **Control frequency:** Recommended release cycle **833 ms (2 Hz)** (see §1.1).
4. **Joint order:** `motor_cmd[12]` is consistent with §5, the topic subscript `k` is the global index `k`: 0-5 left leg (global 0 hip\_pitch starts), 6-11 right leg. See `MotorCmd_` (§2.3) for the field unit; see §1.3 for the position limit.
5. IDL file content:

```c++
cat lower_state.idl
#include "imu_state.idl"
#include "motor_state.idl"
#include "bms_state.idl"

Module dobot_atom{
    module msg{
        module dds_{
            @extensibility(FINAL)
            struct LowerState_ {
                uint16 fsm_id; /*Corresponding algorithm state machine*/
                IMUState_ imu_state;
                MotorState_ motor_state[12];
                BmsState_ bms_state;
                octet wireless_remote[40];
                uint32 reserve; /*cross-border reserve*/
            };
        };
    };
};
```

## 6 Lower limb control

1. Function description: Lower limb control instructions.
2. Topic name: rt/lower/cmd
3. IDL file content:

```shell
cat lower_cmd.idl
#include "motor_cmd.idl"

Struct LowerCmd_ {
    MotorCmd_ motor_cmd[12]; // Motor command, including 12 MotorCmd_ structures
};

```

## 7 Joint and can plate status

1. Function description: Mainly provided for users to assist in debugging.
2. Topic name: rt/main/nodes/state
3. **Feedback frequency:** 5 Hz (200 ms). Covers 29 axes of debugging information for the whole body (6 for each left/right leg, 7 for each waist, left/right arm, and 2 for the head) and 2 channels of ECAT2CAN slave stations.
4. IDL file content:

```c++
   module dobot_atom{
       module msg{
           module dds_{
               @extensibility(FINAL)
               struct AxisStateInfo_
               {
                   uint8 servo_state;
                   uint16 error_code; //Motor alarm error code
                   uint16 warn_code; // Motor warning error code
                   int32 pos_err_code;
                   int32 vel_err_code;
                   int32 torque_err_code;
                   uint8 node_state;
                   uint8 display_op_mode;
                   boolean is_virtual; /* default: false */
                   uint8 mcu_temp;
                   uint8 mos_temp;
                   uint8 motor_temp;
                   uint8 bus_voltage;
                   uint16 software_version;
               };

@extensibility(FINAL)
               struct EcatSlaveInfo_
               {
                   boolean is_virtual;
                   uint8   slave_state;
                   uint16  error_code;
                   uint16  software_version;
               };

@extensibility(FINAL)
               struct MainNodesState_ {
                   AxisStateInfo_ left_leg[6];
                   AxisStateInfo_ right_leg[6];
                   AxisStateInfo_ waist;
                   AxisStateInfo_ left_arm[7];
                   AxisStateInfo_ right_arm[7];
                   AxisStateInfo_ head[2];
                   EcatSlaveInfo_ ecat2can[2];
                   /* AxisStateInfo left_hand[6]; */
                   /* AxisStateInfo right_hand[6]; */
                   /* uint16 power_state[8]; */
               };
           };
       };
   };


   ```

* node\_state enumeration:

* 0: Initialisation
     * 1: Disconnecting
     * 2: Connecting or Preparing
     * 4: Stopped
     * 5: Operational
     * 7: Pre\_operational
     * 其他: UNKNOWN
   * servo\_state枚举:

* 0: disable
     * 1: error
     * 3: enable
     * Others: UNKNOWN
   * pos\_err\_code: Whether the position exceeds the limit, if it exceeds the limit, there is data
   * vel\_err\_code: Speed exceeds limit
   * torque\_err\_code: Whether the torque exceeds the limit

## 8 Clear errors

1. Function description: Only clear the underlying errors.
2. Topic name: rt/clear/errors
3. IDL file content:

```shell
cat clear_errors.idl
struct ClearErrors_ {
    int32 msg_id;
};

```

## 9 Dexterous hand status

1. Function description: Only the angle information of each finger of the dexterous hand.
2. Topic name: rt/hands/state
3. **Feedback frequency:** ≈125 Hz (8000 μs period, see §1.1).
4. **Angle unit:** `HandsState_.hands[i].q` / `.dq` is **rad** / **rad/s**. Dahuan gripper uses `DHGripperState_` (see §18), `realtime_position` is **0~1** opening ratio.
5. **Array order:** `[0–5]` right hand, `[6–11]` left hand; each side: thumb\_bend, index, middle, ring, pinky, thumb\_rotation.
6. IDL file content:

```shell
#include "motor_state.idl"

Module dobot_atom{
    module msg{
        module dds_{
            @extensibility(FINAL)
            struct HandsState_
            {
                MotorState_ hands[12];
            };
        };
    };
};



```

## 10 Dexterous hand control

1. Function description: Only the angle of each finger of the dexterous hand is controlled.
2. Topic name: rt/hands/cmd
3. **Control frequency:** Determined by the publisher (event-driven).
4. **Value range and unit:** `HandsCmd_.hands[i]` reuse `MotorCmd_`:

* `q`: **rad**, target joint angle (the effective range of each finger varies depending on the model, approximately **0.04–3.1 rad**);

* `dq`: **rad/s**, finger angular velocity;
     * Dahuan Clamp: Use `**DHGripperCmd_**` (§17), fill in `position` with **0~1**.
5. **Array order:** Same as §9.
6. IDL file content:

```shell
cat hands_cmd.idl
#include "motor_cmd.idl"

Module dobot_atom{
    module msg{
        module dds_{
            @extensibility(FINAL)
            struct HandsCmd_
            { 
                MotorCmd_ hands[12];
            };
        };
    };
};

```

## 11 Wheeled humanoid chassis status

1. Function description: mainly includes various status information of the chassis
2. Topic name: rt/amr/state
3. **Feedback frequency:** 40 Hz (25 ms).
4. **Unit description:** See §2.6 `AMRBasicStatus_`, `position[]` and `velocity`.
5. IDL file content:

```shell
   module dobot_atom{
       module msg{
           module dds_{
               enum NavigationStatus_ {
                   UNKNOWN, // unknown
                   QUEUING, // queuing
                   RUNNING, // running
                   COMPLETED, // completed
                   FAILED, // failed
                   PAUSED, // pause
                   CANCELED, // Cancel
                   WAITING_CONFIRM, // Waiting for confirmation
                   IDLE, // idle
                   STOPPED // Stop state
               };//The enumeration is only used as a numerical reference for navigation_status

Enum DeviceStatus_ {
                   DEVUNKNOWN, // unknown
                   DEVIDLE, // The car is idle
                   TASKING, // Tasking
                   ERROR, // Faulting
                   OFFLINE, // offline
                   INIT, // device initialization
                   CHARGING, // Charging
                   UPGRADE // Upgrading
               }; // The enumeration is only used as a numerical reference for device_status
               @extensibility(FINAL)
               struct AMREventStatus_{
                   boolean emergency_stop_pressed; // The emergency stop button is pressed
                   boolean enable_pressed; // The enable key is pressed
                   boolean path_blocked; // The path is blocked
                   boolean low_battery; // Battery power is too low
                   boolean obstacle_detected; // blocked by obstacles

               };

@extensibility(FINAL)
               struct AMRBasicStatus_{
                   float battery_level; //Battery power percentage
                   float battery_voltage; // battery voltage
                   float battery_current; // battery current
                   uint16 heartbeat; // heartbeat value
               };

@extensibility(FINAL)
               struct Velocity_ {
                   float linear_vel; // linear velocity (m/s)
                   float angular_vel; // Angular velocity (radians/second)
               };

@extensibility(FINAL)
               struct AMRState_ {
                   DeviceStatus_ device_status; // device status
                   NavigationStatus_ navigation_status; // Navigation status. Refer to the above enum to determine whether the task is completed.
                   AMRBasicStatus_ basic_status; // Basic status
                   float position[3]; // current position {x,y,yaw}
                   Velocity_ velocity; // Fusion velocity (see Velocity_ field description for linear velocity and angular velocity)
                   AMREventStatus_ amr_event; // Chassis related events
                   uint32 error_code[32]; // Error code
                   uint32 task_id; // task ID
                   uint32 work_mode; // Robot working mode The enumeration is as shown on the right: 0: Mission mode || 3: Remote control mode || 4: Maintenance mode
                   uint32 relocate_state; //Chassis relocation status enumeration: 0: No relocation || 1: Relocation in progress || 2: Relocation completed || 3: Relocation failed
                   uint32 map_switch_state; //Map switching state enumeration: 0: Not switching map || 1: Switching map || 2: Switching completed || 3: Switching failed
               };



};
       };
   };
   ```
6. **Relocation/map cutting status field:** `relocate_state`, `map_switch_state` values are shown in the §2.6 table; they are used with the `RELOCATE` and `SWITCH_MAP` commands respectively (see §12.5).

## 12 Wheeled Humanoid Chassis Control

1. Function description: Issue control instructions to the wheeled humanoid robot chassis.
2. Topic name: rt/amr/cmd
3. IDL file content:

```shell
     module dobot_atom{
         module msg{
             module dds_{
                 enum AMRCommandType {
                     CANCEL_TASK, // Cancel task
                     PAUSE_TASK, // Pause the task
                     RESUME_TASK, // Resume task
                     MOVE_TO_TAG, // Move to the mark point, target_id is required, theta is optional, theta is the absolute angle (degrees), if it is 0 or not filled in, the angle is not specified
                     MOVE_TO_CHARGE, // Return to the pile for charging (start the automatic recharging process, see below)
                     REMOTE_CONTROL, // remote control
                     ROTATE, // Rotate in place theta is required, it is the relative angle (degree), relative to the current chassis heading
                     START_REMOTE, // Start remote control
                     STOP_REMOTE, // Turn off remote control
                     SUBSCRIBE_LASER, // Subscribe to chassis lidar data
                     UNSUBSCRIBE_LASER, // Unsubscribe from chassis lidar data
                     START_MAPPING, // Chassis starts scanning
                     SAVE_MAP, // Save the currently scanned map locally
                     STOP_MAPPING, // End scanning
                     SET_VEL, //Set the upper limit of speed (set linear speed and angular speed at the same time, no-load and load)
                     SET_ACCEL, //Set acceleration (set linear and angular acceleration at the same time, no-load and load)
                     SET_DECEL, //Set deceleration (set linear and angular deceleration at the same time, no-load and load)
                     CANCEL_CHARGE, // Cancel charging
                     SWITCH_MAP, // Switch map, target_id is required and is the map number
                     RELOCATE // Relocation, target_id is required to be the node number, theta is optional to be the target angle (degrees)
                 };

@extensibility(FINAL)
                 struct AMRCommand_ {
                     AMRCommandType command_type; //Command type, refer to the above AMRCommandType
                     uint32 target_id; // Target ID (MOVE_TO_TAG/SWITCH_MAP/RELOCATE, etc.)
                     float linear_vel; // Linear speed, m/s (REMOTE_CONTROL/SET_VEL/SET_ACCEL/SET_DECEL)
                     float angular_vel; //Angular velocity, rad/s
                     uint32 command_id; //Command ID (for tracking)
                     uint64 timestamp; // timestamp
                     float theta; // Angle (degree): MOVE_TO_TAG/RELOCATE is the absolute angle, ROTATE is the relative angle
                 };
             };

         };
     };

   ```

1. **Command type and fields:**

| Command | Main fields | Description |
   | -------------------------------------------------- | -------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------ |
   | CANCEL\_TASK | — | Cancel the current walking task; **Only valid when** `navigation_status == RUNNING` |
   | PAUSE\_TASK | — | Pause the current walking task; **Only valid when** `navigation_status == RUNNING` |
   | RESUME\_TASK | — | Resume a suspended walking task |
   | MOVE\_TO\_TAG | `target_id`, `theta`(optional) | Navigate to the topology mark point; `theta` is the absolute orientation to the point (**degree**, 0,360; negative values are modulo 0,360) |
   | MOVE\_TO\_CHARGE | — | Start the automatic recharge process (see §12.4); non-immediate forced issuance of a single recharge |
   | CANCEL\_CHARGE | — | Cancel charging and leave the charging pile; **must first** activate automatic recharge through `MOVE_TO_CHARGE`, otherwise it will be invalid (including the scenario of manually pushing the pile into charging) |
   | REMOTE\_CONTROL | `linear_vel`, `angular_vel` | Remote control speed; must first `START_REMOTE`; if `SET_VEL` has been set, the amplitude will be limited proportionally |
   | ROTATE | `theta` | Rotate in place relative to the current heading (**degree**) |
   | START\_REMOTE / STOP\_REMOTE | — | Enter/exit remote control mode (`work_mode` 3/0); remote control must be stopped before exiting |
   | SUBSCRIBE\_LASER / UNSUBSCRIBE\_LASER | — | Subscribe/unsubscribe chassis lidar data |
   | START\_MAPPING / SAVE\_MAP / STOP\_MAPPING | — | Mapping control |
   | SET\_VEL | `linear_vel`, `angular_vel` | No-load/load linear speed and angular speed upper limit (m/s, rad/s) |
   | SET\_ACCEL | `linear_vel`, `angular_vel` | Linear/angular acceleration (m/s², rad/s²), reusing the same struct field |
   | SET\_DECEL | `linear_vel`, `angular_vel` | Linear/angular deceleration, the fields are the same as above |
   | SWITCH\_MAP | `target_id` | Switch map; if the current map ID is consistent, it will succeed directly |
   | RELOCATE | `target_id`, `theta`(optional) | Relocate to the node; `theta` is the target orientation (**degree**, normalized to 0,360) |

**General prerequisites:** The chassis is online and in a state that can receive commands. Autonomous navigation commands require that the chassis can currently plan new tasks. When the robot upper limb is in damping mode, `MOVE_TO_TAG` will be rejected.
4. **For remote control function**

1. Before starting remote control, send `START_REMOTE`
   2. Then use `REMOTE_CONTROL` to issue according to `linear_vel` (m/s) and `angular_vel` (rad/s); there is no need to send it repeatedly at high frequency, just keep the latest command. If the upper limit has been configured through `SET_VEL`, the excess part will be reduced proportionally.
   3. Please send `STOP_REMOTE` when the remote control is no longer needed, otherwise the chassis cannot receive new tasks
5. **For navigating to a marker (MOVE\_TO\_TAG):**

1. Parameters

1. `target_id` (required): topological node/marker number
      2. `theta` (optional): absolute orientation to the point, unit **degree**; leaving it blank or 0 means no orientation is specified; negative values are treated as 0
   2. How to determine whether the current task has been completed?

1. Subscribe to `rt/amr/state`: `navigation_status` changes to 2 (RUNNING) to indicate execution, to 3 (COMPLETED) to indicate completion; if it fails, it becomes 4 (FAILED)
   3. How to judge that the task has been successfully distributed?

1. Subscribe to `rt/amr/state`: `device_status` becomes 2 (TASKING), indicating that the chassis has received and started executing the task
   4. Is it possible to issue the next task before the previous task is completed?

1. Not recommended; the previous task will not be automatically canceled and the new task will usually be executed after the previous task is completed.
      2. If you need to preempt, please `CANCEL_TASK` first, and then issue a new navigation command
   5. **Pause/Cancel:** `PAUSE_TASK` / `CANCEL_TASK` will only take effect when `navigation_status == RUNNING`
6. **For automatic recharge (MOVE\_TO\_CHARGE):**

After sending `MOVE_TO_CHARGE`, the controller starts the automatic recharging process and decides whether and when to issue the back-to-stake task based on the real-time power and chassis status; **Not** every DDS command is equivalent to an immediate return to the pile.

1. **Battery Threshold:** The low battery threshold and severe low battery threshold are configured on the robot side (typical default values ​​are about 20% / 5%). When the battery level is higher than the low battery threshold and is not charging, the automatic recharging process will end (it will be deemed that no recharging is required)
   2. **Timing of sending back piles:** When the chassis is idle (`device_status==DEVIDLE`) and the battery power ≤ low power threshold, it will try to send back piles; usually only one attempt is made in the same round of automatic charging. If `navigation_status == FAILED` within about 1 s after issuance, it can be considered that the charging point cannot be found or the recharge fails.
   3. **Recharge successful:** `device_status == CHARGING` means **recharge completed** (charging pile has been connected), which can be judged by subscribing to `rt/amr/state`
   4. **Leave the charging pile:** After the recharging is completed and the battery is charged, you need to send `CANCEL_CHARGE` to exit the charging pile. **Must be issued first** `MOVE_TO_CHARGE` **Start automatic recharge**, `CANCEL_CHARGE` will take effect; if charging is entered by manual pushing of piles, etc., and the automatic recharge process is not followed, `CANCEL_CHARGE` will be issued at this time **Invalid**
   5. **Process interrupted (stake failed):**

1. Other navigation tasks (such as `MOVE_TO_TAG`) are received on the way back to the pile → the automatic recharging process is interrupted, and `MOVE_TO_CHARGE` needs to be sent again to re-enter.
      2. On the way back to the pile, the upper body of the robot starts to move → Cancel the chassis return to the pile and interrupt the automatic recharging
      3. Battery ≤ severe low battery threshold → cancel chassis navigation task and trigger upper body safety protection (damping)
7. **For switch map (SWITCH\_MAP) and relocation (RELOCATE): **

After issuing the command, judge the progress by subscribing to `map_switch_state` and `relocate_state` in `rt/amr/state`

1. `SWITCH_MAP`

1. Parameter: `target_id` is the target map ID; if it is consistent with the current map ID, the command side can directly regard it as successful.
      2. Status judgment: read `map_switch_state` - 0: No switching; 1: Switching in progress; 2: Switching completed; 3: Switching failed
   2. `RELOCATE`

1. Parameters: `target_id` is the node number; `theta` is the optional target orientation (degree)
      2. Status judgment: read `relocate_state` - 0: No relocation; 1: Relocation in progress; 2: Relocation completed; 3: Relocation failed

## 13 Joystick Control

1. ~~Function Description: Mainly used to forward the joystick data sent by the app to the algorithm for use~~
2. ~~Topic name: rt/remote/control~~
3. ~~Note, this topic is not open to the public~~
4. ~~IDL file content: ~~

```shell
   module dobot_atom{
       module msg{
           module dds_{
               @extensibility(FINAL)
               struct JoystickValue_ {
                   float x;
                   float y;  
               };

@extensibility(FINAL)
               struct RemoteControl_ {
                   JoystickValue_ btn_move; // Left knob key value
                   JoystickValue_ btn_turn; // Right knob key value
               };
           };
       };
   };

```
5. ~~As above, one joystick has two knobs, the left knob and the right knob, which are stored in btn\_move and btn\_turn respectively, ~~
6. ~~Take the left knob as an example. When the knob is turned to the right, the x value increases, and vice versa. When the knob is turned upward, the y value increases, and vice versa.~~
7. ~~JoystickValue\_Threshold interval: \[-1,1\]~~

## 14 Emergency stop status

1. Function description: Emergency stop information in the entire embedded system.
2. Topic name: rt/emergency/state
3. IDL file content:

```shell
module dobot_atom{
    module msg{
        module dds_{
            @extensibility(FINAL)
            struct EmergencyState_ {
                boolean soft_emergency_triggered; // Soft emergency stop (app triggered)
                boolean hard_emergency_triggered; // Hard emergency stop (triggered by user board)
                boolean amr_emergency_triggered; // AMR emergency stop (wheeled chassis triggered)
            };
        };
    };
};

```

## 15 State machine status

1. Function description: Information about the state machine released by the algorithm
2. Topic name: rt/set/fsm/id
3. IDL file content:

* id: current algorithm state machine installed solid state
   * current\_action: string. When the user executes HandsUp through the app or joystick, the algorithm field is set to "HandsUp". When the action is completed, current\_action is set to empty.

```shell
module dobot_atom{
    module msg{
        module dds_{
            @extensibility(FINAL)
            struct SetFsmId_ {
                uint16 id; // For algorithm state machine
                char current_action; //The action currently being executed by the algorithm, empty means no action
            };
        };
    };
};

```

## 15 AMR lidar data

1. Function description: When passing [**amr\_cmd**](https://alidocs.dingtalk.com/i/nodes/KGZLxjv9VG37o9N6I7N2zj4XV6EDybno?utm_scene= After team_space&utm_medium=main_vertical&utm_source=search) sends a ++**subscribe to lidar**++ command, the thread starts publishing the lidar data and odometer data of the wheeled humanoid chassis.
2. Topic name: rt/amr/laserscan
3. IDL file content:

* timestamp: timestamp of publishing lidar
   * lidar: lidar information, up to 1024 elements, representing laser ranging at each angle, unit mm

```shell
module dobot_atom{
    module msg{
        module dds_{
            @extensibility(FINAL)
            struct LaserScan_ {
                uint64 timestamp; // timestamp (ms)
                int32 lidar[1024]; // Laser data (up to 1024 points, fill with 0 if not full), unit mm
            };
        };
    };
};



```

## 16 AMR odometer data

1. Function description: After sending a ++**subscription lidar**++ command through **amr\_cmd**, the thread begins to publish the lidar data and odometer data of the wheeled humanoid chassis.
2. Topic name: rt/amr/odom
3. IDL file content:

* timestamp: timestamp of publishing lidar

```shell
module dobot_atom{
    module msg{
        module dds_{
            @extensibility(FINAL)
            struct Odom_ {
                uint64 timestamp; // timestamp (ms)
                int32 x; // Odometer X coordinate, unit mm
                int32 y; // Odometer Y coordinate, unit mm
                int32 angle; // Odometer angle, unit 0.001 degrees
            };
        };
    };
};

```

## 17 Dahuan Claw Command

A. **Function description**: Send initialization, target opening position and clamping force to the left and right Dahuan clamping jaws. After the Host subscribes, it writes the gripper register through Modbus.

B. **Topic name**: `rt/hands/cmd`

C. **IDL file content** (`dh_gripper_cmd.idl`):

```shell
module dobot_atom{
    module msg{
        module dds_{
            enum DHGripperInitCmd {
                DH_INIT_NONE, //Do not trigger initialization
                DH_INIT_HOMING // Return to zero position (find one-way position)
            };

@extensibility(FINAL)
            struct DHGripperSingleCmd_ {
                DHgripperInitCmd init_cmd;
                float position; // 0~1, 1.0=fully opened (internal mapping register 0~1000)
                float force;
            };

@extensibility(FINAL)
            struct DHGripperCmd_ {
                DHGripperSingleCmd_ right;
                DHGripperSingleCmd_ left;
            };
        };
    };
};
```

I. Referring to the above IDL, the main fields of the single-sided gripper:

1. `init_cmd`: `DH_INIT_HOMING` triggers zero return initialization; `DH_INIT_NONE` does not trigger
2. `position`: target opening **0~1**
3. `force`: clamping force, commonly used is about **20~100** (onboard default 20)

Ii. Recommended order of use:

1. Release `init_cmd = DH_INIT_HOMING` (left/right can be configured separately)
2. Subscribe to `rt/hands/state` and wait for both sides `init_result_state == DH_INIT_SUCCESS`
3. Set `init_cmd` to `DH_INIT_NONE`, and then release `position` / `force` as needed
4. Before initialization is successful, the location command will be ignored by the Host; the initialization timeout is about **10 s** and you need to HOMING again

## 18 Dahuan clamping jaw status

A. **Function description**: Host periodically releases the left and right gripper initialization results, clamping status, real-time opening and force feedback.

B. **Topic name**: `rt/hands/state`

C. **IDL file content** (`dh_gripper_state.idl`):

```shell
module dobot_atom{
    module msg{
        module dds_{
            enum DHGripperInitResultState {
                DH_INIT_NOT_DONE,
                DH_INIT_SUCCESS,
                DH_INIT_EXECUTING
            };

Enum DHGripperGripState {
                DH_GRIP_MOVING,
                DH_GRIP_IN_PLACE,
                DH_GRIP_HOLDING_OBJECT,
                DH_GRIP_OBJECT_DROPPED
            };

@extensibility(FINAL)
            struct DHGripperSingleState_ {
                DHGripperInitResultState init_result_state;
                DHGripperGripState grip_state;
                float realtime_position;  // 0~1
                float get_force_value;
            };

@extensibility(FINAL)
            struct DHGripperState_ {
                DHGripperSingleState_ right;
                DHGripperSingleState_ left;
            };
        };
    };
};
```

I. `init_result_state`: `0` not initialized, `1` successful, `2` executing

Ii. `grip_state`: `0` is in motion, `1` is in place, `2` clamps the object, `3` the object falls

## 19 Position control joint position control command

A. **Function description**: Switch MIT / position control application mode, or issue position control motion commands to both arms and lifting (AtomW). After the Host subscribes, it calls the position control algorithm and writes the PLC shared memory.

B. **Topic name**: `rt/posctl/cmd`

C. **IDL file content** (`pos_control_cmd.idl`):

```shell
module dobot_atom{
    module msg{
        module dds_{

Enum PosControlCommandType {
                SWITCH_MODE, /* Switch MIT <-> bit control. payload: target_mode */
                MOVE_L, /* Cartesian straight line. payload: left_pose/right_pose/hips_height + speed */
                MOVE_J, /* joint space. payload: left_q/right_q/hips_q (prefix length merge) */
                RUN_TO, /* Cartesian tracing. payload: Same as above; stop=0 starts, stop=1 stops */
                CART_JOG, /* Cartesian jog. payload: left_cart_jog/right_cart_jog/hips_cart_jog; stop Same as above */
                STOP_MOTION, /* Stop the current position control motion (optional extension) */
                SERVO_J /* Joint space streaming servo. payload: left_q/right_q/hips_q (same as MOVE_J); stop is the same as RUN_TO; requires ~10Hz keep-alive */
            };

@extensibility(FINAL)
            struct PosControlCmd_ {
                PosControlCommandType command_type;

/* ---------- MOVE_J / SERVO_J ---------- */
                sequence<double> left_q; /* 1..7, prefix merge; 0=left arm does not participate; SERVO_J multiplexing */
                sequence<double> right_q; /* 1..7; SERVO_J multiplexing */
                sequence<double> hips_q; /* 1..3; SERVO_J multiplexing */

/* ---------- MOVE_L / RUN_TO ---------- */
                sequence<double> left_pose; /* 0 or 6 */
                sequence<double> right_pose; /* 0 or 6 */
                sequence<double> hips_height; /* 0 or 1, allowed value is 0.0 mm */

/* ---------- CART_JOG ---------- */
                sequence<int32> left_cart_jog; /* 0 or 6, value -1/0/1 */
                sequence<int32> right_cart_jog; /* 0 or 6 */
                sequence<int32> hips_cart_jog; /* 0 or 1 */

/** RUN_TO/CART_JOG/SERVO_J: 0=start, 1=stop */
                int32 stop;

Double speed; /* MOVE_L / MOVE_J / RUN_TO speed (algorithm explanation, usually 0~1) */
                int32 target_mode; /* SWITCH_MODE: 0=MIT, 1=position control */

Uint32 command_id;  
                uint64 timestamp;  
            };
        };
    };
};

```

I. Refer to the above IDL, which currently contains seven commands:

1. `SWITCH_MODE`: MIT ↔ position control switching (`target_mode`: 0=MIT, 1=position control)
2. `MOVE_L`: Cartesian straight line, single delivery
3. `MOVE_J`: joint point to point (single planning), `left_q`/`right_q`/`hips_q` is the joint angle **deg**
4. `RUN_TO`: Cartesian tracking, `stop=0` starts, `stop=1` stops; **~10Hz must be repeated during the process**
5. `CART_JOG`: Cartesian jog, each axis **1/0/1**, `stop` has the same semantics as `RUN_TO`; **~10Hz must be repeated during the process**
6. `STOP_MOTION`: Stop all position control movements
7. `SERVO_J`: Joint space streaming servo (algorithm `ServoJ`), the load is the same as `MOVE_J` (`left_q`/`right_q`/`hips_q`); `stop=0` delivers the target, `stop=1` stops; **50Hz must be repeated during the process**, Host will automatically `Stop()` if no new command is received within **300ms**

Ii. Component participation rules: `left_*` / `right_*` / `hips_*` **sequence length 0 means the component does not participate**; fill in at least one set of components that need to move.

Iii. Motion command preconditions:

1. First `SWITCH_MODE` + `target_mode=1`, poll `rt/posctl/state` until `control_mode == 1` (switching is asynchronous, it may take several seconds)
2. The relevant axis has been enabled (`rt/enable/motors`)
3. `error_code` is all 0; if there is an error, send `rt/clear/errors` to clear the error before running.

Iv. For `SERVO_J` / `RUN_TO` / `CART_JOG` (streaming commands, must be kept alive):

1. Start/continue when `stop=0`: The same type of command must be issued repeatedly at about **10Hz** **cycle, and carry the current target (joint angle or Cartesian pose/jog direction)
2. Host monitoring command interval: If there is no new `SERVO_J` / `RUN_TO` / `CART_JOG` within **300ms**, the movement will automatically stop (equivalent to `STOP_MOTION`)
3. Stop when `stop=1`: **Send it once** (you don’t need to carry the target field)
4. You can also use `STOP_MOTION` or any command `stop!=0` to stop uniformly

### Example 1: Switch to the position control application

```plaintext
command_type  = SWITCH_MODE
target_mode   = 1          // 1=位控，0=MIT
command_id    = 1
```

After publishing, poll `rt/posctl/state` until `control_mode == 1`.

### Example 2: MOVE\_J (left arm 7 joints only)

```plaintext
command_type  = MOVE_J
left_q        = [0, -30, 0, 90, 0, 0, 0]   // deg
speed         = 0.5
command_id    = 2
```

When only moving the left arm joint 0, you can only fill in the prefix: `left_q = [10]` (ungiven joints maintain the current feedback angle).

### Example 3: MOVE\_L (right arm Cartesian line only)

```plaintext
command_type  = MOVE_L
right_pose    = [300, 200, 160, 0, 0, 0]   // [x,y,z] mm + [rx,ry,rz] deg
speed         = 0.5
command_id    = 3
```

### Example 4: RUN\_TO start/stop (left arm tracking only)

Start:

```plaintext
command_type  = RUN_TO
left_pose     = [283.74, 206.5, 157.8, 0, 0, 0]
stop          = 0
command_id    = 4
```

Stop:

```plaintext
command_type  = RUN_TO
stop          = 1
command_id    = 5
```

### Example 5: CART\_JOG jog (left arm +X direction)

Start:

```plaintext
command_type     = CART_JOG
left_cart_jog    = [1, 0, 0, 0, 0, 0]   // 各维 -1 / 0 / 1
stop             = 0
command_id       = 6
```

Stop:

```plaintext
command_type  = CART_JOG
stop          = 1
command_id    = 7
```

### Example 6: STOP\_MOTION (stop all position control motion)

```plaintext
command_type  = STOP_MOTION
command_id    = 8
```

## 20 Position control status

A. **Function Description**: Host periodically releases position control application mode, double arm/lifting joint angle, Cartesian posture, algorithm error code and planning status.

B. **Topic name**: `rt/posctl/state`

C. **IDL file content** (`pos_control_state.idl`):

```shell
module dobot_atom{
    module msg{
        module dds_{
            struct CartTargetPose {
                double left_pose[6];   // mm, deg
                double right_pose[6];
                double hips_height;    // mm
            };

Struct PosControlJointQ {
                double left_q[7];    // deg
                double right_q[7];
                double hips_q[3];
            };

Struct PosControlState_ {
                CartTargetPose pose; //Descartes pose
                int32 error_code[3]; //[0]Left arm [1]Right arm [2]Lifting
                int32 status_code[3]; // Coaxial sequence
                int32 control_mode; // 0=MIT, 1=position control
                PosControlJointQ joint_q;//joint angle
            };
        };
    };
};
```

I. `control_mode`: `0` MIT operation control, `1` position control application

Ii. `error_code` value (if any path is non-0, motion commands should be stopped):

| Value | Meaning |
| -- | ------------------- |
| 0 | Normal |
| 1 | Initialization failed |
| 2 | Servo/enable error |
| 3 | Inverse solution failed |
| 4 | Planning failed |
| 5 | The parameter dimension/length is illegal |
| 6 | Arm joint reference speed exceeds limit |

Iii. `status_code` value:

| Value | Meaning |
| -- | ---------------------------------- |
| 0 | Ready to accept new commands |
| 1 | Running |
| 2 | PlanStop (decelerating to stop) |
| 3 | Error (processed in combination with `error_code`) |

Iv. How to determine when the mode switch is completed? -

1. Subscribe to `rt/posctl/state`, and `control_mode` changes to `1` to indicate that you have entered the position control application

V. How to judge the end of a single exercise?

1. The corresponding component `status_code[i]` changes to `0` (Ready) to indicate the end of the movement
