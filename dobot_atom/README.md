# Dobot Atom ROS2 DDS Interface

## 概述

本包提供了越疆人形机器人的ROS2 DDS接口消息定义，支持上下肢分控的控制方式。该接口基于DDS通信协议，提供实时的机器人状态监控和控制功能。

## 包信息

- **包名**: dobot_atom
- **版本**: 1.1.2.0-beta1
- **维护者**: futingxing<futingxing@dobot_robots.com>

## 消息类型

### 基础消息类型

#### BmsState.msg

电池管理系统状态信息

```
uint16 bms_state                    # BMS状态
uint16 afe_state                    # AFE芯片状态
uint32 bms_alarms                   # BMS故障码
uint16 battery_level                # 电池电量百分比
uint16 battery_health               # 电池健康度
uint16 pcb_board_temp               # PCB板温度
uint16 afe_chip_temp                # AFE芯片温度
uint16 battery_now_current          # 电池包当前电流
uint16[16] cells_voltage            # 16个电芯电压
uint16 battery_pack_current_voltage # 电池包电压
uint16 battery_pack_io_voltage      # 电池包放电/充电接口的电压
uint32 bms_work_time                # BMS运行时间
uint16 bms_hardware_version         # BMS硬件版本号
uint16 bms_software_version         # BMS软件版本号
uint16 heartbeat                    # 心跳
```

#### BmsCmd.msg

电池管理系统控制命令

```
uint16 clear_errors                 # 清除BMS错误
uint16 spare_battery_charge_request # 备电充电请求
```

#### BmsInfo.msg

电池管理系统信息

```
uint16 latch_state                  # 锁存状态
uint16 spare_battery_level          # 备电电量
uint16 spare_battery_exchange_complete  # 备电更换完成标志
BmsLog log                          # BMS日志
```

#### BmsLog.msg

电池管理系统日志

```
uint16 log_index                    # 日志索引
uint16 log_capacity                 # 日志容量
uint32 log_bms_runtime              # BMS运行时间
uint32 log_bms_alarm                # BMS故障码
uint16 log_bms_status               # BMS状态
uint16 log_afe_status               # AFE状态
uint16 log_bms_soc                  # 电池电量
uint16 log_bms_vtop                 # 顶端电压
uint16 log_bms_vpack                # 电池包电压
uint16 log_bms_current              # 电池电流
uint16[16] log_bms_vol_ceil         # 16个电芯电压
uint16 log_bms_temp_pcb             # PCB温度
uint16 log_bms_temp_afe             # AFE温度
uint16 log_bms_temp_ceil            # 电芯温度
uint16 log_bms_reserve_flag         # 保留标志
uint32 log_bms_rtc_time             # RTC时间戳
```

#### MotorState.msg

电机状态信息

```
uint8 mode          # 模式
float32 q           # 角位置
float32 dq          # 角速度
float32 ddq         # 角加速度
float32 tau_est     # 估计的力矩
float32 q_raw       # 原始角位置
float32 dq_raw      # 原始角速度
float32 ddq_raw     # 原始角加速度
uint8 mcu_temp      # 伺服控制板温度
uint8 mos_temp      # MOS管温度
uint8 motor_temp    # 电机温度
uint8 bus_voltage   # 母线电压
```

#### MotorCmd.msg

电机控制命令

```
uint8 mode      # 模式
float32 q       # 位置
float32 dq      # 速度
float32 tau     # 力矩
float32 kp      # 比例增益
float32 kd      # 微分增益
```

#### IMUState.msg

IMU传感器状态

```
float32[4] quaternion    # 四元数
float32[3] gyroscope     # 陀螺仪数据
float32[3] accelerometer # 加速度计数据
float32[3] rpy          # 欧拉角（滚转、俯仰、偏航）
uint8 temperature       # 温度传感器数据
```

#### JoystickValue.msg

摇杆值信息

```
float32 x   # x轴值 [-1, 1]
float32 y   # y轴值 [-1, 1]
```

#### LaserScan.msg

激光雷达扫描数据

```
uint64 timestamp       # 时间戳
int32[1024] lidar      # 1024点激光雷达数据
```

#### Odom.msg

里程计信息

```
uint64 timestamp    # 时间戳
int32 x             # x坐标
int32 y             # y坐标
int32 angle         # 角度
```

#### Velocity.msg

速度信息

```
float32 linear_vel   # 线速度
float32 angular_vel  # 角速度
```

### 控制接口消息

#### SetFsmId.msg

切换底层状态

```
# WorkingState constants
uint8 HIDLE=0          # 空闲状态
uint8 HTASKING=1       # 任务状态

uint16 id              # 对应算法状态机ID
string current_action  # 算法当前正在执行的动作
uint8 state            # 工作状态(HIDLE/HTASKING)
```

#### SwitchUpperControl.msg

切换上肢控制权限

```
bool flag  # true: 上肢有控制，false: 上肢无控制
```

#### EnableMotors.msg

电机使能控制

```
bool[29] flag  # 29个关节的使能标志，true: 使能，false: 去使能
```

#### PosControlCmd.msg

位置控制关节位控命令

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

位控目标笛卡尔位姿

```
float64[6] left_pose
float64[6] right_pose
float64 hips_height
```

#### PosControlJointQ.msg

位控关节角状态

```
float64[7] left_q
float64[7] right_q
float64[3] hips_q
```

#### PosControlState.msg

位控应用状态

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

### 上肢相关消息

#### UpperState.msg

上肢状态信息

```
char[16] robot_type             # 对应产品定义的类型
bool is_upper_control           # 上肢控制状态
uint16 fsm_id                   # 对应算法状态机
MotorState[17] motor_state      # 17个电机状态
BmsState bms_state              # 电池状态
uint8[40] wireless_remote       # 手柄键值
uint32 reserve                  # 越疆保留
```

#### UpperCmd.msg

上肢控制命令

- 左臂7个关节 (索引0-6)
- 右臂7个关节 (索引7-13)
- 头部偏航 (索引14)
- 头部俯仰 (索引15)
- 躯干扭转 (索引16)

```
MotorCmd[17] motor_cmd  # 17个电机命令
```

### 下肢相关消息

#### LowerState.msg

下肢状态信息

```
uint16 fsm_id                   # 对应算法状态机
IMUState imu_state              # IMU状态
MotorState[12] motor_state      # 12个电机状态
BmsState bms_state              # 电池状态
uint8[40] wireless_remote       # 手柄键值
uint32 reserve                  # 越疆保留
```

#### LowerCmd.msg

下肢控制命令

- 左腿7个关节 (索引0-5)
- 右腿7个关节 (索引6-11)

```
MotorCmd[12] motor_cmd  # 12个电机命令
```

### 全身控制消息

#### LowState.msg

全身低层状态信息

```
uint16 algs_mode                # 算法模式
IMUState imu_state              # IMU状态
MotorState[30] motor_state      # 30个电机状态
BmsState bms_state              # 电池状态
uint8[40] wireless_remote       # 手柄键值
uint32 reserve                  # 越疆保留
```

#### LowCmd.msg

全身低层控制命令

```
uint16 algs_mode                # 算法模式
MotorCmd[30] motor_cmd          # 30个电机命令
```

### 轮式底盘(AMR)相关消息

#### AMRState.msg

底盘状态信息

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

uint8 device_status             # 设备状态
uint8 navigation_status         # 导航状态
AMRBasicStatus basic_status     # 基础状态
float32[3] position             # 当前位置 {x,y,yaw}
Velocity velocity               # 速度信息
AMREventStatus amr_event        # 底盘相关事件
uint32[32] error_code           # 错误码
uint32 task_id                  # 任务ID
uint32 work_mode                # 机器人工作模式
```

#### AMRCommand.msg

底盘控制命令

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

uint8 command_type              # 命令类型
uint32 target_id                # 目标ID
float32 linear_vel              # 线速度
float32 angular_vel             # 角速度
uint32 command_id               # 命令ID
uint64 timestamp                # 时间戳
float32 theta                   # 角度
```

#### AMRBasicStatus.msg

底盘基础状态

```
float32 battery_level           # 电池电量
float32 battery_voltage         # 电池电压
float32 battery_current         # 电池电流
uint16 heartbeat                # 心跳值
```

#### AMREventStatus.msg

底盘事件状态

```
bool emergency_stop_pressed     # 急停按钮
bool enable_pressed             # 使能键
bool path_blocked               # 路径被挡
bool low_battery                # 电量低
bool obstacle_detected          # 检测到障碍物
```

### 系统状态消息

#### AxisStateInfo.msg

关节状态详细信息

```
uint8 servo_state           # 使能、下使能、报错状态
uint16 error_code           # 错误码
uint16 warn_code            # 警告码
int32 pos_err_code          # 位置超限错误码
int32 vel_err_code          # 速度超限错误码
int32 torque_err_code       # 扭矩超限错误码
uint8 node_state            # 关节在线状态
uint8 display_op_mode       # 关节状态字
bool is_virtual             # 虚实轴标志
uint8 mcu_temp              # MCU温度
uint8 mos_temp              # MOS管温度
uint8 motor_temp            # 电机温度
uint8 bus_voltage           # 母线电压
uint16 software_version     # 软件版本
```

#### EcatSlaveInfo.msg

EtherCAT从站信息

```
bool is_virtual             # 虚实标志
uint8 slave_state           # 从站状态
uint16 error_code           # 错误码
uint16 software_version     # 软件版本
```

#### MainNodesState.msg

主要节点状态

```
AxisStateInfo[6] left_leg   # 6个左腿关节状态
AxisStateInfo[6] right_leg  # 6个右腿关节状态
AxisStateInfo waist         # 腰部状态
AxisStateInfo[7] left_arm   # 左臂状态
AxisStateInfo[7] right_arm  # 右臂状态
AxisStateInfo[2] head       # 头部状态
EcatSlaveInfo[2] ecat2can   # 2个EtherCAT转CAN模块状态
```

#### ClearErrors.msg

清除错误命令

```
int32 msg_id  # 消息ID（任意值）
```

#### EmergencyState.msg

急停状态信息

```
bool soft_emergency_triggered   # 软急停 (app触发)
bool hard_emergency_triggered   # 硬急停 (用户板触发)
bool amr_emergency_triggered    # AMR急停 (底盘触发)
bool di_emergency_triggered     # DI急停 (传感器触发)
```

### 灵巧手消息

#### HandsState.msg

灵巧手状态

```
MotorState[12] hands  # 12个手指电机状态
```

#### HandsCmd.msg

灵巧手控制命令

```
MotorCmd[12] hands  # 12个手指电机命令
```

### 遥控消息

#### RemoteControl.msg

摇杆控制信息

```
JoystickValue btn_move    # 左旋钮键值
JoystickValue btn_turn    # 右旋钮键值
```

### 遥操作消息

#### TeleopCmd.msg

遥操作控制命令

```
# TeleopCommandType constants
# 0 - START_SYNC        # 启动同步
# 1 - STOP_SYNC         # 停止同步
# 2 - START_TELEOP      # 启动遥操
# 3 - STOP_TELEOP       # 停止遥操
# 4 - START_RECORD      # 开始记录
# 5 - STOP_RECORD       # 停止记录

uint8 command_type
```

#### TeleopState.msg

遥操作状态

```
uint8 teleop_state       # 遥操作状态
uint8 sync_state         # 同步状态
uint8 record_state       # 记录状态
# TeleopState enum:
# 0 - DISABLED
# 1 - ENABLING
# 2 - ENABLED
```

### 大脑推理消息

#### CerebrumCmd.msg

大脑控制命令

```
# CerebrumCommandType constants
# 0 - START_REASONING   # 开始推理
# 1 - STOP_REASONING    # 停止推理
# 2 - BACK_TOSTRT       # 回到起点

uint8 command_type
```

#### CerebrumState.msg

大脑状态

```
uint8 teleop_state       # 遥操作状态
uint8 sync_state         # 同步状态
uint8 record_state       # 记录状态
uint8 reasoning_state    # 推理状态
uint8 backing_state      # 回退状态
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

### 灵巧手(Inspire)消息

#### InspireCmd.msg

Inspire灵巧手控制命令

```
MotorCmd[12] hands  # 12个手指电机命令
```

#### InspireState.msg

Inspire灵巧手状态

```
MotorState[12] hands  # 12个手指电机状态
```

### 电源管理消息

#### PowerCmd.msg

电源控制命令

```
uint16 control_power_supply     # 控制电源开关
uint16 clear_errors             # 清除错误
uint16 set_led_colour           # 设置LED颜色
uint16 set_led_mode             # 设置LED模式
uint16 set_sd_brightness        # 设置屏幕亮度
uint16 set_sd_mode              # 设置屏幕模式
```

#### PowerState.msg

电源状态

```
uint16 power_supply_state           # 电源状态
uint16 strong_power_current         # 强电电流
uint16 weak_power_current           # 弱电电流
uint16 weak_power_voltage           # 弱电电压
uint16 replenishment_port_voltage   # 补电口电压
uint16 hardware_version             # 硬件版本
uint16 software_version             # 软件版本
uint16 error_codes                  # 错误码
uint16 heartbeat                    # 心跳
uint16 strong_power_voltage         # 强电电压
uint16 brk_2v5_voltage              # 抱闸2.5V电压
uint16 brk_state                    # 抱闸状态
```

### 其他消息

#### ChangeMode.msg / GoHome.msg / CalibrateImu.msg / SetCalibrationFlag.msg / ModifyJointOffset.msg / PlayTTS.msg / DHGripperCmd.msg / DHGripperSingleCmd.msg / DHGripperState.msg / DHGripperSingleState.msg

辅助功能消息，用于模式切换、回零、IMU校准、关节偏移修正及语音播报等功能。

## 话题映射

| 话题名称                    | 消息类型           | 功能描述         |
| --------------------------- | ------------------ | ---------------- |
| `rt/set/fsm/id`           | SetFsmId           | 切换底层状态     |
| `rt/switch/upper/control` | SwitchUpperControl | 切换上肢控制权限 |
| `rt/upper/state`          | UpperState         | 上肢状态信息     |
| `rt/upper/cmd`            | UpperCmd           | 上肢控制命令     |
| `rt/lower/state`          | LowerState         | 下肢状态信息     |
| `rt/lower/cmd`            | LowerCmd           | 下肢控制命令     |
| `rt/low/state`            | LowState           | 全身低层状态     |
| `rt/low/cmd`              | LowCmd             | 全身低层控制     |
| `rt/amr/state`            | AMRState           | 轮式底盘状态     |
| `rt/amr/cmd`              | AMRCommand         | 轮式底盘控制     |
| `rt/posctl/cmd`           | PosControlCmd      | 位控命令下发     |
| `rt/posctl/state`         | PosControlState    | 位控状态反馈     |
| `rt/main/nodes/state`     | MainNodesState     | 关节和CAN板状态  |
| `rt/clear/errors`         | ClearErrors        | 清除错误         |
| `rt/hands/state`          | HandsState         | 灵巧手状态       |
| `rt/hands/cmd`            | HandsCmd           | 灵巧手控制命令   |
| `rt/remote/control`       | RemoteControl      | 摇杆控制         |
| `rt/emergency/state`      | EmergencyState     | 急停状态         |
| `rt/laserscan`            | LaserScan          | 激光雷达数据     |
| `rt/odom`                 | Odom               | 里程计           |
| `rt/enable/motors`        | EnableMotors       | 电机使能控制     |
| `rt/bms/state`            | BmsState           | BMS状态          |
| `rt/bms/cmd`              | BmsCmd             | BMS控制          |
| `rt/bms/info`             | BmsInfo            | BMS信息          |
| `rt/teleop/cmd`           | TeleopCmd          | 遥操作控制       |
| `rt/teleop/state`         | TeleopState        | 遥操作状态       |
| `rt/cerebrum/cmd`         | CerebrumCmd        | 大脑控制         |
| `rt/cerebrum/state`       | CerebrumState      | 大脑状态         |
| `rt/inspire/cmd`          | InspireCmd         | Inspire灵巧手控制|
| `rt/inspire/state`        | InspireState       | Inspire灵巧手状态|
| `rt/power/state`          | PowerState         | 电源状态         |
| `rt/power/cmd`            | PowerCmd           | 电源控制         |

## 使用说明

### 1. 安全注意事项

- 在控制关节之前，必须使用 `rt/set/fsm/id` 接口将底层状态设置为非零力矩模式
- 如果需要外部控制上肢，必须通过 `rt/switch/upper/control` 接口打开上肢控制权限

### 2. 手柄键值说明

`wireless_remote[40]` 数组中的键值映射：

- `wireless_remote[2]`：从高位到低位对应 '', '', 'LT', 'RT', 'SELECT', 'START', 'LB', 'RB'
- `wireless_remote[3]`：从高位到低位对应 'LEFT', 'DOWN', 'RIGHT', 'UP', 'Y', 'X', 'B', 'A'
- `wireless_remote[4-7]`：LX键值（[-1,0]范围的浮点值）
- `wireless_remote[8-11]`：RX键值（[0,1]范围的浮点值）
- `wireless_remote[12-15]`：RY键值（[-1,0]范围的浮点值）
- `wireless_remote[16-19]`：LY键值（[0,1]范围的浮点值）

### 3. 位控接口使用

- 先发送 `rt/posctl/cmd`，并设置 `command_type=SWITCH_MODE`、`target_mode=1`
- 轮询 `rt/posctl/state`，直到 `control_mode == CONTROL_MODE_POSITION`
- `MOVE_J` 使用 `left_q/right_q/hips_q`，`MOVE_L` 和 `RUN_TO` 使用 `left_pose/right_pose/hips_height`
- `RUN_TO` 与 `CART_JOG` 使用 `stop=0` 开始，`stop=1` 停止，发一次即可
- `left_*` / `right_*` / `hips_*` 长度为 0 表示该部件不参与
- 长度约束由消息上限和业务逻辑共同保证，例如 `left_q` 最大 7，`left_pose` 最大 6

### 4. 灵巧手数组排序

**关节顺序（从索引0开始）：**

- 右手（索引0-5）：小指、无名指、中指、食指、大拇指弯曲、大拇指旋转
- 左手（索引6-11）：小指、无名指、中指、食指、大拇指弯曲、大拇指旋转

**角度范围：**

- 小指、无名指、中指、食指：19°~176.7°（0.3316rad~3.0815rad）
- 大拇指弯曲角度：-13°~53.6°（-0.2269rad~0.9346rad）
- 大拇指旋转角度：90°~165°（1.5708rad~2.8798rad）

### 5. 轮式底盘(AMR)使用

#### 遥控功能
1. 发送 `START_REMOTE` 请求。
2. 发送 `REMOTE_CONTROL` 请求，设置 `linear_vel` 和 `angular_vel`。
3. 结束时发送 `STOP_REMOTE` 请求。

#### 导航功能
1. 发送 `MOVE_TO_TAG` 请求，设置 `target_id` 和 `theta`。
2. 通过 `AMRState` 中的 `navigation_status` 判断任务状态：
   - `RUNNING (2)`: 运行中
   - `COMPLETED (3)`: 已完成

#### 激光雷达与建图
1. `SUBSCRIBE_LASER` / `UNSUBSCRIBE_LASER` 控制激光雷达数据订阅。
2. `START_MAPPING` / `STOP_MAPPING` / `SAVE_MAP` 控制建图流程。
3. `SET_VEL` / `SET_ACCEL` / `SET_DECEL` 设置底盘运动参数。

### 6. 实时话题标识

所有需要实时发布的话题均以 `rt/` 开头。

### 7. 电机使能

通过 `rt/enable/motors` 话题发送 `EnableMotors` 消息控制关节使能，`flag` 数组长度为 29，对应全身各关节。

## 构建说明

```bash
# 在ROS2工作空间中构建
colcon build --packages-select dobot_atom

source install/setup.bash
```

## 依赖项

- ROS2 (Humble/Iron/Rolling)
- geometry_msgs
- rosidl_default_generators
- rosidl_generator_dds_idl
