# 二、DDS接口整理

# 修改历史

|  |  |  |  |
| - | - | - | - |

# 一、前言

本文为人形使用DDS方式交互的接口说明文档，由于DDS接口是通过IDL文件定义并序列化生成的，所以本文的接口文档全部由IDL文件定义。

所有需要实时发布的话题均以**rt/**开头。

越疆人形因为产品形态的多样化，采用的是上下肢分控的方式，即上肢和下肢使用不同的话题控制。上肢：腰部、左臂、右臂、头部，下肢：左腿、右腿。上肢和下肢可能会不同时存在，因此像电池信息、手柄信息在上下肢状态话题中均存在，即虽然两个话题中均有定义，但实际是同一个东西。

下文中提到的底层是指管理关节、BMS、imu、手柄的程序。如果使用人员熟悉强化学习，可以使用如下接口和底层交换，然后来控制越疆机器人。

算法状态机是内置于PC1的一个程序，它在某种情况下也会对上下肢进行控制，关于算法状态机的介绍另外有文档说明，这里不再赘述。

灵巧手数组排序：右手（大拇指弯曲、食指、中指、无名指、小拇指、大拇指旋转）、左手（大拇指弯曲、食指、中指、无名指、小拇指、大拇指旋转）。

# 二、接口说明

共用的子IDL文件

1. bms信息：bms\_state.idl

   ```shell
   cat bms_state.idl
   struct BmsState_
   module dobot_atom{
       module msg{
           module dds_{
               @extensibility(FINAL)
               struct BmsState_
               {
                   uint16  bms_state;                     /*BMS状态*/
                   uint16  afe_state;                     /*AFE芯片状态*/
                   uint32  bms_alarms;                    /*BMS故障码*/
                   uint16  battery_level;                 /*电池电量百分比*/
                   uint16  battery_health;                /*电池健康度*/
                   uint16  pcb_board_temp;                 /*PCB板温度*/
                   uint16  afe_chip_temp;                  /*AFE芯片温度*/
                   uint16  battery_now_current;            /*电池包当前电流*/
                   uint16  cells_voltage[16];             /*16个电芯电压*/
                   uint16  battery_pack_current_voltage;    /*电池包电压*/
                   uint16  battery_pack_io_voltage;         /*电池包放电/充电接口的电压*/
                   uint32  bms_work_time;                  /*BMS运行时间*/
                   uint16  bms_hardware_version;           /*BMS硬件版本号*/
                   uint16  bms_software_version;           /*BMS软件版本号*/
                   uint16  heartbeat;                    /*心跳*/
               };
           };
       };
   };
   ```
2. 关节信息：motor\_state.idl

   ```shell
   cat motor_state.idl
   module dobot_atom{
       module msg{
           module dds_{
               @extensibility(FINAL)
               struct MotorState_ {
                   uint8 mode;                  // 模式
                   float q;                     // 角位置
                   float dq;                    // 角速度
                   float ddq;                   // 角加速度
                   float tau_est;               // 估计的力矩
                   float q_raw;                 // 原始角位置
                   float dq_raw;                // 原始角速度
                   float ddq_raw;               // 原始角加速度
                   uint8 mcu_temp;               // 伺服控制板温度
                   uint8 mos_temp;               // mos管温度
                   uint8 motor_temp;             // 电机温度
                   uint8 bus_voltage;            // 母线电压数据
               };
           };
       };
   };

   ```
3. 关节指令:motor\_cmd.idl

   ```shell
   cat motor_cmd.idl
   module dobot_atom{
       module msg{
           module dds_{
               @extensibility(FINAL)
               struct MotorCmd_ {
                   uint8 mode;                        // 模式
                   float q;                           // 位置
                   float dq;                          // 速度
                   float tau;                         // 力矩
                   float kp;                          // 比例增益
                   float kd;                          // 微分增益
               };
           };
       };
   };

   ```
4. imu信息：imu\_state.idl

   ```shell
   cat imu_state.idl
   module dobot_atom{
       module msg{
           module dds_{
               @extensibility(FINAL)
               struct IMUState_ {
                   float quaternion[4];        // 四维四元数
                   float gyroscope[3];         // 三维陀螺仪数据，单位deg/s
                   float accelerometer[3];     // 三维加速度计数据，单位m/s2
                   float rpy[3];               // 三维欧拉角（滚转、俯仰、偏航）,单位deg
                   uint8 temperature;          // 温度传感器数据,单位℃
               };
           };
       };
   };
   ```
5. 下文中提到的wireless\_remote说明：

   对应键按下时有值，松开时，恢复为0。

   ```shell
   wireless_remote[2]，从高位到低位依次对应键：'', '', 'LT', 'RT', 'SELECT', 'START', 'LB', 'RB';
   wireless_remote[3]，从高位到低位依次对应键：'LEFT', 'DOWN', 'RIGHT', 'UP', 'Y', 'X', 'B', 'A';
   wireless_remote[4->7]，存放的是LX键转成[-1,0]之间的浮点值value，wireless_remote[4]是value低位；
   wireless_remote[8->11]，存放的是RX键的值[0,1]之间的浮点值value，wireless_remote[8]是value低位；
   wireless_remote[12->15]，存放的是RY键的值[-1,0]之间的浮点值value，wireless_remote[12]是value低位；
   wireless_remote[16->19]，存放的是LY键的值[0,1]之间的浮点值value，wireless_remote[16]是value低位。
   ```
6. 底盘状态:amr\_state.idl

   1. AMRState\_为整个底盘的主要状态

```shell
module dobot_atom{
    module msg{
        module dds_{
            enum NavigationStatus_ {
                UNKNOWN,            // 未知
                QUEUING,            // 排队
                RUNNING,            // 运行中
                COMPLETED,          // 完成
                FAILED,             // 失败
                PAUSED,             // 暂停
                CANCELED,           // 取消
                WAITING_CONFIRM,    // 等待确认
                IDLE,               // 空闲
                STOPPED             // 停止状态
            };// 枚举仅作navigation_status的数值参考

            enum DeviceStatus_ {
                DEVUNKNOWN,        // 未知
                DEVIDLE,           // 小车空闲
                TASKING,        // 任务中
                ERROR,          // 故障中
                OFFLINE,        // 离线
                INIT,           // 设备初始化
                CHARGING,       // 充电中
                UPGRADE         // 升级中
            }; // 枚举仅作device_status的数值参考
            @extensibility(FINAL)
            struct AMREventStatus_{
                boolean emergency_stop_pressed;             // 急停按钮被按下
                boolean enable_pressed;                     // 使能键被按下
                boolean path_blocked;                        // 路径被挡住
                boolean low_battery;                         // 电池电量过低
                boolean obstacle_detected;                   // 被障碍物挡住
              
            };
          
            @extensibility(FINAL)
            struct AMRBasicStatus_{
                float battery_level;        // 电池电量百分比
                float battery_voltage;      // 电池电压
                float battery_current;      // 电池电流
                uint16 heartbeat;          // 心跳值
            };

            @extensibility(FINAL)
            struct AMRState_ {
                DeviceStatus_ device_status;                         // 设备状态
                NavigationStatus_ navigation_status;                    // 导航状态 参考上述enum，用于判断任务是否完成
                AMRBasicStatus_ basic_status;                // 基础状态
                float position[3];                          // 当前位置 {x,y,yaw}
                AMREventStatus_ amr_event;                   // 底盘相关事件 
                uint32 error_code[32];                        // 错误码
                uint32 task_id;                           // 任务ID
                uint32 work_mode;                         // 机器人工作模式 枚举如右： 0：任务模式 || 3：遥控模式 || 4:检修模式
            };



        };
    };
};
```

## 1 切换底层状态

1. 功能说明：涉及到安全防护的原因，如果需要控制关节，要使用该接口将底层状态设置为非零力矩模式，否则底层不将控制指令下发到关节。
2. 话题名称：rt/set/fsm/id
3. IDL文件内容：

```shell
cat set_fsm_id.idl
module dobot_atom{
    module msg{
        module dds_{
            @extensibility(FINAL)
            struct SetFsmId_ {
                uint16 id;                  // 对算法状态机
            };
        };
    };
};


```

## 2 切换上肢控制

1. 功能说明：避免外部（用户或者遥操作）和算法状态机同时控制上肢，所以需要增加该接口，如果外部包括PC2要对上肢进行控制时，要打开该开关。
2. 话题名称：rt/switch/upper/control
3. IDL文件内容：

```shell
cat switch_upper_control.idl
module dobot_atom{
    module msg{
        module dds_{
            @extensibility(FINAL)
            struct SwitchUpperControl_ {
                boolean flag;  /* true: 上肢有控制，false: 上肢无控制*/
            };
        };
    };
};
```

## 3 上肢状态

1. 功能说明：上肢底层信息。
2. 话题名称：rt/upper/state
3. IDL文件内容：

```shell
cat upper_state.idl

#include "imu_state.idl"
#include "motor_state.idl"
#include "bms_state.idl"

module dobot_atom{
    module msg{
        module dds_{
            @extensibility(FINAL)
            struct SwitchUpperControl_ {
                boolean flag;  /* true: 上肢有控制，false: 上肢无控制*/
            };
        };
    };
};
```

说明：is\_upper\_control表示是否打开了遥操作，true表示打开了遥操作。

## 4 上肢控制

1. 功能说明：上肢控制指令。
2. 话题名称：rt/upper/cmd
3. IDL文件内容：

```shell
cat upper_cmd.idl
#include "motor_cmd.idl"

module dobot_atom{
    module msg{
        module dds_{
            @extensibility(FINAL)
            struct UpperCmd_ {
                MotorCmd_ motor_cmd[17];           // 电机命令，包含 17 个 MotorCmd_ 结构体
            };
        };
    };
};

```

## 5 下肢状态

1. 功能说明：下肢状态信息。
2. 话题名称：rt/lower/state
3. IDL文件内容：

```c++
cat lower_state.idl
#include "imu_state.idl"
#include "motor_state.idl"
#include "bms_state.idl"

module dobot_atom{
    module msg{
        module dds_{
            @extensibility(FINAL)
            struct LowerState_ {
                uint16 fsm_id;          /*对应算法状态机*/
                IMUState_ imu_state;
                MotorState_ motor_state[12];
                BmsState_ bms_state;
                octet wireless_remote[40];
                uint32 reserve;               /*越疆保留*/
            };
        };
    };
};
```

## 6 下肢控制

1. 功能说明：下肢控制指令。
2. 话题名称：rt/lower/cmd
3. IDL文件内容：

```shell
cat lower_cmd.idl
#include "motor_cmd.idl"

struct LowerCmd_ {
    MotorCmd_ motor_cmd[12];           // 电机命令，包含 12 个 MotorCmd_ 结构体
};

```

## 7 关节、can板状态

1. 功能说明：主要是提供给用户辅助调试使用的。
2. 话题名称：rt/main/nodes/state
3. IDL文件内容：

   ```c++
     $ cat main_nodes_state.idl
   module dobot_atom{
       module msg{
           module dds_{
               @extensibility(FINAL)
               struct AxisStateInfo_
               {
                   uint8  servo_state;
                   uint16 error_code;  //电机报警错误码
                   uint16 warn_code;  // 电机警告错误码
                   int32  pos_err_code;
                   int32  vel_err_code;
                   int32  torque_err_code;
                   uint8  node_state;
                   uint8  display_op_mode;
                   boolean is_virtual;   /* default: false */
                   uint8  mcu_temp;
                   uint8  mos_temp;
                   uint8  motor_temp;
                   uint8  bus_voltage;
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

   * node\_state枚举：

     * 0: Initialisation
     * 1: Disconnecting
     * 2: Connecting or Preparing
     * 4: Stopped
     * 5: Operational
     * 7: Pre\_operational
     * 其他: UNKNOWN
   * servo\_state枚举:    

     * 0: disable
     * 1: error
     * 3: enable
     * 其他: UNKNOWN
   * pos\_err\_code: 位置是否超限，超限则有数据
   * vel\_err\_code: 速度超限
   * torque\_err\_code: 扭矩是否超限

## 8 清错

1. 功能说明：仅清除底层的错误。
2. 话题名称：rt/clear/errors
3. IDL文件内容：

```shell
cat clear_errors.idl
struct ClearErrors_ {
    int32 msg_id;
};

```

## 9 灵巧手状态

1. 功能说明：仅灵巧手各手指角度信息。
2. 话题名称：rt/hands/state
3. IDL文件内容：

```shell
cat hands_state.idl
#include "motor_state.idl"

module dobot_atom{
    module msg{
        module dds_{
            @extensibility(FINAL)
            struct ClearErrors_ {
                int32 msg_id;
            };
        };
    };
};


```

## 10 灵巧手控制

1. 功能说明：仅灵巧手各手指角度控制。
2. 话题名称：rt/hands/cmd
3. IDL文件内容：

```shell
cat hands_cmd.idl
#include "motor_cmd.idl"

module dobot_atom{
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

## 11 轮式人形底盘状态

1. 功能说明：主要包含底盘的各个状态信息
2. 话题名称：rt/amr/state
3. IDL文件内容：

```shell
module dobot_atom{
    module msg{
        module dds_{
            enum NavigationStatus_ {
                UNKNOWN,            // 未知
                QUEUING,            // 排队
                RUNNING,            // 运行中
                COMPLETED,          // 完成
                FAILED,             // 失败
                PAUSED,             // 暂停
                CANCELED,           // 取消
                WAITING_CONFIRM,    // 等待确认
                IDLE,               // 空闲
                STOPPED             // 停止状态
            };// 枚举仅作navigation_status的数值参考

            enum DeviceStatus_ {
                DEVUNKNOWN,        // 未知
                DEVIDLE,           // 小车空闲
                TASKING,        // 任务中
                ERROR,          // 故障中
                OFFLINE,        // 离线
                INIT,           // 设备初始化
                CHARGING,       // 充电中
                UPGRADE         // 升级中
            }; // 枚举仅作device_status的数值参考
            @extensibility(FINAL)
            struct AMREventStatus_{
                boolean emergency_stop_pressed;             // 急停按钮被按下
                boolean enable_pressed;                     // 使能键被按下
                boolean path_blocked;                        // 路径被挡住
                boolean low_battery;                         // 电池电量过低
                boolean obstacle_detected;                   // 被障碍物挡住
              
            };
          
            @extensibility(FINAL)
            struct AMRBasicStatus_{
                float battery_level;        // 电池电量百分比
                float battery_voltage;      // 电池电压
                float battery_current;      // 电池电流
                uint16 heartbeat;          // 心跳值
            };

            @extensibility(FINAL)
            struct AMRState_ {
                DeviceStatus_ device_status;                         // 设备状态
                NavigationStatus_ navigation_status;                    // 导航状态 参考上述enum，用于判断任务是否完成
                AMRBasicStatus_ basic_status;                // 基础状态
                float position[3];                          // 当前位置 {x,y,yaw}
                AMREventStatus_ amr_event;                   // 底盘相关事件 
                uint32 error_code[32];                        // 错误码
                uint32 task_id;                           // 任务ID
                uint32 work_mode;                         // 机器人工作模式 枚举如右： 0：任务模式 || 3：遥控模式 || 4:检修模式
            };



        };
    };
};
```

## 12 轮式人形底盘控制

1. 功能说明：主要用于给轮式人形机器人的底盘发送指令
2. 话题名称：rt/amr/cmd
3. IDL文件内容：

   ```shell
   module dobot_atom{
       module msg{
           module dds_{
               enum AMRCommandType {
                   CANCEL_TASK,      // 取消任务
                   PAUSE_TASK,       // 暂停任务
                   RESUME_TASK,      // 恢复任务
                   MOVE_TO_TAG,      // 移动到标记点, target_id必填，theta选填，theta为绝对角度,如果为0或不填则不指定角度
                   REMOTE_CONTROL,    // 遥控
                   ROTATE,            // 原地旋转 theta必填，为相对角度，相对当前底盘的角度 -- 华睿不可用
                   START_REMOTE,      // 开始遥控
                   STOP_REMOTE,       // 关闭遥控
                   SUBSCRIBE_LASER,     // 订阅底盘激光雷达数据
                   UNSUBSCRIBE_LASER,   // 取消订阅底盘激光雷达数据
                   START_MAPPING,       // 底盘开始扫图
                   SAVE_MAP,            // 保存当前扫的图在本地
                   STOP_MAPPING,        // 结束扫图
                   SET_VEL,             // 设置速度（同时设置线速度和角速度，空载和负载）
                   SET_ACCEL,           // 设置加速度（同时设置线性和角度的加速度，空载和负载）
                   SET_DECEL            // 设置减速度（同时设置线性和角度的减速度，空载和负载）
               };

               @extensibility(FINAL)
               struct AMRCommand_ {
                   AMRCommandType command_type;    // 命令类型,参考上述AMRCommandType
                   uint32 target_id;             // 目标ID（用于MOVE_TO_TAG）
                   float linear_vel;             // 线速度 ,m/s
                   float angular_vel;            //角速度  ,rad/s
                   uint32 command_id;            // 命令ID（用于跟踪）
                   uint64 timestamp;             // 时间戳
                   float theta;                //传入角度
               };
           };

       };
   };

   ```

   1. **参考上述amr\_cmd.idl，目前包含六个功能，分别是：**

      1. 取消任务
      2. 暂停任务
      3. 继续任务
      4. 导航到标记点(需要传入task\_id和theta，theta为绝对角度)
      5. 遥控（需要传入linear\_vel\[m/s\]和angular\_vel\[rad/s\]）,遥控前需要发送START\_REMOTE请求
      6. 原地旋转(需要传入theta)，这个旋转是相对与当前角度的旋转 -- 华睿不提供接口，不可用
      7. 开始遥控
      8. 停止遥控
      9. 订阅激光雷达
      10. 取消订阅激光雷达
      11. 开始建图
      12. 结束建图
      13. 设置底盘速度
      14. 设置底盘加速度
      15. 设置底盘减速度
   2. **对于遥控功能**

      1. 开始遥控前，要发送START\_REMOTE请求,++下发后，可看到底盘灯变为绿灯频闪，此时需要按一下确认键++
      2. 然后通过REMOTE\_CONTROL请求，根据传入的linear\_vel和angular\_vel下发遥控请求给底盘，不需要频繁发送，内部有一个50hz的线程在根据传入的数值下发遥控指令
      3. 不再需要遥控时请发送STOP\_REMOTE请求，否则底盘无法接收新任务
   3. **对于导航到标记点：**

      1. 必填参数

         1. task\_id
         2. theta（不填的话会默认转到0度）
      2. 如何判断当前任务已完成？ 

         1. 订阅amr\_state，当任务开始时，navigation\_status会变为2（RUNNING），当任务完成时，navigataion\_status会变成3（COMPLETED）
      3. 如何判断任务已经下发成功？

         1. 订阅amr\_state，当amr\_state返回device\_status变成了2（TASKING），表示任务已下发，底盘开始运动了
      4. 是否可以在上一次任务未完成时就下发下一次任务？

         1. 不建议，这样不会取消掉上一次任务，反之，会完成上一次任务之后，再继续执行下一次任务
         2. 如果有提前执行下一个任务的需求，建议先发送取消任务，再发送执行下一次任务的请求

## 13 摇杆控制

1. 功能说明：主要用于将app发送的摇杆的数据，转发给到算法使用
2. 话题名称：rt/remote/control
3. IDL文件内容：

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
                   JoystickValue_ btn_move;    // 左旋钮键值
                   JoystickValue_ btn_turn;    // 右旋钮键值
               };
           };
       };
   };


   ```
4. 如上，其中，一个摇杆有两个旋钮分别是左旋钮和右旋钮，分别存储在btn\_move和btn\_turn，
5. 以左旋钮为例，当旋钮往右拨动时，x值增加，反之则减，当往上拨动时，y值增加，反之则减
6. JoystickValue\_阈值区间： \[-1,1\]

## 14 急停状态

1. 功能说明：整个嵌入式系统中的急停信息。
2. 话题名称：rt/emergency/state
3. IDL文件内容：

```shell
module dobot_atom{
    module msg{
        module dds_{
            @extensibility(FINAL)
            struct EmergencyState_ {
                boolean soft_emergency_triggered;   // 软紧急停止 (app触发)
                boolean hard_emergency_triggered;   // 硬紧急停止 (用户板触发)
                boolean amr_emergency_triggered;    // AMR紧急停止 (轮式底盘触发)
            };
        };
    };
};

```

## 15 状态机状态

1. 功能说明：由算法发布的状态机的信息
2. 话题名称：rt/set/fsm/id
3. IDL文件内容：

   * id : 当前算法状态机装固态
   * current\_action: string，当用户通过app或者摇杆执行HandsUp时，算法这个字段设为“HandsUp",当动作执行完，current\_action置为空

```shell
module dobot_atom{
    module msg{
        module dds_{
            @extensibility(FINAL)
            struct SetFsmId_ {
                uint16 id;         // 对算法状态机
                char current_action; // 算法当前正在执行的动作，空表示没有动作
            };
        };
    };
};

```
