# Atom Control Examples

这是一个用于 Dobot Atom 机器人的 ROS2 控制示例包，提供了状态读取和基础控制功能。

## 功能特性

### 状态读取程序

本包提供了四个独立的状态读取程序，分别用于监控机器人的不同部分：

1. **下肢状态读取** (`lower_state_reader`)

   - 订阅话题：`/lower/state`
   - 功能：读取下肢12个电机状态、IMU数据、BMS电池信息
   - 包含：腿部关节位置、速度、力矩、温度等详细信息
2. **上肢状态读取** (`upper_state_reader`)

   - 订阅话题：`/upper/state`
   - 功能：读取上肢17个电机状态（机械臂+头部+躯干）、BMS电池信息
   - 包含：机械臂、头部和躯干关节的详细状态信息
3. **关节CAN板状态读取** (`main_nodes_state_reader`)

   - 订阅话题：`/main/nodes/state`
   - 功能：读取关节CAN板状态和EtherCAT从站信息
   - 包含：左右腿、躯干、左右臂、头部的伺服状态、错误码、警告码等
4. **灵巧手状态读取** (`hands_state_reader`)

   - 订阅话题：`/hands/state`
   - 功能：读取灵巧手12个手指电机状态
   - 包含：左右手各6个自由度的位置、速度、力矩、温度信息

### 控制程序

6. **全身轨迹跟踪控制器** (`atom_full_body_trajectory_controller`)

   - 订阅话题：`/lower/state`, `/upper/state`, `/hands/state`
   - 发布话题：`/lower/cmd`, `/upper/cmd`, `/hands/cmd`, `/set/fsm/id`
   - 功能：全身关节轨迹跟踪控制（腿部12关节+手臂17关节+灵巧手12关节）
   - 特性：
     - 内置正弦波轨迹生成
     - 平滑初始化过程（5秒）
     - 可配置PID参数
     - 实时轨迹跟踪（10ms控制周期）
     - 循环播放轨迹
7. **关节数据记录器** (`atom_joint_recorder`)

   - 订阅话题：`/lower/state`, `/upper/state`, `/hands/state`
   - 功能：记录机器人所有关节角度数据到文件
   - 特性：
     - 可配置记录参数（持续时间、频率、延迟）
     - 完整关节数据记录（41个关节）
     - 带时间戳的数据格式
     - 详细的文件头信息
     - 实时进度显示
8. **灵巧手控制程序** (`atom_dexterous_hands_controller`)

   - 发布话题：`/hands/cmd`
   - 功能：控制机器人灵巧手进行抓握和松开动作
   - 特性：
     - 双手同步控制（左右手各6个自由度）
     - 平滑运动轨迹（缓动函数优化）
     - 智能时序控制（拇指延迟/提前动作）
     - 自动状态切换（抓握 ↔ 松开，5秒间隔）
     - 多线程安全设计
     - 符合手指角度限制范围

## 编译和运行

### 依赖项

确保以下依赖项已安装：

- ROS2 (Humble)
- rclcpp
- dobot_atom (消息包)
- std_msgs
- geometry_msgs

### 编译

```bash
colcon build 
source install/setup.bash
```

### 运行状态读取程序

```bash
# 下肢状态读取
ros2 run atom_control_examples lower_state_reader

# 上肢状态读取
ros2 run atom_control_examples upper_state_reader

# 关节CAN板状态读取
ros2 run atom_control_examples main_nodes_state_reader

# 灵巧手状态读取
ros2 run atom_control_examples hands_state_reader
```

### 运行控制程序

```bash
# 灵巧手控制程序
ros2 run atom_control_examples atom_dexterous_hands_controller
```

## 话题说明

### 状态话题（订阅）

- `/lower/state` - 下肢状态信息
- `/upper/state` - 上肢状态信息
- `/main/nodes/state` - 关节CAN板状态
- `/hands/state` - 灵巧手状态
- `/fsm/status` - 状态机状态

### 控制话题（发布）

- `/lower/cmd` - 下肢控制命令
- `/upper/cmd` - 上肢控制命令
- `/hands/cmd` - 灵巧手控制命令
- `/set/fsm/id` - 设置FSM状态ID
- `/switch/upper/control` - 切换上肢控制权
- `/cmd_vel` - 速度控制命令

**注意**：ROS2中话题名称不包含 `rt` 前缀，这与原始DDS接口有所不同。

## 安全注意事项

⚠️ **重要安全提醒**：

1. 运行控制程序前，确保机器人处于安全环境
2. 随时准备按下急停按钮
3. 首次运行时建议降低控制参数
4. 确保机器人周围有足够的活动空间
5. 运行前检查机器人硬件状态是否正常

## 关节信息

### 机器人关节布局

- **下肢关节**：12个关节（左右腿各6个）
- **上肢关节**：17个关节（左右臂各7个+头部2个+躯干1个）
- **灵巧手关节**：12个关节（左右手各6个）

### 关节限位

详细的关节限位信息请参考 `Atom关节顺序名称与关节限位.md` 文件。

## 程序参数

### 轨迹控制参数

- `trajectory_amplitude`: 轨迹幅度
- `trajectory_frequency`: 轨迹频率
- `control_frequency`: 控制频率
- `leg_kp`, `leg_kd`: 腿部PID参数
- `arm_kp`, `arm_kd`: 手臂PID参数

### 记录器参数

- `record_duration`: 记录持续时间
- `record_frequency`: 记录频率
- `start_delay`: 开始延迟
- `output_file`: 输出文件路径

### 灵巧手控制参数

- `STATE_SWITCH_INTERVAL`: 状态切换间隔（秒）
- `MOTION_STEPS`: 动作总步数
- `STEP_DELAY_MS`: 步间延迟（毫秒）
- `THUMB_DELAY_STEPS`: 抓握时拇指延迟步数
- `THUMB_LEAD_STEPS`: 松开时拇指提前完成步数
- `CONTROL_FREQUENCY`: 控制频率（Hz）

### 信息打印控制

每个程序都有对应的信息打印开关：

- `INFO_LOWER_STATE`: 下肢状态信息打印
- `INFO_UPPER_STATE`: 上肢状态信息打印
- `INFO_MAIN_NODES`: 关节CAN板信息打印
- `INFO_HANDS_STATE`: 灵巧手信息打印

## 故障排除

### 常见问题

1. **话题连接失败**

   - 检查 `dobot_atom` 包是否正确安装
   - 确认机器人硬件连接正常
   - 检查话题名称是否正确（不包含 `rt` 前缀）
2. **编译错误**

   - 确保所有依赖项已安装
   - 检查 ROS2 环境是否正确设置
   - 确认 `dobot_atom` 消息包已编译
3. **控制无响应**

   - 检查机器人是否处于正确的控制模式
   - 确认上肢控制权已正确切换
   - 检查FSM状态是否正确设置

### 调试命令

```bash
# 查看可用话题
ros2 topic list

# 查看话题信息
ros2 topic info /upper/state

# 监听话题数据
ros2 topic echo /upper/state

```

## 扩展开发

### 添加新的控制模式

1. 在相应的控制器文件中添加新的控制逻辑
2. 修改控制回调函数
3. 添加相应的参数配置
4. 更新CMakeLists.txt文件

### 创建新的状态读取程序

1. 参考现有的状态读取程序结构
2. 订阅相应的状态话题
3. 实现状态信息的解析和显示
4. 在 `CMakeLists.txt` 中添加新的可执行文件
