# Atom Gazebo 仿真包

本包为 Dobot Atom 人形机器人提供完整的 Gazebo 仿真环境支持，包括机器人模型加载、控制器配置和仿真环境设置。

## 🚀 功能特性

- **完整仿真环境**: 提供 Gazebo 中的完整机器人仿真
- **ROS2 Control 集成**: 支持标准化的机器人控制接口
- **双臂控制**: 独立的左右手臂轨迹规划和执行
- **灵活配置**: 支持全身和仅手臂的仿真模式

## 📁 文件结构

```
atom_gazebo/
├── CMakeLists.txt              # 构建配置文件
├── package.xml                 # 包依赖声明
├── launch/                     # 启动文件目录
│   ├── atom_gazebo.launch.py   # 完整仿真启动文件
│   └── atom_arms_only.launch.py # 仅手臂仿真启动文件
├── config/                     # 配置文件目录
│   └── atom_moveit.rviz        # MoveIt RViz 配置
└── scripts/                    # 控制脚本目录
    └── control_arms.py         # 双臂控制脚本
```

## 🎯 快速开始

### 1. 启动仿真

#### 完整机器人仿真

```bash
ros2 launch atom_gazebo atom_gazebo.launch.py
```

#### 仅手臂仿真

```bash
ros2 launch atom_gazebo atom_arms_only.launch.py
```

## 🔧 启动文件说明

### atom_gazebo.launch.py

完整的 Gazebo 仿真启动文件，包含：

- **参数配置**:

  - `use_sim_time`: 使用仿真时间（默认: true）
  - `gui`: 是否显示 Gazebo GUI（默认: true）
  - `headless`: 无头模式运行（默认: false）
- **功能模块**:

  - Gazebo 世界环境加载
  - 机器人 URDF 模型加载
  - Robot State Publisher 启动
  - ROS2 Control 控制器管理

### atom_arms_only.launch.py

仅手臂部分的仿真启动文件，适用于：

- 手臂控制算法开发
- 轻量级仿真测试
- 运动规划验证

## 🎮 控制脚本

### control_arms.py

双臂控制脚本，提供以下功能：

#### 主要特性

- **双臂独立控制**: 支持左右手臂独立轨迹规划
- **轨迹插值**: 自动进行平滑轨迹插值
- **状态监控**: 实时监控手臂执行状态
- **错误处理**: 完善的错误检测和恢复机制

#### 使用方法

```bash
# 启动仿真后，运行控制脚本
ros2 run atom_gazebo control_arms.py
```

#### 控制接口

脚本通过以下 Action 接口控制手臂：

- `/left_arm_controller/follow_joint_trajectory`
- `/right_arm_controller/follow_joint_trajectory`

#### 预定义动作

脚本包含以下预定义动作序列：

1. **上升动作**: 手臂向上运动
2. **下降动作**: 手臂向下运动
3. **复位动作**: 返回初始位置

## ⚙️ 配置文件

### atom_moveit.rviz

MoveIt 集成的 RViz 配置文件，包含：

- **显示配置**:

  - 机器人模型显示
  - 运动规划可视化
  - 轨迹显示
  - 碰撞检测可视化
- **交互工具**:

  - 交互式标记器
  - 运动规划面板
  - 场景规划工具

## 💡 使用示例

### 示例 1: 基本仿真启动

```bash
# 终端 1: 启动 Gazebo 仿真
ros2 launch atom_gazebo atom_gazebo.launch.py

# 终端 2: 启动 RViz 可视化
ros2 launch dobot_atom_rviz atom_rviz.launch.py

# 终端 3: 运行控制脚本
ros2 run atom_gazebo control_arms.py
```

### 示例 2: 无头模式仿真

```bash
# 无 GUI 模式启动，适用于服务器环境
ros2 launch atom_gazebo atom_gazebo.launch.py gui:=false headless:=true
```

### 示例 3: 手臂控制测试

```bash
# 启动仅手臂仿真
ros2 launch atom_gazebo atom_arms_only.launch.py

# 检查控制器状态
ros2 control list_controllers

# 手动发送轨迹命令
ros2 topic pub /left_arm_controller/follow_joint_trajectory/goal ...
```

## 🔍 话题和服务

### 主要话题

- `/joint_states`: 关节状态信息
- `/robot_description`: 机器人模型描述
- `/left_arm_controller/follow_joint_trajectory/goal`: 左臂轨迹目标
- `/right_arm_controller/follow_joint_trajectory/goal`: 右臂轨迹目标

### 控制器服务

- `/controller_manager/list_controllers`: 列出所有控制器
- `/controller_manager/load_controller`: 加载控制器
- `/controller_manager/switch_controller`: 切换控制器状态

## 🐛 故障排除

### 常见问题

#### 1. Gazebo 启动失败

```bash
# 检查 Gazebo 版本
gazebo --version

# 重新安装 Gazebo
sudo apt install --reinstall gazebo
```

#### 2. 控制器加载失败

```bash
# 检查控制器状态
ros2 control list_controllers

# 手动加载控制器
ros2 control load_controller joint_state_broadcaster
ros2 control set_controller_state joint_state_broadcaster active
```

#### 3. 机器人模型不显示

```bash
# 检查 robot_description 话题
ros2 topic echo /robot_description

# 检查 TF 树
ros2 run tf2_tools view_frames
```

#### 4. 轨迹执行失败

```bash
# 检查 Action 服务器状态
ros2 action list

# 查看错误日志
ros2 topic echo /left_arm_controller/follow_joint_trajectory/feedback
```

### 性能优化

#### 1. Gazebo 性能优化

- 关闭不必要的插件
- 降低物理引擎更新频率
- 使用简化的碰撞模型

#### 2. 控制器优化

- 调整轨迹插值参数
- 优化控制器更新频率
- 使用适当的 PID 参数

## 📚 扩展开发

### 添加新的控制器

1. 在 `atom_urdf/config/atom_controllers.yaml` 中定义控制器
2. 修改启动文件加载新控制器
3. 编写相应的控制脚本

### 自定义仿真环境

1. 创建新的世界文件（.world）
2. 修改启动文件中的世界文件路径
3. 添加环境物体和传感器

**维护者**: Dobot_futingxing
**最后更新**: 2025-09-02
