# Dobot Atom 机器人 ROS2 支持包

本项目为 Dobot Atom 人形机器人提供完整的 ROS2 支持，包括机器人模型、仿真环境、控制接口和示例程序。

## 📋 目录

- [环境配置](#环境配置)
- [安装说明](#安装说明)
- [快速开始](#快速开始)
- [包说明](#包说明)
- [使用示例](#使用示例)
- [故障排除](#故障排除)

## 🚀 环境配置

### 系统要求

系统和 ROS2 版本：

| 系统         | ROS2 版本 |
| ------------ | --------- |
| Ubuntu 20.04 | Foxy      |
| Ubuntu 22.04 | Humble    |

### 依赖安装

#### 1. 安装 ROS2

注意：本安装方法仅供参考如安装报错，请自行搜索教程安装。

以 ROS2 Humble 为例（推荐）：

```bash
# 添加 ROS2 apt 仓库
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装 ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop

#设置环境变量
source /opt/ros/humble/setup.bash
echo " source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

#### 2. 安装依赖

```bash
# ROS2 控制相关
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt install ros-humble-gazebo-*
sudo apt-get install ros-humble-moveit

#设置环境变量
echo "source /usr/share/gazebo/setup.bash" >> ~/.bashrc

# URDF 和可视化工具
sudo apt install ros-humble-urdf ros-humble-xacro
sudo apt install ros-humble-robot-state-publisher ros-humble-joint-state-publisher
sudo apt install ros-humble-joint-state-publisher-gui

# 构建工具
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep python3-vcstool

# DDS 实现（可选，用于更好的性能）
sudo apt install ros-humble-rmw-cyclonedds-cpp

```

## 📦 安装说明

### 1. 克隆仓库

```bash
# 创建工作空间
mkdir -p ~/atom_ros2_ws/src
cd ~/atom_ros2_ws/src

# 克隆本仓库
git clone <your-repository-url> atom_ros2
```

### 2. 安装依赖

```bash
cd ~/atom_ros2_ws

# 初始化 rosdep（如果是第一次使用）
sudo rosdep init
rosdep update

# 安装包依赖
rosdep install --from-paths src --ignore-src -r -y
```

### 3. 编译工作空间

```bash
cd ~/atom_ros2_ws
colcon build

# 设置环境变量
source install/setup.bash
```

### 4. 配置环境

```bash
# 复制配置文件到用户目录
cp src/atom_ros2/setup*.sh ~/

# 根据需要选择配置文件：
# 1. 连接真实机器人
source ~/setup.sh

# 2. 本地仿真环境
source ~/setup_local.sh

# 3. 默认配置
source ~/setup_default.sh

#如需永久生效可写入环境变量中
sudo gedit ~/.bashrc
```

## 🎯 快速开始

### 1. 启动 Gazebo 仿真

```bash
# 设置环境
source ~/atom_ros2_ws/install/setup.bash
source ~/setup_local.sh

# 启动 Gazebo 仿真
ros2 launch atom_gazebo atom_gazebo.launch.py
```

### 2. 启动 RViz 可视化

```bash
# 新终端
source ~/atom_ros2_ws/install/setup.bash
ros2 launch atom_urdf display.launch.py
```

### 3. 测试招手动作

```bash
# 新终端
source ~/atom_ros2_ws/install/setup.bash
ros2 run atom_gazebo atom_wave_controller.py

# 或运行测试脚本
ros2 run atom_gazebo test_wave.py
```

## 📁 包说明

### atom_urdf

- **功能**: Atom 机器人的 URDF/XACRO 模型定义
- **内容**:
  - `urdf/atom.urdf`: 主要机器人模型文件urdf
  - `urdf/atom.xacro`: 主要机器人模型文件xacro
  - `urdf/atom_gazebo.xacro`: Gazebo 仿真配置

### atom_gazebo

- **功能**: Gazebo 仿真支持
- **内容**:
  - `launch/atom_gazebo.launch.py`: Gazebo 仿真启动文件
  - `config/atom_controllers.yaml`: 控制器配置
  - `scripts/atom_wave_controller.py`: 招手动作控制器
  - `scripts/test_wave.py`: 招手测试脚本

### dobot_atom

- **功能**: Atom 机器人消息定义
- **内容**:
  - `msg/`: 自定义消息类型
  - 机器人状态和控制消息定义

    ![interface](/image/interface.jpg)

### dobot_atom_rviz

- **功能**: RViz 可视化配置
- **内容**:
  - `launch/dobot_rviz.launch.py`: RViz 启动文件
  - `rviz/`: RViz 配置文件

    ![rviz](/image/rviz.jpg)

### atom_control_examples

- **功能**: 控制示例程序(详细说明见atom_control_examples/README.md)
- **内容**:
  - 各种控制算法示例
  - 运动规划示例
  - 传感器数据处理示例

## 🎮 使用示例

### 仿真关节控制

```bash
# 控制手臂关节
ros2 topic pub /arm_position_controller/commands std_msgs/msg/Float64MultiArray \
  "data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.5, 0.0, 1.5, 0.0, 0.0]"

# 控制腿部关节
ros2 topic pub /leg_position_controller/commands std_msgs/msg/Float64MultiArray \
  "data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"

# 控制头部关节
ros2 topic pub /head_position_controller/commands std_msgs/msg/Float64MultiArray \
  "data: [0.0, 0.0]"
```

### 状态监控

```bash
# 查看关节状态
ros2 topic echo /joint_states

# 查看可用控制器
ros2 control list_controllers

# 查看话题列表
ros2 topic list
```

### 自定义动作

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class CustomMotionController(Node):
    def __init__(self):
        super().__init__('custom_motion_controller')
        self.arm_pub = self.create_publisher(
            Float64MultiArray, 
            '/arm_position_controller/commands', 
            10
        )
  
    def move_arms(self, positions):
        msg = Float64MultiArray()
        msg.data = positions
        self.arm_pub.publish(msg)

def main():
    rclpy.init()
    controller = CustomMotionController()
  
    # 自定义手臂位置
    arm_positions = [0.0] * 14  # 14个手臂关节
    arm_positions[8] = -1.0     # 右肩抬起
    arm_positions[11] = 1.5     # 右肘弯曲
  
    controller.move_arms(arm_positions)
    rclpy.spin_once(controller)
  
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 🔧 配置说明

### 网络配置

如果连接真实机器人，需要配置网络接口：

   1.设置网段至192.168.8.xx：
   ![rviz](/image/IP.jpg)

2. 查看网络接口：

```bash
ifconfig
# 或
ip addr show
```

3. 修改 `setup.sh` 中的网络接口名称：

```bash
# 编辑配置文件
gedit ~/setup.sh

# 修改这一行中的 "eth0" 为实际的网络接口名
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
    <NetworkInterface name="eth0" priority="default" multicast="default" />
</Interfaces></General></Domain></CycloneDDS>'
```

### 连接测试

完成上面的配置后，先重启一下 `ros2 daemon`： `ros2 daemon stop` 然后执行 `ros2 daemon start`

```bash
source ~/setup.sh
ros2 topic list
```

可以看见如下话题：

![topic](/image/topic.jpg)

打开终端随意查看一个话题：ros2 topic echo /xxxx 有数据说明通讯正常例如：

![topic_info.](/image/topic_info.jpg)

### 虚拟控制配置

配置文件位于 `atom_gazebo/config/atom_controllers.yaml`，包含：

- `joint_state_broadcaster`: 关节状态广播器
- `arm_position_controller`: 手臂位置控制器
- `leg_position_controller`: 腿部位置控制器
- `head_position_controller`: 头部位置控制器

## 🐛 故障排除

### 常见问题

#### 1. Gazebo 启动失败

```bash
# 检查 Gazebo 是否正确安装
gazebo --version

# 重新安装 Gazebo
sudo apt install gazebo
```

#### 2. 控制器加载失败

```bash
# 检查控制器状态
ros2 control list_controllers

# 手动加载控制器
ros2 control load_controller joint_state_broadcaster
ros2 control set_controller_state joint_state_broadcaster active
```

#### 3. 网络连接问题

```bash
# 检查 DDS 发现
ros2 daemon stop
ros2 daemon start

# 检查话题
ros2 topic list
```

#### 4. 编译错误

```bash
# 清理并重新编译
cd ~/atom_ros2_ws
rm -rf build install log
colcon build
```

### 性能优化

#### 1. Gazebo 性能

- 关闭不必要的 GUI 面板
- 降低物理引擎更新频率
- 使用无头模式：`gui:=false`

#### 2. DDS 优化

- 配置合适的网络接口
- 调整 DDS 域 ID

## 📚 参考资料

- [ROS2 官方文档](https://docs.ros.org/en/humble/)
- [Gazebo 仿真教程](http://gazebosim.org/tutorials)
- [ros2_control 文档](https://control.ros.org/)
- [URDF 教程](http://wiki.ros.org/urdf/Tutorials)

## 📄 许可证

本项目采用 Apache License 2.0 许可证。

## 📞 支持

如有问题，请：

1. 查看本 README 的故障排除部分
2. 搜索已有的 Issues
3. 创建新的 Issue 并提供详细信息

**更新日期**: 2025-9-2
**维护**: dobot_futingxing

---

**注意**: 请根据实际的机器人硬件接口和通信协议调整配置文件和示例代码。
