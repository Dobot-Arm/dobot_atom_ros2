#!/usr/bin/env python3
"""
简单的手臂控制脚本
提供基本的左右手臂控制指令
"""
# 作者：Futingxing
# 日期：2025-09-03
# 版本：1.0.0
# 版权：Copyright (c) 2025 Dobot. All rights reserved.

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
import sys

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        
        # 左臂Action客户端
        self.left_arm_client = ActionClient(
            self, FollowJointTrajectory, '/left_arm_controller/follow_joint_trajectory'
        )
        
        # 右臂Action客户端
        self.right_arm_client = ActionClient(
            self, FollowJointTrajectory, '/right_arm_controller/follow_joint_trajectory'
        )
        
        # 左臂关节名称（按照xacro文件中的定义顺序）
        self.left_arm_joints = [
            'left_shoulder_pitch_joint',
            'left_shoulder_roll_joint',
            'left_shoulder_yaw_joint',
            'left_elbow_pitch_joint',
            'left_elbow_roll_joint',
            'left_wrist_pitch_joint',
            'left_wrist_yaw_joint'
        ]
        
        # 右臂关节名称（按照xacro文件中的定义顺序）
        self.right_arm_joints = [
            'right_shoulder_pitch_joint',
            'right_shoulder_roll_joint',
            'right_shoulder_yaw_joint',
            'right_elbow_pitch_joint',
            'right_elbow_roll_joint',
            'right_wrist_pitch_joint',
            'right_wrist_yaw_joint'
        ]
        
        self.get_logger().info("🤖 手臂控制器已启动")
        
    def wait_for_servers(self):
        """等待Action服务器"""
        print("⏳ 等待左臂控制器 Waiting for left arm controller...")
        if not self.left_arm_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("❌ 左臂控制器不可用 Left arm controller unavailable")
            return False
            
        print("⏳ 等待右臂控制器 Waiting for right arm controller...")
        if not self.right_arm_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("❌ 右臂控制器不可用 Right arm controller unavailable")
            return False
            
        print("✅ 所有控制器已就绪 All controllers ready")
        return True
        
    def move_arm(self, arm, positions, duration=3.0):
        """移动手臂到指定位置"""
        if arm == 'left':
            client = self.left_arm_client
            joints = self.left_arm_joints
            arm_name = "左臂"
        elif arm == 'right':
            client = self.right_arm_client
            joints = self.right_arm_joints
            arm_name = "右臂"
        else:
            self.get_logger().error("❌ 无效的手臂选择 Invalid arm selection, please use 'left' or 'right'")
            return False
            
        if len(positions) != len(joints):
            self.get_logger().error(f"❌ 位置数量({len(positions)})与关节数量({len(joints)})不匹配 Position count mismatch with joint count")
            return False
            
        # 创建轨迹
        trajectory = JointTrajectory()
        trajectory.joint_names = joints
        
        # 起始点（当前位置）- 不设置起始点，让控制器从当前位置开始
        # 直接从第一个中间点开始，让控制器自动从当前位置插值
        
        # 中间点（25%进度）
        point2 = JointTrajectoryPoint()
        point2.positions = [pos * 0.25 for pos in positions]
        point2.velocities = [0.0] * len(joints)
        point2.time_from_start.sec = int(duration * 0.25)
        point2.time_from_start.nanosec = int((duration * 0.25 - int(duration * 0.25)) * 1e9)
        
        # 中间点（50%进度）
        point3 = JointTrajectoryPoint()
        point3.positions = [pos * 0.5 for pos in positions]
        point3.velocities = [0.0] * len(joints)
        point3.time_from_start.sec = int(duration * 0.5)
        point3.time_from_start.nanosec = int((duration * 0.5 - int(duration * 0.5)) * 1e9)
        
        # 中间点（75%进度）
        point4 = JointTrajectoryPoint()
        point4.positions = [pos * 0.75 for pos in positions]
        point4.velocities = [0.0] * len(joints)
        point4.time_from_start.sec = int(duration * 0.75)
        point4.time_from_start.nanosec = int((duration * 0.75 - int(duration * 0.75)) * 1e9)
        
        # 目标点（100%进度）
        point5 = JointTrajectoryPoint()
        point5.positions = positions
        point5.velocities = [0.0] * len(joints)
        point5.time_from_start.sec = int(duration)
        point5.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        
        trajectory.points = [point2, point3, point4, point5]
        
        # 发送目标
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory
        
        print(f"🎯 发送{arm_name}运动目标 Sending {arm_name} motion goal...")
        future = client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"❌ {arm_name}目标被拒绝 {arm_name} goal rejected")
            return False
            
        print(f"✅ {arm_name}目标已接受，等待执行完成 {arm_name} goal accepted, waiting for completion...")
        
        # 等待执行完成
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        if result.result.error_code == 0:
            print(f"🎉 {arm_name}运动完成 {arm_name} motion completed!")
            return True
        else:
            self.get_logger().error(f"❌ {arm_name}运动失败 {arm_name} motion failed, error code: {result.result.error_code}")
            return False
            
    def reset_arms(self):
        """重置双臂到初始位置（根据图片中的零点姿态）"""
        print("🔄 重置双臂到初始位置 Resetting both arms to initial position...")
        # 根据图片调整的零点姿态：双臂自然下垂
        # [shoulder_pitch, shoulder_roll, shoulder_yaw, elbow_pitch, elbow_roll, wrist_pitch, wrist_yaw]
        zero_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # 缓慢重置双臂
        left_success = self.move_arm('left', zero_positions, 4.0)  # 增加到4秒，更缓慢
        time.sleep(0.1)
        right_success = self.move_arm('right', zero_positions, 4.0)
        
        if left_success and right_success:
            print("✅ 双臂重置完成 Both arms reset completed")
        else:
            print("❌ 双臂重置失败 Both arms reset failed")
            
    def wave_hand(self, arm, cycles=3):
        """单臂展开动作 - 手臂展开再回来"""
        arm_name = "左手" if arm == 'left' else "右手"
        print(f"🤲 {arm_name}开始展开动作 {arm_name} starting expand motion...")
        
        # 展开动作循环
        for i in range(cycles):
            print(f"🤲 {arm_name}展开 {arm_name} expanding {i+1}/{cycles}")
            
            # 展开姿态：手臂向侧面展开
            if arm == 'left':
                # 左手展开：肩膀pitch 90度，roll外展，肘部pitch 90度伸直
                # [shoulder_pitch, shoulder_roll, shoulder_yaw, elbow_pitch, elbow_roll, wrist_pitch, wrist_yaw]
                expand_position = [0.0, 1.57, 0.0, 1.57, 0.0, 0.0, 0.0]
            else:
                # 右手展开：肩膀pitch 90度，roll外展，肘部pitch 90度伸直
                expand_position = [0.0, -1.57, 0.0, 1.57, 0.0, 0.0, 0.0]
            
            # 展开动作
            print(f"📍 {arm_name}展开 {arm_name} expanding...")
            self.move_arm(arm, expand_position, 2.0)
            time.sleep(1.0)
            
            # 缓慢回到零点位置
            print(f"🔄 {arm_name}缓慢回到初始位置 {arm_name} slowly returning to initial position...")
            zero_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.move_arm(arm, zero_position, 4.0)  # 增加到4秒，更缓慢放下
            time.sleep(1.5)  # 增加等待时间
        
        print(f"✅ {arm_name}展开动作完成 {arm_name} expand motion completed")
        
    def lower_arm_slowly(self, arm):
        """缓慢放下手臂"""
        arm_name = "左手" if arm == 'left' else "右手"
        print(f"⬇️ {arm_name}缓慢放下 {arm_name} slowly lowering...")
        
        # 缓慢放下到零点位置
        # 顺序：shoulder_pitch, shoulder_roll, shoulder_yaw, elbow_pitch, elbow_roll, wrist_pitch, wrist_yaw
        zero_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.move_arm(arm, zero_positions, 5.0)  # 5秒缓慢放下
        
        print(f"✅ {arm_name}已放下 {arm_name} lowered")
            
    def demo_movements(self):
        """演示动作 - 双臂展开演示"""
        print("\n🎭 开始展开动作演示 Starting expand motion demo...")
        
        # 1. 左手展开
        print("\n1️⃣ 左手展开 Left hand expanding")
        self.wave_hand('left', cycles=2)
        time.sleep(1)
        
        # 2. 右手展开
        print("\n2️⃣ 右手展开 Right hand expanding")
        self.wave_hand('right', cycles=2)
        time.sleep(1)
        
        # 3. 最终重置确保回到零点
        print("\n3️⃣ 最终重置 Final reset")
        self.reset_arms()
        
        print("\n🎉 展开动作演示完成 Expand motion demo completed!")
        
    def wave_both_hands_alternately(self):
        """双手交替展开"""
        print("\n🤲🤲 双手交替展开演示 Alternating hands expand demo...")
        
        for round_num in range(2):  # 进行2轮交替
            print(f"\n🔄 第 {round_num + 1} 轮交替展开 Round {round_num + 1} alternating expand")
            
            # 左手展开
            print("🤲 左手展开 Left hand expanding")
            self.wave_hand('left', cycles=1)
            time.sleep(0.5)
            
            # 右手展开
            print("🤲 右手展开 Right hand expanding")
            self.wave_hand('right', cycles=1)
            time.sleep(0.5)
        
        # 最终重置
        print("\n🔄 最终重置 Final reset")
        self.reset_arms()
        print("\n🎉 交替展开演示完成 Alternating expand demo completed!")

def print_usage():
    """打印使用说明"""
    print("\n" + "="*60)
    print("🤖 Atom机器人手臂控制器 - 双臂控制 Atom Robot Arm Controller - Dual Arm Control")
    print("="*60)
    print("使用方法 Usage:")
    print("  python3 control_arms.py demo        # 运行展开演示（左右手依次）Run expand demo (left and right hands sequentially)")
    print("  python3 control_arms.py alternate   # 双手交替展开演示 Alternating hands expand demo")
    print("  python3 control_arms.py reset       # 重置双臂位置 Reset both arms position")
    print("  python3 control_arms.py wave_left   # 左手展开 Left hand expand")
    print("  python3 control_arms.py wave_right  # 右手展开 Right hand expand")
    print("  python3 control_arms.py lower_left  # 左手缓慢放下 Left hand slowly lower")
    print("  python3 control_arms.py lower_right # 右手缓慢放下 Right hand slowly lower")
    print("\n注意 Note: 请确保已启动 atom_arms_only.launch.py Please ensure atom_arms_only.launch.py is started")
    print("="*60)

def main():
    if len(sys.argv) < 2:
        print_usage()
        return
        
    command = sys.argv[1].lower()
    
    rclpy.init()
    controller = ArmController()
    
    try:
        if not controller.wait_for_servers():
            print("❌ 控制器服务不可用，请检查启动文件是否正确运行 Controller service unavailable, please check if launch file is running correctly")
            return
            
        if command == 'demo':
            controller.demo_movements()
        elif command == 'alternate':
            controller.wave_both_hands_alternately()
        elif command == 'reset':
            controller.reset_arms()
        elif command == 'wave_left':
            print("🤲 左手展开")
            controller.wave_hand('left', cycles=3)
        elif command == 'wave_right':
            print("🤲 右手展开")
            controller.wave_hand('right', cycles=3)
        elif command == 'lower_left':
            print("⬇️ 左手缓慢放下")
            controller.lower_arm_slowly('left')
        elif command == 'lower_right':
            print("⬇️ 右手缓慢放下")
            controller.lower_arm_slowly('right')
        # 保留旧命令以兼容性
        elif command == 'left':
            print("🤲 左手展开（兼容模式）")
            controller.wave_hand('left', cycles=3)
        elif command == 'right':
            print("🤲 右手展开（兼容模式）")
            controller.wave_hand('right', cycles=3)
        else:
            print(f"❌ 未知命令 Unknown command: {command}")
            print_usage()
            
    except KeyboardInterrupt:
        print("\n🛑 用户中断 User interrupted")
    except Exception as e:
        print(f"\n💥 错误 Error: {e}")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
