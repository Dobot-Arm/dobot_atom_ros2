#!/usr/bin/env python3
"""
Atom机器人手臂控制启动文件
只启动手臂控制器，自动激活，无需额外操作
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import time
import random
import string

def generate_launch_description():
    # 生成唯一实体名称避免冲突
    timestamp = str(int(time.time()))
    random_suffix = ''.join(random.choices(string.ascii_lowercase, k=4))
    robot_name_in_model = f'atom_arms_{timestamp}_{random_suffix}'
    
    package_name = 'atom_urdf'
    urdf_name = "atom_gazebo_control.xacro"
    
    # 启动参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='使用仿真时间'
    )
    
    paused_arg = DeclareLaunchArgument(
        'paused',
        default_value='false',
        description='启动时暂停Gazebo'
    )
    
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='启动Gazebo GUI'
    )
    
    # Gazebo启动
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'paused': LaunchConfiguration('paused'),
            'gui': LaunchConfiguration('gui'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'verbose': 'true'
        }.items(),
    )
    
    # 机器人描述
    robot_description_content = Command([
        'xacro ', 
        PathJoinSubstitution([FindPackageShare('atom_urdf'), 'urdf', urdf_name])
    ])
    
    robot_description = {'robot_description': robot_description_content}
    
    # 机器人状态发布器
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # 生成机器人实体
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', robot_name_in_model,
            '-z', '0.1',
            '-timeout', '60'
        ],
        output='screen'
    )
    
    # 加载关节状态广播器（必需）
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )
    
    # 加载并激活左臂控制器
    load_left_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'left_arm_controller'],
        output='screen'
    )
    
    # 加载并激活右臂控制器
    load_right_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'right_arm_controller'],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        paused_arg,
        gui_arg,
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        
        # 延时加载控制器确保机器人已生成
        TimerAction(
            period=3.0,
            actions=[load_joint_state_broadcaster]
        ),
        TimerAction(
            period=4.0,
            actions=[
                load_left_arm_controller,
                load_right_arm_controller
            ]
        )
    ])