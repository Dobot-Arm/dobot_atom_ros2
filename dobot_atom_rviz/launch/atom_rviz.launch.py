#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot Atom机器人RViz可视化启动文件

功能描述:
- 启动robot_state_publisher节点发布机器人状态
- 启动joint_state_publisher节点发布关节状态
- 启动RViz2进行机器人模型可视化
- 支持GUI模式的关节状态发布器

作者: futingxing
日期: 2024-09-01
版本: 1.0.0
Copyright (c) 2024 Dobot. All rights reserved.
"""

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    """
    生成Atom机器人RViz可视化的启动描述
    
    Returns:
        LaunchDescription: ROS2启动描述对象
    """
    
    # 获取atom_urdf包的路径
    atom_urdf_path = get_package_share_path('atom_urdf')
    # 设置默认的URDF模型文件路径
    default_model_path = atom_urdf_path / 'urdf/atom.urdf'
    
    # 获取当前包的路径用于RViz配置文件
    dobot_atom_rviz_path = get_package_share_path('dobot_atom_rviz')
    default_rviz_config_path = dobot_atom_rviz_path / 'rviz/atom.rviz'

    # 声明启动参数
    # GUI参数：是否启用图形化关节状态发布器
    gui_arg = DeclareLaunchArgument(
        name='gui', 
        default_value='true', 
        choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui for interactive joint control'
    )
    
    # 模型参数：URDF文件的绝对路径
    model_arg = DeclareLaunchArgument(
        name='model', 
        default_value=str(default_model_path),
        description='Absolute path to Atom robot urdf file'
    )
    
    # RViz配置参数：RViz配置文件的绝对路径
    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig', 
        default_value=str(default_rviz_config_path),
        description='Absolute path to rviz config file'
    )

    # 机器人描述参数：使用xacro处理URDF文件
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    # 机器人状态发布器节点：发布机器人的TF变换和状态信息
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
        arguments=['--ros-args', '--log-level', 'info']
    )

    # 关节状态发布器节点（非GUI模式）：发布关节状态信息
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('gui')),
        arguments=['--ros-args', '--log-level', 'info']
    )

    # 关节状态发布器GUI节点：提供图形化界面控制关节
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui')),
        arguments=['--ros-args', '--log-level', 'info']
    )

    # RViz2可视化节点：启动RViz进行机器人模型可视化
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    # 返回启动描述
    return LaunchDescription([
        gui_arg,
        model_arg,
        rviz_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])