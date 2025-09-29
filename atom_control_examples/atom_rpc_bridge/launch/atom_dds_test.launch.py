#!/usr/bin/env python3
"""
Atom DDS测试程序启动文件

使用方法:
ros2 launch atom_rpc_bridge atom_dds_test.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明启动参数
    node_name_arg = DeclareLaunchArgument(
        'node_name',
        default_value='atom_dds_test_node',
        description='DDS测试节点名称'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='日志级别 (debug, info, warn, error)'
    )
    
    rpc_ip_arg = DeclareLaunchArgument(
        'rpc_ip',
        default_value='192.168.8.234',
        description='RPC服务器IP地址'
    )
    
    rpc_port_arg = DeclareLaunchArgument(
        'rpc_port',
        default_value='51234',
        description='RPC服务器端口'
    )
    
    # DDS测试节点
    atom_dds_test_node = Node(
        package='atom_rpc_bridge',
        executable='atom_dds_test',
        name=LaunchConfiguration('node_name'),
        output='screen',
        parameters=[
            {
                'use_sim_time': False,
                'rpc_ip': LaunchConfiguration('rpc_ip'),
                'rpc_port': LaunchConfiguration('rpc_port')
            }
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    
    return LaunchDescription([
        node_name_arg,
        log_level_arg,
        rpc_ip_arg,
        rpc_port_arg,
        LogInfo(msg='启动Atom DDS测试程序...'),
        atom_dds_test_node
    ])