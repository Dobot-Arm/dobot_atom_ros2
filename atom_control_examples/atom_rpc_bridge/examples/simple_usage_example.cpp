/**
 * @file simple_usage_example.cpp
 * @brief 简单的Atom DDS客户端使用示例
 * @author futingxing
 * @date 2025-09-01
 * @version 1.0.0
 * @copyright Copyright (c) 2025 Dobot. All rights reserved.
 * 
 * 本示例展示了如何在您的项目中使用AtomDdsClient来控制Atom机器人
 */

#include "atom_rpc_bridge/atom_dds_client.hpp"
#include <iostream>
#include <thread>
#include <chrono>

using namespace atom_rpc_bridge;
using namespace std::chrono_literals;

int main() {
    std::cout << "=== Atom DDS客户端简单使用示例 ===" << std::endl;
    
    // 1. 创建DDS客户端实例
    AtomDdsClient robot_client("simple_example_node");
    
    // 2. 初始化客户端
    std::cout << "正在初始化DDS客户端..." << std::endl;
    if (!robot_client.initialize()) {
        std::cerr << "错误: 初始化失败!" << std::endl;
        return -1;
    }
    
    // 3. 等待连接建立
    std::cout << "等待连接建立..." << std::endl;
    int retry_count = 0;
    while (!robot_client.IsConnected() && retry_count < 10) {
        std::this_thread::sleep_for(500ms);
        retry_count++;
    }
    
    if (!robot_client.IsConnected()) {
        std::cout << "警告: 无法建立连接，继续演示..." << std::endl;
    } else {
        std::cout << "连接建立成功!" << std::endl;
    }
    
    // 4. 基本控制示例
    std::cout << "\n开始控制演示..." << std::endl;
    
    // 设置为阻尼模式
    std::cout << "1. 设置为阻尼模式 (FSM ID: 1)" << std::endl;
    auto result = robot_client.SetFsmId(1);
    if (result == AtomRpcErrorCode::SUCCESS) {
        std::cout << "   ✓ FSM状态设置成功" << std::endl;
    } else {
        std::cout << "   ✗ FSM状态设置失败" << std::endl;
    }
    
    std::this_thread::sleep_for(2s);
    
    // 启用上肢控制
    std::cout << "2. 启用上肢控制" << std::endl;
    result = robot_client.SwitchUpperLimbControl(true);
    if (result == AtomRpcErrorCode::SUCCESS) {
        std::cout << "   ✓ 上肢控制启用成功" << std::endl;
    } else {
        std::cout << "   ✗ 上肢控制启用失败" << std::endl;
    }
    
    std::this_thread::sleep_for(1s);
    
    // 速度控制示例
    std::cout << "3. 速度控制演示" << std::endl;
    
    // 前进
    std::cout << "   前进 (0.1 m/s)" << std::endl;
    robot_client.SetVel(0.1f, 0.0f, 0.0f);
    std::this_thread::sleep_for(3s);
    
    // 停止
    std::cout << "   停止" << std::endl;
    robot_client.SetVel(0.0f, 0.0f, 0.0f);
    std::this_thread::sleep_for(1s);
    
    // 左转
    std::cout << "   左转 (0.2 rad/s)" << std::endl;
    robot_client.SetVel(0.0f, 0.0f, 0.2f);
    std::this_thread::sleep_for(3s);
    
    // 停止
    std::cout << "   停止" << std::endl;
    robot_client.SetVel(0.0f, 0.0f, 0.0f);
    
    // 5. 查询当前状态
    std::cout << "4. 查询当前状态" << std::endl;
    uint32_t current_fsm;
    result = robot_client.GetFsmId(current_fsm);
    if (result == AtomRpcErrorCode::SUCCESS) {
        std::cout << "   当前FSM状态ID: " << current_fsm << std::endl;
    } else {
        std::cout << "   无法获取FSM状态" << std::endl;
    }
    
    // 6. 安全停止
    std::cout << "5. 安全停止" << std::endl;
    robot_client.SetVel(0.0f, 0.0f, 0.0f);  // 停止运动
    robot_client.SwitchUpperLimbControl(false);  // 禁用上肢控制
    
    std::cout << "\n演示完成!" << std::endl;
    
    return 0;
}