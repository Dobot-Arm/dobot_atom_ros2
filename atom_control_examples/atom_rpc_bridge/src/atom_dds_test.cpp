/**
 * @file atom_dds_test.cpp
 * @brief Atom机器人DDS高层控制测试程序
 * @author futingxing
 * @date 2025-09-01
 * @version 1.0.0
 * @copyright Copyright (c) 2025 Dobot. All rights reserved.
 * 
 * 本程序将原有的RPC控制接口改写为DDS实现，提供与原rpc_test.cpp相同的功能
 * 主要功能：
 * - FSM状态控制
 * - 速度控制
 * - 上肢控制开关
 * 
 * 使用方法：
 * 1. 启动机器人底层控制程序
 * 2. 运行本程序
 * 3. 根据提示输入控制命令
 */

#include "atom_rpc_bridge/atom_dds_client.hpp"
#include <iostream>
#include <string>
#include <thread>
#include <chrono>

using namespace atom_rpc_bridge;
using namespace std::chrono_literals;

class AtomDdsTestApp {
public:
    AtomDdsTestApp(const std::string& rpc_ip = "192.168.8.234", int rpc_port = 51234) 
        : dds_client_("atom_dds_test_node", rpc_ip, rpc_port), running_(true), rpc_ip_(rpc_ip), rpc_port_(rpc_port) {}
    
    bool initialize() {
        std::cout << "正在初始化Atom DDS客户端..." << std::endl;
        
        if (!dds_client_.initialize()) {
            std::cerr << "错误: DDS客户端初始化失败!" << std::endl;
            return false;
        }
        
        // 等待连接建立
        int retry_count = 0;
        while (!dds_client_.IsConnected() && retry_count < 10) {
            std::cout << "等待连接建立... (" << (retry_count + 1) << "/10)" << std::endl;
            std::this_thread::sleep_for(1s);
            retry_count++;
        }
        
        if (!dds_client_.IsConnected()) {
            std::cerr << "警告: 无法建立DDS连接，某些功能可能不可用" << std::endl;
        } else {
            std::cout << "DDS连接建立成功!" << std::endl;
        }
        
        return true;
    }
    
    void run() {
        printHelp();
        
        std::string input;
        while (running_ && std::getline(std::cin, input)) {
            processCommand(input);
            if (running_) {
                std::cout << "\n请输入命令 (输入 'h' 查看帮助): ";
            }
        }
    }
    
private:
    AtomDdsClient dds_client_;
    bool running_;
    std::string rpc_ip_;
    int rpc_port_;
    
    void printHelp() {
        std::cout << "\n=== Atom机器人DDS控制测试程序 ===" << std::endl;
        std::cout << "可用命令:" << std::endl;
        std::cout << "  h          - 显示帮助信息" << std::endl;
        std::cout << "  status     - 查看当前状态" << std::endl;
        std::cout << "  fsm <id>   - 设置FSM状态ID" << std::endl;
        std::cout << "  vel <x> <y> <z> - 设置速度 (线速度x,y 角速度z)" << std::endl;
        std::cout << "  upper <0|1> - 上肢控制开关 (0=关闭, 1=开启)" << std::endl;
        std::cout << "  demo       - 运行演示程序" << std::endl;
        std::cout << "  quit       - 退出程序" << std::endl;
        std::cout << "\n常用FSM状态ID:" << std::endl;
        std::cout << "  0  - 零力矩模式 (阻尼状态)" << std::endl;
        std::cout << "  1  - 阻尼模式" << std::endl;
        std::cout << "  2  - 底层控制状态 (调试模式)" << std::endl;
        std::cout << "  100 - 预备锁定站立状态" << std::endl;
        std::cout << "  101 - 准备站立状态" << std::endl;
        std::cout << "  102 - 坐姿状态" << std::endl;
        std::cout << "  200 - 制动起立状态" << std::endl;
        std::cout << "  201 - 蹲起状态" << std::endl;
        std::cout << "请输入命令: ";
    }
    
    void processCommand(const std::string& input) {
        if (input.empty()) return;
        
        std::istringstream iss(input);
        std::string command;
        iss >> command;
        
        if (command == "h" || command == "help") {
            printHelp();
        }
        else if (command == "status") {
            showStatus();
        }
        else if (command == "fsm") {
            uint32_t fsm_id;
            if (iss >> fsm_id) {
                setFsmId(fsm_id);
            } else {
                std::cout << "错误: 请提供FSM状态ID" << std::endl;
            }
        }
        else if (command == "vel") {
            float x, y, z;
            if (iss >> x >> y >> z) {
                setVelocity(x, y, z);
            } else {
                std::cout << "错误: 请提供三个速度参数 (x y z)" << std::endl;
            }
        }
        else if (command == "upper") {
            int enable;
            if (iss >> enable) {
                switchUpperLimb(enable != 0);
            } else {
                std::cout << "错误: 请提供开关参数 (0或1)" << std::endl;
            }
        }
        else if (command == "demo") {
            runDemo();
        }
        else if (command == "quit" || command == "exit") {
            std::cout << "正在退出..." << std::endl;
            running_ = false;
        }
        else {
            std::cout << "未知命令: " << command << std::endl;
            std::cout << "输入 'h' 查看帮助" << std::endl;
        }
    }
    
    void showStatus() {
        std::cout << "\n=== 当前状态 ===" << std::endl;
        std::cout << "DDS连接状态: " << (dds_client_.IsConnected() ? "已连接" : "未连接") << std::endl;
        
        uint32_t current_fsm;
        auto result = dds_client_.GetFsmId(current_fsm);
        if (result == AtomRpcErrorCode::SUCCESS) {
            std::cout << "当前FSM状态ID: " << current_fsm << std::endl;
        } else {
            std::cout << "无法获取FSM状态" << std::endl;
        }
    }
    
    void setFsmId(uint32_t fsm_id) {
        std::cout << "设置FSM状态ID为: " << fsm_id << std::endl;
        
        auto result = dds_client_.SetFsmId(fsm_id);
        if (result == AtomRpcErrorCode::SUCCESS) {
            std::cout << "FSM状态设置成功" << std::endl;
        } else {
            std::cout << "FSM状态设置失败, 错误码: " << static_cast<int>(result) << std::endl;
        }
    }
    
    void setVelocity(float x, float y, float z) {
        std::cout << "设置速度: x=" << x << ", y=" << y << ", z=" << z << std::endl;
        
        auto result = dds_client_.SetVel(x, y, z);
        if (result == AtomRpcErrorCode::SUCCESS) {
            std::cout << "速度设置成功" << std::endl;
        } else {
            std::cout << "速度设置失败, 错误码: " << static_cast<int>(result) << std::endl;
        }
    }
    
    void switchUpperLimb(bool enable) {
        std::cout << (enable ? "启用" : "禁用") << "上肢控制" << std::endl;
        
        auto result = dds_client_.SwitchUpperLimbControl(enable);
        if (result == AtomRpcErrorCode::SUCCESS) {
            std::cout << "上肢控制开关设置成功" << std::endl;
        } else {
            std::cout << "上肢控制开关设置失败, 错误码: " << static_cast<int>(result) << std::endl;
        }
    }
    
    void runDemo() {
        std::cout << "\n=== 运行演示程序 ===" << std::endl;
        
        // 演示1: 状态切换
        std::cout << "1. 切换到阻尼模式..." << std::endl;
        setFsmId(1);
        std::this_thread::sleep_for(2s);
        
        // 演示2: 启用上肢控制
        std::cout << "2. 启用上肢控制..." << std::endl;
        switchUpperLimb(true);
        std::this_thread::sleep_for(1s);
        
        // 演示3: 速度控制
        std::cout << "3. 前进运动..." << std::endl;
        setVelocity(0.1f, 0.0f, 0.0f);
        std::this_thread::sleep_for(2s);
        
        std::cout << "4. 停止运动..." << std::endl;
        setVelocity(0.0f, 0.0f, 0.0f);
        std::this_thread::sleep_for(1s);
        
        // 演示4: 旋转运动
        std::cout << "5. 旋转运动..." << std::endl;
        setVelocity(0.0f, 0.0f, 0.2f);
        std::this_thread::sleep_for(2s);
        
        std::cout << "6. 停止运动..." << std::endl;
        setVelocity(0.0f, 0.0f, 0.0f);
        
        std::cout << "演示程序完成!" << std::endl;
    }
};

int main(int argc, char** argv) {
    try {
        // 解析命令行参数
        std::string rpc_ip = "192.168.8.234";
        int rpc_port = 51234;
        
        for (int i = 1; i < argc; i++) {
            std::string arg = argv[i];
            if (arg == "--rpc-ip" && i + 1 < argc) {
                rpc_ip = argv[++i];
            } else if (arg == "--rpc-port" && i + 1 < argc) {
                rpc_port = std::stoi(argv[++i]);
            } else if (arg == "--help" || arg == "-h") {
                std::cout << "用法: " << argv[0] << " [选项]" << std::endl;
                std::cout << "选项:" << std::endl;
                std::cout << "  --rpc-ip <IP>     RPC服务器IP地址 (默认: 192.168.8.234)" << std::endl;
                std::cout << "  --rpc-port <PORT> RPC服务器端口 (默认: 51234)" << std::endl;
                std::cout << "  --help, -h        显示此帮助信息" << std::endl;
                return 0;
            }
        }
        
        std::cout << "连接到RPC服务器: " << rpc_ip << ":" << rpc_port << std::endl;
        
        AtomDdsTestApp app(rpc_ip, rpc_port);
        
        if (!app.initialize()) {
            std::cerr << "应用程序初始化失败!" << std::endl;
            return -1;
        }
        
        app.run();
        
    } catch (const std::exception& e) {
        std::cerr << "程序异常: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}