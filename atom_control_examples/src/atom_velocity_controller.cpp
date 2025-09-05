/**
 * @file atom_velocity_controller.cpp
 * @brief Atom机器人速度控制器
 * @details 实现机器人速度控制和状态机管理，支持前后左右移动和旋转控制
 * @author futingxing
 * @date 2025-09-01
 * @version 1.0.0
 * @copyright Copyright (c) 2025 Dobot. All rights reserved.
 */

#include <rclcpp/rclcpp.hpp>
#include <dobot_atom/msg/set_fsm_id.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>

class AtomVelocityController : public rclcpp::Node
{
public:
    AtomVelocityController() : Node("atom_velocity_controller")
    {
        // 发布者
        fsm_pub_ = this->create_publisher<dobot_atom::msg::SetFsmId>("/set/fsm/id", 10);
        velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // 订阅者 - 监听当前FSM状态（如果有的话）
        fsm_status_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/fsm/status", 10,
            std::bind(&AtomVelocityController::fsmStatusCallback, this, std::placeholders::_1));
        
        // 订阅速度命令
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_input", 10,
            std::bind(&AtomVelocityController::cmdVelCallback, this, std::placeholders::_1));
        
        // 初始化参数
        current_fsm_id_ = 0;
        default_velocity_ = 0.3;  // 默认速度 m/s
        default_duration_ = 2.0;  // 默认持续时间 s
        
        // 创建交互式控制定时器
        interactive_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&AtomVelocityController::interactiveControl, this));
        
        // 初始化用户输入状态
        user_input_mode_ = 0;
        waiting_for_input_ = true;
        
        RCLCPP_INFO(this->get_logger(), "Atom Velocity Controller initialized");
        RCLCPP_INFO(this->get_logger(), "Available commands:");
        RCLCPP_INFO(this->get_logger(), "  1 = FSM Control");
        RCLCPP_INFO(this->get_logger(), "  2 = Velocity Control");
        RCLCPP_INFO(this->get_logger(), "  3 = Continuous Velocity Mode");
        RCLCPP_INFO(this->get_logger(), "  0 = Exit");
        
        printMenu();
    }

private:
    // ROS2 发布者和订阅者
    rclcpp::Publisher<dobot_atom::msg::SetFsmId>::SharedPtr fsm_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr fsm_status_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    
    rclcpp::TimerBase::SharedPtr interactive_timer_;
    
    // 控制参数
    int32_t current_fsm_id_;
    double default_velocity_;
    double default_duration_;
    
    // 用户交互状态
    int user_input_mode_;
    bool waiting_for_input_;
    
    void fsmStatusCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        current_fsm_id_ = msg->data;
        RCLCPP_DEBUG(this->get_logger(), "Current FSM ID: %d", current_fsm_id_);
    }
    
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // 检查FSM状态是否允许速度控制
        if (current_fsm_id_ != 301 && current_fsm_id_ != 302) {
            RCLCPP_WARN(this->get_logger(), 
                       "Velocity control not allowed in current FSM state (%d). Need FSM 301 or 302.", 
                       current_fsm_id_);
            return;
        }
        
        // 转发速度命令
        velocity_pub_->publish(*msg);
        
        RCLCPP_INFO(this->get_logger(), 
                   "Publishing velocity: vx=%.2f, vy=%.2f, vyaw=%.2f", 
                   msg->linear.x, msg->linear.y, msg->angular.z);
    }
    
    void printMenu()
    {
        std::cout << "\n=== Atom Velocity Controller ===" << std::endl;
        std::cout << "Enter command:" << std::endl;
        std::cout << "  1 = FSM Control" << std::endl;
        std::cout << "  2 = Single Velocity Command" << std::endl;
        std::cout << "  3 = Continuous Velocity Mode" << std::endl;
        std::cout << "  0 = Exit" << std::endl;
        std::cout << "Choice: ";
    }
    
    void interactiveControl()
    {
        if (!waiting_for_input_) {
            return;
        }
        
        // 这里简化处理，实际应用中可以使用更复杂的输入处理
        // 为了演示，我们提供一些预设的控制序列
        static int demo_step = 0;
        static auto last_demo_time = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        
        // 每5秒执行一个演示步骤
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_demo_time).count() >= 5) {
            executeDemoSequence(demo_step);
            demo_step = (demo_step + 1) % 4; // 循环4个步骤
            last_demo_time = now;
        }
    }
    
    void executeDemoSequence(int step)
    {
        switch (step) {
            case 0:
                RCLCPP_INFO(this->get_logger(), "Demo Step 1: Setting FSM to 301 (Walking mode)");
                setFsmId(301);
                break;
                
            case 1:
                RCLCPP_INFO(this->get_logger(), "Demo Step 2: Moving forward");
                sendVelocityCommand(0.3, 0.0, 0.0, 2.0);
                break;
                
            case 2:
                RCLCPP_INFO(this->get_logger(), "Demo Step 3: Turning left");
                sendVelocityCommand(0.0, 0.0, 0.5, 2.0);
                break;
                
            case 3:
                RCLCPP_INFO(this->get_logger(), "Demo Step 4: Stopping");
                sendVelocityCommand(0.0, 0.0, 0.0, 1.0);
                break;
        }
    }
    
    void setFsmId(int32_t fsm_id)
    {
        auto msg = dobot_atom::msg::SetFsmId();
        msg.id = fsm_id;
        
        fsm_pub_->publish(msg);
        current_fsm_id_ = fsm_id;
        
        RCLCPP_INFO(this->get_logger(), "Setting FSM ID to: %d", fsm_id);
        
        // 等待FSM切换完成
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    void sendVelocityCommand(double vx, double vy, double vyaw, double duration)
    {
        // 检查FSM状态
        if (current_fsm_id_ != 301 && current_fsm_id_ != 302) {
            RCLCPP_ERROR(this->get_logger(), 
                        "Cannot send velocity command. Current FSM ID: %d. Need 301 or 302.", 
                        current_fsm_id_);
            return;
        }
        
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = vx;
        twist_msg.linear.y = vy;
        twist_msg.angular.z = vyaw;
        
        RCLCPP_INFO(this->get_logger(), 
                   "Sending velocity command: vx=%.2f m/s, vy=%.2f m/s, vyaw=%.2f rad/s for %.1f seconds", 
                   vx, vy, vyaw, duration);
        
        // 发送速度命令持续指定时间
        auto start_time = std::chrono::steady_clock::now();
        auto end_time = start_time + std::chrono::milliseconds(static_cast<int>(duration * 1000));
        
        while (std::chrono::steady_clock::now() < end_time) {
            velocity_pub_->publish(twist_msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 20Hz发送频率
        }
        
        // 发送停止命令
        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.angular.z = 0.0;
        velocity_pub_->publish(twist_msg);
        
        RCLCPP_INFO(this->get_logger(), "Velocity command completed");
    }
    
    void handleFsmControl()
    {
        RCLCPP_INFO(this->get_logger(), "FSM Control Mode");
        RCLCPP_INFO(this->get_logger(), "Current FSM ID: %d", current_fsm_id_);
        RCLCPP_INFO(this->get_logger(), "Common FSM IDs:");
        RCLCPP_INFO(this->get_logger(), "  0 = Idle");
        RCLCPP_INFO(this->get_logger(), "  301 = Walking Mode");
        RCLCPP_INFO(this->get_logger(), "  302 = Running Mode");
        
        // 示例：设置为行走模式
        setFsmId(301);
    }
    
    void handleVelocityControl()
    {
        RCLCPP_INFO(this->get_logger(), "Single Velocity Command Mode");
        
        if (current_fsm_id_ != 301 && current_fsm_id_ != 302) {
            RCLCPP_WARN(this->get_logger(), "Setting FSM to 301 first...");
            setFsmId(301);
        }
        
        // 发送示例速度命令
        sendVelocityCommand(default_velocity_, 0.0, 0.0, default_duration_);
    }
    
    void handleContinuousVelocityMode()
    {
        RCLCPP_INFO(this->get_logger(), "Continuous Velocity Mode");
        RCLCPP_INFO(this->get_logger(), "Subscribe to /cmd_vel_input to send velocity commands");
        RCLCPP_INFO(this->get_logger(), "Example: ros2 topic pub /cmd_vel_input geometry_msgs/Twist '{linear: {x: 0.3}}'");
        
        if (current_fsm_id_ != 301 && current_fsm_id_ != 302) {
            RCLCPP_WARN(this->get_logger(), "Setting FSM to 301 first...");
            setFsmId(301);
        }
    }
    
public:
    // 公共接口用于外部调用
    void executeCommand(int command)
    {
        switch (command) {
            case 1:
                handleFsmControl();
                break;
            case 2:
                handleVelocityControl();
                break;
            case 3:
                handleContinuousVelocityMode();
                break;
            case 0:
                RCLCPP_INFO(this->get_logger(), "Exiting...");
                rclcpp::shutdown();
                break;
            default:
                RCLCPP_WARN(this->get_logger(), "Unknown command: %d", command);
                break;
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AtomVelocityController>();
    
    RCLCPP_INFO(node->get_logger(), "Starting Atom Velocity Controller...");
    RCLCPP_WARN(node->get_logger(), "WARNING: Robot will move! Ensure safe environment!");
    
    // 如果有命令行参数，直接执行对应命令
    if (argc > 1) {
        int command = std::atoi(argv[1]);
        node->executeCommand(command);
    }
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}