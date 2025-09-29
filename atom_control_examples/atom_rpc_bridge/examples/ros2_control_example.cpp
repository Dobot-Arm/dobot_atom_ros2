/**
 * @file ros2_control_example.cpp
 * @brief ROS2话题控制Atom机器人的示例程序
 * @author futingxing
 * @date 2025-09-01
 * @version 1.0.0
 * @copyright Copyright (c) 2025 Dobot. All rights reserved.
 * 
 * 此示例展示如何通过ROS2话题来控制Atom机器人，
 * 无需直接使用RPC接口，通过DDS桥接中间层实现控制。
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <chrono>
#include <thread>

class AtomRos2Controller : public rclcpp::Node {
public:
    AtomRos2Controller() : Node("atom_ros2_controller") {
        // 创建发布者
        fsm_cmd_pub_ = this->create_publisher<std_msgs::msg::UInt32>("/atom/fsm_cmd", 10);
        vel_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/atom/vel_cmd", 10);
        upper_limb_cmd_pub_ = this->create_publisher<std_msgs::msg::Bool>("/atom/upper_limb_cmd", 10);
        
        // 创建订阅者
        fsm_state_sub_ = this->create_subscription<std_msgs::msg::UInt32>(
            "/atom/fsm_state", 10,
            std::bind(&AtomRos2Controller::fsmStateCallback, this, std::placeholders::_1));
        
        connection_state_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/atom/connection_state", 10,
            std::bind(&AtomRos2Controller::connectionStateCallback, this, std::placeholders::_1));
        
        // 创建服务客户端
        upper_limb_service_client_ = this->create_client<std_srvs::srv::SetBool>("/atom/upper_limb_service");
        
        // 初始化状态
        current_fsm_state_ = 0;
        is_connected_ = false;
        
        RCLCPP_INFO(this->get_logger(), "Atom ROS2 Controller initialized");
        RCLCPP_INFO(this->get_logger(), "Waiting for DDS bridge connection...");
    }
    
    /**
     * @brief 设置FSM状态
     * @param fsm_id FSM状态ID
     */
    void setFsmState(uint32_t fsm_id) {
        auto msg = std_msgs::msg::UInt32();
        msg.data = fsm_id;
        fsm_cmd_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published FSM command: %u", fsm_id);
    }
    
    /**
     * @brief 设置机器人速度
     * @param linear_x 前进速度 (m/s)
     * @param linear_y 侧移速度 (m/s)
     * @param angular_z 旋转速度 (rad/s)
     */
    void setVelocity(double linear_x, double linear_y, double angular_z) {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = linear_x;
        msg.linear.y = linear_y;
        msg.linear.z = 0.0;
        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = angular_z;
        
        vel_cmd_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published velocity command: [%.2f, %.2f, %.2f]", 
                   linear_x, linear_y, angular_z);
    }
    
    /**
     * @brief 启用/禁用上肢控制
     * @param enable true为启用，false为禁用
     */
    void setUpperLimbEnable(bool enable) {
        auto msg = std_msgs::msg::Bool();
        msg.data = enable;
        upper_limb_cmd_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published upper limb command: %s", 
                   enable ? "enabled" : "disabled");
    }
    
    /**
     * @brief 通过服务调用启用/禁用上肢控制
     * @param enable true为启用，false为禁用
     */
    void setUpperLimbEnableService(bool enable) {
        if (!upper_limb_service_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Upper limb service not available");
            return;
        }
        
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = enable;
        
        auto future = upper_limb_service_client_->async_send_request(request);
        
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Upper limb service response: %s - %s",
                       response->success ? "SUCCESS" : "FAILED", response->message.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call upper limb service");
        }
    }
    
    /**
     * @brief 停止机器人运动
     */
    void stopRobot() {
        setVelocity(0.0, 0.0, 0.0);
        RCLCPP_INFO(this->get_logger(), "Robot stopped");
    }
    
    /**
     * @brief 获取当前FSM状态
     */
    uint32_t getCurrentFsmState() const {
        return current_fsm_state_;
    }
    
    /**
     * @brief 检查是否连接到DDS桥接
     */
    bool isConnected() const {
        return is_connected_;
    }
    
    /**
     * @brief 等待连接建立
     * @param timeout_seconds 超时时间（秒）
     * @return true如果连接成功，false如果超时
     */
    bool waitForConnection(int timeout_seconds = 10) {
        auto start_time = std::chrono::steady_clock::now();
        auto timeout = std::chrono::seconds(timeout_seconds);
        
        while (!is_connected_) {
            rclcpp::spin_some(this->get_node_base_interface());
            
            if (std::chrono::steady_clock::now() - start_time > timeout) {
                return false;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        return true;
    }

private:
    // 发布者
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr fsm_cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr upper_limb_cmd_pub_;
    
    // 订阅者
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr fsm_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr connection_state_sub_;
    
    // 服务客户端
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr upper_limb_service_client_;
    
    // 状态变量
    uint32_t current_fsm_state_;
    bool is_connected_;
    
    void fsmStateCallback(const std_msgs::msg::UInt32::SharedPtr msg) {
        current_fsm_state_ = msg->data;
        RCLCPP_DEBUG(this->get_logger(), "Received FSM state: %u", msg->data);
    }
    
    void connectionStateCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        bool prev_connected = is_connected_;
        is_connected_ = msg->data;
        
        if (is_connected_ && !prev_connected) {
            RCLCPP_INFO(this->get_logger(), "Connected to DDS bridge");
        } else if (!is_connected_ && prev_connected) {
            RCLCPP_WARN(this->get_logger(), "Disconnected from DDS bridge");
        }
    }
};

/**
 * @brief 演示基本控制序列
 */
void demonstrateBasicControl(std::shared_ptr<AtomRos2Controller> controller) {
    RCLCPP_INFO(controller->get_logger(), "=== 开始基本控制演示 ===");
    
    // 1. 设置为站立模式
    RCLCPP_INFO(controller->get_logger(), "1. 设置为站立模式 (FSM ID: 100)");
    controller->setFsmState(100);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // 2. 启用上肢控制
    RCLCPP_INFO(controller->get_logger(), "2. 启用上肢控制");
    controller->setUpperLimbEnable(true);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // 3. 前进
    RCLCPP_INFO(controller->get_logger(), "3. 前进 0.1 m/s，持续3秒");
    controller->setVelocity(0.1, 0.0, 0.0);
    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    // 4. 停止
    RCLCPP_INFO(controller->get_logger(), "4. 停止运动");
    controller->stopRobot();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // 5. 左转
    RCLCPP_INFO(controller->get_logger(), "5. 左转 0.2 rad/s，持续2秒");
    controller->setVelocity(0.0, 0.0, 0.2);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // 6. 停止
    RCLCPP_INFO(controller->get_logger(), "6. 停止运动");
    controller->stopRobot();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // 7. 禁用上肢控制
    RCLCPP_INFO(controller->get_logger(), "7. 禁用上肢控制");
    controller->setUpperLimbEnable(false);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // 8. 设置为零力矩模式
    RCLCPP_INFO(controller->get_logger(), "8. 设置为零力矩模式 (FSM ID: 0)");
    controller->setFsmState(0);
    
    RCLCPP_INFO(controller->get_logger(), "=== 基本控制演示完成 ===");
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    try {
        auto controller = std::make_shared<AtomRos2Controller>();
        
        // 等待连接到DDS桥接
        RCLCPP_INFO(controller->get_logger(), "等待连接到DDS桥接...");
        if (!controller->waitForConnection(10)) {
            RCLCPP_WARN(controller->get_logger(), "未能连接到DDS桥接，继续运行演示");
        } else {
            RCLCPP_INFO(controller->get_logger(), "已连接到DDS桥接");
        }
        
        // 检查命令行参数
        if (argc > 1 && std::string(argv[1]) == "--demo") {
            // 运行演示
            demonstrateBasicControl(controller);
        } else {
            // 交互式控制
            RCLCPP_INFO(controller->get_logger(), "交互式控制模式");
            RCLCPP_INFO(controller->get_logger(), "使用以下命令控制机器人:");
            RCLCPP_INFO(controller->get_logger(), "  ros2 topic pub /atom/fsm_cmd std_msgs/msg/UInt32 \"data: 100\" --once");
            RCLCPP_INFO(controller->get_logger(), "  ros2 topic pub /atom/vel_cmd geometry_msgs/msg/Twist \"linear: {x: 0.1}\" --once");
            RCLCPP_INFO(controller->get_logger(), "  ros2 topic pub /atom/upper_limb_cmd std_msgs/msg/Bool \"data: true\" --once");
            RCLCPP_INFO(controller->get_logger(), "按 Ctrl+C 退出");
            
            rclcpp::spin(controller);
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("atom_ros2_controller"), "异常: %s", e.what());
        return -1;
    }
    
    rclcpp::shutdown();
    return 0;
}