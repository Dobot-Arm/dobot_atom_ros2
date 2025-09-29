/**
 * @file atom_dds_client.cpp
 * @brief Atom DDS客户端实现
 * @author futingxing
 * @date 2025-09-01
 * @version 1.0.0
 * @copyright Copyright (c) 2025 Dobot. All rights reserved.
 */

#include "atom_rpc_bridge/atom_dds_client.hpp"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

namespace atom_rpc_bridge {

AtomDdsClient::AtomDdsClient(const std::string& node_name, const std::string& rpc_ip, int rpc_port)
    : rpc_ip_(rpc_ip)
    , rpc_port_(rpc_port)
    , rpc_connected_(false)
    , current_fsm_id_(0)
    , is_connected_(false)
    , is_initialized_(false) {
    
    // 初始化ROS2
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }
    
    node_ = rclcpp::Node::make_shared(node_name);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    
    // 创建独立RPC客户端
    rpc_client_ = std::make_unique<StandaloneRpcClient>(rpc_ip_, rpc_port_);
}

AtomDdsClient::~AtomDdsClient() {
    is_initialized_ = false;
    is_connected_ = false;
    rpc_connected_ = false;
    
    // 停止RPC桥接线程
    if (rpc_bridge_thread_.joinable()) {
        rpc_bridge_thread_.join();
    }
    
    // 断开RPC连接
    disconnectFromRpcServer();
    
    if (executor_thread_.joinable()) {
        executor_->cancel();
        executor_thread_.join();
    }
}

bool AtomDdsClient::initialize() {
    if (is_initialized_) {
        return true;
    }
    
    try {
        // 创建订阅者 - 接收外部控制命令
        fsm_cmd_sub_ = node_->create_subscription<std_msgs::msg::UInt32>(
            "/atom/fsm_cmd", 10,
            std::bind(&AtomDdsClient::fsmCmdCallback, this, std::placeholders::_1));
        
        vel_cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
            "/atom/vel_cmd", 10,
            std::bind(&AtomDdsClient::velCmdCallback, this, std::placeholders::_1));
        
        upper_limb_cmd_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/atom/upper_limb_cmd", 10,
            std::bind(&AtomDdsClient::upperLimbCmdCallback, this, std::placeholders::_1));
        
        // 创建发布者 - 发布机器人状态
        fsm_state_pub_ = node_->create_publisher<std_msgs::msg::UInt32>("/atom/fsm_state", 10);
        connection_state_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/atom/connection_state", 10);
        
        // 创建服务服务器
        upper_limb_service_ = node_->create_service<std_srvs::srv::SetBool>(
            "/atom/upper_limb_service",
            std::bind(&AtomDdsClient::upperLimbServiceCallback, this, 
                     std::placeholders::_1, std::placeholders::_2));
        
        // 创建状态查询定时器
        status_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&AtomDdsClient::statusTimerCallback, this));
        
        // 启动执行器线程
        executor_thread_ = std::thread([this]() {
            executor_->spin();
        });
        
        // 连接到RPC服务器
        if (connectToRpcServer()) {
            // 启动RPC桥接线程
            rpc_bridge_thread_ = std::thread(&AtomDdsClient::rpcBridgeLoop, this);
            RCLCPP_INFO(node_->get_logger(), "Connected to RPC server at %s:%d", rpc_ip_.c_str(), rpc_port_);
        } else {
            RCLCPP_WARN(node_->get_logger(), "Failed to connect to RPC server, running in simulation mode");
        }
        
        is_initialized_ = true;
        RCLCPP_INFO(node_->get_logger(), "AtomDdsClient initialized successfully");
        
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to initialize AtomDdsClient: %s", e.what());
        return false;
    }
}

AtomRpcErrorCode AtomDdsClient::SwitchUpperLimbControl(bool enable) {
    if (!is_initialized_) {
        return AtomRpcErrorCode::CONNECTION_FAILED;
    }
    
    try {
        // 通过话题发布控制命令
        auto msg = std_msgs::msg::Bool();
        msg.data = enable;
        upper_limb_cmd_pub_->publish(msg);
        
        RCLCPP_INFO(node_->get_logger(), "Upper limb control %s via topic", 
                   enable ? "enabled" : "disabled");
        return AtomRpcErrorCode::SUCCESS;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to switch upper limb control: %s", e.what());
        return AtomRpcErrorCode::UNKNOWN_ERROR;
    }
}

AtomRpcErrorCode AtomDdsClient::GetFsmId(uint32_t& fsm_id) {
    if (!is_initialized_) {
        return AtomRpcErrorCode::CONNECTION_FAILED;
    }
    
    fsm_id = current_fsm_id_.load();
    return AtomRpcErrorCode::SUCCESS;
}

AtomRpcErrorCode AtomDdsClient::SetFsmId(uint32_t fsm_id) {
    if (!is_initialized_) {
        return AtomRpcErrorCode::CONNECTION_FAILED;
    }
    
    try {
        auto msg = std_msgs::msg::UInt32();
        msg.data = fsm_id;
        fsm_cmd_pub_->publish(msg);
        
        RCLCPP_INFO(node_->get_logger(), "FSM ID set to: %u", fsm_id);
        return AtomRpcErrorCode::SUCCESS;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to set FSM ID: %s", e.what());
        return AtomRpcErrorCode::UNKNOWN_ERROR;
    }
}

AtomRpcErrorCode AtomDdsClient::SetVel(float linear_x, float linear_y, float angular_z) {
    if (!is_initialized_) {
        return AtomRpcErrorCode::CONNECTION_FAILED;
    }
    
    try {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = linear_x;
        msg.linear.y = linear_y;
        msg.linear.z = 0.0;
        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = angular_z;
        
        vel_cmd_pub_->publish(msg);
        
        RCLCPP_DEBUG(node_->get_logger(), "Velocity set: linear_x=%.2f, linear_y=%.2f, angular_z=%.2f",
                    linear_x, linear_y, angular_z);
        return AtomRpcErrorCode::SUCCESS;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to set velocity: %s", e.what());
        return AtomRpcErrorCode::UNKNOWN_ERROR;
    }
}

bool AtomDdsClient::IsConnected() const {
    return is_connected_ && is_initialized_;
}

// ROS2话题回调函数
void AtomDdsClient::fsmCmdCallback(const std_msgs::msg::UInt32::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(rpc_mutex_);
    if (rpc_client_ && rpc_connected_) {
        try {
            auto result = rpc_client_->setFsmId(static_cast<int32_t>(msg->data));
            if (result == RpcErrorCode::SUCCESS) {
                RCLCPP_INFO(node_->get_logger(), "FSM state set to: %u", msg->data);
            } else {
                RCLCPP_WARN(node_->get_logger(), "Failed to set FSM state: %d", static_cast<int>(result));
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Exception in FSM command: %s", e.what());
        }
    }
}

void AtomDdsClient::velCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(rpc_mutex_);
    if (rpc_client_ && rpc_connected_) {
        try {
            auto result = rpc_client_->setVel(static_cast<float>(msg->linear.x), 
                                            static_cast<float>(msg->linear.y), 
                                            static_cast<float>(msg->angular.z));
            if (result != RpcErrorCode::SUCCESS) {
                RCLCPP_WARN(node_->get_logger(), "Failed to set velocity: %d", static_cast<int>(result));
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Exception in velocity command: %s", e.what());
        }
    }
}

void AtomDdsClient::upperLimbCmdCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(rpc_mutex_);
    if (rpc_client_ && rpc_connected_) {
        try {
            auto result = rpc_client_->switchUpperLimbControl(msg->data);
            if (result == RpcErrorCode::SUCCESS) {
                RCLCPP_INFO(node_->get_logger(), "Upper limb %s", msg->data ? "enabled" : "disabled");
            } else {
                RCLCPP_WARN(node_->get_logger(), "Failed to switch upper limb: %d", static_cast<int>(result));
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Exception in upper limb command: %s", e.what());
        }
    }
}

void AtomDdsClient::upperLimbServiceCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    
    std::lock_guard<std::mutex> lock(rpc_mutex_);
    if (rpc_client_ && rpc_connected_) {
        try {
            auto result = rpc_client_->switchUpperLimbControl(request->data);
            response->success = (result == RpcErrorCode::SUCCESS);
            response->message = response->success ? "Upper limb switched successfully" : "Failed to switch upper limb";
        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("Exception: ") + e.what();
        }
    } else {
        response->success = false;
        response->message = "RPC client not connected";
    }
}

void AtomDdsClient::statusTimerCallback() {
    // 发布连接状态
    auto connection_msg = std_msgs::msg::Bool();
    connection_msg.data = rpc_connected_;
    connection_state_pub_->publish(connection_msg);
    
    // 查询并发布FSM状态
    std::lock_guard<std::mutex> lock(rpc_mutex_);
    if (rpc_client_ && rpc_connected_) {
        try {
            int32_t fsm_id;
            auto result = rpc_client_->getFsmId(fsm_id);
            if (result == RpcErrorCode::SUCCESS) {
                current_fsm_id_ = static_cast<uint32_t>(fsm_id);
                auto fsm_msg = std_msgs::msg::UInt32();
                fsm_msg.data = static_cast<uint32_t>(fsm_id);
                fsm_state_pub_->publish(fsm_msg);
            }
        } catch (const std::exception& e) {
            RCLCPP_DEBUG(node_->get_logger(), "Exception in status query: %s", e.what());
        }
    }
}   
// RPC桥接函数实现
bool AtomDdsClient::connectToRpcServer() {
    if (!rpc_client_) {
        return false;
    }
    
    try {
        // 尝试连接RPC服务器
        auto result = rpc_client_->connect();
        
        if (result == RpcErrorCode::SUCCESS) {
            // 测试连接是否正常
            int32_t test_fsm_id;
            auto test_result = rpc_client_->getFsmId(test_fsm_id);
            
            if (test_result == RpcErrorCode::SUCCESS) {
                rpc_connected_ = true;
                is_connected_ = true;
                RCLCPP_INFO(node_->get_logger(), "Successfully connected to RPC server at %s:%d", rpc_ip_.c_str(), rpc_port_);
                return true;
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(), "Failed to connect to RPC server: %s", e.what());
    }
    
    rpc_connected_ = false;
    return false;
}

void AtomDdsClient::disconnectFromRpcServer() {
    rpc_connected_ = false;
    is_connected_ = false;
    
    if (rpc_client_) {
        rpc_client_->disconnect();
        RCLCPP_INFO(node_->get_logger(), "Disconnected from RPC server");
    }
}

void AtomDdsClient::rpcBridgeLoop() {
    RCLCPP_INFO(node_->get_logger(), "RPC bridge loop started");
    
    while (rpc_connected_ && is_initialized_) {
        try {
            // 检查RPC连接状态
            if (rpc_client_ && rpc_client_->isConnected()) {
                int32_t fsm_id;
                auto result = rpc_client_->getFsmId(fsm_id);
                
                if (result != RpcErrorCode::SUCCESS) {
                    // 连接丢失，尝试重连
                    RCLCPP_WARN(node_->get_logger(), "RPC connection lost, attempting to reconnect...");
                    rpc_connected_ = false;
                    is_connected_ = false;
                    
                    // 等待一段时间后重试
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    
                    if (connectToRpcServer()) {
                        RCLCPP_INFO(node_->get_logger(), "RPC connection restored");
                    }
                }
            } else {
                // 尝试重新连接
                RCLCPP_DEBUG(node_->get_logger(), "RPC client not connected, attempting to connect...");
                if (!connectToRpcServer()) {
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                }
            }
            
            // 休眠一段时间
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Exception in RPC bridge loop: %s", e.what());
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    
    RCLCPP_INFO(node_->get_logger(), "RPC bridge loop stopped");
}

AtomRpcErrorCode AtomDdsClient::convertRpcError(RpcErrorCode rpc_error_code) {
    switch (rpc_error_code) {
        case RpcErrorCode::SUCCESS: return AtomRpcErrorCode::SUCCESS;
        case RpcErrorCode::CONNECT_FAILED: 
        case RpcErrorCode::CONNECTION_FAILED: return AtomRpcErrorCode::CONNECTION_FAILED;
        case RpcErrorCode::TIMEOUT: return AtomRpcErrorCode::TIMEOUT;
        case RpcErrorCode::INVALID_PARAMETER: return AtomRpcErrorCode::INVALID_PARAMETER;
        case RpcErrorCode::SERVICE_UNAVAILABLE: return AtomRpcErrorCode::SERVICE_UNAVAILABLE;
        default: return AtomRpcErrorCode::UNKNOWN_ERROR;
    }
}



} // namespace atom_rpc_bridge