/**
 * @file atom_dds_client.hpp
 * @brief Atom DDS客户端头文件
 * @author futingxing
 * @date 2025-09-01
 * @version 1.0.0
 * @copyright Copyright (c) 2025 Dobot. All rights reserved.
 */

#ifndef ATOM_DDS_CLIENT_HPP
#define ATOM_DDS_CLIENT_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <memory>
#include <future>
#include <thread>
#include <atomic>
#include <mutex>
#include "atom_high_level_cmd.hpp"

// 包含独立RPC客户端头文件
#include "standalone_rpc_client.hpp"

namespace atom_rpc_bridge {

/**
 * @brief RPC错误码枚举
 */
enum class AtomRpcErrorCode : int32_t {
    SUCCESS = 0,
    TIMEOUT = -1,
    CONNECTION_FAILED = -2,
    INVALID_PARAMETER = -3,
    SERVICE_UNAVAILABLE = -4,
    UNKNOWN_ERROR = -5
};

/**
 * @brief Atom机器人DDS桥接客户端类
 * 作为RPC到DDS的中间层，连接原RPC服务器并提供ROS2话题接口
 */
class AtomDdsClient {
public:
    /**
     * @brief 构造函数
     * @param node_name 节点名称
     * @param rpc_ip RPC服务器IP地址
     * @param rpc_port RPC服务器端口
     */
    explicit AtomDdsClient(const std::string& node_name = "atom_dds_client",
                          const std::string& rpc_ip = "192.168.8.234",
                          int rpc_port = 51234);
    
    /**
     * @brief 析构函数
     */
    ~AtomDdsClient();

    /**
     * @brief 初始化DDS客户端
     * @return 是否初始化成功
     */
    bool initialize();

    /**
     * @brief 切换上肢控制状态
     * @param enable 是否启用上肢控制
     * @return 错误码
     */
    AtomRpcErrorCode SwitchUpperLimbControl(bool enable);

    /**
     * @brief 获取当前FSM状态ID
     * @param fsm_id 输出参数，当前FSM状态ID
     * @return 错误码
     */
    AtomRpcErrorCode GetFsmId(uint32_t& fsm_id);

    /**
     * @brief 设置FSM状态ID
     * @param fsm_id 目标FSM状态ID
     * @return 错误码
     */
    AtomRpcErrorCode SetFsmId(uint32_t fsm_id);

    /**
     * @brief 设置机器人速度
     * @param linear_x 前进速度 (m/s)
     * @param linear_y 侧向速度 (m/s)
     * @param angular_z 旋转速度 (rad/s)
     * @return 错误码
     */
    AtomRpcErrorCode SetVel(float linear_x, float linear_y, float angular_z);

    /**
     * @brief 检查DDS连接状态
     * @return 是否连接正常
     */
    bool IsConnected() const;

private:
    // ROS2相关
    rclcpp::Node::SharedPtr node_;                              // ROS2节点
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;  // 执行器
    std::thread executor_thread_;                               // 执行器线程
    
    // RPC相关
    std::unique_ptr<StandaloneRpcClient> rpc_client_;           // 独立RPC客户端
    std::string rpc_ip_;                                        // RPC服务器IP
    int rpc_port_;                                              // RPC服务器端口
    std::atomic<bool> rpc_connected_;                           // RPC连接状态
    std::thread rpc_bridge_thread_;                             // RPC桥接线程
    std::mutex rpc_mutex_;                                      // RPC操作互斥锁
    
    // 发布者 - 用于向机器人发送命令
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr fsm_cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr upper_limb_cmd_pub_;
    
    // 订阅者 - 用于接收外部控制命令
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr fsm_cmd_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr upper_limb_cmd_sub_;
    
    // 发布者 - 用于发布机器人状态
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr fsm_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connection_state_pub_;
    
    // 服务服务器
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr upper_limb_service_;
    
    // 状态变量
    std::atomic<uint32_t> current_fsm_id_;
    std::atomic<bool> is_connected_;
    std::atomic<bool> is_initialized_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr status_timer_;                 // 状态查询定时器
    
    // 回调函数
    void fsmCmdCallback(const std_msgs::msg::UInt32::SharedPtr msg);
    void velCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void upperLimbCmdCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void upperLimbServiceCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                 std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    void statusTimerCallback();
    
    // RPC桥接函数
    void rpcBridgeLoop();
    bool connectToRpcServer();
    void disconnectFromRpcServer();
    
    // 工具函数
    AtomRpcErrorCode convertRpcError(RpcErrorCode rpc_error_code);
    
    // 模板函数声明和实现
    template<typename ServiceT>
    AtomRpcErrorCode callService(
        typename rclcpp::Client<ServiceT>::SharedPtr client,
        typename ServiceT::Request::SharedPtr request,
        std::chrono::milliseconds timeout = std::chrono::milliseconds(1000)) {
        
        try {
            auto future = client->async_send_request(request);
            
            if (future.wait_for(timeout) == std::future_status::ready) {
                auto response = future.get();
                if (response->success) {
                    return AtomRpcErrorCode::SUCCESS;
                } else {
                    RCLCPP_WARN(node_->get_logger(), "Service call failed: %s", response->message.c_str());
                    return AtomRpcErrorCode::UNKNOWN_ERROR;
                }
            } else {
                RCLCPP_WARN(node_->get_logger(), "Service call timeout");
                return AtomRpcErrorCode::TIMEOUT;
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Service call exception: %s", e.what());
            return AtomRpcErrorCode::UNKNOWN_ERROR;
        }
    }
};

} // namespace atom_rpc_bridge

#endif // ATOM_DDS_CLIENT_HPP