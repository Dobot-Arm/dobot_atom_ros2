/**
 * @file standalone_rpc_client.hpp
 * @brief 独立RPC客户端头文件
 * @author futingxing
 * @date 2025-09-01
 * @version 1.0.0
 * @copyright Copyright (c) 2025 Dobot. All rights reserved.
 */

#ifndef STANDALONE_RPC_CLIENT_HPP
#define STANDALONE_RPC_CLIENT_HPP

#include <string>
#include <memory>
#include <functional>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <chrono>

namespace atom_rpc_bridge {

/**
 * @brief RPC错误码枚举
 */
enum class RpcErrorCode : int32_t {
    SUCCESS = 0,
    SOCKET_CREATION_FAILED = 1,
    CONNECT_FAILED = 2,
    NO_RESPONSE = 3,
    JSON_PARSE_ERROR = 4,
    UNEXPECTED_RESPONSE = 5,
    SERVER_INTERNAL_ERROR = 6,
    FSM_TRANSITION_FAILED = 100,
    SWITCH_UPPER_LIMB_CONTROL_FAILED = 101,
    SET_VEL_FAILED = 102,
    TIMEOUT = -1,
    CONNECTION_FAILED = -2,
    INVALID_PARAMETER = -3,
    SERVICE_UNAVAILABLE = -4,
    UNKNOWN_ERROR = -5
};

/**
 * @brief 独立的RPC客户端实现
 * 不依赖外部atom_sdk库，提供基本的RPC通信功能
 */
class StandaloneRpcClient {
public:
    /**
     * @brief 构造函数
     * @param ip RPC服务器IP地址
     * @param port RPC服务器端口
     */
    explicit StandaloneRpcClient(const std::string& ip = "192.168.8.234", int port = 51234);
    
    /**
     * @brief 析构函数
     */
    ~StandaloneRpcClient();
    
    /**
     * @brief 连接到RPC服务器
     * @return 错误码
     */
    RpcErrorCode connect();
    
    /**
     * @brief 断开RPC连接
     */
    void disconnect();
    
    /**
     * @brief 检查连接状态
     * @return 是否已连接
     */
    bool isConnected() const { return connected_.load(); }
    
    /**
     * @brief 切换上肢控制
     * @param is_on 是否启用
     * @return 错误码
     */
    RpcErrorCode switchUpperLimbControl(bool is_on);
    
    /**
     * @brief 获取FSM状态ID
     * @param fsm_id 输出的FSM ID
     * @return 错误码
     */
    RpcErrorCode getFsmId(int32_t& fsm_id);
    
    /**
     * @brief 设置FSM状态ID
     * @param fsm_id FSM ID
     * @return 错误码
     */
    RpcErrorCode setFsmId(int32_t fsm_id);
    
    /**
     * @brief 设置速度命令
     * @param vx 前后速度 (m/s)
     * @param vy 左右速度 (m/s)
     * @param vyaw 旋转速度 (rad/s)
     * @param duration 持续时间 (s)
     * @return 错误码
     */
    RpcErrorCode setVel(float vx, float vy, float vyaw, float duration = 1.0f);
    
private:
    /**
     * @brief 发送RPC请求
     * @param method 方法名
     * @param params 参数JSON字符串
     * @param response 响应JSON字符串
     * @param timeout_ms 超时时间(毫秒)
     * @return 错误码
     */
    RpcErrorCode sendRequest(const std::string& method, 
                           const std::string& params,
                           std::string& response,
                           int timeout_ms = 5000);
    
    /**
     * @brief 创建JSON-RPC请求
     * @param method 方法名
     * @param params 参数
     * @return JSON字符串
     */
    std::string createJsonRpcRequest(const std::string& method, const std::string& params);
    
    /**
     * @brief 解析JSON-RPC响应
     * @param response 响应字符串
     * @param result 解析结果
     * @return 是否成功
     */
    bool parseJsonRpcResponse(const std::string& response, std::string& result);
    
    std::string ip_;                    // 服务器IP
    int port_;                          // 服务器端口
    std::atomic<bool> connected_;       // 连接状态
    std::atomic<int> request_id_;       // 请求ID计数器
    mutable std::mutex mutex_;          // 互斥锁
    
    // 禁用拷贝构造和赋值
    StandaloneRpcClient(const StandaloneRpcClient&) = delete;
    StandaloneRpcClient& operator=(const StandaloneRpcClient&) = delete;
};

} // namespace atom_rpc_bridge

#endif // STANDALONE_RPC_CLIENT_HPP