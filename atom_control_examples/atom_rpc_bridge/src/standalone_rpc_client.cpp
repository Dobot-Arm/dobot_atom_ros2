/**
 * @file standalone_rpc_client.cpp
 * @brief 独立RPC客户端实现
 * @author futingxing
 * @date 2025-09-01
 * @version 1.0.0
 * @copyright Copyright (c) 2025 Dobot. All rights reserved.
 */

#include "atom_rpc_bridge/standalone_rpc_client.hpp"
#include <sstream>
#include <iostream>
#include <cstring>

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
    #define close closesocket
    #define SHUT_RDWR SD_BOTH
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    #include <fcntl.h>
    #include <errno.h>
#endif

namespace atom_rpc_bridge {

StandaloneRpcClient::StandaloneRpcClient(const std::string& ip, int port)
    : ip_(ip)
    , port_(port)
    , connected_(false)
    , request_id_(1) {
#ifdef _WIN32
    // 初始化Winsock
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif
}

StandaloneRpcClient::~StandaloneRpcClient() {
    disconnect();
#ifdef _WIN32
    // 清理Winsock
    WSACleanup();
#endif
}

RpcErrorCode StandaloneRpcClient::connect() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (connected_.load()) {
        return RpcErrorCode::SUCCESS;
    }
    
    // 这里只是模拟连接，实际的socket连接在每次请求时建立
    // 因为RPC通常是短连接模式
    connected_.store(true);
    
    std::cout << "[StandaloneRpcClient] Connected to RPC server at " << ip_ << ":" << port_ << std::endl;
    return RpcErrorCode::SUCCESS;
}

void StandaloneRpcClient::disconnect() {
    std::lock_guard<std::mutex> lock(mutex_);
    connected_.store(false);
    std::cout << "[StandaloneRpcClient] Disconnected from RPC server" << std::endl;
}

RpcErrorCode StandaloneRpcClient::switchUpperLimbControl(bool is_on) {
    if (!connected_.load()) {
        return RpcErrorCode::CONNECTION_FAILED;
    }
    
    std::string params = "{\"is_on\": " + std::string(is_on ? "true" : "false") + "}";
    std::string response;
    
    RpcErrorCode result = sendRequest("SwitchUpperLimbControl", params, response);
    if (result != RpcErrorCode::SUCCESS) {
        std::cerr << "[StandaloneRpcClient] Failed to switch upper limb control: " << static_cast<int>(result) << std::endl;
        return RpcErrorCode::SWITCH_UPPER_LIMB_CONTROL_FAILED;
    }
    
    return RpcErrorCode::SUCCESS;
}

RpcErrorCode StandaloneRpcClient::getFsmId(int32_t& fsm_id) {
    if (!connected_.load()) {
        return RpcErrorCode::CONNECTION_FAILED;
    }
    
    std::string params = "{}";
    std::string response;
    
    RpcErrorCode result = sendRequest("GetFsmId", params, response);
    if (result != RpcErrorCode::SUCCESS) {
        std::cerr << "[StandaloneRpcClient] Failed to get FSM ID: " << static_cast<int>(result) << std::endl;
        return result;
    }
    
    // 简单解析响应中的fsm_id
    // 实际实现中应该使用JSON解析库
    try {
        size_t pos = response.find("\"fsm_id\":");
        if (pos != std::string::npos) {
            pos += 9; // 跳过"fsm_id":
            size_t end_pos = response.find_first_of(",}", pos);
            if (end_pos != std::string::npos) {
                std::string fsm_str = response.substr(pos, end_pos - pos);
                fsm_id = std::stoi(fsm_str);
                return RpcErrorCode::SUCCESS;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "[StandaloneRpcClient] Failed to parse FSM ID from response: " << e.what() << std::endl;
    }
    
    return RpcErrorCode::JSON_PARSE_ERROR;
}

RpcErrorCode StandaloneRpcClient::setFsmId(int32_t fsm_id) {
    if (!connected_.load()) {
        return RpcErrorCode::CONNECTION_FAILED;
    }
    
    std::string params = "{\"fsm_id\": " + std::to_string(fsm_id) + "}";
    std::string response;
    
    RpcErrorCode result = sendRequest("SetFsmId", params, response);
    if (result != RpcErrorCode::SUCCESS) {
        std::cerr << "[StandaloneRpcClient] Failed to set FSM ID: " << static_cast<int>(result) << std::endl;
        return RpcErrorCode::FSM_TRANSITION_FAILED;
    }
    
    return RpcErrorCode::SUCCESS;
}

RpcErrorCode StandaloneRpcClient::setVel(float vx, float vy, float vyaw, float duration) {
    if (!connected_.load()) {
        return RpcErrorCode::CONNECTION_FAILED;
    }
    
    std::ostringstream params_stream;
    params_stream << "{\"vx\": " << vx 
                  << ", \"vy\": " << vy 
                  << ", \"vyaw\": " << vyaw 
                  << ", \"duration\": " << duration << "}";
    
    std::string response;
    RpcErrorCode result = sendRequest("SetVel", params_stream.str(), response);
    if (result != RpcErrorCode::SUCCESS) {
        std::cerr << "[StandaloneRpcClient] Failed to set velocity: " << static_cast<int>(result) << std::endl;
        return RpcErrorCode::SET_VEL_FAILED;
    }
    
    return RpcErrorCode::SUCCESS;
}

RpcErrorCode StandaloneRpcClient::sendRequest(const std::string& method, 
                                            const std::string& params,
                                            std::string& response,
                                            int timeout_ms) {
    // 创建socket
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
#ifdef _WIN32
    if (sockfd == INVALID_SOCKET) {
        std::cerr << "[StandaloneRpcClient] Failed to create socket: " << WSAGetLastError() << std::endl;
        return RpcErrorCode::SOCKET_CREATION_FAILED;
    }
#else
    if (sockfd < 0) {
        std::cerr << "[StandaloneRpcClient] Failed to create socket: " << strerror(errno) << std::endl;
        return RpcErrorCode::SOCKET_CREATION_FAILED;
    }
#endif
    
    // 设置超时
    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    
    // 连接到服务器
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port_);
    
#ifdef _WIN32
    server_addr.sin_addr.s_addr = inet_addr(ip_.c_str());
    if (server_addr.sin_addr.s_addr == INADDR_NONE) {
        std::cerr << "[StandaloneRpcClient] Invalid IP address: " << ip_ << std::endl;
        close(sockfd);
        return RpcErrorCode::CONNECT_FAILED;
    }
#else
    if (inet_pton(AF_INET, ip_.c_str(), &server_addr.sin_addr) <= 0) {
        std::cerr << "[StandaloneRpcClient] Invalid IP address: " << ip_ << std::endl;
        close(sockfd);
        return RpcErrorCode::CONNECT_FAILED;
    }
#endif
    
    if (::connect(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
#ifdef _WIN32
        std::cerr << "[StandaloneRpcClient] Failed to connect to server: " << WSAGetLastError() << std::endl;
#else
        std::cerr << "[StandaloneRpcClient] Failed to connect to server: " << strerror(errno) << std::endl;
#endif
        close(sockfd);
        return RpcErrorCode::CONNECT_FAILED;
    }
    
    // 创建JSON-RPC请求
    std::string request = createJsonRpcRequest(method, params);
    
    // 发送请求
    int sent = send(sockfd, request.c_str(), static_cast<int>(request.length()), 0);
    if (sent < 0) {
#ifdef _WIN32
        std::cerr << "[StandaloneRpcClient] Failed to send request: " << WSAGetLastError() << std::endl;
#else
        std::cerr << "[StandaloneRpcClient] Failed to send request: " << strerror(errno) << std::endl;
#endif
        close(sockfd);
        return RpcErrorCode::NO_RESPONSE;
    }
    
    // 接收响应
    char buffer[4096];
    int received = recv(sockfd, buffer, sizeof(buffer) - 1, 0);
    if (received < 0) {
#ifdef _WIN32
        std::cerr << "[StandaloneRpcClient] Failed to receive response: " << WSAGetLastError() << std::endl;
#else
        std::cerr << "[StandaloneRpcClient] Failed to receive response: " << strerror(errno) << std::endl;
#endif
        close(sockfd);
        return RpcErrorCode::NO_RESPONSE;
    }
    
    buffer[received] = '\0';
    response = std::string(buffer);
    
    close(sockfd);
    
    // 解析响应
    std::string result;
    if (!parseJsonRpcResponse(response, result)) {
        return RpcErrorCode::JSON_PARSE_ERROR;
    }
    
    return RpcErrorCode::SUCCESS;
}

std::string StandaloneRpcClient::createJsonRpcRequest(const std::string& method, const std::string& params) {
    int id = request_id_.fetch_add(1);
    
    std::ostringstream request;
    request << "{\"jsonrpc\": \"2.0\", "
            << "\"method\": \"" << method << "\", "
            << "\"params\": " << params << ", "
            << "\"id\": " << id << "}";
    
    return request.str();
}

bool StandaloneRpcClient::parseJsonRpcResponse(const std::string& response, std::string& result) {
    // 简单的JSON解析，实际应该使用专业的JSON库
    size_t result_pos = response.find("\"result\":");
    if (result_pos == std::string::npos) {
        // 检查是否有错误
        size_t error_pos = response.find("\"error\":");
        if (error_pos != std::string::npos) {
            std::cerr << "[StandaloneRpcClient] RPC error in response: " << response << std::endl;
            return false;
        }
        return false;
    }
    
    result = response;
    return true;
}

} // namespace atom_rpc_bridge