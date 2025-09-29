/**
 * @file atom_high_level_cmd.hpp
 * @brief Atom高层命令定义头文件
 * @author futingxing
 * @date 2025-09-01
 * @version 1.0.0
 * @copyright Copyright (c) 2025 Dobot. All rights reserved.
 */

#ifndef ATOM_HIGH_LEVEL_CMD_HPP
#define ATOM_HIGH_LEVEL_CMD_HPP

#include <cstdint>
#include <array>

namespace atom_rpc_bridge {

/**
 * @brief FSM状态控制命令
 */
struct AtomFsmCmd {
    uint32_t fsm_id;        // FSM状态ID
    uint32_t timestamp;     // 时间戳
    uint8_t reserved[8];    // 保留字段
};

/**
 * @brief 速度控制命令
 */
struct AtomVelocityCmd {
    float linear_x;         // 前进速度 (m/s)
    float linear_y;         // 侧向速度 (m/s)
    float angular_z;        // 旋转速度 (rad/s)
    uint32_t timestamp;     // 时间戳
    uint8_t reserved[4];    // 保留字段
};

/**
 * @brief 上肢控制开关命令
 */
struct AtomUpperLimbCmd {
    bool enable;            // 是否启用上肢控制
    uint32_t timestamp;     // 时间戳
    uint8_t reserved[7];    // 保留字段
};

/**
 * @brief FSM状态查询响应
 */
struct AtomFsmState {
    uint32_t current_fsm_id;    // 当前FSM状态ID
    uint32_t timestamp;         // 时间戳
    bool is_valid;              // 状态是否有效
    uint8_t reserved[7];        // 保留字段
};

/**
 * @brief 通用响应结构
 */
struct AtomResponse {
    int32_t error_code;     // 错误码 (0表示成功)
    uint32_t timestamp;     // 时间戳
    char message[64];       // 响应消息
};

} // namespace atom_rpc_bridge

#endif // ATOM_HIGH_LEVEL_CMD_HPP