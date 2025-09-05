/**
 * @file lower_state_reader.cpp
 * @brief Atom机器人下肢状态读取程序
 * @details 读取并显示下肢12个电机状态、IMU数据、BMS电池信息
 * @author futingxing
 * @date 2025-09-01
 * @version 1.0.0
 * @copyright Copyright (c) 2025 Dobot. All rights reserved.
 */
#include "rclcpp/rclcpp.hpp"
#include "dobot_atom/msg/lower_state.hpp"
#include "dobot_atom/msg/upper_state.hpp"
#include "dobot_atom/msg/main_nodes_state.hpp"
#include "dobot_atom/msg/hands_state.hpp"
#include "dobot_atom/msg/motor_state.hpp"
#include "dobot_atom/msg/imu_state.hpp"
#include "dobot_atom/msg/bms_state.hpp"

#define INFO_LOWER_STATE 1     // Set 1 to info lower states
#define INFO_UPPER_STATE 1     // Set 1 to info upper states
#define INFO_MAIN_NODES 1      // Set 1 to info main nodes states
#define INFO_HANDS_STATE 1     // Set 1 to info hands states

using std::placeholders::_1;

class LowerStateReader : public rclcpp::Node
{
public:
    LowerStateReader() : Node("lower_state_reader")
    {
        // Subscribe to lower state topic
        lower_state_sub_ = this->create_subscription<dobot_atom::msg::LowerState>(
            "/lower/state", 10, std::bind(&LowerStateReader::lower_state_callback, this, _1));
        
        RCLCPP_INFO(this->get_logger(), "Lower State Reader Node Started");
        RCLCPP_INFO(this->get_logger(), "Subscribing to topic: /lower/state");
        

    }

private:
    void lower_state_callback(const dobot_atom::msg::LowerState::SharedPtr msg)
    {
        if (INFO_LOWER_STATE)
        {
            RCLCPP_INFO(this->get_logger(), "=== Lower State ===");
            
            // IMU State
            RCLCPP_INFO(this->get_logger(), "IMU - RPY: [%.3f, %.3f, %.3f], Temp: %d°C",
                       msg->imu_state.rpy[0], msg->imu_state.rpy[1], msg->imu_state.rpy[2], msg->imu_state.temperature);
            
            // Motor States (first 6 motors for legs)
            for (int i = 0; i < 6 && i < 12; i++)
            {
                RCLCPP_INFO(this->get_logger(), "Motor[%d] - Mode: %d, Pos: %.3f, Vel: %.3f, Tau: %.3f, Temp: %d°C",
                           i, msg->motor_state[i].mode, msg->motor_state[i].q, 
                           msg->motor_state[i].dq, msg->motor_state[i].tau_est, msg->motor_state[i].motor_temp);
            }
            
            // BMS State
            RCLCPP_INFO(this->get_logger(), "BMS - SOC: %d%%, Voltage: %dV, Current: %dA, Temp: %d°C",
                       msg->bms_state.battery_level, msg->bms_state.battery_pack_current_voltage, 
                       msg->bms_state.battery_now_current, msg->bms_state.pcb_board_temp);
        }
    }
    

    
    // Subscriber
    rclcpp::Subscription<dobot_atom::msg::LowerState>::SharedPtr lower_state_sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);                                    // Initialize rclcpp
    rclcpp::spin(std::make_shared<LowerStateReader>());          // Run ROS2 node
    rclcpp::shutdown();
    return 0;
}