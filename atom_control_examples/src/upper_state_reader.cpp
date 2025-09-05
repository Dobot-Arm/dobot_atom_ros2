/**
 * @file upper_state_reader.cpp
 * @brief Atom机器人上肢状态读取程序
 * @details 读取并显示上肢17个电机状态（机械臂+头部+躯干）、BMS电池信息
 * @author futingxing
 * @date 2025-09-01
 * @version 1.0.0
 * @copyright Copyright (c) 2025 Dobot. All rights reserved.
 */

#include "rclcpp/rclcpp.hpp"
#include "dobot_atom/msg/upper_state.hpp"

// 信息打印控制开关
const bool INFO_UPPER_STATE = true;

using std::placeholders::_1;

class UpperStateReader : public rclcpp::Node
{
public:
    UpperStateReader() : Node("upper_state_reader")
    {
        // Subscribe to upper state topic
        upper_state_sub_ = this->create_subscription<dobot_atom::msg::UpperState>(
            "/upper/state", 10, std::bind(&UpperStateReader::upper_state_callback, this, _1));
        
        RCLCPP_INFO(this->get_logger(), "Upper State Reader Node Started");
        RCLCPP_INFO(this->get_logger(), "Subscribing to topic: /upper/state");
    }

private:
    void upper_state_callback(const dobot_atom::msg::UpperState::SharedPtr msg)
    {
        if (INFO_UPPER_STATE)
        {
            RCLCPP_INFO(this->get_logger(), "=== Upper State ===");
            RCLCPP_INFO(this->get_logger(), "Upper Control: %s, FSM ID: %d", 
                       msg->is_upper_control ? "Enabled" : "Disabled", msg->fsm_id);
            
            // Motor States (arms and head - 17 motors total)
            for (int i = 0; i < 17; i++)
            {
                RCLCPP_INFO(this->get_logger(), "Upper Motor[%d] - Mode: %d, Pos: %.3f, Vel: %.3f, Tau: %.3f, Temp: %d°C",
                           i, msg->motor_state[i].mode, msg->motor_state[i].q, 
                           msg->motor_state[i].dq, msg->motor_state[i].tau_est, msg->motor_state[i].motor_temp);
            }
            
            // BMS State
            RCLCPP_INFO(this->get_logger(), "Upper BMS - SOC: %d%%, Voltage: %dV, Current: %dA, Temp: %d°C",
                       msg->bms_state.battery_level, msg->bms_state.battery_pack_current_voltage, 
                       msg->bms_state.battery_now_current, msg->bms_state.pcb_board_temp);
        }
    }
    
    // Subscriber
    rclcpp::Subscription<dobot_atom::msg::UpperState>::SharedPtr upper_state_sub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);                                    // Initialize rclcpp
    rclcpp::spin(std::make_shared<UpperStateReader>());          // Run ROS2 node
    rclcpp::shutdown();
    return 0;
}