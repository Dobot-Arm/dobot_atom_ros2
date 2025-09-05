/**
 * @file hands_state_reader.cpp
 * @brief Atom机器人灵巧手状态读取程序
 * @details 读取并显示灵巧手12个手指电机状态，包含左右手各6个自由度的详细信息
 * @author futingxing
 * @date 2025-09-01
 * @version 1.0.0
 * @copyright Copyright (c) 2025 Dobot. All rights reserved.
 */

#include "rclcpp/rclcpp.hpp"
#include "dobot_atom/msg/hands_state.hpp"

// 信息打印控制开关
const bool INFO_HANDS_STATE = true;

using std::placeholders::_1;

class HandsStateReader : public rclcpp::Node
{
public:
    HandsStateReader() : Node("hands_state_reader")
    {
        // Subscribe to hands state topic
        hands_state_sub_ = this->create_subscription<dobot_atom::msg::HandsState>(
            "/hands/state", 10, std::bind(&HandsStateReader::hands_state_callback, this, _1));
        
        RCLCPP_INFO(this->get_logger(), "Hands State Reader Node Started");
        RCLCPP_INFO(this->get_logger(), "Subscribing to topic: /hands/state");
    }

private:
    void hands_state_callback(const dobot_atom::msg::HandsState::SharedPtr msg)
    {
        if (INFO_HANDS_STATE)
        {
            RCLCPP_INFO(this->get_logger(), "=== Hands State ===");
            
            // Dexterous hand motors (12 fingers)
            // 根据灵巧手数组排序：拇指、食指、中指、无名指、小指（每只手5个手指 + 2个额外自由度）
            const char* finger_names[12] = {
                "L_Thumb", "L_Index", "L_Middle", "L_Ring", "L_Pinky", "L_Extra1",
                "R_Thumb", "R_Index", "R_Middle", "R_Ring", "R_Pinky", "R_Extra1"
            };
            
            RCLCPP_INFO(this->get_logger(), "--- Left Hand ---");
            for (int i = 0; i < 6; i++)
            {
                RCLCPP_INFO(this->get_logger(), "%s[%d] - Mode: %d, Pos: %.3f, Vel: %.3f, Tau: %.3f, Temp: %d°C",
                           finger_names[i], i, msg->hands[i].mode, msg->hands[i].q, 
                           msg->hands[i].dq, msg->hands[i].tau_est, msg->hands[i].motor_temp);
            }
            
            RCLCPP_INFO(this->get_logger(), "--- Right Hand ---");
            for (int i = 6; i < 12; i++)
            {
                RCLCPP_INFO(this->get_logger(), "%s[%d] - Mode: %d, Pos: %.3f, Vel: %.3f, Tau: %.3f, Temp: %d°C",
                           finger_names[i], i, msg->hands[i].mode, msg->hands[i].q, 
                           msg->hands[i].dq, msg->hands[i].tau_est, msg->hands[i].motor_temp);
            }
        }
    }
    
    // Subscriber
    rclcpp::Subscription<dobot_atom::msg::HandsState>::SharedPtr hands_state_sub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);                                    // Initialize rclcpp
    rclcpp::spin(std::make_shared<HandsStateReader>());          // Run ROS2 node
    rclcpp::shutdown();
    return 0;
}