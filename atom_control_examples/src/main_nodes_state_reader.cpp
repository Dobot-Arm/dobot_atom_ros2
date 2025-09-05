/**
 * @file main_nodes_state_reader.cpp
 * @brief Atom机器人关节CAN板状态读取程序
 * @details 读取并显示关节CAN板状态和EtherCAT从站信息，包含各部位伺服状态、错误码等
 * @author futingxing
 * @date 2025-09-01
 * @version 1.0.0
 * @copyright Copyright (c) 2025 Dobot. All rights reserved.
 */

#include "rclcpp/rclcpp.hpp"
#include "dobot_atom/msg/main_nodes_state.hpp"

// 信息打印控制开关
const bool INFO_MAIN_NODES = true;

using std::placeholders::_1;

class MainNodesStateReader : public rclcpp::Node
{
public:
    MainNodesStateReader() : Node("main_nodes_state_reader")
    {
        // Subscribe to main nodes state topic
        main_nodes_sub_ = this->create_subscription<dobot_atom::msg::MainNodesState>(
            "/main/nodes/state", 10, std::bind(&MainNodesStateReader::main_nodes_callback, this, _1));
        
        RCLCPP_INFO(this->get_logger(), "Main Nodes State Reader Node Started");
        RCLCPP_INFO(this->get_logger(), "Subscribing to topic: /main/nodes/state");
    }

private:
    void main_nodes_callback(const dobot_atom::msg::MainNodesState::SharedPtr msg)
    {
        if (INFO_MAIN_NODES)
        {
            RCLCPP_INFO(this->get_logger(), "=== Main Nodes State (CAN Board Status) ===");
            
            // Left Leg
            RCLCPP_INFO(this->get_logger(), "--- Left Leg ---");
            for (int i = 0; i < 6; i++)
            {
                RCLCPP_INFO(this->get_logger(), "LeftLeg[%d] - Error: 0x%X, Warn: 0x%X, NodeState: %d, OpMode: %d",
                           i, msg->left_leg[i].error_code, 
                           msg->left_leg[i].warn_code, msg->left_leg[i].node_state, msg->left_leg[i].display_op_mode);
            }
            
            // Right Leg
            RCLCPP_INFO(this->get_logger(), "--- Right Leg ---");
            for (int i = 0; i < 6; i++)
            {
                RCLCPP_INFO(this->get_logger(), "RightLeg[%d] - Error: 0x%X, Warn: 0x%X, NodeState: %d, OpMode: %d",
                           i, msg->right_leg[i].error_code, 
                           msg->right_leg[i].warn_code, msg->right_leg[i].node_state, msg->right_leg[i].display_op_mode);
            }
            
            // Waist (single joint)
            RCLCPP_INFO(this->get_logger(), "--- Waist ---");
            RCLCPP_INFO(this->get_logger(), "Waist - Error: 0x%X, Warn: 0x%X, NodeState: %d, OpMode: %d",
                       msg->waist.error_code, 
                       msg->waist.warn_code, msg->waist.node_state, msg->waist.display_op_mode);
            
            // Left Arm
            RCLCPP_INFO(this->get_logger(), "--- Left Arm ---");
            for (int i = 0; i < 7; i++)
            {
                RCLCPP_INFO(this->get_logger(), "LeftArm[%d] - Error: 0x%X, Warn: 0x%X, NodeState: %d, OpMode: %d",
                           i, msg->left_arm[i].error_code, 
                           msg->left_arm[i].warn_code, msg->left_arm[i].node_state, msg->left_arm[i].display_op_mode);
            }
            
            // Right Arm
            RCLCPP_INFO(this->get_logger(), "--- Right Arm ---");
            for (int i = 0; i < 7; i++)
            {
                RCLCPP_INFO(this->get_logger(), "RightArm[%d] - Error: 0x%X, Warn: 0x%X, NodeState: %d, OpMode: %d",
                           i, msg->right_arm[i].error_code, 
                           msg->right_arm[i].warn_code, msg->right_arm[i].node_state, msg->right_arm[i].display_op_mode);
            }
            
            // Head
            RCLCPP_INFO(this->get_logger(), "--- Head ---");
            for (int i = 0; i < 2; i++)
            {
                RCLCPP_INFO(this->get_logger(), "Head[%d] - Error: 0x%X, Warn: 0x%X, NodeState: %d, OpMode: %d",
                           i, msg->head[i].error_code, 
                           msg->head[i].warn_code, msg->head[i].node_state, msg->head[i].display_op_mode);
            }
            
            // EtherCAT Slave Info
            RCLCPP_INFO(this->get_logger(), "--- EtherCAT Slaves ---");
            for (int i = 0; i < 2; i++)
            {
                RCLCPP_INFO(this->get_logger(), "EtherCAT[%d] - Virtual: %s, State: %d, Error: 0x%X, SW Ver: %d",
                           i, msg->ecat2can[i].is_virtual ? "true" : "false", msg->ecat2can[i].slave_state,
                           msg->ecat2can[i].error_code, msg->ecat2can[i].software_version);
            }
        }
    }
    
    // Subscriber
    rclcpp::Subscription<dobot_atom::msg::MainNodesState>::SharedPtr main_nodes_sub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);                                    // Initialize rclcpp
    rclcpp::spin(std::make_shared<MainNodesStateReader>());      // Run ROS2 node
    rclcpp::shutdown();
    return 0;
}