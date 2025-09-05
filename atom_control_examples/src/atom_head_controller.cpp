/**
 * @file atom_head_controller.cpp
 * @brief Atom机器人头部控制程序
 * @details 演示如何使用ROS2控制Atom机器人头部运动，包括摇头（偏航）和点头（俯仰）动作
 * @author futingxing
 * @date 2025-09-01
 * @version 1.0.0
 * @copyright Copyright (c) 2025 Dobot. All rights reserved.
 */
#include "rclcpp/rclcpp.hpp"
#include "dobot_atom/msg/upper_cmd.hpp"
#include "dobot_atom/msg/upper_state.hpp"
#include "dobot_atom/msg/motor_cmd.hpp"
#include "dobot_atom/msg/set_fsm_id.hpp"
#include "dobot_atom/msg/switch_upper_control.hpp"
#include <cmath>

#define HEAD_YAW_INDEX 14    
#define HEAD_PITCH_INDEX 15  

// Motion parameters
#define MOTION_FREQUENCY 1.0  // Hz, motion frequency
#define YAW_AMPLITUDE 0.5     // rad, head shake amplitude (±0.5 rad)
#define PITCH_AMPLITUDE 0.3   // rad, head nod amplitude (±0.3 rad)
#define CONTROL_FREQUENCY 50  // Hz, control loop frequency

using std::placeholders::_1;

class AtomHeadController : public rclcpp::Node
{
public:
    AtomHeadController() : Node("atom_head_controller")
    {
        // Publishers
        upper_cmd_pub_ = this->create_publisher<dobot_atom::msg::UpperCmd>("/upper/cmd", 10);
        fsm_pub_ = this->create_publisher<dobot_atom::msg::SetFsmId>("/set/fsm/id", 10);
        switch_control_pub_ = this->create_publisher<dobot_atom::msg::SwitchUpperControl>("/switch/upper/control", 10);
        
        // Subscriber to get current upper state
        upper_state_sub_ = this->create_subscription<dobot_atom::msg::UpperState>(
            "/upper/state", 10, std::bind(&AtomHeadController::upper_state_callback, this, _1));
        
        // Control timer
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / CONTROL_FREQUENCY), 
            std::bind(&AtomHeadController::control_callback, this));
        
        // Initialize variables
        time_count_ = 0.0;
        motion_mode_ = 0; // 0: shake head, 1: nod head, 2: stop
        state_received_ = false;
        
        RCLCPP_INFO(this->get_logger(), "Atom Head Controller Node Started");
        RCLCPP_INFO(this->get_logger(), "Publishing to topics: /upper/cmd, /set/fsm/id, /switch/upper/control");
        RCLCPP_INFO(this->get_logger(), "Control modes: 0=shake head, 1=nod head, 2=stop");
        
        // Initialize robot state
        initialize_robot();
    }

private:
    void initialize_robot()
    {
        // Wait a moment for connections
        rclcpp::sleep_for(std::chrono::milliseconds(1000));
        
        // Enable upper body control
        auto switch_msg = dobot_atom::msg::SwitchUpperControl();
        switch_msg.flag = true;
        switch_control_pub_->publish(switch_msg);
        RCLCPP_INFO(this->get_logger(), "Enabled upper body control");
        
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        
        // Set FSM to appropriate state (assuming 1 is control mode)
        auto fsm_msg = dobot_atom::msg::SetFsmId();
        fsm_msg.id = 1;
        fsm_pub_->publish(fsm_msg);
        RCLCPP_INFO(this->get_logger(), "Set FSM ID to 1");
        
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        
        // Initialize motor commands
        init_motor_commands();
    }
    
    void init_motor_commands()
    {
        for (int i = 0; i < 17; i++)
        {
            cmd_msg_.motor_cmd[i].mode = 1; 
            cmd_msg_.motor_cmd[i].q = 0.0;  
            cmd_msg_.motor_cmd[i].dq = 0.0; 
            cmd_msg_.motor_cmd[i].tau = 0.0;
            cmd_msg_.motor_cmd[i].kp = 20.0;
            cmd_msg_.motor_cmd[i].kd = 2.0; 
        }
        
        RCLCPP_INFO(this->get_logger(), "Initialized 17 motor commands (left_arm: 0-6, right_arm: 7-13, head: 14-15, torso: 16)");
    }
    
    void upper_state_callback(const dobot_atom::msg::UpperState::SharedPtr msg)
    {
        current_state_ = *msg;
        state_received_ = true;
        
        // Print current head joint positions occasionally
        static int print_counter = 0;
        if (++print_counter >= 100) // Print every 2 seconds at 50Hz
        {
            RCLCPP_INFO(this->get_logger(), "Head Yaw: %.3f rad, Head Pitch: %.3f rad", 
                       msg->motor_state[HEAD_YAW_INDEX].q, msg->motor_state[HEAD_PITCH_INDEX].q);
            print_counter = 0;
        }
    }
    
    void control_callback()
    {
        if (!state_received_)
        {
            return; // Wait for first state message
        }
        
        time_count_ += 1.0 / CONTROL_FREQUENCY;
        
        // Generate motion based on current mode
        switch (motion_mode_)
        {
            case 0: // Shake head (yaw motion)
                generate_shake_motion();
                break;
            case 1: // Nod head (pitch motion)
                generate_nod_motion();
                break;
            case 2: // Stop motion
                generate_stop_motion();
                break;
        }
        
        // Publish command
        upper_cmd_pub_->publish(cmd_msg_);
        
        // Switch motion mode every 5 seconds
        static double mode_switch_time = 0.0;
        mode_switch_time += 1.0 / CONTROL_FREQUENCY;
        if (mode_switch_time >= 5.0)
        {
            motion_mode_ = (motion_mode_ + 1) % 3;
            mode_switch_time = 0.0;
            
            switch (motion_mode_)
            {
                case 0:
                    RCLCPP_INFO(this->get_logger(), "Switching to SHAKE HEAD mode");
                    break;
                case 1:
                    RCLCPP_INFO(this->get_logger(), "Switching to NOD HEAD mode");
                    break;
                case 2:
                    RCLCPP_INFO(this->get_logger(), "Switching to STOP mode");
                    break;
            }
        }
    }
    
    void generate_shake_motion()
    {
        // Sinusoidal yaw motion (shake head left-right)
        double yaw_target = YAW_AMPLITUDE * sin(2.0 * M_PI * MOTION_FREQUENCY * time_count_);
        
        // Set head yaw command
        cmd_msg_.motor_cmd[HEAD_YAW_INDEX].q = yaw_target;
        cmd_msg_.motor_cmd[HEAD_YAW_INDEX].dq = YAW_AMPLITUDE * 2.0 * M_PI * MOTION_FREQUENCY * 
                                               cos(2.0 * M_PI * MOTION_FREQUENCY * time_count_);
        
        // Keep pitch at neutral
        cmd_msg_.motor_cmd[HEAD_PITCH_INDEX].q = 0.0;
        cmd_msg_.motor_cmd[HEAD_PITCH_INDEX].dq = 0.0;
    }
    
    void generate_nod_motion()
    {
        // Sinusoidal pitch motion (nod head up-down)
        double pitch_target = PITCH_AMPLITUDE * sin(2.0 * M_PI * MOTION_FREQUENCY * time_count_);
        
        // Set head pitch command
        cmd_msg_.motor_cmd[HEAD_PITCH_INDEX].q = pitch_target;
        cmd_msg_.motor_cmd[HEAD_PITCH_INDEX].dq = PITCH_AMPLITUDE * 2.0 * M_PI * MOTION_FREQUENCY * 
                                                 cos(2.0 * M_PI * MOTION_FREQUENCY * time_count_);
        
        // Keep yaw at neutral
        cmd_msg_.motor_cmd[HEAD_YAW_INDEX].q = 0.0;
        cmd_msg_.motor_cmd[HEAD_YAW_INDEX].dq = 0.0;
    }
    
    void generate_stop_motion()
    {
        // Return to neutral position
        cmd_msg_.motor_cmd[HEAD_YAW_INDEX].q = 0.0;
        cmd_msg_.motor_cmd[HEAD_YAW_INDEX].dq = 0.0;
        cmd_msg_.motor_cmd[HEAD_PITCH_INDEX].q = 0.0;
        cmd_msg_.motor_cmd[HEAD_PITCH_INDEX].dq = 0.0;
    }
    
    // Publishers
    rclcpp::Publisher<dobot_atom::msg::UpperCmd>::SharedPtr upper_cmd_pub_;
    rclcpp::Publisher<dobot_atom::msg::SetFsmId>::SharedPtr fsm_pub_;
    rclcpp::Publisher<dobot_atom::msg::SwitchUpperControl>::SharedPtr switch_control_pub_;
    
    // Subscriber
    rclcpp::Subscription<dobot_atom::msg::UpperState>::SharedPtr upper_state_sub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // State variables
    dobot_atom::msg::UpperCmd cmd_msg_;
    dobot_atom::msg::UpperState current_state_;
    double time_count_;
    int motion_mode_;
    bool state_received_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);                                      // Initialize rclcpp
    auto node = std::make_shared<AtomHeadController>();            // Create ROS2 node
    
    RCLCPP_INFO(node->get_logger(), "Starting Atom Head Controller...");
    RCLCPP_INFO(node->get_logger(), "The robot will cycle through: shake head -> nod head -> stop every 5 seconds");
    RCLCPP_INFO(node->get_logger(), "Press Ctrl+C to stop");
    
    rclcpp::spin(node);                                           // Run ROS2 node
    rclcpp::shutdown();                                           // Exit
    return 0;
}