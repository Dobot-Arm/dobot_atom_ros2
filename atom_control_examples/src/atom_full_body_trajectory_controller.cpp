/**
 * @file atom_full_body_trajectory_controller.cpp
 * @brief Atom机器人全身轨迹跟踪控制器
 * @details 实现全身关节轨迹跟踪控制，包括腿部12关节、手臂17关节和灵巧手12关节
 * @author futingxing
 * @date 2025-09-01
 * @version 1.0.0
 * @copyright Copyright (c) 2025 Dobot. All rights reserved.
 */

#include <rclcpp/rclcpp.hpp>
#include <dobot_atom/msg/lower_cmd.hpp>
#include <dobot_atom/msg/upper_cmd.hpp>
#include <dobot_atom/msg/hands_cmd.hpp>
#include <dobot_atom/msg/lower_state.hpp>
#include <dobot_atom/msg/upper_state.hpp>
#include <dobot_atom/msg/hands_state.hpp>
#include <dobot_atom/msg/set_fsm_id.hpp>
#include <std_msgs/msg/string.hpp>
#include <fstream>
#include <vector>
#include <sstream>
#include <chrono>
#include <memory>

class AtomFullBodyTrajectoryController : public rclcpp::Node
{
public:
    AtomFullBodyTrajectoryController() : Node("atom_full_body_trajectory_controller")
    {
        // 发布者
        lower_cmd_pub_ = this->create_publisher<dobot_atom::msg::LowerCmd>("/lower/cmd", 10);
        upper_cmd_pub_ = this->create_publisher<dobot_atom::msg::UpperCmd>("/upper/cmd", 10);
        hands_cmd_pub_ = this->create_publisher<dobot_atom::msg::HandsCmd>("/hands/cmd", 10);
        fsm_pub_ = this->create_publisher<dobot_atom::msg::SetFsmId>("/set/fsm/id", 10);
        
        // 订阅者
        lower_state_sub_ = this->create_subscription<dobot_atom::msg::LowerState>(
            "/lower/state", 10,
            std::bind(&AtomFullBodyTrajectoryController::lowerStateCallback, this, std::placeholders::_1));
        
        upper_state_sub_ = this->create_subscription<dobot_atom::msg::UpperState>(
            "/upper/state", 10,
            std::bind(&AtomFullBodyTrajectoryController::upperStateCallback, this, std::placeholders::_1));
        
        hands_state_sub_ = this->create_subscription<dobot_atom::msg::HandsState>(
            "/hands/state", 10,
            std::bind(&AtomFullBodyTrajectoryController::handsStateCallback, this, std::placeholders::_1));
        
        // 初始化参数
        init_duration_ = 5.0;  // 初始化持续时间（秒）
        control_dt_ = 0.01;    // 控制周期（秒）
        time_ = 0.0;
        trajectory_row_ = 0;
        
        // 加载轨迹数据和PID参数
        loadTrajectoryData();
        loadPIDParameters();
        
        // 创建控制定时器
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(control_dt_ * 1000)),
            std::bind(&AtomFullBodyTrajectoryController::controlLoop, this));
        
        // 初始化命令消息
        initializeCommands();
        
        RCLCPP_INFO(this->get_logger(), "Atom Full Body Trajectory Controller initialized");
        RCLCPP_INFO(this->get_logger(), "Loaded %zu trajectory points", trajectory_data_.size());
    }

private:
    // ROS2 发布者和订阅者
    rclcpp::Publisher<dobot_atom::msg::LowerCmd>::SharedPtr lower_cmd_pub_;
    rclcpp::Publisher<dobot_atom::msg::UpperCmd>::SharedPtr upper_cmd_pub_;
    rclcpp::Publisher<dobot_atom::msg::HandsCmd>::SharedPtr hands_cmd_pub_;
    rclcpp::Publisher<dobot_atom::msg::SetFsmId>::SharedPtr fsm_pub_;
    
    rclcpp::Subscription<dobot_atom::msg::LowerState>::SharedPtr lower_state_sub_;
    rclcpp::Subscription<dobot_atom::msg::UpperState>::SharedPtr upper_state_sub_;
    rclcpp::Subscription<dobot_atom::msg::HandsState>::SharedPtr hands_state_sub_;
    
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // 控制参数
    double init_duration_;
    double control_dt_;
    double time_;
    size_t trajectory_row_;
    
    // 状态数据
    dobot_atom::msg::LowerState::SharedPtr current_lower_state_;
    dobot_atom::msg::UpperState::SharedPtr current_upper_state_;
    dobot_atom::msg::HandsState::SharedPtr current_hands_state_;
    
    // 命令消息
    dobot_atom::msg::LowerCmd lower_cmd_;
    dobot_atom::msg::UpperCmd upper_cmd_;
    dobot_atom::msg::HandsCmd hands_cmd_;
    
    // 轨迹数据和PID参数
    std::vector<std::vector<double>> trajectory_data_;
    std::vector<double> leg_kp_, leg_kd_;
    std::vector<double> arm_kp_, arm_kd_;
    std::vector<double> leg_init_, arm_init_;
    
    void loadTrajectoryData()
    {
        // 创建默认轨迹数据（如果文件不存在）
        // 这里创建一个简单的正弦波轨迹作为示例
        const int num_points = 1000;
        const int total_dofs = 12 + 17; // 腿部12 + 手臂17
        
        trajectory_data_.clear();
        for (int i = 0; i < num_points; ++i) {
            std::vector<double> point(total_dofs, 0.0);
            double t = static_cast<double>(i) / num_points * 2.0 * M_PI;
            
            // 为腿部关节创建简单的正弦波轨迹
            for (int j = 0; j < 12; ++j) {
                point[j] = 0.1 * sin(t + j * 0.1); // 小幅度运动
            }
            
            // 为手臂关节创建简单的正弦波轨迹
            for (int j = 12; j < 29; ++j) {
                point[j] = 0.1 * sin(t + (j-12) * 0.1); // 小幅度运动
            }
            
            trajectory_data_.push_back(point);
        }
        
        RCLCPP_INFO(this->get_logger(), "Generated default trajectory with %d points", num_points);
    }
    
    void loadPIDParameters()
    {
        // 默认PID参数（基于atom_sdk的controlParams.json）
        // 腿部PID参数
        leg_kp_ = {200, 200, 200, 300, 40, 40, 200, 200, 200, 300, 40, 40};
        leg_kd_ = {5, 5, 5, 7.5, 1, 1, 5, 5, 5, 7.5, 1, 1};
        
        // 手臂PID参数（17个关节）
        arm_kp_ = {100, 100, 100, 100, 50, 50, 50, 100, 100, 100, 100, 50, 50, 50, 50, 50, 50};
        arm_kd_ = {3, 3, 3, 3, 1.5, 1.5, 1.5, 3, 3, 3, 3, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5};
        
        // 初始位置（零位）
        leg_init_.resize(12, 0.0);
        arm_init_.resize(17, 0.0);
    }
    
    void initializeCommands()
    {
        // 初始化腿部命令
        for (int i = 0; i < 12; ++i) {
            lower_cmd_.motor_cmd[i].mode = 1; // 位置控制模式
            lower_cmd_.motor_cmd[i].q = 0.0;
            lower_cmd_.motor_cmd[i].dq = 0.0;
            lower_cmd_.motor_cmd[i].tau = 0.0;
            lower_cmd_.motor_cmd[i].kp = leg_kp_[i];
            lower_cmd_.motor_cmd[i].kd = leg_kd_[i];
        }
        
        // 初始化手臂命令
        for (int i = 0; i < 17; ++i) {
            upper_cmd_.motor_cmd[i].mode = 1; // 位置控制模式
            upper_cmd_.motor_cmd[i].q = 0.0;
            upper_cmd_.motor_cmd[i].dq = 0.0;
            upper_cmd_.motor_cmd[i].tau = 0.0;
            upper_cmd_.motor_cmd[i].kp = arm_kp_[i];
            upper_cmd_.motor_cmd[i].kd = arm_kd_[i];
        }
        
        // 初始化灵巧手命令
        for (int i = 0; i < 12; ++i) {
            hands_cmd_.hands[i].mode = 1; // 位置控制模式
            hands_cmd_.hands[i].q = 0.0;
            hands_cmd_.hands[i].dq = 0.0;
            hands_cmd_.hands[i].tau = 0.0;
            hands_cmd_.hands[i].kp = 50.0; // 默认手部刚度
            hands_cmd_.hands[i].kd = 1.0;  // 默认手部阻尼
        }
    }
    
    void lowerStateCallback(const dobot_atom::msg::LowerState::SharedPtr msg)
    {
        current_lower_state_ = msg;
    }
    
    void upperStateCallback(const dobot_atom::msg::UpperState::SharedPtr msg)
    {
        current_upper_state_ = msg;
    }
    
    void handsStateCallback(const dobot_atom::msg::HandsState::SharedPtr msg)
    {
        current_hands_state_ = msg;
    }
    
    void controlLoop()
    {
        if (!current_lower_state_ || !current_upper_state_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                "Waiting for robot state data...");
            return;
        }
        
        time_ += control_dt_;
        
        // 计算目标位置
        std::vector<double> leg_target(12), arm_target(17);
        
        if (time_ < init_duration_) {
            // 初始化阶段：平滑移动到轨迹起始位置
            double alpha = time_ / init_duration_;
            
            for (int i = 0; i < 12; ++i) {
                double current_pos = current_lower_state_->motor_state[i].q;
                double target_pos = trajectory_data_[0][i];
                leg_target[i] = current_pos + alpha * (target_pos - current_pos);
            }
            
            for (int i = 0; i < 17; ++i) {
                double current_pos = current_upper_state_->motor_state[i].q;
                double target_pos = trajectory_data_[0][i + 12];
                arm_target[i] = current_pos + alpha * (target_pos - current_pos);
            }
        } else {
            // 轨迹跟踪阶段
            for (int i = 0; i < 12; ++i) {
                leg_target[i] = trajectory_data_[trajectory_row_][i];
            }
            
            for (int i = 0; i < 17; ++i) {
                arm_target[i] = trajectory_data_[trajectory_row_][i + 12];
            }
            
            // 更新轨迹索引
            trajectory_row_++;
            if (trajectory_row_ >= trajectory_data_.size()) {
                trajectory_row_ = 0; // 循环播放
            }
        }
        
        // 设置腿部命令
        for (int i = 0; i < 12; ++i) {
            lower_cmd_.motor_cmd[i].q = leg_target[i];
            lower_cmd_.motor_cmd[i].dq = 0.0;
            lower_cmd_.motor_cmd[i].tau = 0.0;
        }
        
        // 设置手臂命令
        for (int i = 0; i < 17; ++i) {
            upper_cmd_.motor_cmd[i].q = arm_target[i];
            upper_cmd_.motor_cmd[i].dq = 0.0;
            upper_cmd_.motor_cmd[i].tau = 0.0;
        }
        
        // 设置灵巧手命令（保持零位）
        for (int i = 0; i < 12; ++i) {
            hands_cmd_.hands[i].q = 0.0;
            hands_cmd_.hands[i].dq = 0.0;
            hands_cmd_.hands[i].tau = 0.0;
        }
        
        // 发布命令
        lower_cmd_pub_->publish(lower_cmd_);
        upper_cmd_pub_->publish(upper_cmd_);
        hands_cmd_pub_->publish(hands_cmd_);
        
        // 每秒输出一次调试信息
        if (static_cast<int>(time_ * 100) % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), "Time: %.2f, Trajectory row: %zu/%zu", 
                       time_, trajectory_row_, trajectory_data_.size());
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AtomFullBodyTrajectoryController>();
    
    RCLCPP_INFO(node->get_logger(), "Starting Atom Full Body Trajectory Controller...");
    RCLCPP_WARN(node->get_logger(), "WARNING: Robot will move! Ensure safe environment!");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}