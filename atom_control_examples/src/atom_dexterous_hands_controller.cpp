/**
 * @file atom_dexterous_hands_controller.cpp
 * @brief Atom机器人灵巧手控制程序
 * @details 演示如何使用ROS2控制Atom机器人灵巧手进行抓握和松开动作，支持平滑运动和时序控制
 * @author futingxing
 * @date 2025-09-01
 * @version 1.0.0
 * @copyright Copyright (c) 2025 Dobot. All rights reserved.
 */

#include "rclcpp/rclcpp.hpp"
#include "dobot_atom/msg/hands_cmd.hpp"
#include "dobot_atom/msg/motor_cmd.hpp"
#include <cmath>
#include <thread>
#include <atomic>
#include <map>
#include <string>

// 手指索引定义
#define RIGHT_PINKY 0        // 右手小指
#define RIGHT_RING 1         // 右手无名指
#define RIGHT_MIDDLE 2       // 右手中指
#define RIGHT_INDEX 3        // 右手食指
#define RIGHT_THUMB_FLEX 4   // 右手大拇指弯曲
#define RIGHT_THUMB_ROTATE 5 // 右手大拇指旋转
#define LEFT_PINKY 6         // 左手小指
#define LEFT_RING 7          // 左手无名指
#define LEFT_MIDDLE 8        // 左手中指
#define LEFT_INDEX 9         // 左手食指
#define LEFT_THUMB_FLEX 10   // 左手大拇指弯曲
#define LEFT_THUMB_ROTATE 11 // 左手大拇指旋转

// 控制参数
#define CONTROL_FREQUENCY 20    // Hz, 控制频率
#define STATE_SWITCH_INTERVAL 5 // 秒, 状态切换间隔
#define MOTION_STEPS 30         // 动作总步数
#define STEP_DELAY_MS 50        // 步间延迟（毫秒）
#define THUMB_DELAY_STEPS 12    // 抓握时拇指延迟步数
#define THUMB_LEAD_STEPS 12     // 松开时拇指提前完成步数

using std::placeholders::_1;

class AtomDexterousHandsController : public rclcpp::Node
{
public:
    AtomDexterousHandsController() : Node("atom_dexterous_hands_controller")
    {
        // 发布器
        hands_cmd_pub_ = this->create_publisher<dobot_atom::msg::HandsCmd>("/hands/cmd", 10);
        
        RCLCPP_INFO(this->get_logger(), "灵巧手控制节点已启动 - 使用话题 /hands/cmd");
        
        // 初始化角度限制（弧度）
        init_angle_limits();
        
        // 初始化目标位置（初始为松开状态 - 100%张开）
        init_target_positions();
        
        // 创建状态切换定时器
        state_timer_ = this->create_wall_timer(
            std::chrono::seconds(STATE_SWITCH_INTERVAL),
            std::bind(&AtomDexterousHandsController::toggle_grasp_state, this));
        
        // 初始化状态
        is_grasping_ = false;
        motion_in_progress_ = false;
    }
    
    ~AtomDexterousHandsController()
    {
        // 确保线程安全退出
        if (motion_thread_.joinable()) {
            motion_thread_.join();
        }
    }

private:
    void init_angle_limits()
    {
        // 小拇指、无名指、中指、食指范围: 19°~176.7°
        angle_limits_["pinky"] = {deg_to_rad(19), deg_to_rad(176.7)};
        angle_limits_["ring"] = {deg_to_rad(19), deg_to_rad(176.7)};
        angle_limits_["middle"] = {deg_to_rad(19), deg_to_rad(176.7)};
        angle_limits_["index"] = {deg_to_rad(19), deg_to_rad(176.7)};
        // 大拇指弯曲范围: -13°~53.6°
        angle_limits_["thumb_flex"] = {deg_to_rad(-13), deg_to_rad(53.6)};
        // 大拇指旋转范围: 90°~165°
        angle_limits_["thumb_rotate"] = {deg_to_rad(90), deg_to_rad(165)};
    }
    
    void init_target_positions()
    {
        // 初始为松开状态（最大角度）
        target_positions_[RIGHT_PINKY] = angle_limits_["pinky"].second;
        target_positions_[RIGHT_RING] = angle_limits_["ring"].second;
        target_positions_[RIGHT_MIDDLE] = angle_limits_["middle"].second;
        target_positions_[RIGHT_INDEX] = angle_limits_["index"].second;
        target_positions_[RIGHT_THUMB_FLEX] = angle_limits_["thumb_flex"].second;
        target_positions_[RIGHT_THUMB_ROTATE] = angle_limits_["thumb_rotate"].second;
        
        target_positions_[LEFT_PINKY] = angle_limits_["pinky"].second;
        target_positions_[LEFT_RING] = angle_limits_["ring"].second;
        target_positions_[LEFT_MIDDLE] = angle_limits_["middle"].second;
        target_positions_[LEFT_INDEX] = angle_limits_["index"].second;
        target_positions_[LEFT_THUMB_FLEX] = angle_limits_["thumb_flex"].second;
        target_positions_[LEFT_THUMB_ROTATE] = angle_limits_["thumb_rotate"].second;
    }
    
    void toggle_grasp_state()
    {
        if (motion_in_progress_) {
            return;
        }
        
        is_grasping_ = !is_grasping_;
        std::string state = is_grasping_ ? "抓握" : "松开";
        RCLCPP_INFO(this->get_logger(), "开始执行: %s动作", state.c_str());
        
        // 启动动作执行线程
        motion_in_progress_ = true;
        if (motion_thread_.joinable()) {
            motion_thread_.join();
        }
        motion_thread_ = std::thread(&AtomDexterousHandsController::execute_smooth_motion, this);
    }
    
    void execute_smooth_motion()
    {
        // 计算目标位置
        std::map<int, double> target_positions;
        calculate_target_positions(target_positions);
        
        // 执行动作序列
        int total_steps = MOTION_STEPS;
        if (!is_grasping_) {
            total_steps += THUMB_LEAD_STEPS; // 松开时增加步数让拇指提前完成
        }
        
        for (int step = 0; step < total_steps; ++step) {
            auto cmd = dobot_atom::msg::HandsCmd();
            // hands是固定大小数组MotorCmd[12]，无需resize
            
            // 设置所有手指参数
            set_finger_commands(cmd, target_positions, step, total_steps);
            
            // 发布指令
            hands_cmd_pub_->publish(cmd);
            
            // 日志记录
            if (step % 4 == 0 || step == total_steps - 1) {
                log_motion_progress(cmd, step, total_steps);
            }
            
            // 步间延迟
            std::this_thread::sleep_for(std::chrono::milliseconds(STEP_DELAY_MS));
        }
        
        // 动作完成
        motion_in_progress_ = false;
        std::string state = is_grasping_ ? "抓握" : "松开";
        RCLCPP_INFO(this->get_logger(), "%s动作完成", state.c_str());
    }
    
    void calculate_target_positions(std::map<int, double>& target_positions)
    {
        // 右手
        target_positions[RIGHT_PINKY] = is_grasping_ ? angle_limits_["pinky"].first : angle_limits_["pinky"].second;
        target_positions[RIGHT_RING] = is_grasping_ ? angle_limits_["ring"].first : angle_limits_["ring"].second;
        target_positions[RIGHT_MIDDLE] = is_grasping_ ? angle_limits_["middle"].first : angle_limits_["middle"].second;
        target_positions[RIGHT_INDEX] = is_grasping_ ? angle_limits_["index"].first : angle_limits_["index"].second;
        target_positions[RIGHT_THUMB_FLEX] = is_grasping_ ? angle_limits_["thumb_flex"].first : angle_limits_["thumb_flex"].second;
        target_positions[RIGHT_THUMB_ROTATE] = is_grasping_ ? angle_limits_["thumb_rotate"].first : angle_limits_["thumb_rotate"].second;
        
        // 左手
        target_positions[LEFT_PINKY] = is_grasping_ ? angle_limits_["pinky"].first : angle_limits_["pinky"].second;
        target_positions[LEFT_RING] = is_grasping_ ? angle_limits_["ring"].first : angle_limits_["ring"].second;
        target_positions[LEFT_MIDDLE] = is_grasping_ ? angle_limits_["middle"].first : angle_limits_["middle"].second;
        target_positions[LEFT_INDEX] = is_grasping_ ? angle_limits_["index"].first : angle_limits_["index"].second;
        target_positions[LEFT_THUMB_FLEX] = is_grasping_ ? angle_limits_["thumb_flex"].first : angle_limits_["thumb_flex"].second;
        target_positions[LEFT_THUMB_ROTATE] = is_grasping_ ? angle_limits_["thumb_rotate"].first : angle_limits_["thumb_rotate"].second;
    }
    
    void set_finger_commands(dobot_atom::msg::HandsCmd& cmd, 
                           const std::map<int, double>& target_positions, 
                           int step, int /* total_steps */)
    {
        for (int idx = 0; idx < 12; ++idx) {
            double current = target_positions_[idx];
            double target = target_positions.at(idx);
            
            // 计算进度
            double progress = calculate_finger_progress(idx, step);
            
            // 使用缓动函数优化运动曲线
            double eased_progress = ease_in_out(progress);
            
            // 计算目标位置
            double new_target = current + (target - current) * eased_progress;
            
            // 设置基本参数
            cmd.hands[idx].mode = 1;      // 位置控制模式
            cmd.hands[idx].q = new_target;
            cmd.hands[idx].tau = 0.05;    // 小力矩避免过冲
            cmd.hands[idx].kp = 40.0;     // 刚度
            cmd.hands[idx].kd = 0.5;      // 阻尼
            cmd.hands[idx].dq = 0.0;      // 速度设为0
            
            // 更新当前位置
            target_positions_[idx] = new_target;
        }
    }
    
    double calculate_finger_progress(int finger_idx, int step)
    {
        bool is_thumb = (finger_idx == RIGHT_THUMB_FLEX || finger_idx == RIGHT_THUMB_ROTATE ||
                        finger_idx == LEFT_THUMB_FLEX || finger_idx == LEFT_THUMB_ROTATE);
        
        double progress = 0.0;
        
        if (is_grasping_) {
            // 抓握动作：四指正常进度，拇指延迟
            if (is_thumb) {
                int effective_step = std::max(0, step - THUMB_DELAY_STEPS);
                progress = std::min(1.0, static_cast<double>(effective_step) / MOTION_STEPS);
            } else {
                progress = std::min(1.0, static_cast<double>(step) / MOTION_STEPS);
            }
        } else {
            // 松开动作：拇指快速提前完成，四指正常进度
            if (is_thumb) {
                progress = std::min(1.0, static_cast<double>(step) / THUMB_LEAD_STEPS);
            } else {
                int effective_step = std::max(0, step - THUMB_LEAD_STEPS);
                progress = std::min(1.0, static_cast<double>(effective_step) / MOTION_STEPS);
            }
        }
        
        return progress;
    }
    
    void log_motion_progress(const dobot_atom::msg::HandsCmd& cmd, int step, int total_steps)
    {
        double thumb_angle = rad_to_deg(cmd.hands[RIGHT_THUMB_FLEX].q);
        double index_angle = rad_to_deg(cmd.hands[RIGHT_INDEX].q);
        
        RCLCPP_INFO(this->get_logger(), 
                   "步骤 %d/%d: 食指=%.1f°, 拇指=%.1f°", 
                   step + 1, total_steps, index_angle, thumb_angle);
    }
    
    double ease_in_out(double t)
    {
        // 缓动函数：开始慢，中间快，结束慢
        return t * t * (3 - 2 * t);
    }
    
    double deg_to_rad(double degrees)
    {
        return degrees * M_PI / 180.0;
    }
    
    double rad_to_deg(double radians)
    {
        return radians * 180.0 / M_PI;
    }
    
    // 成员变量
    rclcpp::Publisher<dobot_atom::msg::HandsCmd>::SharedPtr hands_cmd_pub_;
    rclcpp::TimerBase::SharedPtr state_timer_;
    
    std::map<std::string, std::pair<double, double>> angle_limits_;
    std::map<int, double> target_positions_;
    
    std::atomic<bool> is_grasping_;
    std::atomic<bool> motion_in_progress_;
    std::thread motion_thread_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AtomDexterousHandsController>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "控制器异常: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}