#include <rclcpp/logging.hpp>  // ROS 2日志库
#include <rclcpp/qos.hpp>  // ROS 2 QoS设置
#include <rclcpp/utilities.hpp>  // ROS 2工具库
#include <cmath>  // 数学库

#include "decision.hpp"  // 包含头文件

using namespace std::chrono_literals;  // 使用时间字面量

namespace rm_decision
{

RMDecision::RMDecision(const rclcpp::NodeOptions & options)
: Node("rm_decision", options), current_state_(RobotState::IDLE)  // 初始化节点和状态
{
    RCLCPP_INFO(this->get_logger(), "RMDecision node has been started.");

    // 初始化订阅器
    from_sentry_sub_ = this->create_subscription<rm_serial_driver_nav_msgs::msg::FromSentry>(
        "/from_sentry", 10, std::bind(&RMDecision::fromSentryCallback, this, std::placeholders::_1));  // 订阅哨兵消息
    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/Odometry", 10, std::bind(&RMDecision::getLocalPosition, this, std::placeholders::_1));  // 订阅里程计消息

    // 初始化Action客户端
    action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");  // 创建Action客户端
    send_goal_options_.result_callback = std::bind(&RMDecision::goalResultCallback, this, std::placeholders::_1);  // 设置结果回调函数

    // 初始化关键点位置
    point_home_.x = 0.0;  // 家（起点）位置
    point_home_.y = 0.0;
    point_center_.x = 3.0;  // 中心位置
    point_center_.y = 0.54;
    point_supply_area_.x = -2.5;  // 补给区位置
    point_supply_area_.y = 2.85;
    point_enemy_supply_area_.x = 5.3;  // 敌方补给区位置
    point_enemy_supply_area_.y = -1.35;
    point_wall_center_.x = 0.45;  // 墙中心位置
    point_wall_center_.y = 0.15;
    point_right_.x = 2.90;  // 右侧位置
    point_right_.y = -2.10;
    point_base_.x = 0.03;  // 基地位置
    point_base_.y = -0.60;
    point_wall_left_.x = 0.23;  // 左侧墙位置
    point_wall_left_.y = 2.22;

    // 初始化变量
    hp_ = 0;  // 当前血量
    time_ = 0;  // 当前时间
    last_hp_ = 0;  // 上一次的血量
    added_hp_ = 0;  // 增加的血量
    return_state_ = false;  // 返回补给区的状态
    return_time_ = 0;  // 返回补给区的时间
    last_send_time_ = 0;  // 上一次发送目标的时间
    last_state_ = 0;  // 上一次的状态

    // 启动决策线程
    decision_thread_ = std::thread(&RMDecision::decision, this);  // 启动决策线程
}

RMDecision::~RMDecision()
{
    if (decision_thread_.joinable()) {
        decision_thread_.join();  // 等待决策线程结束
    }
    RCLCPP_INFO(this->get_logger(), "RMDecision node has been destroyed.");
}

void RMDecision::fromSentryCallback(const rm_serial_driver_nav_msgs::msg::FromSentry::SharedPtr msg)
{
    last_hp_ = hp_;  // 记录上一次的血量
    hp_ = msg->hp;  // 更新当前血量
    time_ = msg->time;  // 更新当前时间
    mode_ = msg->mode;  // 更新当前模式

    if (hp_ - last_hp_ > 0) {
        added_hp_ += hp_ - last_hp_;  // 计算增加的血量
        RCLCPP_INFO(this->get_logger(), "HP added: %d, Total added HP: %d", hp_ - last_hp_, added_hp_);
    }

    RCLCPP_INFO(this->get_logger(), "Received message: HP=%d, Time=%d, Mode=%d", hp_, time_, mode_);
}

void RMDecision::getLocalPosition(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    localPosition_x_ = msg->pose.pose.position.x;  // 更新本地位置x坐标
    localPosition_y_ = msg->pose.pose.position.y;  // 更新本地位置y坐标
    RCLCPP_INFO(this->get_logger(), "Current position: x=%.2f, y=%.2f", localPosition_x_, localPosition_y_);
}

void RMDecision::decision()
{
    rclcpp::WallRate loop_rate(5.0);  // 设置决策频率为5Hz
    while (rclcpp::ok()) {
        // 根据当前状态和条件切换状态
        switch (current_state_) {
            case RobotState::IDLE:
                current_state_ = RobotState::PATROL;  // 从空闲状态切换到巡逻状态
                break;
            case RobotState::PATROL:
                if (hp_ < 340 && added_hp_ <= 580 && time_ <= 240) {
                    current_state_ = RobotState::RETURN_TO_BASE;  // 血量不足时返回补给区
                } else if (time_ > 240) {
                    current_state_ = RobotState::PROTECT_BASE;  // 时间超过240秒时保护基地
                } else if (mode_ == 2) {
                    current_state_ = RobotState::ATTACK;  // 模式为2时进入攻击状态
                }
                break;
            case RobotState::RETURN_TO_BASE:
                if (isGoalReached(point_supply_area_)) {
                    current_state_ = RobotState::PATROL;  // 到达补给区后返回巡逻状态
                }
                break;
            case RobotState::ATTACK:
                if (time_ > 82) {
                    current_state_ = RobotState::PATROL;  // 时间超过82秒时返回巡逻状态
                }
                break;
            case RobotState::PROTECT_BASE:
                if (time_ <= 240) {
                    current_state_ = RobotState::PATROL;  // 时间小于240秒时返回巡逻状态
                }
                break;
        }

        // 设置目标点并发送
        setNavigationTarget();
        if (!isGoalReached(goal_msg_.pose.pose.position)) {
            sendGoal();  // 如果未到达目标点，则发送目标
        }

        loop_rate.sleep();  // 控制决策频率
    }
}

void RMDecision::setNavigationTarget()
{
    switch (current_state_) {
        case RobotState::PATROL:
            goal_msg_.pose.pose.position = point_center_;  // 巡逻时前往中心点
            break;
        case RobotState::RETURN_TO_BASE:
            goal_msg_.pose.pose.position = point_supply_area_;  // 返回补给区
            break;
        case RobotState::ATTACK:
            if (time_ <= 40) {
                goal_msg_.pose.pose.position = point_enemy_supply_area_;  // 攻击敌方补给区
            } else if (time_ <= 65) {
                goal_msg_.pose.pose.position = point_right_;  // 攻击右侧位置
            } else {
                goal_msg_.pose.pose.position = point_center_;  // 返回中心点
            }
            break;
        case RobotState::PROTECT_BASE:
            goal_msg_.pose.pose.position = point_base_;  // 保护基地
            break;
        default:
            goal_msg_.pose.pose.position = point_home_;  // 默认返回家（起点）
            break;
    }
}

void RMDecision::sendGoal()
{
    if (!action_client_->wait_for_action_server(10s)) {
        RCLCPP_ERROR(get_logger(), "Action server not available.");  // 检查Action服务器是否可用
        return;
    }
    action_client_->async_send_goal(goal_msg_, send_goal_options_);  // 发送目标点
}

bool RMDecision::isGoalReached(const geometry_msgs::msg::Point& target)
{
    float error = 0.1;  // 允许的误差范围
    return (fabs(localPosition_x_ - target.x) < error) &&
           (fabs(localPosition_y_ - target.y) < error);  // 判断是否到达目标点
}

void RMDecision::goalResultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result)
{
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(get_logger(), "Goal reached.");  // 目标达成
    } else {
        RCLCPP_WARN(get_logger(), "Goal failed with code: %d", static_cast<int>(result.code));  // 目标失败
    }
}

} // namespace rm_decision

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);  // 初始化ROS 2
    auto node_options = rclcpp::NodeOptions();
    auto node = std::make_shared<rm_decision::RMDecision>(node_options);  // 创建节点
    rclcpp::spin(node);  // 运行节点
    rclcpp::shutdown();  // 关闭节点
    return 0;
}