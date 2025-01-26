#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <cmath>  // 用于数学计算

#include "decision.hpp"  // 包含头文件

using namespace std::chrono_literals;  // 使用时间字面量

namespace rm_decision
{

// RMDecision 类的构造函数
RMDecision::RMDecision(const rclcpp::NodeOptions & options)
: Node("rm_decision", options), current_state_(RobotState::IDLE)  // 初始化节点和状态
{
    // 打印日志，表示节点已启动
    RCLCPP_INFO(this->get_logger(), "RMDecision 节点已启动。");

    // 创建订阅器，订阅来自哨兵的消息，话题名为 "/from_sentry"，队列长度为 10
    from_sentry_sub_ = this->create_subscription<rm_serial_driver_nav_msgs::msg::FromSentry>(
        "/from_sentry", 10, std::bind(&RMDecision::fromSentryCallback, this, std::placeholders::_1));

    // 创建动作客户端，用于发送导航目标，动作名为 "navigate_to_pose"
    action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

    // 创建订阅器，订阅里程计消息，话题名为 "/Odometry"，队列长度为 10
    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/Odometry", 10, std::bind(&RMDecision::getLocalPosition, this, std::placeholders::_1));

    // 初始化目标消息的帧 ID 为 "map"
    goal_msg_.pose.header.frame_id = "map";

    // 初始化目标位置和姿态
    goal_msg_.pose.pose.position.x = 0.0;
    goal_msg_.pose.pose.position.y = 0.0;
    goal_msg_.pose.pose.position.z = 0.0;
    goal_msg_.pose.pose.orientation.x = 0.0;
    goal_msg_.pose.pose.orientation.y = 0.0;
    goal_msg_.pose.pose.orientation.z = 0.0;
    goal_msg_.pose.pose.orientation.w = 1.0;

    // 初始化一些关键点位置
    point_home_.x = 0.0;  // 家（起点）位置
    point_home_.y = 0.0;

    point_test1_.x = 0.23;
    point_test1_.y = -5.13;

    point_test2_.x = 0.98;
    point_test2_.y = 3.83;

    // 初始化变量
    hp_ = 0;  // 当前血量
    time_ = 0;  // 当前时间
    last_hp_ = 0;  // 上一次的血量
    added_hp_ = 0;  // 增加的血量
    return_state_ = false;  // 返回补给区的状态
    return_time_ = 0;  // 返回补给区的时间
    last_send_time_ = 0;  // 上一次发送目标的时间
    last_state_ = 0;  // 上一次的状态

    // 设置动作结果回调函数
    send_goal_options_.result_callback = std::bind(&RMDecision::goalResultCallback, this, std::placeholders::_1);

    // 启动决策线程
    decision_thread_ = std::thread(&RMDecision::decision, this);
}

// 哨兵消息回调函数
void RMDecision::fromSentryCallback(const rm_serial_driver_nav_msgs::msg::FromSentry::SharedPtr msg)
{
    last_hp_ = hp_;  // 记录上一次的血量
    hp_ = msg->hp;  // 更新当前血量
    time_ = msg->time;  // 更新当前时间
    mode_ = msg->mode;  // 更新当前模式

    // 如果血量增加，记录增加的血量
    if (hp_ - last_hp_ > 0) {
        added_hp_ += hp_ - last_hp_;
        RCLCPP_INFO(this->get_logger(), "血量增加: %d, 总增加血量: %d", hp_ - last_hp_, added_hp_);
    }

    // 打印接收到的消息
    RCLCPP_INFO(this->get_logger(), "收到哨兵消息: 血量=%d, 时间=%d, 模式=%d", hp_, time_, mode_);
}

// 动作结果回调函数
void RMDecision::goalResultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result)
{
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(get_logger(), "目标已达成。");  // 目标达成
    } else {
        // 将 rclcpp_action::ResultCode 转换为整数输出
        RCLCPP_WARN(get_logger(), "目标失败，错误码: %d", static_cast<int>(result.code));  // 目标失败
    }
}

// 发送目标函数
void RMDecision::sendGoal()
{
    // 打印日志，表示正在发送目标
    RCLCPP_INFO(this->get_logger(), "正在发送目标。");

    // 等待动作服务器可用
    while (!action_client_->wait_for_action_server(10s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(get_logger(), "等待动作服务器超时。");
            return;
        }
        RCLCPP_INFO(get_logger(), "等待动作服务器可用...");
    }

    // 异步发送目标
    action_client_->async_send_goal(goal_msg_, send_goal_options_);
}

// 获取本地位置的回调函数
void RMDecision::getLocalPosition(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // 更新本地位置
    localPosition_x_ = msg->pose.pose.position.x;
    localPosition_y_ = msg->pose.pose.position.y;

    // 打印当前位置
    RCLCPP_INFO(this->get_logger(), "当前位置: x=%.2f, y=%.2f", localPosition_x_, localPosition_y_);
}

// 决策函数
void RMDecision::decision()
{
    // 设置循环频率为 10Hz
    rclcpp::WallRate loop_rate(10.0);

    // 主循环
    while (rclcpp::ok()) {
        // 打印日志，表示正在决策
        RCLCPP_INFO(this->get_logger(), "正在决策。");

        // 根据时间和血量进行决策
        if (time_ > 0 && time_ < 295) {
            if (hp_ == 600) {
                // 如果血量为 600，前往家的位置
                goal_msg_.pose.pose.position = point_home_;
            } else if (hp_ <= 480) {
                // 如果血量低于 480，前往测试点 2
                goal_msg_.pose.pose.position = point_test2_;
            } else if (hp_ >= 500 && hp_ < 600) {
                // 如果血量在 500 到 600 之间，前往测试点 1
                goal_msg_.pose.pose.position = point_test1_;
            }

            // 发送目标
            sendGoal();
        }

        // 休眠以保持循环频率
        loop_rate.sleep();
    }
}

// 析构函数
RMDecision::~RMDecision()
{
    // 确保线程在销毁前结束
    if (decision_thread_.joinable()) {
        decision_thread_.join();
    }

    // 打印日志，表示节点已销毁
    RCLCPP_INFO(this->get_logger(), "RMDecision 节点已销毁。");
}

} // namespace rm_decision

// 主函数
int main(int argc, char * argv[])
{
    // 初始化 ROS 2
    rclcpp::init(argc, argv);

    // 创建节点选项
    auto node_options = rclcpp::NodeOptions();

    // 创建 RMDecision 节点
    auto node = std::make_shared<rm_decision::RMDecision>(node_options);

    // 进入 ROS 2 事件循环
    rclcpp::spin(node);

    // 关闭 ROS 2
    rclcpp::shutdown();

    return 0;
}
