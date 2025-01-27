#ifndef RM_DICISION__DECISION_HPP_
#define RM_DICISION__DECISION_HPP_

#include <rclcpp/rclcpp.hpp>  // ROS 2核心库
#include <rclcpp/publisher.hpp>  // 发布者相关库
#include <rclcpp/subscription.hpp>  // 订阅者相关库
#include <rclcpp_action/rclcpp_action.hpp>  // ROS 2 Action相关库

#include <nav2_msgs/action/navigate_to_pose.hpp>  // Nav2导航目标消息
#include <geometry_msgs/msg/pose_stamped.hpp>  // 位姿消息
#include <geometry_msgs/msg/point.hpp>  // 点消息
#include <nav_msgs/msg/odometry.hpp>  // 里程计消息

#include <thread>  // 线程库
#include <memory>  // 智能指针库

#include "rm_serial_driver_nav_msgs/msg/from_sentry.hpp"  // 自定义消息，来自哨兵

namespace rm_decision
{

class RMDecision : public rclcpp::Node
{
public:
    // 构造函数，接受NodeOptions作为参数
    explicit RMDecision(const rclcpp::NodeOptions & options);
    // 析构函数
    ~RMDecision();

private:
    // 定义机器人状态枚举
    enum class RobotState {
        PATROL,  // 巡逻状态
        RETURN_TO_BASE,  // 返回补给区状态
        ATTACK,  // 攻击状态
        PROTECT_BASE,  // 保护基地状态
        IDLE  // 空闲状态
    };

    // 订阅器和回调函数
    rclcpp::Subscription<rm_serial_driver_nav_msgs::msg::FromSentry>::SharedPtr from_sentry_sub_;  // 订阅来自哨兵的消息
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;  // 订阅里程计消息
    void fromSentryCallback(const rm_serial_driver_nav_msgs::msg::FromSentry::SharedPtr msg);  // 处理哨兵消息的回调函数
    void getLocalPosition(const nav_msgs::msg::Odometry::SharedPtr msg);  // 获取本地位置的回调函数

    // 决策逻辑
    void decision();  // 决策函数，根据当前状态做出决策
    void setNavigationTarget();  // 设置导航目标点
    void sendGoal();  // 发送目标点给导航系统
    bool isGoalReached(const geometry_msgs::msg::Point& target);  // 判断是否到达目标点

    // Action客户端
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;  // Action客户端，用于与导航系统交互
    nav2_msgs::action::NavigateToPose::Goal goal_msg_;  // 导航目标消息
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions send_goal_options_;  // 发送目标选项
    void goalResultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result);  // 处理导航结果的回调函数

    // 机器人状态和变量
    RobotState current_state_;  // 当前机器人状态
    float localPosition_x_;  // 本地位置x坐标
    float localPosition_y_;  // 本地位置y坐标
    uint16_t hp_;  // 当前血量
    uint16_t time_;  // 当前时间
    uint8_t mode_;  // 当前模式
    uint16_t last_hp_;  // 上一次的血量
    uint16_t added_hp_;  // 增加的血量
    bool return_state_;  // 返回补给区的状态
    uint16_t return_time_;  // 返回补给区的时间
    uint16_t last_send_time_;  // 上一次发送目标的时间
    uint8_t last_state_;  // 上一次的状态

    // 关键点位置
    geometry_msgs::msg::Point point_home_;  // 家（起点）位置
    geometry_msgs::msg::Point point_center_;  // 中心位置
    geometry_msgs::msg::Point point_supply_area_;  // 补给区位置
    geometry_msgs::msg::Point point_enemy_supply_area_;  // 敌方补给区位置
    geometry_msgs::msg::Point point_wall_center_;  // 墙中心位置
    geometry_msgs::msg::Point point_right_;  // 右侧位置
    geometry_msgs::msg::Point point_base_;  // 基地位置
    geometry_msgs::msg::Point point_wall_left_;  // 左侧墙位置
    //测试用点
    geometry_msgs::msg::Point point_test1_;//测试点1
    geometry_msgs::msg::Point point_test2_;//测试点2


    // 接收线程
    std::thread decision_thread_;  // 决策线程
};

}  // namespace rm_decision

#endif  // RM_DICISION__DECISION_HPP_