// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// C++ system
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"
#include "rm_serial_driver/rm_serial_driver.hpp"

namespace rm_serial_driver
{

// RMSerialDriver 构造函数，初始化节点和串口驱动
RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & options) : Node("rm_serial_driver", options),
  owned_ctx_{new IoContext(2)},  // 创建 IoContext 对象，用于管理串口通信的上下文
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}  // 创建串口驱动对象
{
  RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");

  getParams();  // 获取并设置串口参数

  // 创建发布器
  sentry_pub_ = this->create_publisher<rm_serial_driver_nav_msgs::msg::FromSentry>("/from_sentry", 10);  // 发布哨兵信息

  try {
    // 初始化串口
    serial_driver_->init_port(device_name_, *device_config_);
    if (!serial_driver_->port()->is_open()) {
      serial_driver_->port()->open();  // 打开串口
      receive_thread_ = std::thread(&RMSerialDriver::receiveData, this);  // 启动接收数据的线程
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
    throw ex;
  }

  // 创建订阅器
  sentry_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel_nav", rclcpp::SensorDataQoS(),
    std::bind(&RMSerialDriver::sendDataSentry, this, std::placeholders::_1));  // 订阅哨兵控制命令
}

// RMSerialDriver 析构函数，清理资源
RMSerialDriver::~RMSerialDriver()
{
  if (receive_thread_.joinable()) {
    receive_thread_.join();  // 等待接收线程结束
  }

  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();  // 关闭串口
  }

  if (owned_ctx_) {
    owned_ctx_->waitForExit();  // 等待 IoContext 退出
  }
}

// 接收数据的线程函数
void RMSerialDriver::receiveData()
{
  std::vector<uint8_t> header(1);  // 数据头
  std::vector<uint8_t> data;  // 数据
  data.reserve(sizeof(ReceivePacket));  // 预留数据包大小

  while (rclcpp::ok()) {
    try {
      serial_driver_->port()->receive(header);  // 接收数据头

      if (header[0] == 0x5A) {  // 检查数据头是否为 0x5A
        data.resize(sizeof(ReceivePacket) - 1);
        serial_driver_->port()->receive(data);  // 接收数据

        data.insert(data.begin(), header[0]);  // 将数据头插入数据包
        ReceivePacket packet = fromVector(data);  // 将数据转换为 ReceivePacket 结构体

        // 校验 CRC
        bool crc_ok =
          crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
        if (crc_ok) {
          // 如果 CRC 校验通过，处理接收到的数据
          // 发布哨兵信息
          sentry_msg_.hp = packet.hp;
          sentry_msg_.time = packet.time;
          sentry_msg_.mode = packet.mode;
          sentry_pub_->publish(sentry_msg_);

        } else {
          RCLCPP_ERROR(get_logger(), "CRC error!");  // CRC 校验失败
        }
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "Invalid header: %02X", header[0]);  // 无效的数据头
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());  // 接收数据时出错
      reopenPort();  // 重新打开串口
    }
  }
}

// 发送哨兵控制数据到串口
void RMSerialDriver::sendDataSentry(geometry_msgs::msg::Twist::SharedPtr msg)
{
  try {
    SendSentry packet;
    packet.vx = msg->linear.x;
    packet.vy = msg->linear.y;
    crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));  // 计算并附加 CRC
    std::vector<uint8_t> data = toVectorSentry(packet);
    serial_driver_->port()->send(data);  // 通过串口发送数据
    RCLCPP_DEBUG(get_logger(),"已经发送雷达数据");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "当发送数据时出错: %s", ex.what());  // 发送数据时出错
    reopenPort();  // 重新打开串口
  }
}

// 获取并设置串口参数
void RMSerialDriver::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try {
    device_name_ = declare_parameter<std::string>("device_name", "");  // 获取设备名称
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);  // 获取波特率
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");  // 获取流控制设置

    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "");  // 获取奇偶校验设置

    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");  // 获取停止位设置

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);  // 创建串口配置对象
}

// 重新打开串口
void RMSerialDriver::reopenPort()
{
  RCLCPP_WARN(get_logger(), "Attempting to reopen port");
  try {
    if (serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();  // 关闭串口
    }
    serial_driver_->port()->open();  // 重新打开串口
    RCLCPP_INFO(get_logger(), "Successfully reopened port");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
    if (rclcpp::ok()) {
      rclcpp::sleep_for(std::chrono::seconds(1));
      reopenPort();  // 重试重新打开串口
    }
  }
}

}  // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// 注册节点，使其可被类加载器发现
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)