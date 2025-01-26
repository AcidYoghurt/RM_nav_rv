// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace rm_serial_driver
{
struct ReceivePacket
{
  uint8_t header = 0x5A;
  uint8_t detect_color : 1;  // 0-red 1-blue
  bool reset_tracker : 1;
  uint8_t reserved : 6;
  float Yun_yaw;  //大云台
  float roll;
  float pitch;
  float yaw;
  float roll2;
  float pitch2;
  float yaw2;
  float aim_x;
  float aim_y;
  float aim_z;
  uint16_t hp;
  uint16_t time;
  uint8_t mode; //0-原地 1-进攻 2-防守
  uint16_t checksum = 0;
} __attribute__((packed));

struct SendSentry   //发送导航包
{
  uint8_t header = 0xA6;
  float vx;
  float vy;
  uint16_t checksum = 0;
  /* data */
}__attribute__((packed));

inline ReceivePacket fromVector(const std::vector<uint8_t> & data)
{
  ReceivePacket packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

inline std::vector<uint8_t> toVectorSentry(const SendSentry & data)
{
  std::vector<uint8_t> packet(sizeof(SendSentry));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SendSentry), packet.begin());
  return packet;
}

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
