#pragma once

#include <cstdint>
#include <optional>
#include <vector>

namespace skye_robot_driver {
namespace modbus_rtu {

inline std::uint16_t crc16(const std::uint8_t *data, std::size_t len) {
  std::uint16_t crc = 0xFFFF;
  for (std::size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      if (crc & 1) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

inline std::vector<std::uint8_t> read_holding(
    int slave, std::uint16_t addr, std::uint16_t count = 1) {
  std::vector<std::uint8_t> frame{
      static_cast<std::uint8_t>(slave), 0x03,
      static_cast<std::uint8_t>((addr >> 8) & 0xFF),
      static_cast<std::uint8_t>(addr & 0xFF),
      static_cast<std::uint8_t>((count >> 8) & 0xFF),
      static_cast<std::uint8_t>(count & 0xFF)};
  const auto crc = crc16(frame.data(), frame.size());
  frame.push_back(static_cast<std::uint8_t>(crc & 0xFF));
  frame.push_back(static_cast<std::uint8_t>((crc >> 8) & 0xFF));
  return frame;
}

inline std::vector<std::uint8_t> write_single(
    int slave, std::uint16_t addr, std::uint16_t value) {
  std::vector<std::uint8_t> frame{
      static_cast<std::uint8_t>(slave), 0x06,
      static_cast<std::uint8_t>((addr >> 8) & 0xFF),
      static_cast<std::uint8_t>(addr & 0xFF),
      static_cast<std::uint8_t>((value >> 8) & 0xFF),
      static_cast<std::uint8_t>(value & 0xFF)};
  const auto crc = crc16(frame.data(), frame.size());
  frame.push_back(static_cast<std::uint8_t>(crc & 0xFF));
  frame.push_back(static_cast<std::uint8_t>((crc >> 8) & 0xFF));
  return frame;
}

inline std::optional<std::uint16_t> parse_read_u16(
    const std::uint8_t *data, std::size_t len, int slave) {
  if (len < 7 || data[0] != static_cast<std::uint8_t>(slave) || data[1] != 0x03) {
    return std::nullopt;
  }
  const std::size_t byte_count = data[2];
  if (byte_count != 2 || len < 3 + byte_count + 2) {
    return std::nullopt;
  }
  const std::size_t body_len = 3 + byte_count;
  if (crc16(data, body_len) !=
      static_cast<std::uint16_t>(data[body_len]) |
          (static_cast<std::uint16_t>(data[body_len + 1]) << 8)) {
    return std::nullopt;
  }
  return static_cast<std::uint16_t>((data[3] << 8) | data[4]);
}

}  // namespace modbus_rtu
}  // namespace skye_robot_driver
