#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace skye_robot_driver {
namespace dm_mit {

// DM4310 MIT limits (aligned with Thor gripper_bridge.py / DM_CAN.py).
constexpr double kQMax = 12.5;
constexpr double kDqMax = 30.0;
constexpr double kTauMax = 10.0;
constexpr std::size_t kCanMtu = 8;
constexpr std::size_t kTerminalPackedLen = 12;  // 4B CAN ID LE + 8B data

double clamp(double x, double lo, double hi);
int float_to_uint(double x, double x_min, double x_max, int bits);
double uint_to_float(int x, double vmin, double vmax, int bits);

// Encode MIT control frame (8 bytes).
std::array<std::uint8_t, 8> encode_mit(
    double kp, double kd, double q, double dq, double tau);

struct Feedback {
  double pos{0.0};
  double vel{0.0};
  double torque{0.0};
  int err_code{0};
  std::string err_msg;
  int temp_mos{0};
  int temp_motor{0};
};

std::optional<Feedback> decode_feedback(const std::uint8_t *data, std::size_t len);

// CAN ID (LE 4B) + data[0..7] → terminal payload.
std::vector<std::uint8_t> pack_terminal(std::uint32_t can_id, const std::uint8_t *data,
                                        std::size_t data_len);

std::optional<std::pair<std::uint32_t, std::array<std::uint8_t, 8>>> unpack_terminal(
    const std::uint8_t *payload, std::size_t len);

// Enable / disable frames for DM motor (data only, 8 bytes).
inline std::array<std::uint8_t, 8> enable_frame() {
  return {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
}
inline std::array<std::uint8_t, 8> disable_frame() {
  return {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
}

}  // namespace dm_mit
}  // namespace skye_robot_driver
