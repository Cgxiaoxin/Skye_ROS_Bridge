#include "skye_robot_driver/dm_mit.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace skye_robot_driver {
namespace dm_mit {
namespace {

const std::unordered_map<int, const char *> kErrorMessages{
    {0x0, "Off"},       {0x1, "On"},        {0x8, "过压"},
    {0x9, "欠压"},      {0xA, "过流"},      {0xB, "MOS过温"},
    {0xC, "线圈过温"},  {0xD, "通讯丢失"},  {0xE, "过载"},
};

}  // namespace

double clamp(double x, double lo, double hi) {
  if (x <= lo) {
    return lo;
  }
  if (x > hi) {
    return hi;
  }
  return x;
}

int float_to_uint(double x, double x_min, double x_max, int bits) {
  x = clamp(x, x_min, x_max);
  const double span = x_max - x_min;
  return static_cast<int>((x - x_min) / span * ((1 << bits) - 1));
}

double uint_to_float(int x, double vmin, double vmax, int bits) {
  const double span = vmax - vmin;
  return static_cast<double>(x) / static_cast<double>((1 << bits) - 1) * span +
         vmin;
}

std::array<std::uint8_t, 8> encode_mit(
    double kp, double kd, double q, double dq, double tau) {
  const int kp_u = float_to_uint(kp, 0.0, 500.0, 12);
  const int kd_u = float_to_uint(kd, 0.0, 5.0, 12);
  const int q_u = float_to_uint(q, -kQMax, kQMax, 16);
  const int dq_u = float_to_uint(dq, -kDqMax, kDqMax, 12);
  const int tau_u = float_to_uint(tau, -kTauMax, kTauMax, 12);

  std::array<std::uint8_t, 8> buf{};
  buf[0] = static_cast<std::uint8_t>((q_u >> 8) & 0xFF);
  buf[1] = static_cast<std::uint8_t>(q_u & 0xFF);
  buf[2] = static_cast<std::uint8_t>(dq_u >> 4);
  buf[3] = static_cast<std::uint8_t>(((dq_u & 0xF) << 4) | ((kp_u >> 8) & 0xF));
  buf[4] = static_cast<std::uint8_t>(kp_u & 0xFF);
  buf[5] = static_cast<std::uint8_t>(kd_u >> 4);
  buf[6] = static_cast<std::uint8_t>(((kd_u & 0xF) << 4) | ((tau_u >> 8) & 0xF));
  buf[7] = static_cast<std::uint8_t>(tau_u & 0xFF);
  return buf;
}

std::optional<Feedback> decode_feedback(const std::uint8_t *data, std::size_t len) {
  if (data == nullptr || len < 8) {
    return std::nullopt;
  }
  Feedback fb;
  fb.err_code = (data[0] & 0xF0) >> 4;
  fb.pos = uint_to_float((data[1] << 8) | data[2], -kQMax, kQMax, 16);
  fb.vel = uint_to_float((data[3] << 4) | (data[4] >> 4), -kDqMax, kDqMax, 12);
  fb.torque =
      uint_to_float(((data[4] & 0xF) << 8) | data[5], -kTauMax, kTauMax, 12);
  fb.temp_mos = data[6];
  fb.temp_motor = data[7];
  const auto it = kErrorMessages.find(fb.err_code);
  fb.err_msg = it != kErrorMessages.end()
                   ? it->second
                   : ("未知(0x" + std::to_string(fb.err_code) + ")");
  return fb;
}

std::vector<std::uint8_t> pack_terminal(
    std::uint32_t can_id, const std::uint8_t *data, std::size_t data_len) {
  std::vector<std::uint8_t> out(4 + kCanMtu, 0);
  out[0] = static_cast<std::uint8_t>(can_id & 0xFF);
  out[1] = static_cast<std::uint8_t>((can_id >> 8) & 0xFF);
  out[2] = static_cast<std::uint8_t>((can_id >> 16) & 0xFF);
  out[3] = static_cast<std::uint8_t>((can_id >> 24) & 0xFF);
  const std::size_t n = std::min(data_len, kCanMtu);
  if (data != nullptr && n > 0) {
    std::copy(data, data + n, out.begin() + 4);
  }
  return out;
}

std::optional<std::pair<std::uint32_t, std::array<std::uint8_t, 8>>> unpack_terminal(
    const std::uint8_t *payload, std::size_t len) {
  if (payload == nullptr || len < 5) {
    return std::nullopt;
  }
  const std::uint32_t can_id =
      static_cast<std::uint32_t>(payload[0]) |
      (static_cast<std::uint32_t>(payload[1]) << 8) |
      (static_cast<std::uint32_t>(payload[2]) << 16) |
      (static_cast<std::uint32_t>(payload[3]) << 24);
  std::array<std::uint8_t, 8> data{};
  const std::size_t n = std::min(len - 4, kCanMtu);
  std::copy(payload + 4, payload + 4 + n, data.begin());
  return std::make_pair(can_id, data);
}

}  // namespace dm_mit
}  // namespace skye_robot_driver
