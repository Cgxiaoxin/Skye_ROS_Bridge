#include "skye_robot_driver/robotiq_gripper_arm.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <thread>

#include "skye_robot_driver/modbus_rtu.hpp"

namespace skye_robot_driver {
namespace {

constexpr int kFeedbackEveryTicks = 10;

}  // namespace

RobotiqGripperArm::RobotiqGripperArm(
    DriverCore &core, DriverCore::Arm arm, Config config)
    : core_(core), arm_(arm), config_(config) {}

FXTerminalType RobotiqGripperArm::terminal() const {
  return config_.terminal == 0 ? FX_TERMINAL_ARM0 : FX_TERMINAL_ARM1;
}

std::uint16_t RobotiqGripperArm::action_reg(
    int r_act, int r_gto, int r_atr, int r_ard) {
  return static_cast<std::uint16_t>(
      (r_act << 8) | (r_gto << 11) | (r_atr << 12) | (r_ard << 13));
}

bool RobotiqGripperArm::tx_modbus(
    const std::uint8_t *data, std::size_t len) {
  const auto tx_timeout_ms = std::max(
      config_.modbus_timeout_ms, static_cast<unsigned int>(200));
  return core_.terminal_set(
      terminal(), config_.channel, data, len, tx_timeout_ms);
}

std::optional<std::uint16_t> RobotiqGripperArm::modbus_read(
    std::uint16_t addr) {
  return modbus_read(addr, nullptr);
}

std::optional<std::uint16_t> RobotiqGripperArm::modbus_read(
    std::uint16_t addr, std::string *comm_detail) {
  core_.terminal_clear(terminal());
  const auto req = modbus_rtu::read_holding(config_.slave_id, addr, 1);
  if (!tx_modbus(req.data(), req.size())) {
    if (comm_detail != nullptr) {
      *comm_detail = "tx_setdata_failed";
    }
    return std::nullopt;
  }
  // RS-485 往返在 SDK mutex 竞争下常 >150ms；与 test_robotiq_right_485.py 对齐 1s。
  const auto read_budget_ms = std::max(
      config_.modbus_timeout_ms, static_cast<unsigned int>(1000));
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(read_budget_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    const auto packet = core_.terminal_get(terminal(), 100);
    if (!packet || packet->data.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }
    const auto value = modbus_rtu::parse_read_u16(
        packet->data.data(), packet->data.size(), config_.slave_id);
    if (value) {
      return value;
    }
    if (comm_detail != nullptr && comm_detail->empty()) {
      std::ostringstream oss;
      oss << "rx_unparsed n=" << packet->data.size() << " hex=";
      for (const auto byte : packet->data) {
        oss << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<int>(byte) << ' ';
      }
      *comm_detail = oss.str();
    }
  }
  if (comm_detail != nullptr && comm_detail->empty()) {
    *comm_detail = "rx_timeout";
  }
  return std::nullopt;
}

bool RobotiqGripperArm::modbus_write(std::uint16_t addr, std::uint16_t value) {
  core_.terminal_clear(terminal());
  const auto req = modbus_rtu::write_single(config_.slave_id, addr, value);
  if (!tx_modbus(req.data(), req.size())) {
    return false;
  }
  const auto write_budget_ms = std::max(
      config_.modbus_timeout_ms, static_cast<unsigned int>(1000));
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(write_budget_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    const auto packet = core_.terminal_get(terminal(), 100);
    if (packet && packet->data.size() >= 8 &&
        packet->data[0] == static_cast<std::uint8_t>(config_.slave_id) &&
        packet->data[1] == 0x06) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return false;
}

bool RobotiqGripperArm::reset_gripper() {
  if (!modbus_write(kRegAction, action_reg(0))) {
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  return true;
}

bool RobotiqGripperArm::activate_gripper() {
  if (!modbus_write(kRegAction, action_reg(1))) {
    return false;
  }
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (std::chrono::steady_clock::now() < deadline) {
    const auto status = modbus_read(kRegStatus);
    if (status) {
      const int g_sta = ((status.value() >> 8) & 0x30) >> 4;
      if (g_sta == kGstaActivated) {
        return true;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  const auto status = modbus_read(kRegStatus);
  if (status) {
    const int g_sta = ((status.value() >> 8) & 0x30) >> 4;
    return g_sta == kGstaActivated;
  }
  return false;
}

double RobotiqGripperArm::norm_to_mm(double norm) const {
  return config_.pos_max_mm - norm * (config_.pos_max_mm - config_.pos_min_mm);
}

double RobotiqGripperArm::mm_to_norm(double opening_mm) const {
  const double span = config_.pos_max_mm - config_.pos_min_mm;
  if (span <= 0.0) {
    return 0.0;
  }
  const double norm = (config_.pos_max_mm - opening_mm) / span;
  if (norm < 0.0) {
    return 0.0;
  }
  if (norm > 1.0) {
    return 1.0;
  }
  return norm;
}

double RobotiqGripperArm::read_opening_mm() {
  const auto reg = modbus_read(kRegPosCur);
  if (!reg) {
    return config_.pos_max_mm;
  }
  const int pos_byte = (reg.value() >> 8) & 0xFF;
  return kFullPosMm - pos_byte * kPosRatio;
}

bool RobotiqGripperArm::write_pending() {
  double norm = 0.0;
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    norm = target_;
    dirty_ = false;
  }
  const double opening_mm = norm_to_mm(norm);
  const int reg_pos = std::max(
      0, std::min(
             255, static_cast<int>(std::lround(
                      (kFullPosMm - opening_mm) / kPosRatio))));
  if (!modbus_write(kRegPos, static_cast<std::uint16_t>(reg_pos))) {
    std::lock_guard<std::mutex> lock(target_mutex_);
    dirty_ = true;
    return false;
  }
  const std::uint16_t speed_force =
      static_cast<std::uint16_t>(
          ((config_.speed & 0xFF) << 8) | (config_.force & 0xFF));
  if (!modbus_write(kRegSpeed, speed_force)) {
    std::lock_guard<std::mutex> lock(target_mutex_);
    dirty_ = true;
    return false;
  }
  if (!modbus_write(kRegAction, action_reg(1, 1))) {
    std::lock_guard<std::mutex> lock(target_mutex_);
    dirty_ = true;
    return false;
  }
  return true;
}

bool RobotiqGripperArm::start(std::string *report) {
  if (started_) {
    return true;
  }
  if (config_.slave_id <= 0 || config_.slave_id > 247 ||
      config_.pos_max_mm <= config_.pos_min_mm ||
      config_.close_limit <= 0.0 || config_.close_limit > 1.0 ||
      config_.modbus_timeout_ms == 0 ||
      (config_.terminal != 0 && config_.terminal != 1)) {
    if (report != nullptr) {
      *report = "invalid robotiq config";
    }
    return false;
  }
  core_.terminal_clear(terminal());
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::string comm_detail;
  const auto pre_status = modbus_read(kRegStatus, &comm_detail);
  bool ok = false;
  if (!pre_status) {
    if (report != nullptr) {
      *report = "robotiq comm=FAIL (" + comm_detail + ")";
    }
    return false;
  }
  const int pre_gsta = ((pre_status.value() >> 8) & 0x30) >> 4;
  if (pre_gsta == kGstaActivated) {
    ok = true;
  } else {
    ok = reset_gripper() && activate_gripper();
  }
  const auto status = modbus_read(kRegStatus);
  std::ostringstream oss;
  oss << "robotiq slave=" << config_.slave_id
      << " term=ARM" << (config_.terminal == 0 ? 0 : 1)
      << " chn=" << (config_.channel == FX_CHN_485B ? "485B" : "485A");
  if (status) {
    oss << " status=0x" << std::hex << status.value() << std::dec;
  }
  oss << (ok ? " activated=ok" : " activated=FAIL");
  if (report != nullptr) {
    *report = oss.str();
  }
  if (!ok) {
    return false;
  }
  // Teleop default / FACTR released: motor norm 0 → fully open (pos_max_mm).
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    target_ = 0.0;
    dirty_ = true;
  }
  if (!write_pending()) {
    if (report != nullptr) {
      *report += " open_cmd=FAIL";
    }
  }
  const double opening = read_opening_mm();
  GripperFeedback parsed;
  parsed.valid = true;
  parsed.position = mm_to_norm(opening);
  parsed.device_id = static_cast<std::uint32_t>(config_.slave_id);
  parsed.frame_tag = "robotiq_" + std::to_string(config_.slave_id);
  {
    std::lock_guard<std::mutex> lock(fb_mutex_);
    fb_ = parsed;
  }
  started_ = true;
  return true;
}

void RobotiqGripperArm::stop() {
  if (!started_) {
    return;
  }
  modbus_write(kRegAction, action_reg(1, 0));
  started_ = false;
}

void RobotiqGripperArm::set_target(double norm) {
  const double clamped = std::max(0.0, std::min(config_.close_limit, norm));
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    if (clamped != target_) {
      target_ = clamped;
      dirty_ = true;
    }
  }
}

double RobotiqGripperArm::target() const {
  std::lock_guard<std::mutex> lock(target_mutex_);
  return target_;
}

void RobotiqGripperArm::tick_control() {
  if (!started_) {
    return;
  }
  bool should_write = false;
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    should_write = dirty_;
  }
  if (should_write) {
    write_pending();
  }
}

void RobotiqGripperArm::tick_feedback() {
  if (!started_) {
    return;
  }
  ++feedback_ticks_;
  if (feedback_ticks_ % kFeedbackEveryTicks != 0) {
    return;
  }
  const double opening = read_opening_mm();
  GripperFeedback parsed;
  parsed.valid = true;
  parsed.position = mm_to_norm(opening);
  parsed.device_id = static_cast<std::uint32_t>(config_.slave_id);
  parsed.frame_tag = "robotiq_" + std::to_string(config_.slave_id);
  std::lock_guard<std::mutex> lock(fb_mutex_);
  fb_ = parsed;
}

GripperFeedback RobotiqGripperArm::feedback() const {
  std::lock_guard<std::mutex> lock(fb_mutex_);
  return fb_;
}

const char *RobotiqGripperArm::type_name() const { return "robotiq"; }

std::string RobotiqGripperArm::describe() const {
  return "robotiq slave=" + std::to_string(config_.slave_id) +
         " term=ARM" + std::to_string(config_.terminal);
}

}  // namespace skye_robot_driver
