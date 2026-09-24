#include "robocraft_hardware/robocraft_i2c_system.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <sstream>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robocraft_hardware
{
namespace
{
const rclcpp::Logger kLog = rclcpp::get_logger("RoboCraftI2CSystem");

std::vector<std::string> split(const std::string & s)
{
  std::vector<std::string> out;
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, ',')) {
    item.erase(std::remove_if(item.begin(), item.end(), ::isspace), item.end());
    if (!item.empty()) {out.push_back(item);}
  }
  return out;
}

std::string param(const hardware_interface::HardwareInfo & info, const std::string & k, const std::string & d)
{
  auto it = info.hardware_parameters.find(k);
  return it == info.hardware_parameters.end() ? d : it->second;
}

int16_t to_centideg(double rad)
{
  const double v = std::round(rad * 18000.0 / M_PI);
  return static_cast<int16_t>(std::clamp(v, -32768.0, 32767.0));
}
}  // namespace

// ------------------------------------------------------------------ protocol
namespace protocol
{
uint8_t crc8(const uint8_t * data, size_t len)
{
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x07) : static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}

std::array<uint8_t, kCmdLen> encode_command(
  uint8_t seq, double q1, double q2, uint8_t grip, bool enable, bool brake, bool clear_fault)
{
  std::array<uint8_t, kCmdLen> f{};
  const int16_t a = to_centideg(q1), b = to_centideg(q2);
  f[0] = kCmdHeader;
  f[1] = seq;
  f[2] = static_cast<uint8_t>((a >> 8) & 0xFF);
  f[3] = static_cast<uint8_t>(a & 0xFF);
  f[4] = static_cast<uint8_t>((b >> 8) & 0xFF);
  f[5] = static_cast<uint8_t>(b & 0xFF);
  f[6] = std::min<uint8_t>(grip, 180);
  f[7] = static_cast<uint8_t>((enable ? 1 : 0) | (brake ? 2 : 0) | (clear_fault ? 4 : 0));
  f[8] = crc8(f.data(), kCmdLen - 1);
  return f;
}

bool decode_state(const uint8_t * f, uint8_t & seq, double & q1, double & q2, uint8_t & status)
{
  if (f[0] != kStateHeader || crc8(f, kStateLen - 1) != f[kStateLen - 1]) {return false;}
  seq = f[1];
  const int16_t a = static_cast<int16_t>((f[2] << 8) | f[3]);
  const int16_t b = static_cast<int16_t>((f[4] << 8) | f[5]);
  q1 = a * M_PI / 18000.0;
  q2 = b * M_PI / 18000.0;
  status = f[6];
  return true;
}
}  // namespace protocol

// ------------------------------------------------------------------ lifecycle
hardware_interface::CallbackReturn RoboCraftI2CSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {return CallbackReturn::ERROR;}

  device_ = param(info, "i2c_device", "/dev/i2c-1");
  loopback_ = param(info, "loopback", "false") == "true";
  max_errors_ = std::stoi(param(info, "max_consecutive_errors", "5"));
  virtual_suffixes_ = split(param(info, "virtual_joints", "joint3,joint4"));
  const auto prefixes = split(param(info, "arm_prefixes", "arm1,arm2,arm3"));
  const auto addresses = split(param(info, "i2c_addresses", "8,9,10"));
  if (prefixes.size() != addresses.size()) {
    RCLCPP_FATAL(kLog, "arm_prefixes and i2c_addresses must have the same length");
    return CallbackReturn::ERROR;
  }

  const size_t n = info.joints.size();
  names_.resize(n);
  pos_.assign(n, 0.0);
  vel_.assign(n, 0.0);
  cmd_.assign(n, 0.0);
  prev_pos_.assign(n, 0.0);
  virtual_.assign(n, false);
  mimic_of_.assign(n, -1);
  mimic_mult_.assign(n, 1.0);
  for (size_t i = 0; i < n; ++i) {
    const auto & j = info.joints[i];
    names_[i] = j.name;
    for (const auto & si : j.state_interfaces) {
      if (si.name == hardware_interface::HW_IF_POSITION && !si.initial_value.empty()) {
        pos_[i] = cmd_[i] = prev_pos_[i] = std::stod(si.initial_value);
      }
    }
    for (const auto & s : virtual_suffixes_) {
      if (j.name.size() >= s.size() && j.name.compare(j.name.size() - s.size(), s.size(), s) == 0) {
        virtual_[i] = true;
      }
    }
  }
  auto index_of = [&](const std::string & name) -> int {
      auto it = std::find(names_.begin(), names_.end(), name);
      return it == names_.end() ? -1 : static_cast<int>(it - names_.begin());
    };
  for (size_t i = 0; i < n; ++i) {
    const auto & p = info.joints[i].parameters;
    if (p.count("mimic")) {
      mimic_of_[i] = index_of(p.at("mimic"));
      if (p.count("multiplier")) {mimic_mult_[i] = std::stod(p.at("multiplier"));}
    }
  }
  for (size_t k = 0; k < prefixes.size(); ++k) {
    Arm a;
    a.prefix = prefixes[k];
    a.address = std::stoi(addresses[k], nullptr, 0);
    const int j1 = index_of(a.prefix + "_joint1"), j2 = index_of(a.prefix + "_joint2");
    const int fg = index_of(a.prefix + "_finger_left_joint");
    if (j1 < 0 || j2 < 0 || fg < 0) {
      RCLCPP_FATAL(kLog, "joints of arm '%s' missing in the ros2_control tag", a.prefix.c_str());
      return CallbackReturn::ERROR;
    }
    a.j1 = j1;
    a.j2 = j2;
    a.finger = fg;
    arms_.push_back(a);
  }
  RCLCPP_INFO(kLog, "%zu arms on %s%s", arms_.size(), device_.c_str(), loopback_ ? " (LOOPBACK)" : "");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoboCraftI2CSystem::on_configure(const rclcpp_lifecycle::State &)
{
  if (loopback_) {return CallbackReturn::SUCCESS;}
  fd_ = ::open(device_.c_str(), O_RDWR);
  if (fd_ < 0) {
    RCLCPP_FATAL(kLog, "cannot open %s: %s", device_.c_str(), std::strerror(errno));
    return CallbackReturn::ERROR;
  }
  // read the actual joint angles once so the first command does not jump
  for (auto & a : arms_) {
    if (!transfer(a, false, true)) {
      RCLCPP_FATAL(kLog, "arm %s (0x%02x) does not answer", a.prefix.c_str(), a.address);
      return CallbackReturn::ERROR;
    }
    cmd_[a.j1] = pos_[a.j1];
    cmd_[a.j2] = pos_[a.j2];
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoboCraftI2CSystem::on_activate(const rclcpp_lifecycle::State &)
{
  for (size_t i = 0; i < pos_.size(); ++i) {cmd_[i] = pos_[i];}
  for (auto & a : arms_) {a.errors = 0;}
  RCLCPP_INFO(kLog, "motors enabled");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoboCraftI2CSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  for (auto & a : arms_) {
    if (!loopback_) {transfer(a, false, false);}   // disable = H-bridges off
  }
  RCLCPP_INFO(kLog, "motors disabled");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoboCraftI2CSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  if (fd_ >= 0) {::close(fd_);}
  fd_ = -1;
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RoboCraftI2CSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  for (size_t i = 0; i < names_.size(); ++i) {
    out.emplace_back(names_[i], hardware_interface::HW_IF_POSITION, &pos_[i]);
    out.emplace_back(names_[i], hardware_interface::HW_IF_VELOCITY, &vel_[i]);
  }
  return out;
}

std::vector<hardware_interface::CommandInterface> RoboCraftI2CSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  for (size_t i = 0; i < names_.size(); ++i) {
    out.emplace_back(names_[i], hardware_interface::HW_IF_POSITION, &cmd_[i]);
  }
  return out;
}

// ------------------------------------------------------------------ I/O
bool RoboCraftI2CSystem::transfer(Arm & a, bool enable, bool clear_fault)
{
  // finger opening 0..0.022 m per finger  ->  servo 0..180 deg (rack & pinion)
  const double open = std::clamp(cmd_[a.finger] / 0.022, 0.0, 1.0);
  const auto frame = protocol::encode_command(
    ++a.seq, cmd_[a.j1], cmd_[a.j2], static_cast<uint8_t>(std::lround(open * 180.0)), enable, false,
    clear_fault);
  uint8_t rx[protocol::kStateLen];
  if (ioctl(fd_, I2C_SLAVE, a.address) < 0 ||
    ::write(fd_, frame.data(), frame.size()) != static_cast<ssize_t>(frame.size()) ||
    ::read(fd_, rx, sizeof(rx)) != static_cast<ssize_t>(sizeof(rx)))
  {
    return false;
  }
  uint8_t seq, status;
  double q1, q2;
  if (!protocol::decode_state(rx, seq, q1, q2, status)) {return false;}
  pos_[a.j1] = q1;
  pos_[a.j2] = q2;
  a.status = status;
  return true;
}

hardware_interface::return_type RoboCraftI2CSystem::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  const double dt = std::max(period.seconds(), 1e-4);
  for (size_t i = 0; i < pos_.size(); ++i) {
    // virtual joints (J3/J4 not on the 2019 prototype), fingers without encoder, loopback
    const bool measured = !loopback_ && std::any_of(arms_.begin(), arms_.end(), [&](const Arm & a) {
          return a.j1 == i || a.j2 == i;
        });
    if (!measured && mimic_of_[i] < 0) {pos_[i] = cmd_[i];}
  }
  for (size_t i = 0; i < pos_.size(); ++i) {
    if (mimic_of_[i] >= 0) {pos_[i] = mimic_mult_[i] * pos_[mimic_of_[i]];}
    vel_[i] = (pos_[i] - prev_pos_[i]) / dt;
    prev_pos_[i] = pos_[i];
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RoboCraftI2CSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (loopback_) {return hardware_interface::return_type::OK;}
  for (auto & a : arms_) {
    if (transfer(a, true, false)) {
      a.errors = 0;
      if (a.status & 0x02) {
        RCLCPP_ERROR_THROTTLE(kLog, *rclcpp::Clock::make_shared(), 1000, "arm %s reports FAULT (status 0x%02x)",
          a.prefix.c_str(), a.status);
      }
    } else if (++a.errors > max_errors_) {
      RCLCPP_FATAL(kLog, "arm %s: %d consecutive I2C/CRC errors -> stopping", a.prefix.c_str(), a.errors);
      return hardware_interface::return_type::ERROR;
    }
  }
  return hardware_interface::return_type::OK;
}

}  // namespace robocraft_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(robocraft_hardware::RoboCraftI2CSystem, hardware_interface::SystemInterface)
