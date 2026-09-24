// RoboCraft ros2_control hardware interface: Raspberry Pi (I2C master) -> one
// ATmega328 joint controller per arm (I2C slave), exactly the thesis topology,
// with a framed + CRC-8 protected protocol (see robocraft_kinematics/protocol.py).
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace robocraft_hardware
{

namespace protocol
{
constexpr uint8_t kCmdHeader = 0xA5;
constexpr uint8_t kStateHeader = 0x5A;
constexpr size_t kCmdLen = 9;
constexpr size_t kStateLen = 8;

uint8_t crc8(const uint8_t * data, size_t len);
std::array<uint8_t, kCmdLen> encode_command(
  uint8_t seq, double q1_rad, double q2_rad, uint8_t gripper_deg, bool enable, bool brake,
  bool clear_fault);
// Returns false on header/CRC error.
bool decode_state(
  const uint8_t * frame, uint8_t & seq, double & q1_rad, double & q2_rad, uint8_t & status);
}  // namespace protocol

class RoboCraftI2CSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RoboCraftI2CSystem)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct Arm
  {
    std::string prefix;
    int address{0};
    size_t j1{0}, j2{0}, finger{0};   // indices into joint arrays
    uint8_t seq{0};
    int errors{0};
    uint8_t status{0};
  };

  bool transfer(Arm & arm, bool enable, bool clear_fault);

  std::string device_;
  int fd_{-1};
  bool loopback_{false};
  int max_errors_{5};
  std::vector<std::string> virtual_suffixes_;

  std::vector<Arm> arms_;
  std::vector<std::string> names_;
  std::vector<double> pos_, vel_, cmd_, prev_pos_;
  std::vector<bool> virtual_;
  std::vector<int> mimic_of_;       // -1 if not a mimic joint
  std::vector<double> mimic_mult_;
};

}  // namespace robocraft_hardware
