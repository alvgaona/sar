#ifndef SAR_CONTROL__SAR_SYSTEM_HPP_
#define SAR_CONTROL__SAR_SYSTEM_HPP_

#include "sar_control/arduino_comms.hpp"

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <string>
#include <vector>

namespace sar_control
{

class SarSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SarSystemHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  ArduinoComms comms_;
  std::string device_;
  int baud_rate_ = 115200;
  bool mock_ = false;

  struct WheelState
  {
    double position = 0.0;
    double velocity = 0.0;
    double velocity_command = 0.0;
  };

  WheelState front_left_;
  WheelState front_right_;
  WheelState rear_left_;
  WheelState rear_right_;
};

}  // namespace sar_control

#endif  // SAR_CONTROL__SAR_SYSTEM_HPP_
