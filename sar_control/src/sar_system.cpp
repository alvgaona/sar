#include "sar_control/sar_system.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace sar_control
{

hardware_interface::CallbackReturn SarSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  device_ = info_.hardware_parameters["device"];
  baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);

  auto it = info_.hardware_parameters.find("mock");
  mock_ = (it != info_.hardware_parameters.end() &&
    (it->second == "true" || it->second == "True"));

  auto wr = info_.hardware_parameters.find("wheel_radius");
  if (wr != info_.hardware_parameters.end()) {
    wheel_radius_ = std::stod(wr->second);
  }

  if (info_.joints.size() != 4) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SarSystemHardware"),
      "Expected 4 joints, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("SarSystemHardware"),
        "Joint '%s' must have exactly one velocity command interface", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2) {
      RCLCPP_ERROR(
        rclcpp::get_logger("SarSystemHardware"),
        "Joint '%s' must have exactly two state interfaces", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SarSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  comms_.connect(device_, baud_rate_, mock_);
  if (!comms_.connected()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SarSystemHardware"),
      "Failed to connect to Arduino on %s", device_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("SarSystemHardware"),
    "Connected to Arduino on %s at %d baud", device_.c_str(), baud_rate_);

  char *color_env = std::getenv("RCUTILS_COLORIZED_OUTPUT");
  if ((color_env == nullptr) || (atoi(color_env)!=0)) {
    RCLCPP_INFO(
      rclcpp::get_logger("SarSystemHardware"),
      "\033[96m");
  }
  RCLCPP_INFO(
  rclcpp::get_logger("SarSystemHardware"),
  "Sleeping for 2 seconds to allow Arduino to reset...\033[0m");

  sleep(2); // Wait for Arduino to reset after serial connection is opened
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SarSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  comms_.disconnect();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SarSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  front_left_ = {};
  front_right_ = {};
  rear_left_ = {};
  rear_right_ = {};
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SarSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  WheelValues zero;
  comms_.send_velocities(zero);
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SarSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(
    info_.joints[0].name, hardware_interface::HW_IF_POSITION, &front_left_.position);
  state_interfaces.emplace_back(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &front_left_.velocity);

  state_interfaces.emplace_back(
    info_.joints[1].name, hardware_interface::HW_IF_POSITION, &front_right_.position);
  state_interfaces.emplace_back(
    info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &front_right_.velocity);

  state_interfaces.emplace_back(
    info_.joints[2].name, hardware_interface::HW_IF_POSITION, &rear_left_.position);
  state_interfaces.emplace_back(
    info_.joints[2].name, hardware_interface::HW_IF_VELOCITY, &rear_left_.velocity);

  state_interfaces.emplace_back(
    info_.joints[3].name, hardware_interface::HW_IF_POSITION, &rear_right_.position);
  state_interfaces.emplace_back(
    info_.joints[3].name, hardware_interface::HW_IF_VELOCITY, &rear_right_.velocity);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SarSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &front_left_.velocity_command);
  command_interfaces.emplace_back(
    info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &front_right_.velocity_command);
  command_interfaces.emplace_back(
    info_.joints[2].name, hardware_interface::HW_IF_VELOCITY, &rear_left_.velocity_command);
  command_interfaces.emplace_back(
    info_.joints[3].name, hardware_interface::HW_IF_VELOCITY, &rear_right_.velocity_command);

  return command_interfaces;
}

hardware_interface::return_type SarSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!comms_.connected()) {
    return hardware_interface::return_type::ERROR;
  }

  WheelValues old_values = {
    front_left_.position,
    front_right_.position,
    rear_left_.position,
    rear_right_.position
  };

  double inv_scale = (wheel_radius_ > 0.0) ? 1.0 / (wheel_radius_ * 1000.0) : 1.0;
  WheelValues encoder_values = comms_.read_encoder_values();
  front_left_.position = encoder_values.front_left * inv_scale;
  front_right_.position = encoder_values.front_right * inv_scale;
  rear_left_.position = encoder_values.rear_left * inv_scale;
  rear_right_.position = encoder_values.rear_right * inv_scale;

  front_left_.velocity = (front_left_.position - old_values.front_left) / period.seconds();
  front_right_.velocity = (front_right_.position - old_values.front_right) / period.seconds();
  rear_left_.velocity = (rear_left_.position - old_values.rear_left) / period.seconds();
  rear_right_.velocity = (rear_right_.position - old_values.rear_right) / period.seconds();

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SarSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comms_.connected()) {
    return hardware_interface::return_type::ERROR;
  }

  double scale = (wheel_radius_ > 0.0) ? wheel_radius_ * 1000.0 : 1.0;
  WheelValues cmd;
  cmd.front_left = front_left_.velocity_command * scale;
  cmd.front_right = front_right_.velocity_command * scale;
  cmd.rear_left = rear_left_.velocity_command * scale;
  cmd.rear_right = rear_right_.velocity_command * scale;
  comms_.send_velocities(cmd);

  return hardware_interface::return_type::OK;
}

}  // namespace sar_control

PLUGINLIB_EXPORT_CLASS(
  sar_control::SarSystemHardware,
  hardware_interface::SystemInterface)
