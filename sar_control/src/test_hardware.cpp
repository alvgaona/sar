#include <chrono>
#include <thread>

#include <pluginlib/class_loader.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
    "hardware_interface", "hardware_interface::SystemInterface");

  auto hw = loader.createUniqueInstance("sar_control/SarSystemHardware");

  hardware_interface::HardwareInfo info;
  info.name = "sar_system";
  info.type = "system";
  info.hardware_parameters["device"] = "/dev/ttyACM0";
  info.hardware_parameters["baud_rate"] = "115200";
  info.hardware_parameters["mock"] = "True";

  auto make_joint = [](const std::string & name) {
    hardware_interface::ComponentInfo joint;
    joint.name = name;
    joint.command_interfaces.push_back({"velocity", "", ""});
    joint.state_interfaces.push_back({"position", "", ""});
    joint.state_interfaces.push_back({"velocity", "", ""});
    return joint;
  };

  info.joints.push_back(make_joint("fl_wheel_joint"));
  info.joints.push_back(make_joint("fr_wheel_joint"));
  info.joints.push_back(make_joint("rl_wheel_joint"));
  info.joints.push_back(make_joint("rr_wheel_joint"));

  RCLCPP_INFO(rclcpp::get_logger("test"), "Initializing...");
  hw->on_init(info);

  RCLCPP_INFO(rclcpp::get_logger("test"), "Configuring...");
  hw->on_configure(rclcpp_lifecycle::State());

  RCLCPP_INFO(rclcpp::get_logger("test"), "Activating...");
  hw->on_activate(rclcpp_lifecycle::State());

  auto state_interfaces = hw->export_state_interfaces();
  auto command_interfaces = hw->export_command_interfaces();

  RCLCPP_INFO(rclcpp::get_logger("test"), "Setting velocity commands: 100, -100, 100, -100 mm/s");
  command_interfaces[0].set_value(100.0);
  command_interfaces[1].set_value(-100.0);
  command_interfaces[2].set_value(100.0);
  command_interfaces[3].set_value(-100.0);

  rclcpp::Time now(0, 0, RCL_ROS_TIME);
  rclcpp::Duration period(0, 20000000);

  for (int i = 0; i < 5; ++i) {
    hw->read(now, period);
    hw->write(now, period);
    RCLCPP_INFO(rclcpp::get_logger("test"), "Cycle %d — positions: [%.1f, %.1f, %.1f, %.1f]",
      i + 1,
      state_interfaces[0].get_value(),
      state_interfaces[2].get_value(),
      state_interfaces[4].get_value(),
      state_interfaces[6].get_value());
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  RCLCPP_INFO(rclcpp::get_logger("test"), "Deactivating...");
  hw->on_deactivate(rclcpp_lifecycle::State());

  RCLCPP_INFO(rclcpp::get_logger("test"), "Cleaning up...");
  hw->on_cleanup(rclcpp_lifecycle::State());

  RCLCPP_INFO(rclcpp::get_logger("test"), "Done!");
  rclcpp::shutdown();
  return 0;
}
