#include "sar_control/arduino_comms.hpp"

#include <iostream>
#include <sstream>
#include <vector>

namespace sar_control
{

ArduinoComms::~ArduinoComms()
{
  disconnect();
}

void ArduinoComms::connect(const std::string & device, int baud_rate, bool mock)
{
  mock_ = mock;

  if (mock_) {
    mock_connected_ = true;
    std::cout << "[MOCK] Connected to " << device << " at " << baud_rate << " baud" << std::endl;
    return;
  }

  serial_port_ = std::make_unique<boost::asio::serial_port>(io_context_);
  serial_port_->open(device);
  serial_port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
  serial_port_->set_option(
    boost::asio::serial_port_base::character_size(8));
  serial_port_->set_option(
    boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
  serial_port_->set_option(
    boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
  serial_port_->set_option(
    boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
}

void ArduinoComms::disconnect()
{
  if (mock_) {
    if (mock_connected_) {
      std::cout << "[MOCK] Disconnected" << std::endl;
      mock_connected_ = false;
    }
    return;
  }

  if (serial_port_ && serial_port_->is_open()) {
    serial_port_->close();
  }
  serial_port_.reset();
}

bool ArduinoComms::connected() const
{
  if (mock_) {
    return mock_connected_;
  }
  return serial_port_ && serial_port_->is_open();
}

void ArduinoComms::send_velocities(const WheelValues & velocities_mm_s)
{
  std::ostringstream oss;
  oss << "v:"
      << static_cast<int>(velocities_mm_s.front_left) << ","
      << static_cast<int>(velocities_mm_s.front_right) << ","
      << static_cast<int>(velocities_mm_s.rear_left) << ","
      << static_cast<int>(velocities_mm_s.rear_right);

  if (mock_) {
    if (++mock_counter_ % 50 == 0) {
      std::cout << "[MOCK] TX: " << oss.str() << std::endl;
    }
    return;
  }

  write_line(oss.str());
}

WheelValues ArduinoComms::read_encoder_values()
{
  if (mock_) {
    return {};
  }

  write_line("e");
  std::string response = read_line();

  WheelValues values;
  if (response.substr(0, 2) != "e:") {
    return values;
  }

  std::istringstream iss(response.substr(2));
  std::string token;
  std::vector<double> parsed;
  while (std::getline(iss, token, ',')) {
    parsed.push_back(std::stod(token));
  }

  if (parsed.size() == 4) {
    values.front_left = parsed[0];
    values.front_right = parsed[1];
    values.rear_left = parsed[2];
    values.rear_right = parsed[3];
  }

  return values;
}

std::string ArduinoComms::read_line()
{
  boost::asio::streambuf buf;
  boost::asio::read_until(*serial_port_, buf, '\n');
  std::string line(
    boost::asio::buffers_begin(buf.data()),
    boost::asio::buffers_end(buf.data()));

  if (!line.empty() && line.back() == '\n') {
    line.pop_back();
  }
  if (!line.empty() && line.back() == '\r') {
    line.pop_back();
  }

  return line;
}

void ArduinoComms::write_line(const std::string & msg)
{
  std::string data = msg + "\n";
  boost::asio::write(*serial_port_, boost::asio::buffer(data));
}

}  // namespace sar_control
