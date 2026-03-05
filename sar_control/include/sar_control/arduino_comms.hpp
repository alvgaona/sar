#ifndef SAR_CONTROL__ARDUINO_COMMS_HPP_
#define SAR_CONTROL__ARDUINO_COMMS_HPP_

#include <boost/asio.hpp>
#include <string>

namespace sar_control
{

struct WheelValues
{
  double front_left = 0.0;
  double front_right = 0.0;
  double rear_left = 0.0;
  double rear_right = 0.0;
};

class ArduinoComms
{
public:
  ArduinoComms() = default;
  ~ArduinoComms();

  void connect(const std::string & device, int baud_rate, bool mock = false);
  void disconnect();
  bool connected() const;

  void send_velocities(const WheelValues & velocities_mm_s);
  WheelValues read_encoder_values();

private:
  std::string read_line();
  void write_line(const std::string & msg);

  bool mock_ = false;
  bool mock_connected_ = false;
  uint64_t mock_counter_ = 0;
  boost::asio::io_context io_context_;
  std::unique_ptr<boost::asio::serial_port> serial_port_;
};

}  // namespace sar_control

#endif  // SAR_CONTROL__ARDUINO_COMMS_HPP_
