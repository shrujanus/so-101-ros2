#ifndef SO101_HARDWARE_INTERFACE_HPP
#define SO101_HARDWARE_INTERFACE_HPP

#include "driver/SMS_STS.h"
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>

using hardware_interface::CallbackReturn;

namespace so101_control {

class so101_hardware_interface final
    : public hardware_interface::SystemInterface {
private:
  SMS_STS motor_driver_;
  std::string usb_port_ = "/dev/ttyUSB0";
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;
  std::vector<double> hw_commands_;

public:
  so101_hardware_interface();

  // Lifecycle methods
  CallbackReturn on_configure(const hardware_interface::HardwareInfo &info);
  CallbackReturn on_activate(const hardware_interface::HardwareInfo &info);
  CallbackReturn on_deactivate(const hardware_interface::HardwareInfo &info);
  CallbackReturn on_cleanup(const hardware_interface::HardwareInfo &info);
  CallbackReturn on_shutdown(const hardware_interface::HardwareInfo &info);
  CallbackReturn on_error(const hardware_interface::HardwareInfo &info);

  // Export interfaces
  CallbackReturn
  on_init(const hardware_interface::HardwareInfo &params) override;
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  // Read/Write methods
  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;
  hardware_interface::return_type
  write(const rclcpp::Time &time, const rclcpp::Duration &period) override;
};

} // namespace so101_control

#endif