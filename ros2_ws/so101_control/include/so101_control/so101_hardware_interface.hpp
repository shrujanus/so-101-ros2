#ifndef SO101_HARDWARE_INTERFACE_HPP
#define SO101_HARDWARE_INTERFACE_HPP

#include "driver/SMS_STS.h"
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

using hardware_interface::CallbackReturn;

namespace so101_control {

struct Servo {
  Servo(int servo_id)
      : id(servo_id), position(0.0), velocity(0.0), effort(0.0),
        temperature(0.0), voltage(0.0), error_code(0.0), cmd_position(0.0),
        cmd_velocity(0.0), cmd_effort(0.0) {}
  int id;
  double position_offset;

  double cmd_id; // 0 = position, 1 = velocity
  double cmd_position;
  double cmd_velocity;

  double position_state;
  double velocity_state;
  double torque_state;
  double temperature_state;
  double voltage_state;
};

class so101_hardware_interface final
    : public hardware_interface::SystemInterface {
private:
  SMS_STS motor_driver_;
  std::string usb_port_ = "/dev/ttyUSB0";
  std::vector<Servo> servos_;
  int baud_rate_ = 115200;
  int num_joints_ = 0;
  CallbackReturn update_hardware_status(int i);

public:
  so101_hardware_interface();

  // Lifecycle methods
  CallbackReturn
  on_configure(const hardware_interface::HardwareInfo &info) override;
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