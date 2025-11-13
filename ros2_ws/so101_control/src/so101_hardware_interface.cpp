#include "so101_control/so101_hardware_interface.hpp"

so101_control::so101_hardware_interface::so101_hardware_interface() {
  // Constructor implementation (if needed)
}

so101_control::CallbackReturn so101_control::so101_hardware_interface::on_init(
    const hardware_interface::HardwareInfo &params) {
  if (hardware_interface::SystemInterface::on_init(params) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  usb_port_ = params.hardware_parameters["usb_port"];

  return CallbackReturn::SUCCESS;
}

so101_control::CallbackReturn
so101_control::so101_hardware_interface::on_configure(
    const hardware_interface::HardwareInfo &info) {
  // on_configure implementation
  return CallbackReturn::SUCCESS;
}

so101_control::CallbackReturn
so101_control::so101_hardware_interface::on_activate(
    const hardware_interface::HardwareInfo &info) {
  // on_activate implementation
  return CallbackReturn::SUCCESS;
}