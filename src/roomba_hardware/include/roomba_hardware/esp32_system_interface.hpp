#ifndef ROOMBA_HARDWARE__ESP32_SYSTEM_INTERFACE_HPP_
#define ROOMBA_HARDWARE__ESP32_SYSTEM_INTERFACE_HPP_

#include <memory>
#include <string>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "roomba_hardware/serial_link.hpp"

namespace roomba_hardware
{

/// Real-hardware counterpart of gz_ros2_control/GazeboSimSystem for the
/// roomba diff-drive base: same two joints, same velocity command / position
/// + velocity state interface shape, but backed by an ESP32 over serial
/// instead of the simulator. See ros2_hardwareinterface.xacro for the
/// <hardware><param> entries this reads in on_init (serial_port, baud_rate,
/// left_motor_channel, right_motor_channel).
class ESP32SystemInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ESP32SystemInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Resolved once in on_init from the joint name (must contain "left" or
  // "right", matching base_left_wheel_joint/base_right_wheel_joint) plus the
  // left_motor_channel/right_motor_channel hardware params. Keeping this as
  // data -- not hardcoded if/else on joint name -- is what makes flipping
  // the physical Motor A/B <-> left/right mapping during bring-up a
  // one-line xacro param change instead of a rebuild.
  struct WheelJoint
  {
    std::string joint_name;
    char motor_channel = 'A';  // 'A' or 'B'
  };
  WheelJoint left_;
  WheelJoint right_;

  std::string serial_port_;
  int baud_rate_ = 115200;

  std::unique_ptr<SerialLink> link_;

  static constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
  static constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
};

}  // namespace roomba_hardware

#endif  // ROOMBA_HARDWARE__ESP32_SYSTEM_INTERFACE_HPP_
