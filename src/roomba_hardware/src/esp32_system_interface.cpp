#include "roomba_hardware/esp32_system_interface.hpp"

#include <algorithm>
#include <cctype>
#include <set>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace roomba_hardware
{

namespace
{
std::string to_lower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
  return s;
}

std::string get_param(
  const std::unordered_map<std::string, std::string> & params, const std::string & key,
  const std::string & default_value)
{
  auto it = params.find(key);
  return it != params.end() ? it->second : default_value;
}
}  // namespace

hardware_interface::CallbackReturn ESP32SystemInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & hp = info_.hardware_parameters;

  auto it = hp.find("serial_port");
  if (it == hp.end() || it->second.empty()) {
    RCLCPP_FATAL(
      get_logger(),
      "roomba_hardware requires a <param name=\"serial_port\"> entry under <hardware> "
      "(e.g. /dev/ttyUSB0) -- none was found.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  serial_port_ = it->second;

  const std::string baud_str = get_param(hp, "baud_rate", "115200");
  try {
    baud_rate_ = std::stoi(baud_str);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      get_logger(), "Invalid baud_rate param '%s': %s", baud_str.c_str(), e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  const std::string left_channel = to_lower(get_param(hp, "left_motor_channel", "A"));
  const std::string right_channel = to_lower(get_param(hp, "right_motor_channel", "B"));
  if (
    (left_channel != "a" && left_channel != "b") ||
    (right_channel != "a" && right_channel != "b") || left_channel == right_channel)
  {
    RCLCPP_FATAL(
      get_logger(),
      "left_motor_channel/right_motor_channel must be one 'A' and one 'B' (got '%s'/'%s'). "
      "This is the unverified Motor A/B <-> left/right wheel mapping from the firmware side "
      "(see open question in the hardware-interface prompt) -- flip it here, not in code, "
      "if the robot spins in place or drives backwards on first test.",
      left_channel.c_str(), right_channel.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  const char left_motor_channel = left_channel == "a" ? 'A' : 'B';
  const char right_motor_channel = right_channel == "a" ? 'A' : 'B';

  if (info_.joints.size() != 2) {
    RCLCPP_FATAL(
      get_logger(), "ESP32SystemInterface expects exactly 2 joints, got %zu.",
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  left_ = WheelJoint{};
  right_ = WheelJoint{};
  bool have_left = false;
  bool have_right = false;

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(),
        "Joint '%s' must have exactly 1 command interface ('%s'). Check "
        "ros2_hardwareinterface.xacro.",
        joint.name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    std::set<std::string> state_names;
    for (const auto & si : joint.state_interfaces) {
      state_names.insert(si.name);
    }
    if (
      state_names.size() != 2 || !state_names.count(hardware_interface::HW_IF_POSITION) ||
      !state_names.count(hardware_interface::HW_IF_VELOCITY))
    {
      RCLCPP_FATAL(
        get_logger(),
        "Joint '%s' must have exactly the '%s' and '%s' state interfaces. Check "
        "ros2_hardwareinterface.xacro.",
        joint.name.c_str(), hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    const std::string lname = to_lower(joint.name);
    const bool is_left = lname.find("left") != std::string::npos;
    const bool is_right = lname.find("right") != std::string::npos;
    if (is_left == is_right) {
      // neither matched, or (pathologically) both did
      RCLCPP_FATAL(
        get_logger(),
        "Joint '%s' name must unambiguously contain 'left' or 'right' so "
        "ESP32SystemInterface can pick the correct motor mapping.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (is_left) {
      left_.joint_name = joint.name;
      left_.motor_channel = left_motor_channel;
      have_left = true;
    } else {
      right_.joint_name = joint.name;
      right_.motor_channel = right_motor_channel;
      have_right = true;
    }
  }

  if (!have_left || !have_right) {
    RCLCPP_FATAL(
      get_logger(),
      "Could not resolve one left and one right wheel joint from the 2 joints in the URDF.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    get_logger(), "left joint '%s' -> Motor %c, right joint '%s' -> Motor %c",
    left_.joint_name.c_str(), left_.motor_channel, right_.joint_name.c_str(),
    right_.motor_channel);

  link_ = std::make_unique<SerialLink>(serial_port_, baud_rate_, get_logger());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ESP32SystemInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  set_state(left_.joint_name + "/" + hardware_interface::HW_IF_POSITION, 0.0);
  set_state(left_.joint_name + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);
  set_state(right_.joint_name + "/" + hardware_interface::HW_IF_POSITION, 0.0);
  set_state(right_.joint_name + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);
  set_command(left_.joint_name + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);
  set_command(right_.joint_name + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);

  // Start the link here (not on_activate): per SystemInterface's lifecycle
  // contract, INACTIVE already means "communication started, states can be
  // read" -- and it lets telemetry be flowing (or a missing-device warning
  // already visible) before anything is spawned/activated on top of us.
  // SerialLink::start() is idempotent, retries the open on its own if the
  // ESP32 isn't plugged in yet, and never blocks this call.
  link_->start();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ESP32SystemInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (link_) {
    link_->stop();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ESP32SystemInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Command interfaces become available starting now -- make sure we don't
  // hand the controller a stale nonzero command left over from a previous
  // activation.
  set_command(left_.joint_name + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);
  set_command(right_.joint_name + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);

  // Zero the encoder reference once on activate, per the hardware-interface
  // requirement: everything downstream (relay.py's position integration,
  // ekf_node) should see position start from 0 at this point, not wherever
  // the wheels happened to be since the last "Z".
  link_->request_zero();

  RCLCPP_INFO(get_logger(), "ESP32SystemInterface activated, encoder zero requested.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ESP32SystemInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Ramped stop so the robot doesn't keep coasting on its last velocity
  // command. Non-blocking: queues "S" for the background thread to send on
  // its next loop iteration (up to kPollTimeoutMs later).
  link_->request_stop();
  RCLCPP_INFO(get_logger(), "ESP32SystemInterface deactivated, stop requested.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ESP32SystemInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!link_->is_connected()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "ESP32 serial link ('%s') not connected -- holding last known joint state.",
      serial_port_.c_str());
    return hardware_interface::return_type::OK;
  }

  const SerialLink::Telemetry t = link_->get_telemetry();
  if (!t.ever_received) {
    // Connected but no telemetry line parsed yet (ESP32 only pushes ~every
    // 150ms) -- hold last known state rather than reporting a bogus zero.
    return hardware_interface::return_type::OK;
  }

  const double left_pos_deg = left_.motor_channel == 'A' ? t.pos_a_deg : t.pos_b_deg;
  const double left_vel_deg_s = left_.motor_channel == 'A' ? t.vel_a_deg_s : t.vel_b_deg_s;
  const double right_pos_deg = right_.motor_channel == 'A' ? t.pos_a_deg : t.pos_b_deg;
  const double right_vel_deg_s = right_.motor_channel == 'A' ? t.vel_a_deg_s : t.vel_b_deg_s;

  set_state(left_.joint_name + "/" + hardware_interface::HW_IF_POSITION, left_pos_deg * kDegToRad);
  set_state(
    left_.joint_name + "/" + hardware_interface::HW_IF_VELOCITY, left_vel_deg_s * kDegToRad);
  set_state(
    right_.joint_name + "/" + hardware_interface::HW_IF_POSITION, right_pos_deg * kDegToRad);
  set_state(
    right_.joint_name + "/" + hardware_interface::HW_IF_VELOCITY, right_vel_deg_s * kDegToRad);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ESP32SystemInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const double left_cmd_rad_s =
    get_command<double>(left_.joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
  const double right_cmd_rad_s =
    get_command<double>(right_.joint_name + "/" + hardware_interface::HW_IF_VELOCITY);

  const double left_cmd_deg_s = left_cmd_rad_s * kRadToDeg;
  const double right_cmd_deg_s = right_cmd_rad_s * kRadToDeg;

  double motor_a_deg_s = 0.0;
  double motor_b_deg_s = 0.0;
  (left_.motor_channel == 'A' ? motor_a_deg_s : motor_b_deg_s) = left_cmd_deg_s;
  (right_.motor_channel == 'A' ? motor_a_deg_s : motor_b_deg_s) = right_cmd_deg_s;

  // Non-blocking: just latches the desired speeds. SerialLink's background
  // thread decides when to actually put V1/V2 on the wire (rate-limited --
  // see serial_link.cpp).
  link_->set_command_deg_s(motor_a_deg_s, motor_b_deg_s);

  return hardware_interface::return_type::OK;
}

}  // namespace roomba_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  roomba_hardware::ESP32SystemInterface, hardware_interface::SystemInterface)
