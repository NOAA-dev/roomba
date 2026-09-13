cmake_minimum_required(VERSION 3.16)
project(roomba_hardware LANGUAGES CXX)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(controller_manager 4.0.0)
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
  "Are you using the correct ROS 2 distro (this package targets Jazzy)?")
endif()

find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  roomba_hardware
  SHARED
  src/serial_link.cpp
  src/esp32_system_interface.cpp
)
target_compile_features(roomba_hardware PUBLIC cxx_std_17)
target_include_directories(roomba_hardware PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/roomba_hardware>
)
target_link_libraries(roomba_hardware PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

# Export hardware plugin description so pluginlib/ros2_control_node can find it
pluginlib_export_plugin_description_file(hardware_interface roomba_hardware.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/roomba_hardware
)
install(TARGETS roomba_hardware
  EXPORT export_roomba_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

## EXPORTS
ament_export_targets(export_roomba_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
