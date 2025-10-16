cmake_minimum_required(VERSION 3.5)
project(mappingbot_hardware)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
# find_package(controller_manager REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(serial QUIET)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/mappingbot_hardware.cpp
  src/mappingbot_serial_port.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)
ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  # controller_manager
  pluginlib
  rclcpp
  rclcpp_lifecycle
  serial
)

pluginlib_export_plugin_description_file(hardware_interface mappingbot_hardware.xml)

install(
  TARGETS
    ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY
    include
  DESTINATION include
)
# install(
#   DIRECTORY controllers launch/
#   DESTINATION share/${PROJECT_NAME}
# )

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  serial
  # controller_manager
)

ament_package()
