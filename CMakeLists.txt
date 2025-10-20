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

# dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ament_index_cpp REQUIRED)


add_library(${PROJECT_NAME} SHARED
  src/mappingbot_hardware.cpp
  src/mappingbot_serial_port.cpp
)

target_compile_definitions(${PROJECT_NAME}
  PRIVATE "MAPPINGBOT_HARDWARE_BUILDING_DLL")

target_include_directories(${PROJECT_NAME}
  PRIVATE
  include
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ament_index_cpp
)

# ---- PLUGINLIB EXPORT ----
pluginlib_export_plugin_description_file(hardware_interface mappingbot_hardware.xml)

# ---- INSTALLATION ----
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES mappingbot_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

# ---- EXPORTS ----
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ament_index_cpp
)

ament_package()
