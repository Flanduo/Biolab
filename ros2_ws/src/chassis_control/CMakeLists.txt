cmake_minimum_required(VERSION 3.8)
project(chassis_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/MotorFeedback.msg"
#   DEPENDENCIES std_msgs
# )

# Create a ament index resource to make the package findable
ament_export_dependencies(rosidl_default_runtime)

# Include directories
include_directories(
  include
)

# Declare a cpp library with a different name to avoid conflict
add_library(${PROJECT_NAME}_lib
  src/steering_motor.cpp
  src/wheel_motor.cpp
  src/filters.cpp
  src/chassis_control.cpp
  src/zlac8015d_rpm_node.cpp
  src/JK_BMS_Status.cpp
)

# Rename the output to avoid confusion
set_target_properties(${PROJECT_NAME}_lib PROPERTIES
  OUTPUT_NAME ${PROJECT_NAME}
)

# Use the generated interfaces

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  std_msgs
  sensor_msgs
)

# rosidl_target_interfaces(${PROJECT_NAME}_lib 
#   ${PROJECT_NAME} "rosidl_typesupport_cpp")


# Motor control test executable
add_executable(chassis_control
  src/main.cpp
)

add_executable(lift_state_node
  src/aimotor_position_node.cpp
)

add_executable(zlac8015d_rpm_node
  src/zlac8015d_rpm_node.cpp
)

add_executable(JK_BMS_Status_node
  src/JK_BMS_Status.cpp
)

# add_executable(enable_motor
#   src/enable_motor.cpp
# )

ament_target_dependencies(chassis_control
  rclcpp
  std_msgs
)

ament_target_dependencies(lift_state_node
  rclcpp
  std_msgs
)

ament_target_dependencies(zlac8015d_rpm_node
  rclcpp
  std_msgs
)

ament_target_dependencies(JK_BMS_Status_node
  rclcpp
  std_msgs
)

# ament_target_dependencies(enable_motor
#   rclcpp
#   std_msgs
#   can_msgs
# )

target_link_libraries(chassis_control
  ${PROJECT_NAME}_lib
)

# target_link_libraries(enable_motor
#   ${PROJECT_NAME}_lib
# )

# Install include directories
install(
  DIRECTORY include/
  DESTINATION include
)

# Install targets
install(
  TARGETS ${PROJECT_NAME}_lib chassis_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


install(
  TARGETS ${PROJECT_NAME}_lib lift_state_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}_lib zlac8015d_rpm_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}_lib JK_BMS_Status_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

#Install launch files
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/bms_monitor_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
