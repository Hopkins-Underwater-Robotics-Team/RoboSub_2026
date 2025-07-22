cmake_minimum_required(VERSION 3.8)
project(mpu6500_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories - use the inc directory we created
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/../mpu6500_driver/inc
  ${CMAKE_CURRENT_SOURCE_DIR}/../mpu6500_driver/src
)

# Add only the necessary MPU6500 driver source files
set(MPU6500_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/../mpu6500_driver/src/driver_mpu6500.c
  ${CMAKE_CURRENT_SOURCE_DIR}/../mpu6500_driver/src/driver_mpu6500_interface.c
)

# Create executable
add_executable(mpu6500_publisher 
  src/mpu6500_publisher.cpp
  ${MPU6500_SOURCES}
)

# Link dependencies
ament_target_dependencies(mpu6500_publisher
  rclcpp
  sensor_msgs
  std_msgs
)

# Install executable
install(TARGETS
  mpu6500_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.py"
)

ament_package()
