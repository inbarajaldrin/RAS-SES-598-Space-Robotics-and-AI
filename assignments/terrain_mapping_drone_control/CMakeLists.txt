cmake_minimum_required(VERSION 3.5)
project(terrain_mapping_drone_control)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install models (if needed)
install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
