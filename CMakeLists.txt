cmake_minimum_required(VERSION 3.5)
project(steering_interface)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Install Python nodes
install(
  PROGRAMS
    src/steeringPublisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch + config files
install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

