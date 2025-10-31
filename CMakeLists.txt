cmake_minimum_required(VERSION 3.5)
project(sparton_ahrs8_driver)

# Required for Python nodes
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Install Python scripts
install(
  PROGRAMS
    scripts/ahrs8_nmea.py
  DESTINATION lib/${PROJECT_NAME}
)



ament_package()
install(
  DIRECTORY launch udev
  DESTINATION share/${PROJECT_NAME}
)

