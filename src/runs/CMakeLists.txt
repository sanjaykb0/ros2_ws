cmake_minimum_required(VERSION 3.5)
project(runs)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  src/camera_node.py
  src/lidar_node.py
  src/motor_control_node.py
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# Install URDF files
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME})

ament_package()
