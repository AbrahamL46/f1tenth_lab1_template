cmake_minimum_required(VERSION 3.5)
project(lab1_pkg)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)

# --- C++ executable ---
add_executable(cpp_talker src/cpp_talker.cpp)
ament_target_dependencies(cpp_talker rclcpp ackermann_msgs)

install(TARGETS cpp_talker
  DESTINATION lib/${PROJECT_NAME})

# --- Python package & scripts ---
# Install a Python package directory named "lab1_pkg"
ament_python_install_package(${PROJECT_NAME})

# Install standalone Python scripts as executables
install(PROGRAMS
  scripts/py_talker
  DESTINATION lib/${PROJECT_NAME})

ament_package()