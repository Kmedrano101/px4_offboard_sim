cmake_minimum_required(VERSION 3.24)
project(px4_offboard_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

# ── C++ offboard_control_node ────────────────────────────────
add_executable(offboard_control_node
  src/offboard_control_node.cpp
)
ament_target_dependencies(offboard_control_node
  rclcpp px4_msgs geometry_msgs std_msgs
)

# ── C++ joy_control_node ─────────────────────────────────────
add_executable(joy_control_node
  src/joy_control_node.cpp
)
ament_target_dependencies(joy_control_node
  rclcpp sensor_msgs geometry_msgs std_msgs
)

install(TARGETS
  offboard_control_node
  joy_control_node
  DESTINATION lib/${PROJECT_NAME}
)

# ── Python nodes (frame_manager, processes) ──────────────────
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  px4_offboard_sim/frame_manager.py
  px4_offboard_sim/processes.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── Config, launch, resource, worlds, and models ─────────────
install(DIRECTORY config launch resource models worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
