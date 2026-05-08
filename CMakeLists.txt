cmake_minimum_required(VERSION 3.8)
project(com2009_team09_2026)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME}_modules)

# Install Python executables
install(PROGRAMS
  scripts/task1/task1.py
  scripts/task2/task2_explore_map.py
  scripts/task2/task2_obstacle_detection.py
  scripts/task3/task3_beacon.py
  scripts/task3/task3_explorer.py
  scripts/task3/task3_map_saver.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  maps        
  snaps
  DESTINATION share/${PROJECT_NAME}
)
find_package(nav2_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObstacleInfo.msg"
)

ament_package()
