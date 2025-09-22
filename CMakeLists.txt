cmake_minimum_required(VERSION 3.10)
project(chrps_visual_servo CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Find ViSP (only for standalone executable)
find_package(VISP REQUIRED)

include_directories(${VISP_INCLUDE_DIRS})

# Standalone executable (original)
add_executable(servoFrankaIBVS_CHRPS src/servoFrankaIBVS_CHRPS.cpp)
target_link_libraries(servoFrankaIBVS_CHRPS ${VISP_LIBRARIES})

# ROS 2 node executable
add_executable(chrps_visual_servo_node src/chrps_visual_servo_node.cpp)
ament_target_dependencies(chrps_visual_servo_node
  rclcpp
  std_msgs
  std_srvs
  ament_index_cpp
)

# Install executables
install(TARGETS
  servoFrankaIBVS_CHRPS
  chrps_visual_servo_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
