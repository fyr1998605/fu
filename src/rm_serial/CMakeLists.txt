cmake_minimum_required(VERSION 3.8)
project(rm_serial)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rm_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(yaml-cpp REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

add_executable(rm_serial_node src/serial.cpp)
target_include_directories(rm_serial_node PRIVATE include)
ament_target_dependencies(rm_serial_node
  rclcpp
  rm_msgs
  geometry_msgs
  sensor_msgs
  tf2
)
target_link_libraries(rm_serial_node yaml-cpp Boost::system)

enable_testing()
add_test(
  NAME ros2_serial_source_test
  COMMAND ${Python3_EXECUTABLE}
          ${CMAKE_CURRENT_SOURCE_DIR}/test/test_ros2_serial_source.py
)

install(TARGETS rm_serial_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

ament_package()
