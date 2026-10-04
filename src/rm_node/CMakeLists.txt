cmake_minimum_required(VERSION 3.8)
project(rm_node)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

enable_testing()
add_test(
  NAME energy_launch_source_test
  COMMAND ${Python3_EXECUTABLE}
          ${CMAKE_CURRENT_SOURCE_DIR}/test/test_launch_description.py
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
