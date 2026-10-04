cmake_minimum_required(VERSION 3.8)
project(rm_msgs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Armor.msg"
  "msg/ArmorArray.msg"
  "msg/Debug.msg"
  "msg/RmSerial.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime geometry_msgs std_msgs)

enable_testing()
add_test(
  NAME rm_msgs_definition_test
  COMMAND ${Python3_EXECUTABLE}
          ${CMAKE_CURRENT_SOURCE_DIR}/test/test_message_definitions.py
)

ament_package()
