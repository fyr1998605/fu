cmake_minimum_required(VERSION 3.10)
project(rm_msgs)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  Armor.msg
  ArmorArray.msg
  Debug.msg
  RmSerial.msg
  SentrySerial.msg 
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs message_runtime
)

include_directories(${catkin_INCLUDE_DIRS})

SET(CMAKE_BUILD_TYPE "RelWithDebInfo")