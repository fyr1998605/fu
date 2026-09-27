cmake_minimum_required(VERSION 3.10)
project(rm_serial)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-std=c++14)

set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

# 关键修改：在 find_package 中添加 rm_msgs 及哨兵串口所需消息包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  rm_msgs          # 添加这一行！
)

find_package(Eigen3 REQUIRED)

#
# set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0")

# 关键修改：在 catkin_package 中添加 rm_msgs
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs tf2 rm_msgs    # 添加 rm_msgs
)

# 关键修改：添加 rm_msgs 到 include 目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}    # 添加 Eigen3 的 include，如果你需要的话
)

# 这里把 serial 改成 rm_serial_node
add_executable(rm_serial_node src/serial.cpp)

# 关键修改：添加依赖关系，确保 rm_msgs 先生成
add_dependencies(rm_serial_node 
  ${catkin_EXPORTED_TARGETS}
  rm_msgs_generate_messages_cpp    # 添加这一行！
)

# 这里第一参数同步改成 rm_serial_node
target_link_libraries(rm_serial_node
  ${catkin_LIBRARIES}
  yaml-cpp  # 直接链接库名
)