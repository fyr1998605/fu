cmake_minimum_required(VERSION 3.10)
project(rm_identify)

# C++17标准
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-std=c++17)

find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

# ========== 关键修改：在 find_package 中添加 rm_msgs ==========
find_package(catkin REQUIRED COMPONENTS
  tf2_ros
  tf2
  cv_bridge
  image_transport
  roscpp
  rospy
  std_msgs
  camera_info_manager
  sensor_msgs
  serial
  geometry_msgs
  message_generation
  rm_msgs          # 添加这一行！
)

find_package(OpenCV 4.2.0 REQUIRED)
find_package(Ceres REQUIRED)
find_package(GTest REQUIRED)

# ========== catkin_package 保持原样 ==========
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs rm_msgs serial geometry_msgs sensor_msgs
)

# ========== include_directories 保持不变 ==========
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS} 
  /opt/intel/openvino_2022.3.0/runtime/include
  /opt/intel/openvino_2022.3.0/runtime/include/ie
  /opt/intel/openvino_2022.3.0/runtime/include/ngraph
  /opt/MVS/include/
)

message(STATUS "OpenCV library status:")
message(STATUS "version:${OpenCV_VERSION}")
message(STATUS "libraries:${OpenCV_LIBS}")
message(STATUS "include path:${OpenCV_INCLUDE_DIRS}")

# 三个可执行文件
add_executable(identify 
  src/identify.cpp
  include/CoorConverter.cpp
  include/math.cpp
)
add_executable(identify_test 
  src/identify_test.cpp
  include/CoorConverter.cpp
  include/math.cpp
)
add_executable(handeye 
  src/calibrate_handeye.cpp
  include/img_tools.cpp
)

# ========== 关键修改：为每个可执行文件添加依赖 ==========
add_dependencies(identify 
  ${catkin_EXPORTED_TARGETS}
  rm_msgs_generate_messages_cpp
)
add_dependencies(identify_test 
  ${catkin_EXPORTED_TARGETS}
  rm_msgs_generate_messages_cpp
)
add_dependencies(handeye 
  ${catkin_EXPORTED_TARGETS}
  rm_msgs_generate_messages_cpp
)

# 头文件目录
target_include_directories(identify
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
  PUBLIC ${OpenCV_INCLUDE_DIRS}
  /opt/intel/openvino_2022.3.0/runtime/include
  /opt/intel/openvino_2022.3.0/runtime/include/ie
  /opt/intel/openvino_2022.3.0/runtime/include/ngraph
)
target_include_directories(identify_test
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
  PUBLIC ${OpenCV_INCLUDE_DIRS}
  /opt/intel/openvino_2022.3.0/runtime/include
  /opt/intel/openvino_2022.3.0/runtime/include/ie
  /opt/intel/openvino_2022.3.0/runtime/include/ngraph
)
target_include_directories(handeye
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
  PUBLIC ${OpenCV_INCLUDE_DIRS}
  /opt/intel/openvino_2022.3.0/runtime/include
  /opt/intel/openvino_2022.3.0/runtime/include/ie
  /opt/intel/openvino_2022.3.0/runtime/include/ngraph
)

# 链接库
target_link_libraries(identify
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
  ${Protobuf_LIBRARIES}
  yaml-cpp
  /opt/intel/openvino_2022.3.0/runtime/lib/intel64/libopenvino.so
)
target_link_libraries(identify_test
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
  ${Protobuf_LIBRARIES}
  yaml-cpp
  /opt/intel/openvino_2022.3.0/runtime/lib/intel64/libopenvino.so
)
target_link_libraries(handeye
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
  ${Protobuf_LIBRARIES}
  yaml-cpp  
  /opt/intel/openvino_2022.3.0/runtime/lib/intel64/libopenvino.so
)

SET(CMAKE_BUILD_TYPE "RelWithDebInfo")