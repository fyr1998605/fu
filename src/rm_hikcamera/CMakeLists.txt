cmake_minimum_required(VERSION 3.10)
project(rm_hikcamera)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-std=c++14)

find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

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
)

catkin_package(
 CATKIN_DEPENDS roscpp std_msgs 
)

find_package(OpenCV 4.2.0 REQUIRED)
find_package(Ceres REQUIRED)
find_package(GTest REQUIRED)

LINK_DIRECTORIES(
/opt/MVS/lib/64/
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS} 
  find_package(OpenCV 4.2.0 REQUIRED)
  /opt/intel/openvino_2022.3.0/runtime/include
  /opt/intel/openvino_2022.3.0/runtime/include/ie
  /opt/intel/openvino_2022.3.0/runtime/include/ngraph
  /opt/MVS/include/
)

message(STATUS "OpenCV library status:")
message(STATUS "version:${OpenCV_VERSION}")
message(STATUS "libraries:${OpenCV_LIBS}")
message(STATUS "include path:${OpenCV_INCLUDE_DIRS}")

add_executable(hikcamera src/hikcamera.cpp)

target_include_directories(hikcamera
      PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
      PUBLIC  ${OpenCV_INCLUDE_DIRS}
      /opt/MVS/include/
)
  
target_link_libraries(hikcamera
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
 
    MvCameraControl
 
    MVGigEVisionSDK
 
    MVRender
 
    MvUsb3vTL
 
    MediaProcess
 
    X11
  )    
SET(CMAKE_BUILD_TYPE "RelWithDebInfo")