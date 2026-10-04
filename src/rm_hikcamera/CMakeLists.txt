cmake_minimum_required(VERSION 3.8)
project(rm_hikcamera)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

find_library(MVS_CAMERA_CONTROL MvCameraControl PATHS /opt/MVS/lib/64)
find_library(MVS_GIGE MVGigEVisionSDK PATHS /opt/MVS/lib/64)
find_library(MVS_RENDER MVRender PATHS /opt/MVS/lib/64)
find_library(MVS_USB MvUsb3vTL PATHS /opt/MVS/lib/64)
find_library(MVS_MEDIA MediaProcess PATHS /opt/MVS/lib/64)

add_executable(hikcamera src/hikcamera.cpp)
target_include_directories(hikcamera PRIVATE include /opt/MVS/include)
ament_target_dependencies(hikcamera
  rclcpp
  cv_bridge
  image_transport
  sensor_msgs
  camera_info_manager
)
target_link_libraries(hikcamera
  ${OpenCV_LIBS}
  ${MVS_CAMERA_CONTROL}
  ${MVS_GIGE}
  ${MVS_RENDER}
  ${MVS_USB}
  ${MVS_MEDIA}
  X11
  pthread
)

enable_testing()
add_test(
  NAME ros2_camera_source_test
  COMMAND ${Python3_EXECUTABLE}
          ${CMAKE_CURRENT_SOURCE_DIR}/test/test_ros2_camera_source.py
)

install(TARGETS hikcamera
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

ament_package()
