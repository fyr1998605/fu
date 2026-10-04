cmake_minimum_required(VERSION 3.8)
project(rm_tracker)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rm_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui calib3d dnn)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

find_path(FFTW3_INCLUDE_DIR fftw3.h PATHS /usr/include)
find_library(FFTW3_LIB fftw3 PATHS /usr/lib/x86_64-linux-gnu)

set(OPENVINO_ROOT
    "/opt/l_openvino_toolkit_ubuntu20_2024.4.0.16579.c3152d32c9c_x86_64")
set(OPENVINO_INCLUDE_DIR "${OPENVINO_ROOT}/runtime/include")
set(OPENVINO_LIB_DIR "${OPENVINO_ROOT}/runtime/lib/intel64")
set(OPENVINO_TBB_DIR "${OPENVINO_ROOT}/runtime/3rdparty/tbb/lib")
find_library(OPENVINO_LIB openvino PATHS "${OPENVINO_LIB_DIR}" NO_DEFAULT_PATH)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${Protobuf_INCLUDE_DIRS}
  ${FFTW3_INCLUDE_DIR}
  ${OPENVINO_INCLUDE_DIR}
  /opt/MVS/include/
)

link_directories(${OPENVINO_LIB_DIR} ${OPENVINO_TBB_DIR})

set(FU_SOURCES
  src/Fu.cpp
  src/buff_recognizer.cpp
  src/buff_zju_ransac.cpp
  src/buff_big_ransac_predictor.cpp
  include/CoorConverter.cpp
)

add_executable(Fu ${FU_SOURCES})
ament_target_dependencies(Fu
  rclcpp
  rm_msgs
  cv_bridge
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
)
target_link_libraries(Fu
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
  ${Protobuf_LIBRARIES}
  yaml-cpp
  ${FFTW3_LIB}
  ${OPENVINO_LIB}
)
set_target_properties(Fu PROPERTIES
  BUILD_RPATH "${OPENVINO_TBB_DIR};${OPENVINO_LIB_DIR}"
  INSTALL_RPATH "${OPENVINO_TBB_DIR};${OPENVINO_LIB_DIR}"
)

add_executable(ros2_energy_helpers_test test/ros2_energy_helpers_test.cpp)
target_include_directories(ros2_energy_helpers_test PRIVATE include)

enable_testing()
add_test(NAME ros2_energy_helpers_test COMMAND ros2_energy_helpers_test)
add_test(
  NAME ros2_fu_source_test
  COMMAND ${Python3_EXECUTABLE}
          ${CMAKE_CURRENT_SOURCE_DIR}/test/test_ros2_fu_source.py
)

if(BUILD_TESTING)
  add_executable(buff_detection_stabilizer_test
    test/buff_detection_stabilizer_test.cpp)
  target_include_directories(buff_detection_stabilizer_test PRIVATE include)
  add_test(NAME buff_detection_stabilizer_test COMMAND buff_detection_stabilizer_test)

  add_executable(buff_prediction_geometry_test
    test/buff_prediction_geometry_test.cpp)
  target_include_directories(buff_prediction_geometry_test PRIVATE include)
  add_test(NAME buff_prediction_geometry_test COMMAND buff_prediction_geometry_test)

  add_executable(buff_zju_ransac_test
    test/buff_zju_ransac_test.cpp
    src/buff_zju_ransac.cpp)
  target_include_directories(buff_zju_ransac_test PRIVATE include)
  add_test(NAME buff_zju_ransac_test COMMAND buff_zju_ransac_test)

  add_executable(buff_big_ransac_predictor_test
    test/buff_big_ransac_predictor_test.cpp
    src/buff_big_ransac_predictor.cpp
    src/buff_zju_ransac.cpp)
  target_include_directories(buff_big_ransac_predictor_test PRIVATE include)
  add_test(NAME buff_big_ransac_predictor_test COMMAND buff_big_ransac_predictor_test)

  add_executable(buff_angle_processor_test
    test/buff_angle_processor_test.cpp)
  target_include_directories(buff_angle_processor_test PRIVATE include)
  add_test(NAME buff_angle_processor_test COMMAND buff_angle_processor_test)
endif()

install(TARGETS Fu
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../Model/fu_moxi/
  DESTINATION share/${PROJECT_NAME}/models/fu_moxi
  FILES_MATCHING
  PATTERN "*.xml"
  PATTERN "*.bin"
  PATTERN "*.yaml")

ament_package()
