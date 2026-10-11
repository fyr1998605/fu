cmake_minimum_required(VERSION 3.10)
project(rm_tracker)

# C++17标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-std=c++17)

# 用 RPATH 代替 RUNPATH，使传递依赖（libopenvino→libtbb）也能找到库路径
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--disable-new-dtags")

# ========== OpenCV 强制使用系统 4.2 ==========
# 关键：指定正确的 cmake 配置目录，并用 NO_DEFAULT_PATH 忽略 /usr/local 下的 4.7
set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4")
find_package(OpenCV 4.2 REQUIRED COMPONENTS core imgproc highgui calib3d
             PATHS /usr
             NO_DEFAULT_PATH
)

# 第三方依赖
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(yaml-cpp 0.6.2 REQUIRED)
find_package(Protobuf REQUIRED)

# 手动加载FFTW（Ubuntu Noetic 无 FFTWConfig.cmake）
find_path(FFTW3_INCLUDE_DIR fftw3.h PATHS /usr/include)
find_library(FFTW3_LIB fftw3 PATHS /usr/lib/x86_64-linux-gnu)

# ROS 依赖
find_package(catkin REQUIRED COMPONENTS
  tf2_ros tf2 cv_bridge image_transport
  roscpp rospy std_msgs camera_info_manager
  sensor_msgs geometry_msgs message_generation 
  rm_msgs
)

# ========== 修改点1：删除强制指定 Eigen3_DIR 的语句，让系统自动查找 ==========
# set(Eigen3_DIR "/usr/local/eigen-3.4/share/eigen3/cmake")   # 已注释掉

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs tf2_ros tf2 cv_bridge image_transport sensor_msgs geometry_msgs rm_msgs
  DEPENDS OpenCV Eigen3 Ceres yaml-cpp Protobuf
)

# OpenVINO 2024.4 自带 TBB 库路径（编译+运行都需找到 libtbb.so.12）
set(OPENVINO_TBB_DIR /opt/l_openvino_toolkit_ubuntu20_2024.4.0.16579.c3152d32c9c_x86_64/runtime/3rdparty/tbb/lib)
link_directories(${OPENVINO_TBB_DIR})
# 把 TBB 路径写入 RPATH，运行时不再找不到
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(OPENVINO_LIB_DIR /opt/l_openvino_toolkit_ubuntu20_2024.4.0.16579.c3152d32c9c_x86_64/runtime/lib/intel64)
link_directories(${OPENVINO_LIB_DIR})
set(CMAKE_BUILD_RPATH "${OPENVINO_TBB_DIR};${OPENVINO_LIB_DIR}")

# ========== 修改点2：在全局头文件目录中，将 ${Eigen3_INCLUDE_DIRS} 改为 /usr/include/eigen3 ==========
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /usr/include/eigen3               # 直接指定 Eigen 头文件路径
  ${CERES_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
  ${Protobuf_INCLUDE_DIRS}
  ${FFTW3_INCLUDE_DIR}
  /usr/include
  /opt/l_openvino_toolkit_ubuntu20_2024.4.0.16579.c3152d32c9c_x86_64/runtime/include
  /opt/MVS/include/
)

# ========== tracker 可执行文件 ==========
add_executable(tracker 
  src/tracker.cpp
  include/CoorConverter.cpp
  include/math.cpp
  include/MPC.cpp
  include/trajectory_visualizer.cpp
)
add_dependencies(tracker 
  ${catkin_EXPORTED_TARGETS}
  rm_msgs_generate_messages_cpp
)

target_include_directories(tracker
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
  PUBLIC ${OpenCV_INCLUDE_DIRS}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
  /opt/l_openvino_toolkit_ubuntu20_2024.4.0.16579.c3152d32c9c_x86_64/runtime/include
)

target_link_libraries(tracker
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
  ${Protobuf_LIBRARIES}
  yaml-cpp
  ${FFTW3_LIB}
  /opt/l_openvino_toolkit_ubuntu20_2024.4.0.16579.c3152d32c9c_x86_64/runtime/lib/intel64/libopenvino.so
)

# ========== Fu 可执行文件 ==========
add_executable(Fu
  src/Fu.cpp
  src/buff_recognizer.cpp
  src/buff_zju_ransac.cpp
  src/buff_big_ransac_predictor.cpp
  include/CoorConverter.cpp
)
add_dependencies(Fu
  ${catkin_EXPORTED_TARGETS}
  rm_msgs_generate_messages_cpp
)

target_include_directories(Fu
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
  PUBLIC ${OpenCV_INCLUDE_DIRS}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
  /opt/l_openvino_toolkit_ubuntu20_2024.4.0.16579.c3152d32c9c_x86_64/runtime/include
)

target_link_libraries(Fu
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${Protobuf_LIBRARIES}
  yaml-cpp
  /opt/l_openvino_toolkit_ubuntu20_2024.4.0.16579.c3152d32c9c_x86_64/runtime/lib/intel64/libopenvino.so
)

# ========== 识别器独立测试 (无串口依赖) ==========
add_executable(buff_test
  src/buff_recognizer_test.cpp
  src/buff_recognizer.cpp
)
add_dependencies(buff_test ${catkin_EXPORTED_TARGETS})

target_include_directories(buff_test
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
  PUBLIC ${OpenCV_INCLUDE_DIRS}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
  /opt/l_openvino_toolkit_ubuntu20_2024.4.0.16579.c3152d32c9c_x86_64/runtime/include
)

target_link_libraries(buff_test
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  /opt/l_openvino_toolkit_ubuntu20_2024.4.0.16579.c3152d32c9c_x86_64/runtime/lib/intel64/libopenvino.so
)

if(CATKIN_ENABLE_TESTING)
  add_executable(buff_big_predictor_test
    test/buff_big_predictor_test.cpp
    src/buff_zju_ransac.cpp
    src/buff_big_ransac_predictor.cpp
  )
  target_include_directories(buff_big_predictor_test
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  add_test(NAME buff_big_predictor_test COMMAND buff_big_predictor_test)

  add_executable(buff_frame_timing_test test/buff_frame_timing_test.cpp)
  target_include_directories(buff_frame_timing_test
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  add_test(NAME buff_frame_timing_test COMMAND buff_frame_timing_test)

  add_executable(buff_detection_stabilizer_test
    test/buff_detection_stabilizer_test.cpp
  )
  target_include_directories(buff_detection_stabilizer_test
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  add_test(NAME buff_detection_stabilizer_test COMMAND buff_detection_stabilizer_test)

  add_executable(buff_prediction_geometry_test
    test/buff_prediction_geometry_test.cpp
  )
  target_include_directories(buff_prediction_geometry_test
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  add_test(NAME buff_prediction_geometry_test COMMAND buff_prediction_geometry_test)

  add_executable(buff_zju_ransac_test
    test/buff_zju_ransac_test.cpp
    src/buff_zju_ransac.cpp
  )
  target_include_directories(buff_zju_ransac_test
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(buff_zju_ransac_test
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
  )
  add_test(NAME buff_zju_ransac_test COMMAND buff_zju_ransac_test)

  add_executable(buff_big_ransac_predictor_test
    test/buff_big_ransac_predictor_test.cpp
    src/buff_big_ransac_predictor.cpp
    src/buff_zju_ransac.cpp
  )
  target_include_directories(buff_big_ransac_predictor_test
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(buff_big_ransac_predictor_test
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
  )
  add_test(NAME buff_big_ransac_predictor_test COMMAND buff_big_ransac_predictor_test)

  add_executable(buff_big_variable_speed_predictor_test
    test/buff_big_variable_speed_predictor_test.cpp
    src/buff_big_variable_speed_predictor.cpp
  )
  target_include_directories(buff_big_variable_speed_predictor_test
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  add_test(NAME buff_big_variable_speed_predictor_test
    COMMAND buff_big_variable_speed_predictor_test)

  add_executable(buff_big_prediction_gate_test
    test/buff_big_prediction_gate_test.cpp
  )
  target_include_directories(buff_big_prediction_gate_test
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  add_test(NAME buff_big_prediction_gate_test COMMAND buff_big_prediction_gate_test)

  add_executable(buff_angle_processor_test
    test/buff_angle_processor_test.cpp
  )
  target_include_directories(buff_angle_processor_test
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(buff_angle_processor_test
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
  )
  add_test(NAME buff_angle_processor_test COMMAND buff_angle_processor_test)

  # 靶子切换 (five-blade ID switch) regression: the predicted point must follow
  # the blade the detector reports. Uses rosconsole, hence catkin_LIBRARIES.
  add_executable(buff_blade_tracker_test
    test/buff_blade_tracker_test.cpp
  )
  target_include_directories(buff_blade_tracker_test
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(buff_blade_tracker_test
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
  )
  add_test(NAME buff_blade_tracker_test COMMAND buff_blade_tracker_test)
endif()

# 安装规则
install(TARGETS tracker Fu
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
