cmake_minimum_required(VERSION 3.16)
project(buff_regressions LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
option(BUFF_TEST_WITH_ROS "Include the small-buff and legacy angle regressions" ON)
enable_testing()
find_package(Eigen3 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

get_filename_component(BUFF_REPO "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(BUFF_PACKAGE "${BUFF_REPO}/src/rm_tracker")
add_library(buff_sine STATIC
  "${BUFF_PACKAGE}/src/buff_zju_ransac.cpp"
  "${BUFF_PACKAGE}/src/buff_big_ransac_predictor.cpp")
target_include_directories(buff_sine PUBLIC "${BUFF_PACKAGE}/include")
target_link_libraries(buff_sine PUBLIC Eigen3::Eigen)

function(buff_test name)
  add_executable(${name} "${BUFF_PACKAGE}/test/${name}.cpp" ${ARGN})
  target_include_directories(${name} PRIVATE "${BUFF_PACKAGE}/include")
  add_test(NAME ${name} COMMAND ${name})
endfunction()

foreach(name buff_big_predictor_test buff_big_ransac_predictor_test buff_zju_ransac_test
             buff_blade_prediction_pipeline_test)
  buff_test(${name})
  target_link_libraries(${name} PRIVATE buff_sine)
endforeach()
foreach(name buff_big_prediction_gate_test buff_frame_timing_test
             buff_control_state_test
             buff_target_association_test
             buff_detection_stabilizer_test buff_prediction_geometry_test)
  buff_test(${name})
endforeach()
buff_test(buff_big_variable_speed_predictor_test
  "${BUFF_PACKAGE}/src/buff_big_variable_speed_predictor.cpp")
buff_test(buff_time_sync_test)
target_link_libraries(buff_time_sync_test PRIVATE Eigen3::Eigen)

if(BUFF_TEST_WITH_ROS)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(BUFF_ROS REQUIRED IMPORTED_TARGET roscpp tf2_ros cv_bridge angles)
  find_package(GTest REQUIRED)
  find_package(Boost REQUIRED COMPONENTS system)
  add_executable(buff_serial_timestamps_test
    "${BUFF_REPO}/src/rm_serial/test/test_serial_timestamps.cpp")
  # Match the deployed serial package; its legacy helpers target C++14.
  set_property(TARGET buff_serial_timestamps_test PROPERTY CXX_STANDARD 14)
  target_include_directories(buff_serial_timestamps_test PRIVATE
    "${BUFF_REPO}/src/rm_serial/include" "${BUFF_MESSAGE_INCLUDE_DIR}")
  target_link_libraries(buff_serial_timestamps_test PRIVATE
    PkgConfig::BUFF_ROS GTest::GTest Boost::system yaml-cpp util)
  add_test(NAME buff_serial_timestamps_test COMMAND "${Python3_EXECUTABLE}"
    "${BUFF_REPO}/scripts/run_ros_test.py" $<TARGET_FILE:buff_serial_timestamps_test>)
  add_executable(buff_serial_mode_node "${BUFF_REPO}/src/rm_serial/src/serial.cpp")
  set_property(TARGET buff_serial_mode_node PROPERTY CXX_STANDARD 14)
  target_include_directories(buff_serial_mode_node PRIVATE
    "${BUFF_REPO}/src/rm_serial/include" "${BUFF_MESSAGE_INCLUDE_DIR}")
  target_link_libraries(buff_serial_mode_node PRIVATE
    PkgConfig::BUFF_ROS Boost::system yaml-cpp)
  add_test(NAME buff_serial_mode_ros_test COMMAND "${Python3_EXECUTABLE}"
    "${BUFF_REPO}/scripts/run_ros_test.py" "${Python3_EXECUTABLE}"
    "${BUFF_REPO}/scripts/test_serial_mode_ros.py" $<TARGET_FILE:buff_serial_mode_node>)
  set_tests_properties(buff_serial_mode_ros_test PROPERTIES TIMEOUT 45)
  find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc highgui video calib3d
    PATHS "/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/cmake/opencv4" NO_DEFAULT_PATH)
  buff_test(buff_blade_tracker_test)
  target_include_directories(buff_blade_tracker_test PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(buff_blade_tracker_test PRIVATE
    PkgConfig::BUFF_ROS ${OpenCV_LIBS})
  buff_test(buff_angle_processor_test)
  target_include_directories(buff_angle_processor_test PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(buff_angle_processor_test PRIVATE
    ${OpenCV_LIBS} Eigen3::Eigen)
  find_package(Ceres REQUIRED)
  add_executable(armor_input_guard_test
    "${BUFF_PACKAGE}/test/armor_input_guard_test.cpp"
    "${BUFF_PACKAGE}/include/CoorConverter.cpp"
    "${BUFF_PACKAGE}/include/math.cpp"
    "${BUFF_PACKAGE}/include/MPC.cpp"
    "${BUFF_PACKAGE}/include/trajectory_visualizer.cpp")
  target_include_directories(armor_input_guard_test PRIVATE
    "${BUFF_PACKAGE}/include" "${BUFF_MESSAGE_INCLUDE_DIR}"
    ${OpenCV_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})
  target_link_libraries(armor_input_guard_test PRIVATE
    PkgConfig::BUFF_ROS ${OpenCV_LIBS} Eigen3::Eigen ${CERES_LIBRARIES} yaml-cpp)
  add_test(NAME armor_input_guard_test COMMAND "${Python3_EXECUTABLE}"
    "${BUFF_REPO}/scripts/run_ros_test.py" $<TARGET_FILE:armor_input_guard_test>
    "${BUFF_PACKAGE}/config/rm_tracker.yaml")
endif()

add_test(NAME fu_frame_timing_contracts COMMAND "${Python3_EXECUTABLE}"
  "${BUFF_PACKAGE}/test/test_fu_frame_timing.py")
add_test(NAME fu_config_layout_contracts COMMAND "${Python3_EXECUTABLE}"
  "${BUFF_PACKAGE}/test/test_fu_config_layout.py")
