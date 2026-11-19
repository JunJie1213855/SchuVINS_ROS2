cmake_minimum_required(VERSION 3.16)
project(schurvins VERSION 0.2.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs video highgui calib3d)
find_package(yaml-cpp REQUIRED)
find_package(glog REQUIRED)
find_package(gflags REQUIRED)
find_package(Threads REQUIRED)
include(cmake/Dependencies.cmake)
add_library(schurvins_core STATIC
  rpg_common/src/fs.cpp
  vikit/vikit_common/src/homography.cpp
  vikit/vikit_common/src/math_utils.cpp
  vikit/vikit_common/src/performance_monitor.cpp
  vikit/vikit_common/src/sample.cpp
  vikit/vikit_common/src/user_input_thread.cpp
  vikit/vikit_common/src/vision.cpp
  vikit/vikit_cameras/src/camera_factory.cpp
  vikit/vikit_cameras/src/ncamera.cpp
  vikit/vikit_cameras/src/camera_geometry_base.cpp
  vikit/vikit_cameras/src/camera_yaml_serialization.cpp
  vikit/vikit_cameras/src/equidistant_fisheye_geometry.cpp
  vikit/vikit_cameras/src/equidistant_fisheye_projection.cpp
  vikit/vikit_cameras/src/omni_geometry.cpp
  vikit/vikit_cameras/src/omni_projection.cpp
  vikit/vikit_cameras/src/ncamera_yaml_serialization.cpp
  vikit/vikit_solver/src/robust_cost.cpp
  svo_common/src/frame.cpp
  svo_common/src/point.cpp
  svo_common/src/local_feature.cpp
  svo_direct/src/patch_warp.cpp
  svo_direct/src/feature_alignment.cpp
  svo_direct/src/feature_detection.cpp
  svo_direct/src/feature_detection_utils.cpp
  svo_direct/src/matcher.cpp
  svo_direct/src/depth_filter.cpp
  svo_direct/src/depth_estimation.cpp
  svo_direct/src/elder_zucker.cpp
  svo_tracker/src/feature_tracking_types.cpp
  svo_tracker/src/feature_tracking_utils.cpp
  svo_tracker/src/feature_tracking_viz.cpp
  svo_tracker/src/feature_tracker.cpp
  svo_img_align/src/sparse_img_align_base.cpp
  svo_img_align/src/sparse_img_align.cpp
  svo_vio_common/src/vio_common.cpp
  svo/src/frame_handler_mono.cpp
  svo/src/frame_handler_stereo.cpp
  svo/src/frame_handler_array.cpp
  svo/src/frame_handler_base.cpp
  svo/src/map.cpp
  svo/src/pose_optimizer.cpp
  svo/src/schur_vins.cpp
  svo/src/initialization.cpp
  svo/src/reprojector.cpp
  svo/src/imu_handler.cpp
  svo/src/stereo_triangulation.cpp
  svo/src/io.cpp
 )
set_target_properties(schurvins_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_definitions(schurvins_core PUBLIC SVO_USE_OPENGV SVO_DEPTHFILTER_IN_REPROJECTOR)
target_include_directories(schurvins_core PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/rpg_common/include>)
target_include_directories(schurvins_core PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vikit/vikit_common/include>)
target_include_directories(schurvins_core PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vikit/vikit_cameras/include>)
target_include_directories(schurvins_core PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vikit/vikit_solver/include>)
target_include_directories(schurvins_core PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/svo_common/include>)
target_include_directories(schurvins_core PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/svo_direct/include>)
target_include_directories(schurvins_core PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/svo_tracker/include>)
target_include_directories(schurvins_core PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/svo_img_align/include>)
target_include_directories(schurvins_core PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/svo_vio_common/include>)
target_include_directories(schurvins_core PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/svo/include>)
target_include_directories(schurvins_core PUBLIC ${minkindr_SOURCE_DIR}/minkindr/include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(schurvins_core PUBLIC Eigen3::Eigen glog::glog gflags yaml-cpp ${OpenCV_LIBS} Threads::Threads schurvins_fast opengv)
add_executable(svo_node ros2/src/node.cpp ros2/src/factory.cpp ros2/src/visualization.cpp)
target_include_directories(svo_node PRIVATE ros2/include)
target_link_libraries(svo_node schurvins_core)
ament_target_dependencies(svo_node rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs tf2_ros message_filters cv_bridge)
install(TARGETS svo_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY ros2/launch ros2/config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY svo_ros/param/calib DESTINATION share/${PROJECT_NAME}/config)
install(FILES README_ROS2.md LICENSE DESTINATION share/${PROJECT_NAME})
install(FILES ${fast_SOURCE_DIR}/LICENSE DESTINATION share/${PROJECT_NAME}/licenses RENAME fast.txt)
install(FILES ${minkindr_SOURCE_DIR}/LICENSE DESTINATION share/${PROJECT_NAME}/licenses RENAME minkindr.txt)
install(FILES ${opengv_src_SOURCE_DIR}/License.txt DESTINATION share/${PROJECT_NAME}/licenses RENAME opengv.txt)
if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  add_executable(landmark_history_test ros2/test/landmark_history_test.cpp)
  target_include_directories(landmark_history_test PRIVATE ros2/include)
  ament_add_test(landmark_history
    COMMAND $<TARGET_FILE:landmark_history_test>
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_test(ros2_stereo_imu_smoke
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/ros2/test/smoke_test.py
      --node $<TARGET_FILE:svo_node>
      --calib ${CMAKE_CURRENT_SOURCE_DIR}/svo_ros/param/calib/euroc_stereo.yaml
      --params ${CMAKE_CURRENT_SOURCE_DIR}/ros2/config/euroc_stereo.yaml
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 90)
  ament_add_test(ros2_tidesense_imu_smoke
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/ros2/test/smoke_test.py
      --node $<TARGET_FILE:svo_node>
      --calib ${CMAKE_CURRENT_SOURCE_DIR}/ros2/config/tidesense_calib.yaml
      --params ${CMAKE_CURRENT_SOURCE_DIR}/ros2/config/tidesense_stereo.yaml
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 90)
endif()
ament_package()
