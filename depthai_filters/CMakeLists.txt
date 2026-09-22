cmake_minimum_required(VERSION 3.8)
project(
  depthai_filters
  VERSION 3.4.0
  LANGUAGES CXX C)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
# uncomment the following section in order to fill in further dependencies
# manually. find_package(<dependency> REQUIRED)
include_directories(include)
add_library(
  ${PROJECT_NAME}
  SHARED
  src/detection2d_overlay.cpp
  src/segmentation_overlay.cpp
  src/spatial_bb.cpp
  src/wls_filter.cpp
  src/feature_tracker_overlay.cpp
  src/features_3d.cpp
  src/thermal_temp.cpp
  src/utils.cpp)
find_package(OpenCV REQUIRED COMPONENTS core imgproc ximgproc)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME} PUBLIC
  rclcpp::rclcpp rclcpp_components::component message_filters::message_filters
  cv_bridge::cv_bridge image_transport::image_transport
  ${sensor_msgs_TARGETS} ${vision_msgs_TARGETS} ${visualization_msgs_TARGETS}
  ${depthai_ros_msgs_TARGETS} ${OpenCV_LIBRARIES})
rclcpp_components_register_nodes(${PROJECT_NAME}
  "${PROJECT_NAME}::Detection2DOverlay")
rclcpp_components_register_nodes(${PROJECT_NAME}
  "${PROJECT_NAME}::SegmentationOverlay")
rclcpp_components_register_nodes(${PROJECT_NAME} "${PROJECT_NAME}::WLSFilter")
rclcpp_components_register_nodes(${PROJECT_NAME} "${PROJECT_NAME}::SpatialBB")
rclcpp_components_register_nodes(${PROJECT_NAME}
  "${PROJECT_NAME}::FeatureTrackerOverlay")
rclcpp_components_register_nodes(${PROJECT_NAME} "${PROJECT_NAME}::Features3D")
rclcpp_components_register_nodes(${PROJECT_NAME} "${PROJECT_NAME}::ThermalTemp")

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
ament_export_include_directories(include/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_components message_filters cv_bridge image_transport
  sensor_msgs vision_msgs visualization_msgs depthai_ros_msgs OpenCV)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(filter_contract_test test/FilterContractTest.cpp TIMEOUT 30)
  target_link_libraries(filter_contract_test ${PROJECT_NAME})
endif()
ament_package()
