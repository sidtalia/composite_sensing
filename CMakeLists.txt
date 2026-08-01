cmake_minimum_required(VERSION 3.8)
project(composite_sensing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(realsense2 REQUIRED)

include_directories(include)

# In-process stereo + cuVSLAM (same libcuvslam.so as Isaac ROS).
set(CUVSLAM_INCLUDE_DIR "/opt/ros/jazzy/share/isaac_ros_nitros/cuvslam/include")
set(CUVSLAM_LIB "/opt/ros/jazzy/lib/libcuvslam.so")

add_executable(stereo_composite_node
  src/stereo_composite_node.cpp
)
target_include_directories(stereo_composite_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${CUVSLAM_INCLUDE_DIR}
  ${realsense2_INCLUDE_DIR}
)
ament_target_dependencies(stereo_composite_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)
target_link_libraries(stereo_composite_node
  ${realsense2_LIBRARY}
  ${CUVSLAM_LIB}
)
target_compile_features(stereo_composite_node PUBLIC cxx_std_17)

add_executable(stereo_composite_modular_node
  src/stereo_composite_modular_main.cpp
  src/stereo_composite_modular_node.cpp
  src/realsense_camera_device.cpp
  src/cuvslam_backend.cpp
)
target_include_directories(stereo_composite_modular_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${CUVSLAM_INCLUDE_DIR}
  ${realsense2_INCLUDE_DIR}
)
ament_target_dependencies(stereo_composite_modular_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)
target_link_libraries(stereo_composite_modular_node
  ${realsense2_LIBRARY}
  ${CUVSLAM_LIB}
)
target_compile_features(stereo_composite_modular_node PUBLIC cxx_std_17)

install(TARGETS
  stereo_composite_node
  stereo_composite_modular_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_python_install_package(${PROJECT_NAME}
  SCRIPTS_DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/stereo_composite_stereo
  scripts/install_deps.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
