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
find_package(std_srvs REQUIRED)
find_package(assimp REQUIRED)

include_directories(include)

# Embree 4 (mesh PF raycast). Optional: skip lidar_mesh_composite if missing.
find_library(EMBREE_LIB NAMES embree4 embree3)
find_path(EMBREE_INCLUDE_DIR embree4/rtcore.h)
if(NOT EMBREE_INCLUDE_DIR)
  find_path(EMBREE_INCLUDE_DIR embree3/rtcore.h)
endif()
set(UNITREE_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../unilidar_sdk/unitree_lidar_sdk")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(UNITREE_SDK_LIB_DIR "${UNITREE_SDK_ROOT}/lib/aarch64")
else()
  set(UNITREE_SDK_LIB_DIR "${UNITREE_SDK_ROOT}/lib/x86_64")
endif()
set(UNITREE_SDK_LIB "${UNITREE_SDK_LIB_DIR}/libunitree_lidar_sdk.a")

# Prefer official NVIDIA cuVSLAM SDK under /opt/cuvslam; fall back to Isaac ROS.
if(DEFINED ENV{CUVSLAM_INCLUDE_DIR} AND DEFINED ENV{CUVSLAM_LIB})
  set(CUVSLAM_INCLUDE_DIR "$ENV{CUVSLAM_INCLUDE_DIR}")
  set(CUVSLAM_LIB "$ENV{CUVSLAM_LIB}")
elseif(EXISTS "/opt/cuvslam/include/cuvslam/cuvslam2.h" AND EXISTS "/opt/cuvslam/lib/libcuvslam.so")
  set(CUVSLAM_INCLUDE_DIR "/opt/cuvslam/include")
  set(CUVSLAM_LIB "/opt/cuvslam/lib/libcuvslam.so")
elseif(EXISTS "/opt/ros/jazzy/share/isaac_ros_nitros/cuvslam/include/cuvslam/cuvslam2.h"
    AND EXISTS "/opt/ros/jazzy/lib/libcuvslam.so")
  set(CUVSLAM_INCLUDE_DIR "/opt/ros/jazzy/share/isaac_ros_nitros/cuvslam/include")
  set(CUVSLAM_LIB "/opt/ros/jazzy/lib/libcuvslam.so")
else()
  message(FATAL_ERROR
    "cuVSLAM not found. Install official SDK to /opt/cuvslam, or Isaac ROS "
    "(isaac_ros_nitros + libcuvslam.so), or set CUVSLAM_INCLUDE_DIR and CUVSLAM_LIB.")
endif()
message(STATUS "cuVSLAM include: ${CUVSLAM_INCLUDE_DIR}")
message(STATUS "cuVSLAM lib:     ${CUVSLAM_LIB}")

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

# --- Lidar + constant-twist mesh PF composite ---------------------------------
set(BUILD_LIDAR_MESH_COMPOSITE OFF)
if(EMBREE_LIB AND EMBREE_INCLUDE_DIR AND EXISTS "${UNITREE_SDK_LIB}")
  set(BUILD_LIDAR_MESH_COMPOSITE ON)
  message(STATUS "lidar_mesh_composite: embree=${EMBREE_LIB} unitree=${UNITREE_SDK_LIB}")
  add_executable(lidar_mesh_composite_node
    src/lidar_mesh_composite_main.cpp
    src/lidar_mesh_composite_node.cpp
    src/unilidar_lidar_device.cpp
    src/constant_twist_mesh_pf_backend.cpp
    src/embree_mesh_map.cpp
  )
  target_include_directories(lidar_mesh_composite_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${EMBREE_INCLUDE_DIR}
    ${UNITREE_SDK_ROOT}/include
  )
  ament_target_dependencies(lidar_mesh_composite_node
    rclcpp
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf2
    tf2_ros
    std_srvs
  )
  target_link_libraries(lidar_mesh_composite_node
    ${EMBREE_LIB}
    ${UNITREE_SDK_LIB}
    assimp::assimp
  )
  target_compile_features(lidar_mesh_composite_node PUBLIC cxx_std_17)
else()
  message(WARNING
    "lidar_mesh_composite DISABLED (need libembree-dev, unilidar_sdk static lib). "
    "embree_lib=${EMBREE_LIB} embree_inc=${EMBREE_INCLUDE_DIR} unitree=${UNITREE_SDK_LIB}")
endif()

install(TARGETS
  stereo_composite_node
  stereo_composite_modular_node
  DESTINATION lib/${PROJECT_NAME}
)
if(BUILD_LIDAR_MESH_COMPOSITE)
  install(TARGETS lidar_mesh_composite_node
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(DIRECTORY include/
  DESTINATION include
)

ament_python_install_package(${PROJECT_NAME}
  SCRIPTS_DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/stereo_composite_stereo
  scripts/install_deps.sh
  scripts/multicam_rail_replay
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
