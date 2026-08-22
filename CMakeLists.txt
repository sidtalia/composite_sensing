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
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(realsense2 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(assimp REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)

include_directories(include)

# Embree 4 (mesh PF CPU raycast fallback). Optional Vulkan RT if present.
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

# Livox-SDK2 (Mid-360 / Mid-360S). Built separately under ../Livox-SDK2/build.
set(LIVOX_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../Livox-SDK2")
set(LIVOX_SDK_INCLUDE "${LIVOX_SDK_ROOT}/include")
set(LIVOX_SDK_LIB "${LIVOX_SDK_ROOT}/build/sdk_core/liblivox_lidar_sdk_static.a")
if(NOT EXISTS "${LIVOX_SDK_LIB}")
  set(LIVOX_SDK_LIB "${LIVOX_SDK_ROOT}/build/sdk_core/liblivox_lidar_sdk_shared.so")
endif()
if(EXISTS "${LIVOX_SDK_LIB}" AND EXISTS "${LIVOX_SDK_INCLUDE}/livox_lidar_api.h")
  set(HAVE_LIVOX_SDK ON)
  message(STATUS "Livox-SDK2: ${LIVOX_SDK_LIB}")
else()
  set(HAVE_LIVOX_SDK OFF)
  message(WARNING
    "Livox-SDK2 not found (expected ${LIVOX_SDK_ROOT}/build/sdk_core/liblivox*). "
    "Build Livox-SDK2 first; livox backend will fail to link.")
endif()

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
  src/dataset_camera_device.cpp
  src/cuvslam_backend.cpp
)
target_include_directories(stereo_composite_modular_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${CUVSLAM_INCLUDE_DIR}
  ${realsense2_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
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
  ${OpenCV_LIBS}
)
target_compile_features(stereo_composite_modular_node PUBLIC cxx_std_17)

# --- Lidar composite (SDK + static TF + in-process deskew) --------------------
set(BUILD_LIDAR_MESH_COMPOSITE OFF)
if(EXISTS "${UNITREE_SDK_LIB}")
  set(BUILD_LIDAR_MESH_COMPOSITE ON)
  message(STATUS "lidar_mesh_composite: unitree=${UNITREE_SDK_LIB}")
  set(LIDAR_MESH_SRCS
    src/lidar_mesh_composite_main.cpp
    src/lidar_mesh_composite_node.cpp
    src/cloud_deskew.cpp
    src/unilidar_lidar_device.cpp
  )
  if(HAVE_LIVOX_SDK)
    list(APPEND LIDAR_MESH_SRCS src/livox_lidar_device.cpp)
  endif()
  add_executable(lidar_mesh_composite_node ${LIDAR_MESH_SRCS})
  target_include_directories(lidar_mesh_composite_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${UNITREE_SDK_ROOT}/include
  )
  if(HAVE_LIVOX_SDK)
    target_include_directories(lidar_mesh_composite_node PUBLIC ${LIVOX_SDK_INCLUDE})
    target_compile_definitions(lidar_mesh_composite_node PUBLIC HOUND_HAVE_LIVOX_SDK=1)
  endif()
  ament_target_dependencies(lidar_mesh_composite_node
    rclcpp
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf2
    tf2_ros
  )
  target_link_libraries(lidar_mesh_composite_node
    ${UNITREE_SDK_LIB}
    Eigen3::Eigen
  )
  if(HAVE_LIVOX_SDK)
    target_link_libraries(lidar_mesh_composite_node ${LIVOX_SDK_LIB} pthread)
  endif()
  target_compile_features(lidar_mesh_composite_node PUBLIC cxx_std_17)
else()
  message(WARNING
    "lidar_mesh_composite DISABLED (need unilidar_sdk static lib). "
    "unitree=${UNITREE_SDK_LIB}")
endif()

# --- Standalone mesh PF (Embree CPU + optional Vulkan RT) ---------------------
set(BUILD_MESH_PF OFF)
if(EMBREE_LIB AND EMBREE_INCLUDE_DIR)
  set(BUILD_MESH_PF ON)
  message(STATUS "mesh_pf: embree=${EMBREE_LIB}")

  find_package(Vulkan QUIET)
  find_program(GLSLANG_VALIDATOR glslangValidator)
  set(HAVE_VULKAN_RT OFF)
  set(MESH_PF_VULKAN_SRCS)
  set(MESH_PF_SPV_HEADER)
  if(Vulkan_FOUND AND GLSLANG_VALIDATOR)
    set(HAVE_VULKAN_RT ON)
    set(MESH_PF_SPV_HEADER "${CMAKE_CURRENT_BINARY_DIR}/mesh_rayquery.spv.h")
    add_custom_command(
      OUTPUT "${MESH_PF_SPV_HEADER}"
      COMMAND ${GLSLANG_VALIDATOR} -V
        "${CMAKE_CURRENT_SOURCE_DIR}/shaders/mesh_rayquery.comp"
        --target-env vulkan1.2
        --vn kMeshRayquerySpv
        -o "${MESH_PF_SPV_HEADER}"
      DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/shaders/mesh_rayquery.comp"
      COMMENT "SPIR-V: mesh_rayquery.comp"
      VERBATIM
    )
    list(APPEND MESH_PF_VULKAN_SRCS src/vulkan_mesh_raycaster.cpp)
    set_source_files_properties(
      src/vulkan_mesh_raycaster.cpp
      PROPERTIES
        OBJECT_DEPENDS "${MESH_PF_SPV_HEADER}"
        COMPILE_FLAGS "-Wno-missing-field-initializers"
    )
    message(STATUS "mesh_pf: Vulkan RT enabled (${Vulkan_LIBRARY})")
  else()
    message(STATUS
      "mesh_pf: Vulkan RT disabled "
      "(Vulkan_FOUND=${Vulkan_FOUND} glslangValidator=${GLSLANG_VALIDATOR})")
  endif()

  add_executable(mesh_pf_node
    src/mesh_pf_main.cpp
    src/mesh_pf_node.cpp
    src/constant_twist_mesh_pf_backend.cpp
    src/assimp_mesh_load.cpp
    src/embree_mesh_map.cpp
    src/mesh_raycaster.cpp
    ${MESH_PF_VULKAN_SRCS}
  )
  target_include_directories(mesh_pf_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${EMBREE_INCLUDE_DIR}
  )
  target_include_directories(mesh_pf_node PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}
  )
  ament_target_dependencies(mesh_pf_node
    rclcpp
    sensor_msgs
    geometry_msgs
    std_srvs
    tf2
  )
  target_link_libraries(mesh_pf_node
    ${EMBREE_LIB}
    assimp::assimp
  )
  if(HAVE_VULKAN_RT)
    target_compile_definitions(mesh_pf_node PRIVATE HOUND_HAVE_VULKAN_RT=1)
    if(TARGET Vulkan::Vulkan)
      target_link_libraries(mesh_pf_node Vulkan::Vulkan)
    else()
      target_include_directories(mesh_pf_node PRIVATE ${Vulkan_INCLUDE_DIRS})
      target_link_libraries(mesh_pf_node ${Vulkan_LIBRARIES})
    endif()
  endif()
  target_compile_features(mesh_pf_node PUBLIC cxx_std_17)
  if(HAVE_VULKAN_RT)
    add_custom_target(mesh_pf_rayquery_spv DEPENDS "${MESH_PF_SPV_HEADER}")
    add_dependencies(mesh_pf_node mesh_pf_rayquery_spv)
  endif()
else()
  message(WARNING
    "mesh_pf DISABLED (need libembree-dev). "
    "embree_lib=${EMBREE_LIB} embree_inc=${EMBREE_INCLUDE_DIR}")
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
if(BUILD_MESH_PF)
  install(TARGETS mesh_pf_node
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
