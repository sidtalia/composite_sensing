#!/usr/bin/env bash
# Host/Jetson deps for composite_sensing (stereo camera + cuVSLAM).
# Does not install mesh-ICP / lidar composite (future).
set -euo pipefail

echo "[composite_sensing] checking RealSense + Isaac cuVSLAM paths"

if ! pkg-config --exists realsense2 2>/dev/null && [[ ! -f /usr/lib/aarch64-linux-gnu/librealsense2.so ]] && [[ ! -f /usr/local/lib/librealsense2.so ]]; then
  echo "[composite_sensing] WARNING: librealsense2 not found — install Intel RealSense SDK 2"
fi

CUV_INC="${CUVSLAM_INCLUDE_DIR:-/opt/ros/jazzy/share/isaac_ros_nitros/cuvslam/include}"
CUV_LIB="${CUVSLAM_LIB:-/opt/ros/jazzy/lib/libcuvslam.so}"

if [[ ! -f "${CUV_INC}/cuvslam/cuvslam2.h" ]]; then
  echo "[composite_sensing] ERROR: missing ${CUV_INC}/cuvslam/cuvslam2.h"
  echo "  Install Isaac ROS Jazzy (isaac_ros_nitros / visual_slam) or set CUVSLAM_INCLUDE_DIR"
  exit 1
fi
if [[ ! -f "${CUV_LIB}" ]]; then
  echo "[composite_sensing] ERROR: missing ${CUV_LIB}"
  echo "  Install Isaac ROS Jazzy libcuvslam or set CUVSLAM_LIB"
  exit 1
fi

echo "[composite_sensing] cuVSLAM include: ${CUV_INC}"
echo "[composite_sensing] cuVSLAM lib:     ${CUV_LIB}"
echo "[composite_sensing] OK — build with: colcon build --packages-select composite_sensing"
