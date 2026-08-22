#!/usr/bin/env bash
# Install / verify host deps for composite_sensing:
#   - stereo: librealsense2 + cuVSLAM (Isaac or /opt/cuvslam)
#   - lidar mesh PF: libembree-dev, libassimp-dev, libvulkan-dev, glslang-tools, unilidar_sdk
#
# Usage (inside mushr_jazzy / ROS Jazzy env):
#   bash /root/colcon_ws/src/composite_sensing/scripts/install_deps.sh
#   # or after install:
#   ros2 run composite_sensing install_deps.sh
#
# Set SKIP_APT=1 to only verify. Set APT_YES=0 to disable noninteractive apt.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_SRC="$(cd "${PKG_ROOT}/.." && pwd)"
UNITREE_LIB_AARCH64="${WS_SRC}/unilidar_sdk/unitree_lidar_sdk/lib/aarch64/libunitree_lidar_sdk.a"
UNITREE_LIB_X86="${WS_SRC}/unilidar_sdk/unitree_lidar_sdk/lib/x86_64/libunitree_lidar_sdk.a"

APT_YES="${APT_YES:-1}"
SKIP_APT="${SKIP_APT:-0}"
FAILED=0

log() { echo "[composite_sensing] $*"; }
warn() { echo "[composite_sensing] WARNING: $*" >&2; }
err() { echo "[composite_sensing] ERROR: $*" >&2; FAILED=1; }

need_sudo() {
  if [[ "$(id -u)" -eq 0 ]]; then
    "$@"
  elif command -v sudo >/dev/null 2>&1; then
    sudo "$@"
  else
    err "need root or sudo to run: $*"
    return 1
  fi
}

install_apt_packages() {
  local pkgs=("$@")
  local missing=()
  local p
  for p in "${pkgs[@]}"; do
    if ! dpkg -s "$p" >/dev/null 2>&1; then
      missing+=("$p")
    fi
  done
  if [[ ${#missing[@]} -eq 0 ]]; then
    log "apt packages already installed: ${pkgs[*]}"
    return 0
  fi
  if [[ "${SKIP_APT}" == "1" ]]; then
    warn "SKIP_APT=1 — not installing: ${missing[*]}"
    return 0
  fi
  log "apt install: ${missing[*]}"
  need_sudo apt-get update -qq
  local args=(-y)
  if [[ "${APT_YES}" == "1" ]]; then
    export DEBIAN_FRONTEND=noninteractive
  fi
  need_sudo apt-get install "${args[@]}" "${missing[@]}"
}

# --- apt deps (stereo + lidar mesh PF) ---------------------------------------
log "installing apt dependencies"
install_apt_packages \
  libembree-dev \
  libassimp-dev \
  libeigen3-dev \
  libvulkan-dev \
  glslang-tools \
  vulkan-tools \
  pkg-config \
  build-essential

# RealSense: prefer distro / already-present SDK (Jetson images often ship it).
if ! pkg-config --exists realsense2 2>/dev/null \
  && [[ ! -f /usr/lib/aarch64-linux-gnu/librealsense2.so ]] \
  && [[ ! -f /usr/local/lib/librealsense2.so ]]; then
  if apt-cache show librealsense2-dev >/dev/null 2>&1; then
    install_apt_packages librealsense2-dev librealsense2-utils || true
  else
    warn "librealsense2 not found and no apt package — install Intel RealSense SDK 2"
  fi
else
  log "librealsense2 OK"
fi

# --- unilidar static lib (workspace sibling) ---------------------------------
ARCH="$(uname -m)"
UNITREE_LIB=""
if [[ "${ARCH}" == "aarch64" || "${ARCH}" == "arm64" ]]; then
  UNITREE_LIB="${UNITREE_LIB_AARCH64}"
else
  UNITREE_LIB="${UNITREE_LIB_X86}"
fi
if [[ -f "${UNITREE_LIB}" ]]; then
  log "Unitree lidar SDK lib OK: ${UNITREE_LIB}"
else
  warn "missing ${UNITREE_LIB}"
  warn "  clone unilidar_sdk next to composite_sensing (expected under ${WS_SRC}/unilidar_sdk)"
fi

# --- Embree / Assimp / Vulkan verify -----------------------------------------
if [[ -f /usr/include/embree4/rtcore.h || -f /usr/include/embree3/rtcore.h ]]; then
  log "Embree headers OK"
else
  err "Embree headers missing after install (libembree-dev)"
fi
if [[ -f /usr/include/assimp/Importer.hpp ]]; then
  log "Assimp headers OK"
else
  err "Assimp headers missing after install (libassimp-dev)"
fi
if [[ -f /usr/include/vulkan/vulkan.h ]]; then
  log "Vulkan headers OK"
else
  warn "Vulkan headers missing (libvulkan-dev) — mesh_pf will build Embree-only"
fi
if command -v glslangValidator >/dev/null 2>&1; then
  log "glslangValidator OK: $(command -v glslangValidator)"
else
  warn "glslangValidator missing (glslang-tools) — mesh_pf will build Embree-only"
fi

# --- cuVSLAM -----------------------------------------------------------------
if [[ -n "${CUVSLAM_INCLUDE_DIR:-}" && -n "${CUVSLAM_LIB:-}" ]]; then
  CUV_INC="${CUVSLAM_INCLUDE_DIR}"
  CUV_LIB="${CUVSLAM_LIB}"
elif [[ -f /opt/cuvslam/include/cuvslam/cuvslam2.h && -f /opt/cuvslam/lib/libcuvslam.so ]]; then
  CUV_INC="/opt/cuvslam/include"
  CUV_LIB="/opt/cuvslam/lib/libcuvslam.so"
else
  CUV_INC="/opt/ros/jazzy/share/isaac_ros_nitros/cuvslam/include"
  CUV_LIB="/opt/ros/jazzy/lib/libcuvslam.so"
fi

if [[ -f "${CUV_INC}/cuvslam/cuvslam2.h" && -f "${CUV_LIB}" ]]; then
  log "cuVSLAM include: ${CUV_INC}"
  log "cuVSLAM lib:     ${CUV_LIB}"
else
  err "cuVSLAM not found (need ${CUV_INC}/cuvslam/cuvslam2.h and ${CUV_LIB})"
  err "  Install Isaac ROS Jazzy (isaac_ros_nitros + visual_slam), or official SDK to /opt/cuvslam,"
  err "  or export CUVSLAM_INCLUDE_DIR and CUVSLAM_LIB"
fi

# --- summary -----------------------------------------------------------------
if [[ "${FAILED}" -ne 0 ]]; then
  err "dependency check failed — fix errors above before building"
  exit 1
fi

log "OK — all required deps present"
log "build:  cd <ws> && source /opt/ros/jazzy/setup.bash && colcon build --packages-select composite_sensing"
log "stereo: ros2 run composite_sensing stereo_composite_modular_node"
log "lidar:  ros2 run composite_sensing lidar_mesh_composite_node --ros-args -p map_file:=/path/to/map.dae"
exit 0
