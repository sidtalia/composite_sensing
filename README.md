# composite_sensing

Stereo + lidar composite sensing for HOUND.

- **`CameraDevice` / `RealsenseCameraDevice` / `DatasetCameraDevice`** — transport-agnostic stereo/color/depth (RealSense or rail-sim PNG dataset)
- **`VslamBackend` / `CuvslamBackend`** — VO/SLAM from stereo frames (Isaac `libcuvslam`)
- **`LidarDevice` / `UnilidarLidarDevice` / `LivoxLidarDevice`** — Unitree + Livox Mid-360(S) (SDK2); Ouster TBD
- **`MeshLocalizerBackend` / `ConstantTwistMeshPfBackend`** — mesh PF with Vulkan `rayQuery` (Orin RT cores) or Embree CPU fallback; internal constant-twist motion (no EKF odom)
- **Nodes:** `stereo_composite_modular_node`, `lidar_mesh_composite_node`, `multicam_rail_replay`

## Known limitation: cuVSLAM

cuVSLAM can be **overconfident** after large motions: tracking continues in a new attitude reference while reported confidence stays high. With a single VSLAM source into the EKF, glitches are hard to detect (tilt re-align is a partial mitigation). Prefer multi-cam VSLAM or an independent absolute pose (GPS / mesh ICP / mocap). GPS should make disagreement easier to spot; mocap validation is planned. See `hound_core/config/SSoT.yaml` (`stereo_composite` header) and `hound_core/docs/context.md`.

`lidar_mesh_composite_node` publishes `/localization/mesh_pose` (`PoseWithCovarianceStamped`) as a map-absolute aid for the EKF, plus optional `/unilidar/cloud` and static `parent→lidar` TF. Relocalize: `ros2 service call /lidar_mesh_composite/global_localization std_srvs/srv/Empty {}`.

## Install dependencies

From a fresh container / host (needs `sudo` or root for apt):

```bash
bash /root/colcon_ws/src/composite_sensing/scripts/install_deps.sh
```

Or after the package is built/installed:

```bash
ros2 run composite_sensing install_deps.sh
```

The script installs apt packages (`libembree-dev`, `libassimp-dev`, `libvulkan-dev`, `glslang-tools`, `libeigen3-dev`, …), checks RealSense + Unitree SDK + cuVSLAM paths, and exits non-zero if anything required is missing. Vulkan packages are optional at build time: without them `mesh_pf_node` still links Embree.

| Env | Effect |
|-----|--------|
| `SKIP_APT=1` | Verify only; do not run `apt-get` |
| `CUVSLAM_INCLUDE_DIR` / `CUVSLAM_LIB` | Override cuVSLAM discovery |

cuVSLAM resolution order: env overrides → `/opt/cuvslam` → Isaac ROS Jazzy (`isaac_ros_nitros` + `libcuvslam.so`).

Workspace sibling expected for lidar:

- `../unilidar_sdk/unitree_lidar_sdk/lib/<arch>/libunitree_lidar_sdk.a`

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd /root/colcon_ws
colcon build --packages-select composite_sensing
source install/setup.bash
```

## Run

Prefer launching via `hound_core` SSoT (`stereo_composite:` / `lidar.composite:`). Manual:

```bash
ros2 run composite_sensing stereo_composite_modular_node --ros-args \
  -p serial_number:=030422250301 -p enable_color:=true

# DatasetCameraDevice (rail sim) via the same modular node:
ros2 run composite_sensing stereo_composite_modular_node --ros-args \
  -p camera_backend:=dataset \
  -p dataset_path:=/path/to/rail_sim/race-2_multicam \
  -p dataset_view:=front \
  -p dataset_rate_hz:=10.0 \
  -p enable_vslam:=false \
  -p warmup_frames:=0 \
  -p enable_color:=true -p enable_depth:=true \
  -p camera_name:=front -p base_frame:=front_link

ros2 run composite_sensing lidar_mesh_composite_node --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p map_file:=/path/to/map.dae \
  -p localize_hz:=10.0
```

## mesh_pf raycast backend

`mesh_pf_node` (SSoT `mesh_pf:`) traces closest-hit rays against a triangle map.

| `raycast_backend` | Behavior |
|---|---|
| `auto` (default) | Vulkan `rayQuery` (Orin RT cores) if device creation succeeds, else Embree |
| `vulkan` | Vulkan only; node fails if RT is unavailable |
| `embree` | CPU Embree packet intersect (`rtcIntersect4`) |

```bash
ros2 run composite_sensing mesh_pf_node --ros-args \
  -p map_file:=/path/to/map.dae \
  -p cloud_topic:=/livox/cloud \
  -p raycast_backend:=auto
```
