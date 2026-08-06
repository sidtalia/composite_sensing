# composite_sensing

Stereo + lidar composite sensing for HOUND.

- **`CameraDevice` / `RealsenseCameraDevice`** — transport-agnostic stereo/color/depth
- **`VslamBackend` / `CuvslamBackend`** — VO/SLAM from stereo frames (Isaac `libcuvslam`)
- **`LidarDevice` / `UnilidarLidarDevice`** — Unitree SDK hot path (Ouster/Livox adapters TBD)
- **`MeshLocalizerBackend` / `ConstantTwistMeshPfBackend`** — Embree mesh PF, internal constant-twist motion (no EKF odom)
- **Nodes:** `stereo_composite_modular_node`, `lidar_mesh_composite_node`

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

The script installs apt packages (`libembree-dev`, `libassimp-dev`, `libeigen3-dev`, …), checks RealSense + Unitree SDK + cuVSLAM paths, and exits non-zero if anything required is missing.

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

ros2 run composite_sensing lidar_mesh_composite_node --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p map_file:=/path/to/map.dae \
  -p localize_hz:=10.0
```
