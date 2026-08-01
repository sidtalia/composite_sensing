# composite_sensing

Stereo camera + derived sensing for HOUND.

- **`CameraDevice` / `RealsenseCameraDevice`** — transport-agnostic stereo/color/depth
- **`VslamBackend` / `CuvslamBackend`** — VO/SLAM from stereo frames (Isaac `libcuvslam`)
- **Nodes:** `stereo_composite_node` (legacy monolith), `stereo_composite_modular_node` (preferred)

Future: mesh-based ICP and lidar composite nodes belong here.

## Build deps

See `scripts/install_deps.sh`. cuVSLAM headers/libs come from Isaac ROS Jazzy:

- include: `/opt/ros/jazzy/share/isaac_ros_nitros/cuvslam/include`
- lib: `/opt/ros/jazzy/lib/libcuvslam.so`

## Run

Prefer launching via `hound_core` SSoT (`stereo_composite:` block). Manual:

```bash
ros2 run composite_sensing stereo_composite_modular_node --ros-args \
  -p serial_number:=030422250301 -p enable_color:=true
```
