#!/usr/bin/env python3
"""Replay a multicam rail dataset: ROS depth/color/info + TF (GT poses, no VSLAM).

  ros2 run composite_sensing multicam_rail_replay --ros-args \\
    -p dataset:=.../rail_sim/race-2_multicam -p rate_hz:=10.0

ClipSeg encoder packs N ``/{view}/color/image_raw`` into a Dora rgb batch when
``color_topics`` is configured (approximate sync by shared stamp).
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def _load_kv(path: Path) -> dict:
    kv = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if "=" in line:
            k, v = line.split("=", 1)
            kv[k.strip()] = v.strip()
    return kv


def _mat_to_tf(T: np.ndarray, parent: str, child: str, stamp) -> TransformStamped:
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.transform.translation.x = float(T[0, 3])
    msg.transform.translation.y = float(T[1, 3])
    msg.transform.translation.z = float(T[2, 3])
    R = T[:3, :3]
    tr = float(np.trace(R))
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2.0
        qw, qx = 0.25 * s, (R[2, 1] - R[1, 2]) / s
        qy, qz = (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw, qx = (R[2, 1] - R[1, 2]) / s, 0.25 * s
        qy, qz = (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw, qx = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s
        qy, qz = 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw, qx = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s
        qy, qz = (R[1, 2] + R[2, 1]) / s, 0.25 * s
    msg.transform.rotation.x = float(qx)
    msg.transform.rotation.y = float(qy)
    msg.transform.rotation.z = float(qz)
    msg.transform.rotation.w = float(qw)
    return msg


def _camera_info(width, height, fx, fy, cx, cy, frame_id, stamp) -> CameraInfo:
    info = CameraInfo()
    info.header.stamp = stamp
    info.header.frame_id = frame_id
    info.width = width
    info.height = height
    info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    info.distortion_model = "plumb_bob"
    return info


def _depth_msg(depth_mm: np.ndarray, frame_id: str, stamp) -> Image:
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = depth_mm.shape[:2]
    msg.encoding = "16UC1"
    msg.is_bigendian = False
    msg.step = int(msg.width * 2)
    msg.data = np.ascontiguousarray(depth_mm).tobytes()
    return msg


def _rgb_msg(rgb: np.ndarray, frame_id: str, stamp) -> Image:
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = rgb.shape[:2]
    msg.encoding = "rgb8"
    msg.is_bigendian = False
    msg.step = int(msg.width * 3)
    msg.data = np.ascontiguousarray(rgb).tobytes()
    return msg


class MulticamRailReplay(Node):
    def __init__(self):
        super().__init__("multicam_rail_replay")
        self.declare_parameter("dataset", "")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("loop", True)
        self.declare_parameter("global_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        dataset = Path(self.get_parameter("dataset").as_string())
        if not dataset.is_dir():
            raise RuntimeError(f"dataset not found: {dataset}")
        self.dataset = dataset
        self.kv = _load_kv(dataset / "camera.txt")
        self.views = [
            v.strip() for v in self.kv.get("views", "front").split(",") if v.strip()
        ]
        self.n_frames = int(self.kv.get("n_frames", 0))
        self.width = int(self.kv.get("width", 640))
        self.height = int(self.kv.get("height", 360))
        self.fx = float(self.kv.get("fx", 400))
        self.fy = float(self.kv.get("fy", 400))
        self.cx = float(self.kv.get("cx", self.width / 2))
        self.cy = float(self.kv.get("cy", self.height / 2))
        self.global_frame = self.get_parameter("global_frame").as_string()
        self.base_frame = self.get_parameter("base_frame").as_string()
        self.loop = self.get_parameter("loop").as_bool()
        self.idx = 0

        self.tf_static = StaticTransformBroadcaster(self)
        self.tf_dyn = TransformBroadcaster(self)
        self._publish_static_extrinsics()

        self.depth_pubs = {}
        self.depth_info_pubs = {}
        self.color_pubs = {}
        self.color_info_pubs = {}
        for v in self.views:
            self.depth_pubs[v] = self.create_publisher(
                Image, f"/{v}/depth/image_rect_raw", qos_profile_sensor_data
            )
            self.depth_info_pubs[v] = self.create_publisher(
                CameraInfo, f"/{v}/depth/camera_info", qos_profile_sensor_data
            )
            self.color_pubs[v] = self.create_publisher(
                Image, f"/{v}/color/image_raw", qos_profile_sensor_data
            )
            self.color_info_pubs[v] = self.create_publisher(
                CameraInfo, f"/{v}/color/camera_info", qos_profile_sensor_data
            )
        self.odom_pub = self.create_publisher(
            Odometry, "/visual_slam/tracking/odometry", 10
        )

        rate = float(self.get_parameter("rate_hz").as_double())
        self.timer = self.create_timer(1.0 / max(rate, 0.1), self._tick)
        self.get_logger().info(
            f"multicam_rail_replay: {dataset} views={self.views} "
            f"n={self.n_frames} @ {rate} Hz"
        )

    def _publish_static_extrinsics(self):
        stamp = self.get_clock().now().to_msg()
        statics = []
        for v in self.views:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.base_frame
            t.child_frame_id = f"{v}_link"
            t.transform.rotation.w = 1.0
            statics.append(t)
            T = np.eye(4)
            T[:3, :3] = np.array(
                [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]],
                dtype=np.float64,
            )
            statics.append(
                _mat_to_tf(T, f"{v}_link", f"{v}_optical_frame", stamp)
            )
            for suffix in ("depth_optical_frame", "color_optical_frame"):
                alias = TransformStamped()
                alias.header.stamp = stamp
                alias.header.frame_id = f"{v}_optical_frame"
                alias.child_frame_id = f"{v}_{suffix}"
                alias.transform.rotation.w = 1.0
                statics.append(alias)
        self.tf_static.sendTransform(statics)

    def _tick(self):
        if self.n_frames <= 0:
            return
        if self.idx >= self.n_frames:
            if not self.loop:
                return
            self.idx = 0

        import cv2

        stamp = self.get_clock().now().to_msg()
        stem = f"{self.idx:06d}"
        body_bin = self.dataset / "frames" / "poses_body.bin"
        if body_bin.is_file():
            with open(body_bin, "rb") as f:
                f.seek(self.idx * 16 * 8)
                raw = f.read(16 * 8)
            T_body = np.frombuffer(raw, dtype=np.float64).reshape(4, 4)
        else:
            T_body = np.fromfile(
                self.dataset / "frames" / f"{stem}_front_T_wo.bin",
                dtype=np.float64,
            ).reshape(4, 4)

        self.tf_dyn.sendTransform(
            _mat_to_tf(T_body, self.global_frame, self.base_frame, stamp)
        )
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.global_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = float(T_body[0, 3])
        odom.pose.pose.position.y = float(T_body[1, 3])
        odom.pose.pose.position.z = float(T_body[2, 3])
        tf_msg = _mat_to_tf(T_body, self.global_frame, self.base_frame, stamp)
        odom.pose.pose.orientation = tf_msg.transform.rotation
        self.odom_pub.publish(odom)

        for v in self.views:
            prefix = self.dataset / "frames" / f"{stem}_{v}"
            depth_mm = cv2.imread(
                str(prefix) + "_depth_mm.png", cv2.IMREAD_UNCHANGED
            )
            trav_bgr = cv2.imread(str(prefix) + "_trav.png", cv2.IMREAD_COLOR)
            if depth_mm is None or trav_bgr is None:
                self.get_logger().warn(f"missing frame {stem}_{v}")
                self.idx += 1
                return
            opt = f"{v}_optical_frame"
            self.depth_pubs[v].publish(_depth_msg(depth_mm, opt, stamp))
            self.depth_info_pubs[v].publish(
                _camera_info(
                    self.width,
                    self.height,
                    self.fx,
                    self.fy,
                    self.cx,
                    self.cy,
                    opt,
                    stamp,
                )
            )
            rgb = trav_bgr[:, :, ::-1].copy()
            self.color_pubs[v].publish(_rgb_msg(rgb, opt, stamp))
            self.color_info_pubs[v].publish(
                _camera_info(
                    self.width,
                    self.height,
                    self.fx,
                    self.fy,
                    self.cx,
                    self.cy,
                    opt,
                    stamp,
                )
            )

        self.idx += 1


def main(args=None):
    rclpy.init(args=args)
    node = MulticamRailReplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
