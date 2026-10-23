#!/usr/bin/env python3
"""
Publish a Jetson CSI camera (IMX219 / Raspberry Pi v2 class) as sensor_msgs/Image.

Runs ONLY on the Jetson: it needs OpenCV built with GStreamer (JetPack's
python3-opencv has it) and the nvarguscamerasrc plugin. On any other machine
use ``--ros-args -p device:=/dev/video0`` to read a V4L2 camera instead.

Publishes ``image_topic`` (rgb8) at ``fps`` and, when ``camera_info_url``
points at a YAML written by ``ros2 run camera_calibration cameracalibrator``,
``camera_info_topic`` with the calibration so the ground station can put the
real K / distortion into the stitcher manifest.

Mount check (same as in simulation): with the vehicle facing North the image
top must be North and image right East; otherwise set
``image_top_yaw_offset_rad`` on the ground station.
"""

from __future__ import annotations

import os
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
import yaml


def gst_pipeline(sensor_id: int, width: int, height: int, fps: int, flip: int) -> str:
    """Build the nvarguscamerasrc -> BGR pipeline for OpenCV's appsink."""
    return (
        f'nvarguscamerasrc sensor-id={sensor_id} ! '
        f'video/x-raw(memory:NVMM), width={width}, height={height}, framerate={fps}/1 ! '
        f'nvvidconv flip-method={flip} ! video/x-raw, format=BGRx ! '
        f'videoconvert ! video/x-raw, format=BGR ! appsink drop=true max-buffers=1'
    )


def load_camera_info(path: str, frame_id: str) -> Optional[CameraInfo]:
    """Read a ROS camera_calibration YAML (image_width, camera_matrix, ...)."""
    if not path:
        return None
    with open(os.path.expanduser(path)) as handle:
        data = yaml.safe_load(handle)
    info = CameraInfo()
    info.header.frame_id = frame_id
    info.width = int(data['image_width'])
    info.height = int(data['image_height'])
    info.distortion_model = str(data.get('distortion_model', 'plumb_bob'))
    info.k = [float(v) for v in data['camera_matrix']['data']]
    info.d = [float(v) for v in data['distortion_coefficients']['data']]
    identity = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    info.r = [float(v) for v in data.get('rectification_matrix', {}).get('data', identity)]
    info.p = [float(v) for v in data.get('projection_matrix', {}).get('data', [0.0] * 12)]
    return info


class CsiCameraNode(Node):
    """Grab frames with OpenCV and publish them without cv_bridge."""

    def __init__(self) -> None:
        super().__init__('csi_camera')
        self.declare_parameter('sensor_id', 0)
        self.declare_parameter('device', '')          # non-empty: V4L2 device instead of CSI
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 5)
        self.declare_parameter('flip_method', 0)
        self.declare_parameter('frame_id', 'survey_camera_link')
        self.declare_parameter('image_topic', '/survey/camera/image')
        self.declare_parameter('camera_info_topic', '/survey/camera/camera_info')
        self.declare_parameter('camera_info_url', '')

        import cv2  # imported here so the file parses on machines without OpenCV
        self._cv2 = cv2
        width = int(self.get_parameter('width').value)
        height = int(self.get_parameter('height').value)
        fps = int(self.get_parameter('fps').value)
        device = str(self.get_parameter('device').value)
        if device:
            self._capture = cv2.VideoCapture(device)
            self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            source = device
        else:
            source = gst_pipeline(int(self.get_parameter('sensor_id').value), width, height, fps,
                                  int(self.get_parameter('flip_method').value))
            self._capture = cv2.VideoCapture(source, cv2.CAP_GSTREAMER)
        if not self._capture.isOpened():
            raise RuntimeError(f'cannot open camera: {source}')

        self._frame_id = str(self.get_parameter('frame_id').value)
        self._image_pub = self.create_publisher(
            Image, str(self.get_parameter('image_topic').value), qos_profile_sensor_data)
        self._info = load_camera_info(str(self.get_parameter('camera_info_url').value),
                                      self._frame_id)
        self._info_pub = self.create_publisher(
            CameraInfo, str(self.get_parameter('camera_info_topic').value),
            qos_profile_sensor_data) if self._info else None
        self._frames = 0
        self.create_timer(1.0 / max(1, fps), self._on_timer)
        self.get_logger().info(f'camera open: {source}; calibration: '
                               f'{"yes" if self._info else "none (hfov model will be used)"}')

    def _on_timer(self) -> None:
        ok, frame = self._capture.read()
        if not ok:
            self.get_logger().warn('camera read failed', throttle_duration_sec=5.0)
            return
        rgb = self._cv2.cvtColor(frame, self._cv2.COLOR_BGR2RGB)
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.height, msg.width = rgb.shape[:2]
        msg.encoding = 'rgb8'
        msg.step = msg.width * 3
        msg.data = rgb.tobytes()
        self._image_pub.publish(msg)
        if self._info_pub is not None:
            self._info.header.stamp = msg.header.stamp
            self._info_pub.publish(self._info)
        self._frames += 1
        if self._frames == 1:
            self.get_logger().info(f'publishing {msg.width}x{msg.height} rgb8')

    def destroy_node(self) -> bool:
        self._capture.release()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = CsiCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:  # noqa: BLE001 - surface config errors to the user
        print(f'csi_camera failed to start: {error}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
