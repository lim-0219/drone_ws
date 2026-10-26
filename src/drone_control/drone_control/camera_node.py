#!/usr/bin/env python3
"""
Downward survey camera on the Raspberry Pi 5 -> ``sensor_msgs/Image``.

The real camera is an **Arducam B0353** (AR0234 2.3 MP colour global shutter,
1920x1200, 3.6 mm M12 lens, Pivariety board).  On the Pi it works only
through libcamera (Arducam: no raspistill/V4L2 ISP path), so the default
backend is **Picamera2**::

    sudo apt install python3-picamera2            # after Arducam's libcamera packages
    ros2 run drone_control survey_camera --ros-args -p fps:=3

Backends (``backend`` parameter):

* ``picamera2`` - libcamera via Picamera2 (Raspberry Pi OS).  Frames come
  out of the ISP already debayered; the node asks for a stream with R,G,B
  byte order and publishes it as ``rgb8`` without any copy besides the
  optional rotation.
* ``v4l2``      - OpenCV ``VideoCapture`` on ``device`` (a USB webcam, bench
  tests on a laptop).  Not usable for the Pivariety camera itself.

QoS: the image is published **RELIABLE / VOLATILE / KEEP_LAST 1**.  A
1920x1200 rgb8 frame is 6.9 MB (~4,700 UDP fragments); a best-effort reader
loses whole frames on the default socket buffers, and the navigator reads
reliably on purpose, so the publisher matches it.  Keep this topic on the
onboard computer: RViz on the ground PC must not subscribe to it over Wi-Fi
(``config/mosaic.rviz`` ships with that display disabled).

Calibration: when ``camera_info_url`` points at the YAML written by
``ros2 run camera_calibration cameracalibrator`` the matching
``CameraInfo`` is published next to every frame, and the ground station
puts the real K / distortion into the stitcher manifest.  Without it the
navigator and ground station use the pinhole model from ``camera_hfov_rad``
(B0353: 1.3495 rad from the 3.6 mm lens and 3 um pixels).

Mount check (same as in simulation): with the vehicle facing North the image
top must be North and image right East.  Fix a rotated mount with
``rotate_deg`` (0/90/180/270, applied in software); a residual yaw offset
goes into the ground station's ``image_top_yaw_offset_rad``.
"""

from __future__ import annotations

import os
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
import yaml

# Matches the navigator's camera subscription (RELIABLE, see module docstring).
IMAGE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE,
                       history=HistoryPolicy.KEEP_LAST, depth=1)


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


class Picamera2Backend:
    """libcamera through Picamera2; returns H x W x 3 uint8 arrays in R,G,B order."""

    def __init__(self, logger, camera_num: int, width: int, height: int, fps: int,
                 exposure_us: int, analogue_gain: float) -> None:
        from picamera2 import Picamera2  # Raspberry Pi OS: python3-picamera2
        self._camera = Picamera2(camera_num)
        # Picamera2 names formats by libcamera's memory layout: 'BGR888' is
        # the stream whose bytes come out R, G, B (what rgb8 wants); 'RGB888'
        # would give B, G, R (OpenCV order).  Check once with a red object.
        config = self._camera.create_video_configuration(
            main={'size': (width, height), 'format': 'BGR888'},
            buffer_count=3,
            controls={'FrameRate': float(fps)},
        )
        self._camera.configure(config)
        controls = {'AwbEnable': True}
        if exposure_us > 0:
            # Fixed short exposure: no motion blur from the hover drift and no
            # frame-to-frame brightness change for the feature matcher.
            controls.update(AeEnable=False, ExposureTime=int(exposure_us),
                            AnalogueGain=float(analogue_gain))
        self._camera.set_controls(controls)
        self._camera.start()
        actual = self._camera.camera_configuration()['main']
        logger.info(f'picamera2: camera {camera_num} -> {actual["size"]} {actual["format"]} '
                    f'@ {fps} fps, exposure '
                    + (f'{exposure_us} us gain {analogue_gain}' if exposure_us > 0 else 'auto'))

    def read(self):
        return self._camera.capture_array('main')

    def close(self) -> None:
        self._camera.stop()
        self._camera.close()


class V4l2Backend:
    """OpenCV VideoCapture (USB cameras / bench tests); returns R,G,B arrays."""

    def __init__(self, logger, device: str, width: int, height: int, fps: int) -> None:
        import cv2
        self._cv2 = cv2
        self._capture = cv2.VideoCapture(device)
        self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._capture.set(cv2.CAP_PROP_FPS, fps)
        if not self._capture.isOpened():
            raise RuntimeError(f'cannot open V4L2 device {device}')
        logger.info(f'v4l2: {device} -> {int(self._capture.get(cv2.CAP_PROP_FRAME_WIDTH))}x'
                    f'{int(self._capture.get(cv2.CAP_PROP_FRAME_HEIGHT))}')

    def read(self):
        ok, frame = self._capture.read()
        if not ok:
            return None
        return self._cv2.cvtColor(frame, self._cv2.COLOR_BGR2RGB)

    def close(self) -> None:
        self._capture.release()


class SurveyCameraNode(Node):
    """Grab frames from the backend and publish them without cv_bridge."""

    def __init__(self) -> None:
        super().__init__('survey_camera')
        self.declare_parameter('backend', 'picamera2')        # 'picamera2' or 'v4l2'
        self.declare_parameter('camera_num', 0)    # Pi 5 connector CAM/DISP 0 or 1
        self.declare_parameter('device', '/dev/video0')        # v4l2 backend only
        self.declare_parameter('width', 1920)
        self.declare_parameter('height', 1200)
        self.declare_parameter('fps', 3)
        self.declare_parameter('rotate_deg', 0)                # 0, 90, 180, 270 (clockwise)
        self.declare_parameter('swap_rb', False)               # if red shows as blue
        self.declare_parameter('exposure_us', 0)               # 0 = auto exposure
        self.declare_parameter('analogue_gain', 1.0)           # with a fixed exposure
        self.declare_parameter('frame_id', 'survey_camera_link')
        self.declare_parameter('image_topic', '/survey/camera/image')
        self.declare_parameter('camera_info_topic', '/survey/camera/camera_info')
        self.declare_parameter('camera_info_url', '')

        import numpy  # noqa: F401 - checked here so the error names the missing package
        self._np = numpy

        width = int(self.get_parameter('width').value)
        height = int(self.get_parameter('height').value)
        fps = int(self.get_parameter('fps').value)
        if width <= 0 or height <= 0 or fps <= 0:
            raise ValueError('"width", "height" and "fps" must be positive')
        self._rotate = int(self.get_parameter('rotate_deg').value) % 360
        if self._rotate not in (0, 90, 180, 270):
            raise ValueError('"rotate_deg" must be 0, 90, 180 or 270')
        self._swap_rb = bool(self.get_parameter('swap_rb').value)

        backend = str(self.get_parameter('backend').value).strip().lower()
        if backend == 'picamera2':
            self._backend = Picamera2Backend(
                self.get_logger(), int(self.get_parameter('camera_num').value), width, height,
                fps, int(self.get_parameter('exposure_us').value),
                float(self.get_parameter('analogue_gain').value))
        elif backend == 'v4l2':
            self._backend = V4l2Backend(self.get_logger(),
                                        str(self.get_parameter('device').value),
                                        width, height, fps)
        else:
            raise ValueError('"backend" must be "picamera2" or "v4l2"')

        self._frame_id = str(self.get_parameter('frame_id').value)
        self._image_pub = self.create_publisher(
            Image, str(self.get_parameter('image_topic').value), IMAGE_QOS)
        self._info = load_camera_info(str(self.get_parameter('camera_info_url').value),
                                      self._frame_id)
        self._info_pub = self.create_publisher(
            CameraInfo, str(self.get_parameter('camera_info_topic').value),
            IMAGE_QOS) if self._info else None
        self._frames = 0
        self._failures = 0
        self.create_timer(1.0 / fps, self._on_timer)
        self.get_logger().info(
            f'camera: {backend}, {width}x{height} @ {fps} fps, rotate {self._rotate} deg; '
            f'calibration: {"yes" if self._info else "none (hfov pinhole model will be used)"}')

    def _on_timer(self) -> None:
        frame = self._backend.read()
        if frame is None:
            self._failures += 1
            self.get_logger().warn('camera read failed', throttle_duration_sec=5.0)
            return
        if self._rotate:
            frame = self._np.rot90(frame, k=-self._rotate // 90)
        if self._swap_rb:
            frame = frame[:, :, ::-1]
        frame = self._np.ascontiguousarray(frame)

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = 'rgb8'
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = frame.tobytes()
        self._image_pub.publish(msg)
        if self._info_pub is not None:
            if (self._info.width, self._info.height) != (msg.width, msg.height):
                self.get_logger().warn(
                    f'calibration is {self._info.width}x{self._info.height} but frames are '
                    f'{msg.width}x{msg.height}; the ground station will ignore it',
                    throttle_duration_sec=30.0)
            self._info.header.stamp = msg.header.stamp
            self._info_pub.publish(self._info)
        self._frames += 1
        if self._frames == 1:
            self.get_logger().info(
                f'publishing {msg.width}x{msg.height} rgb8 ({len(msg.data) / 1e6:.1f} MB/frame)')

    def destroy_node(self) -> bool:
        try:
            self._backend.close()
        except Exception as error:  # noqa: BLE001 - never block shutdown
            self.get_logger().warn(f'camera close failed: {error}')
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = SurveyCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:  # noqa: BLE001 - surface config errors to the user
        print(f'survey_camera failed to start: {error}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
