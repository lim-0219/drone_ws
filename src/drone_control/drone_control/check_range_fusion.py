#!/usr/bin/env python3
"""
Watch the range-finder path end to end and report whether EKF2 fuses it.

Prints once a second:  bridged scan rate, DistanceSensor rate published to
PX4, and vehicle_local_position.dist_bottom / dist_bottom_valid / the sensor
bitfield.  Exit code 0 if dist_bottom became valid *from the range sensor*.

    ros2 run drone_control check_range_fusion --seconds 20
"""

import argparse
import sys
from typing import Optional

from px4_msgs.msg import DistanceSensor, VehicleLocalPosition
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image, LaserScan


class RangeFusionCheck(Node):
    """Counts messages on the three hops and remembers the latest position."""

    def __init__(self, range_topic: str, camera_topic: str, px4_namespace: str) -> None:
        super().__init__('check_range_fusion')
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.scans = 0
        self.images = 0
        self.last_image_time: Optional[float] = None
        self.distance_msgs = 0
        self.last_range: Optional[float] = None
        self.position: Optional[VehicleLocalPosition] = None
        self.create_subscription(LaserScan, range_topic, self._on_scan, qos_profile_sensor_data)
        # Same reliable profile as the navigator's camera subscription.
        camera_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.VOLATILE,
                                history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, camera_topic, self._on_image, camera_qos)
        self.create_subscription(
            DistanceSensor, f'{px4_namespace}/fmu/in/distance_sensor', self._on_distance, px4_qos)
        self.create_subscription(
            VehicleLocalPosition, f'{px4_namespace}/fmu/out/vehicle_local_position_v1',
            self._on_position, px4_qos)

    def _on_scan(self, msg: LaserScan) -> None:
        self.scans += 1
        if msg.ranges:
            self.last_range = float(msg.ranges[0])

    def _on_image(self, _msg: Image) -> None:
        self.images += 1
        self.last_image_time = self.get_clock().now().nanoseconds * 1e-9

    def _on_distance(self, _msg: DistanceSensor) -> None:
        self.distance_msgs += 1

    def _on_position(self, msg: VehicleLocalPosition) -> None:
        self.position = msg


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument('--seconds', type=float, default=15.0)
    parser.add_argument('--range-topic', default='/survey/range/scan')
    parser.add_argument('--camera-topic', default='/survey/camera/image')
    parser.add_argument('--px4-namespace', default='')
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = RangeFusionCheck(args.range_topic, args.camera_topic, args.px4_namespace.rstrip('/'))
    fused_from_range = False
    try:
        elapsed = 0.0
        while elapsed < args.seconds:
            scans_before, dist_before, img_before = node.scans, node.distance_msgs, node.images
            end = node.get_clock().now().nanoseconds + int(1e9)
            while node.get_clock().now().nanoseconds < end:
                rclpy.spin_once(node, timeout_sec=0.05)
            elapsed += 1.0

            pos = node.position
            if pos is None:
                status = 'no vehicle_local_position yet'
            else:
                from_range = bool(pos.dist_bottom_sensor_bitfield
                                  & VehicleLocalPosition.DIST_BOTTOM_SENSOR_RANGE)
                fused_from_range = fused_from_range or (pos.dist_bottom_valid and from_range)
                status = (f'dist_bottom={pos.dist_bottom:6.2f} m '
                          f'valid={pos.dist_bottom_valid!s:5} '
                          f'range_bit={from_range!s:5} z={pos.z:6.2f}')
            last = f'{node.last_range:.2f}' if node.last_range is not None else '  -- '
            print(f'[{elapsed:4.0f}s] cam {node.images - img_before:3d}/s  '
                  f'scan {node.scans - scans_before:3d}/s (last {last} m)  '
                  f'-> distance_sensor {node.distance_msgs - dist_before:3d}/s  | {status}',
                  flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if node.images == 0:
        print('CAMERA: no image arrived - bridge not running or cannot discover Gazebo '
              '(run the bridge with GZ_IP=127.0.0.1, same as PX4 starts Gazebo).')
    elif node.images < args.seconds:
        print(f'CAMERA: only {node.images} images in {args.seconds:.0f}s - frames are being '
              'lost; raise net.core.rmem_max and set CYCLONEDDS_URI (see README).')
    else:
        print(f'CAMERA: {node.images} images in {args.seconds:.0f}s '
              f'(~{node.images / args.seconds:.1f} Hz).')
    if fused_from_range:
        print('RESULT: dist_bottom is valid and sourced from the range sensor.')
        return 0
    print('RESULT: dist_bottom never became valid from the range sensor. Check: is the '
          'x500_survey model running, the ros_gz bridge up, the navigator node running, '
          'and EKF2_RNG_CTRL != 0?')
    return 1


if __name__ == '__main__':
    sys.exit(main())
