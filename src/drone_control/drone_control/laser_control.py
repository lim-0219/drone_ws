#!/usr/bin/env python3
"""
Laser and servo payload controller for the real vehicle (mock backend for SITL).

Hardware (from the purchase list): three 5 V line lasers switched by a
4-channel MOSFET board from Jetson GPIO (3.3 V logic, 10 kOhm pull-downs so
the lasers stay off while the Jetson boots), a physical kill switch on the
laser DC branch, and three FS90MG servos on Pixhawk AUX outputs that move
the emitters between the "triangle" and "line" arrangement.

Interlocks - every one of them must hold for a laser to be switched on:

* ``hardware_enable`` parameter is true (default false: SITL and bench runs
  never drive a pin),
* the mission node asked for it (``LaserCommand.enable``) less than
  ``command_timeout_s`` ago,
* PX4 reports ARMED and the vehicle is airborne (local z below
  ``-min_altitude_m``).

Anything else switches the lasers off. The kill switch is independent of all
of this and is the only thing you should rely on when people are nearby.

Servos are driven through PX4: assign the AUX outputs the "Peripheral via
Actuator Set 1..3" functions in QGroundControl (Vehicle Setup -> Actuators),
then this node sends ``VEHICLE_CMD_DO_SET_ACTUATOR`` (param1..6 = values in
[-1, 1], param7 = 0) whenever the requested pattern changes.
"""

from __future__ import annotations

import time
from typing import List, Optional

from drone_control_msgs.msg import LaserCommand
from px4_msgs.msg import VehicleCommand, VehicleLocalPosition, VehicleStatus
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

PATTERNS = ('triangle', 'line')


class MockGpio:
    """Logs instead of toggling pins; used in SITL and on the bench."""

    def __init__(self, logger, pins: List[int], active_high: bool) -> None:
        self._logger = logger
        self._pins = pins
        self._state = {pin: False for pin in pins}
        self._logger.info(f'laser GPIO backend: mock (pins {pins}, active_high={active_high})')

    def set_all(self, on: bool) -> None:
        for pin in self._pins:
            if self._state[pin] != on:
                self._state[pin] = on
                self._logger.info(f'[mock] laser pin {pin} -> {"ON" if on else "off"}')

    def close(self) -> None:
        self.set_all(False)


class JetsonGpio:
    """Real pins through Jetson.GPIO (BOARD numbering = the 40-pin header)."""

    def __init__(self, logger, pins: List[int], active_high: bool) -> None:
        import Jetson.GPIO as GPIO  # noqa: N814 - library name
        self._gpio = GPIO
        self._pins = pins
        self._active_high = active_high
        self._logger = logger
        GPIO.setmode(GPIO.BOARD)
        off = GPIO.LOW if active_high else GPIO.HIGH
        GPIO.setup(pins, GPIO.OUT, initial=off)
        self._on = False
        logger.info(f'laser GPIO backend: Jetson.GPIO BOARD pins {pins}, '
                    f'active_high={active_high}')

    def set_all(self, on: bool) -> None:
        if on == self._on:
            return
        self._on = on
        level = self._gpio.HIGH if (on == self._active_high) else self._gpio.LOW
        self._gpio.output(self._pins, level)
        self._logger.info(f'laser pins {self._pins} -> {"ON" if on else "off"}')

    def close(self) -> None:
        self.set_all(False)
        self._gpio.cleanup(self._pins)


class LaserControl(Node):
    """Apply LaserCommand requests under the interlocks above."""

    def __init__(self) -> None:
        super().__init__('laser_control')
        self.declare_parameter('hardware_enable', False)
        self.declare_parameter('interlock_bypass', False)   # bench tests only, never in flight
        self.declare_parameter('backend', 'mock')
        self.declare_parameter('laser_pins', [7, 29, 31])
        self.declare_parameter('active_high', True)
        self.declare_parameter('command_topic', '/laser/command')
        self.declare_parameter('state_topic', '/laser/state')
        self.declare_parameter('command_timeout_s', 1.0)
        self.declare_parameter('min_altitude_m', 0.5)
        self.declare_parameter('servo_enable', True)
        self.declare_parameter('servo_triangle', [-1.0, -1.0, -1.0])
        self.declare_parameter('servo_line', [1.0, 1.0, 1.0])
        self.declare_parameter('px4_namespace', '')
        self.declare_parameter('timer_period', 0.1)

        self._hardware_enable = bool(self.get_parameter('hardware_enable').value)
        self._interlock_bypass = bool(self.get_parameter('interlock_bypass').value)
        backend = str(self.get_parameter('backend').value).strip().lower()
        pins = [int(v) for v in self.get_parameter('laser_pins').value]
        active_high = bool(self.get_parameter('active_high').value)
        if backend == 'jetson':
            self._gpio = JetsonGpio(self.get_logger(), pins, active_high)
        elif backend == 'mock':
            self._gpio = MockGpio(self.get_logger(), pins, active_high)
        else:
            raise ValueError('"backend" must be "mock" or "jetson"')
        self._command_timeout = float(self.get_parameter('command_timeout_s').value)
        self._min_altitude = float(self.get_parameter('min_altitude_m').value)
        self._servo_enable = bool(self.get_parameter('servo_enable').value)
        self._servo_values = {
            'triangle': [float(v) for v in self.get_parameter('servo_triangle').value],
            'line': [float(v) for v in self.get_parameter('servo_line').value],
        }
        for name, values in self._servo_values.items():
            if len(values) != 3 or any(not -1.0 <= v <= 1.0 for v in values):
                raise ValueError(f'"servo_{name}" needs three values in [-1, 1]')
        namespace = str(self.get_parameter('px4_namespace').value).strip().rstrip('/')
        if namespace and not namespace.startswith('/'):
            namespace = '/' + namespace

        px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self._command_pub = self.create_publisher(
            VehicleCommand, f'{namespace}/fmu/in/vehicle_command', px4_qos)
        self.create_subscription(VehicleStatus, f'{namespace}/fmu/out/vehicle_status_v4',
                                 self._on_status, px4_qos)
        self.create_subscription(VehicleLocalPosition,
                                 f'{namespace}/fmu/out/vehicle_local_position_v1',
                                 self._on_position, px4_qos)
        self.create_subscription(LaserCommand, str(self.get_parameter('command_topic').value),
                                 self._on_command, 10)
        self._state_pub = self.create_publisher(
            LaserCommand, str(self.get_parameter('state_topic').value), 10)

        self._armed = False
        self._z: Optional[float] = None
        self._requested_enable = False
        self._requested_pattern = ''
        self._request_time: Optional[float] = None
        self._applied_pattern = ''
        self._lasers_on = False
        self._last_block_reason = ''
        self.create_timer(float(self.get_parameter('timer_period').value), self._on_timer)
        self.get_logger().info(
            f'laser_control: hardware_enable={self._hardware_enable} backend={backend} '
            f'servo_enable={self._servo_enable}' + (
                '' if self._hardware_enable else '  (lasers will never fire until '
                'hardware_enable:=true)'))
        if self._interlock_bypass:
            self.get_logger().warn(
                'interlock_bypass is TRUE: armed/airborne checks are OFF. Bench use only - '
                'point the lasers at a wall and keep the kill switch within reach.')

    # ------------------------------------------------------------------
    def _now(self) -> float:
        return time.monotonic()

    def _on_status(self, msg: VehicleStatus) -> None:
        self._armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def _on_position(self, msg: VehicleLocalPosition) -> None:
        self._z = float(msg.z) if msg.z_valid else None

    def _on_command(self, msg: LaserCommand) -> None:
        self._requested_enable = bool(msg.enable)
        if msg.pattern:
            if msg.pattern not in PATTERNS:
                self.get_logger().warn(f'unknown laser pattern "{msg.pattern}" ignored')
            else:
                self._requested_pattern = msg.pattern
        self._request_time = self._now()

    # ------------------------------------------------------------------
    def _block_reason(self) -> str:
        """Empty string when every interlock holds, else why the lasers stay off."""
        if not self._hardware_enable:
            return 'hardware_enable is false'
        if self._request_time is None or self._now() - self._request_time > self._command_timeout:
            return 'no fresh LaserCommand'
        if not self._requested_enable:
            return 'not requested'
        if self._interlock_bypass:
            return ''
        if not self._armed:
            return 'vehicle not armed'
        if self._z is None or self._z > -self._min_altitude:
            return f'not airborne (z={self._z})'
        return ''

    def _apply_pattern(self, pattern: str) -> None:
        values = self._servo_values[pattern]
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_ACTUATOR
        msg.param1, msg.param2, msg.param3 = values
        msg.param4 = msg.param5 = msg.param6 = 0.0
        msg.param7 = 0.0   # actuator index 0 = "Peripheral via Actuator Set 1..6"
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self._command_pub.publish(msg)
        self._applied_pattern = pattern
        self.get_logger().info(f'laser array -> {pattern} (servo values {values})')

    def _on_timer(self) -> None:
        if self._servo_enable and self._requested_pattern \
                and self._requested_pattern != self._applied_pattern:
            self._apply_pattern(self._requested_pattern)

        reason = self._block_reason()
        want_on = reason == ''
        if want_on != self._lasers_on:
            self._lasers_on = want_on
            self._gpio.set_all(want_on)
            self.get_logger().info('LASERS ON' if want_on else f'lasers off ({reason})')
        elif reason and reason != self._last_block_reason and self._requested_enable:
            self.get_logger().warn(f'laser request blocked: {reason}')
        self._last_block_reason = reason

        state = LaserCommand()
        state.header.stamp = self.get_clock().now().to_msg()
        state.enable = self._lasers_on
        state.pattern = self._applied_pattern
        self._state_pub.publish(state)

    def destroy_node(self) -> bool:
        try:
            self._gpio.close()
        except Exception as error:  # noqa: BLE001 - never block shutdown
            self.get_logger().warn(f'GPIO cleanup failed: {error}')
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = LaserControl()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:  # noqa: BLE001 - surface config errors to the user
        print(f'laser_control failed to start: {error}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
