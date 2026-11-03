#!/usr/bin/env python3
"""
Spin each motor briefly from the companion computer (Pi <-> Pixhawk check).

PROPS OFF.  Uses PX4's actuator test (VEHICLE_CMD_ACTUATOR_TEST, the command
behind QGroundControl's Actuators tab), so the vehicle is never armed.  The
run checks the path one hop at a time and stops at the first failure:

1. link     ``/fmu/out/vehicle_status_v4`` arrives: uXRCE-DDS agent, TELEM2
            wiring and baud, ROS_DOMAIN_ID and px4_msgs all match.
2. command  PX4 answers ACCEPTED on ``/fmu/out/vehicle_command_ack_v1``:
            commands from the Pi reach the commander.
3. motors   each motor spins (look): ESC power, output mapping, motor order
            and spin direction.

PX4 refuses the test while armed or while the safety switch is on, and runs
one command for at most 3 s.  This node repeats a 1 s command every 0.3 s, so
a motor stops within a second if the node dies.

    ros2 run drone_control motor_test                        # motors 1-4, 10 %, 2 s each
    ros2 run drone_control motor_test --motors 1 3 --throttle 0.15
"""

import argparse
import sys
import time
from typing import Callable, Iterable, Optional

from px4_msgs.msg import VehicleCommand, VehicleCommandAck, VehicleStatus
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions

# Same names as waypoint_navigator.py (see README "PX4 topic names"):
# VehicleCommandAck has MESSAGE_VERSION 1, hence the _v1 suffix.
_TOPIC_VEHICLE_COMMAND = 'fmu/in/vehicle_command'
_TOPIC_VEHICLE_COMMAND_ACK = 'fmu/out/vehicle_command_ack_v1'
_TOPIC_VEHICLE_STATUS = 'fmu/out/vehicle_status_v4'

# PX4 quad X (airframe 4019, Holybro X500 V2), seen from above.
_QUAD_X_LAYOUT = {
    1: 'front right, counter-clockwise',
    2: 'rear left, counter-clockwise',
    3: 'front left, clockwise',
    4: 'rear right, clockwise',
}

_RESULT_NAMES = {
    VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED: 'ACCEPTED',
    VehicleCommandAck.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED: 'TEMPORARILY_REJECTED',
    VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED: 'DENIED',
    VehicleCommandAck.VEHICLE_CMD_RESULT_UNSUPPORTED: 'UNSUPPORTED',
    VehicleCommandAck.VEHICLE_CMD_RESULT_FAILED: 'FAILED',
}

MAX_THROTTLE = 0.3          # props are off; this is only "does it spin"
MAX_MOTOR = 12
COMMAND_TIMEOUT_S = 1.0     # PX4 stops the motor this long after the last command
RESEND_PERIOD_S = 0.3
STATUS_WAIT_S = 10.0
SAFETY_WAIT_S = 30.0
ACK_WAIT_S = 2.0
PAUSE_BETWEEN_MOTORS_S = 1.5


class MotorTest(Node):
    """Publishes actuator-test commands and keeps the latest status and ACK."""

    def __init__(self, px4_namespace: str) -> None:
        super().__init__('motor_test')
        # QoS of every /fmu topic on the uXRCE-DDS bridge (as in waypoint_navigator).
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        prefix = px4_namespace.rstrip('/')
        self.status_topic = f'{prefix}/{_TOPIC_VEHICLE_STATUS}'
        self.ack_topic = f'{prefix}/{_TOPIC_VEHICLE_COMMAND_ACK}'
        self._command_pub = self.create_publisher(
            VehicleCommand, f'{prefix}/{_TOPIC_VEHICLE_COMMAND}', qos)
        self.create_subscription(VehicleStatus, self.status_topic, self._on_status, qos)
        self.create_subscription(VehicleCommandAck, self.ack_topic, self._on_ack, qos)
        self.status: Optional[VehicleStatus] = None
        self.ack: Optional[VehicleCommandAck] = None
        self.commands_sent = 0

    def _on_status(self, msg: VehicleStatus) -> None:
        self.status = msg

    def _on_ack(self, msg: VehicleCommandAck) -> None:
        if msg.command == VehicleCommand.VEHICLE_CMD_ACTUATOR_TEST:
            self.ack = msg

    def wait(self, seconds: float, until: Optional[Callable[[], bool]] = None) -> bool:
        """Spin for up to `seconds`; return early (True) once `until()` holds."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if until is not None and until():
                return True
        return until is not None and until()

    def send_test(self, motor: int, throttle: float, timeout_s: float) -> None:
        """Run `motor` at `throttle` (0..1) for `timeout_s`; timeout 0 releases it."""
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_ACTUATOR_TEST
        msg.param1 = float(throttle)
        msg.param2 = float(timeout_s)
        msg.param5 = float(motor)   # MAVLink ACTUATOR_OUTPUT_FUNCTION_MOTOR<n> == n
        system_id = self.status.system_id if self.status is not None else 1
        msg.target_system = system_id
        msg.target_component = 1
        msg.source_system = system_id
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self._command_pub.publish(msg)
        self.commands_sent += 1

    def release(self, motors: Iterable[int]) -> None:
        """Hand the outputs back to PX4 (they also time out on their own)."""
        for _ in range(2):
            for motor in motors:
                self.send_test(motor, 0.0, 0.0)
            self.wait(0.1)


def _is_armed(status: VehicleStatus) -> bool:
    return status.arming_state == VehicleStatus.ARMING_STATE_ARMED


def _check_link(node: MotorTest) -> bool:
    print(f'Waiting for PX4 on {node.status_topic} ...', flush=True)
    if not node.wait(STATUS_WAIT_S, until=lambda: node.status is not None):
        print(f'FAIL [1/3 link]: nothing on {node.status_topic} in {STATUS_WAIT_S:.0f} s.\n'
              '  - Is MicroXRCEAgent running (serial --dev /dev/ttyAMA0 -b 921600)?\n'
              '  - Does the agent log show "session established"? If not: TELEM2 wiring '
              '(TX/RX crossed, GND), UXRCE_DDS_CFG=102, SER_TEL2_BAUD=921600.\n'
              '  - ROS_DOMAIN_ID here must equal UXRCE_DDS_DOM_ID on PX4 (7 on the vehicle).\n'
              '  - Check the topic names with  ros2 topic list | grep fmu  (px4_msgs '
              'built from the same PX4 version?).')
        return False
    status = node.status
    safety = 'none'
    if status.safety_button_available:
        safety = 'off' if status.safety_off else 'ON'
    print(f'OK   [1/3 link]: PX4 system {status.system_id}, '
          f'{"ARMED" if _is_armed(status) else "disarmed"}, nav_state {status.nav_state}, '
          f'safety switch {safety}', flush=True)
    return True


def _wait_for_safety_off(node: MotorTest) -> bool:
    status = node.status
    if not status.safety_button_available or status.safety_off:
        return True
    print(f'Safety switch is ON. Press and hold the safety button on the GPS until its LED '
          f'stays lit (waiting {SAFETY_WAIT_S:.0f} s) ...', flush=True)
    if node.wait(SAFETY_WAIT_S, until=lambda: bool(node.status.safety_off)):
        print('Safety switch is off.', flush=True)
        return True
    print('FAIL: safety switch still on; PX4 would deny the motor test.')
    return False


def _confirm(args: argparse.Namespace) -> bool:
    print(f'\n*** PROPELLERS MUST BE OFF ***\n'
          f'Motors {", ".join(map(str, args.motors))} will spin one at a time at '
          f'{args.throttle * 100:.0f} % for {args.seconds:g} s each.')
    try:
        answer = input('Props removed and everyone clear? Type "yes": ')
    except EOFError:
        answer = ''
    if answer.strip().lower() != 'yes':
        print('Aborted, nothing was sent.')
        return False
    return True


def _spin_motor(node: MotorTest, motor: int, args: argparse.Namespace, first: bool) -> bool:
    where = _QUAD_X_LAYOUT.get(motor, 'see QGC Actuators tab for its position')
    print(f'\nMotor {motor} ({where}): {args.throttle * 100:.0f} % for {args.seconds:g} s',
          flush=True)
    node.ack = None
    node.send_test(motor, args.throttle, COMMAND_TIMEOUT_S)
    if not node.wait(ACK_WAIT_S, until=lambda: node.ack is not None):
        print(f'FAIL [2/3 command]: no answer on {node.ack_topic} in {ACK_WAIT_S:.0f} s. '
              'Status arrives but commands do not: check that /fmu/in/vehicle_command '
              'is listed by  ros2 topic list  and that only one agent is running.')
        return False
    result = node.ack.result
    if result != VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
        name = _RESULT_NAMES.get(result, f'result {result}')
        hint = {
            VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED:
                'PX4 denies the actuator test while armed or while the safety switch is on.',
            VehicleCommandAck.VEHICLE_CMD_RESULT_UNSUPPORTED:
                f'Motor {motor} is not a valid output function on this firmware.',
        }.get(result, '')
        print(f'FAIL [2/3 command]: PX4 answered {name}. {hint}')
        return False
    if first:
        print('OK   [2/3 command]: PX4 accepted the command from this computer.', flush=True)

    end = time.monotonic() + args.seconds
    while True:
        remaining = end - time.monotonic()
        if remaining <= 0.0:
            break
        node.wait(min(RESEND_PERIOD_S, remaining))
        if time.monotonic() < end:
            node.send_test(motor, args.throttle, COMMAND_TIMEOUT_S)
    node.release([motor])
    print(f'Motor {motor} stopped.', flush=True)
    return True


def _run(node: MotorTest, args: argparse.Namespace) -> int:
    if not _check_link(node):
        return 1
    if _is_armed(node.status):
        print('FAIL: the vehicle is ARMED. Disarm it first; PX4 only runs the test disarmed.')
        return 1
    if not _wait_for_safety_off(node):
        return 1
    if not args.yes and not _confirm(args):
        return 1

    for index, motor in enumerate(args.motors):
        if index > 0:
            node.wait(PAUSE_BETWEEN_MOTORS_S)
        if not _spin_motor(node, motor, args, first=index == 0):
            return 1

    print('\nOK   [3/3 motors]: every command was accepted. Now judge by eye:\n'
          '  - None spun: kill switch (SD) on? Battery connected (USB alone does not power '
          'the ESCs)? PWM_MAIN_FUNC1-4 = Motor 1-4? ESC calibration done? Try --throttle 0.15.\n'
          '  - Wrong position: motor wires on the wrong outputs (or PWM_MAIN_FUNCn mapping).\n'
          '  - Wrong direction: swap any two of the three motor wires on that ESC.')
    return 0


def _parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument('--motors', type=int, nargs='+', default=[1, 2, 3, 4],
                        help='motor numbers in test order (default: 1 2 3 4)')
    parser.add_argument('--throttle', type=float, default=0.10,
                        help=f'0 < throttle <= {MAX_THROTTLE} (default 0.10 = 10 %%)')
    parser.add_argument('--seconds', type=float, default=2.0,
                        help='spin time per motor, 0.5-10 s (default 2)')
    parser.add_argument('--yes', action='store_true', help='skip the props-off prompt')
    parser.add_argument('--px4-namespace', default='', help="e.g. '/px4_1' (default none)")
    args, ros_args = parser.parse_known_args(argv)
    if not 0.0 < args.throttle <= MAX_THROTTLE:   # also rejects NaN
        parser.error(f'--throttle must be in (0, {MAX_THROTTLE}]')
    if not 0.5 <= args.seconds <= 10.0:
        parser.error('--seconds must be between 0.5 and 10')
    bad = [m for m in args.motors if not 1 <= m <= MAX_MOTOR]
    if bad:
        parser.error(f'--motors must be between 1 and {MAX_MOTOR}, got {bad}')
    return args, ros_args


def main(argv=None) -> int:
    args, ros_args = _parse_args(argv)
    # Keep the ROS context alive on Ctrl+C so the motors can still be released.
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = MotorTest(args.px4_namespace)
    try:
        return _run(node, args)
    except KeyboardInterrupt:
        print('\nInterrupted.')
        return 1
    finally:
        if node.commands_sent:
            node.release(args.motors)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
