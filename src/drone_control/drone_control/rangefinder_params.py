#!/usr/bin/env python3
"""
Set (or just read) the EKF2 range-finder parameters on a running PX4.

The airframe file (4022_gz_x500_survey) already sets these as boot defaults;
this tool is the runtime alternative for a SITL that was started with another
airframe, or to check what the vehicle is actually using.

Talks MAVLink to PX4's *onboard* link (SITL instance 0 sends it to UDP 14540),
so it works while QGroundControl owns the GCS link on 14550.

EKF2_HGT_REF is reboot-required: after setting it, restart PX4.
"""

import argparse
import struct
import sys
from typing import Dict, Optional, Tuple, Union

Number = Union[int, float]

# name -> desired value.  ints are MAV_PARAM_TYPE_INT32, floats REAL32.
RANGEFINDER_PARAMS: Dict[str, Number] = {
    'EKF2_RNG_CTRL': 2,      # 0 off, 1 conditional, 2 always fuse
    'EKF2_HGT_REF': 2,       # 0 baro, 1 GPS, 2 range sensor, 3 vision (reboot required)
    'EKF2_RNG_POS_Z': 0.05,  # sensor 5 cm below the body origin (matches the SDF)
}


def _encode(value: Number, mavlink) -> Tuple[float, int]:
    """
    Return (param_value, param_type) for PARAM_SET.

    PX4 sends and expects integer parameters as the raw int32 bit pattern
    stored in the float field, not as a converted float.
    """
    if isinstance(value, int):
        as_float = struct.unpack('<f', struct.pack('<i', value))[0]
        return as_float, mavlink.MAV_PARAM_TYPE_INT32
    return float(value), mavlink.MAV_PARAM_TYPE_REAL32


def _decode(message, mavlink) -> Number:
    if message.param_type in (mavlink.MAV_PARAM_TYPE_INT32, mavlink.MAV_PARAM_TYPE_INT16,
                              mavlink.MAV_PARAM_TYPE_INT8, mavlink.MAV_PARAM_TYPE_UINT8,
                              mavlink.MAV_PARAM_TYPE_UINT16, mavlink.MAV_PARAM_TYPE_UINT32):
        return struct.unpack('<i', struct.pack('<f', message.param_value))[0]
    return float(message.param_value)


def _read_param(link, name: str, timeout: float) -> Optional[Number]:
    link.mav.param_request_read_send(link.target_system, link.target_component,
                                     name.encode('ascii'), -1)
    deadline_attempts = max(1, int(timeout / 0.5))
    for _ in range(deadline_attempts):
        reply = link.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
        if reply is not None and reply.param_id.strip('\x00') == name:
            return _decode(reply, link.mav.mavlink)  # type: ignore[attr-defined]
    return None


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument('--port', type=int, default=14540,
                        help='UDP port PX4 sends its onboard MAVLink link to (default 14540)')
    parser.add_argument('--read', action='store_true',
                        help='only read the current values, do not change anything')
    parser.add_argument('--timeout', type=float, default=10.0,
                        help='seconds to wait for a PX4 heartbeat')
    args = parser.parse_args(argv)

    try:
        from pymavlink import mavutil
    except ImportError:
        print('pymavlink is not installed: pip install pymavlink', file=sys.stderr)
        return 2
    mavlink = mavutil.mavlink

    link = mavutil.mavlink_connection(f'udpin:127.0.0.1:{args.port}',
                                      source_system=255, source_component=191)
    if link.recv_match(type='HEARTBEAT', blocking=True, timeout=args.timeout) is None:
        print(f'no PX4 heartbeat on UDP {args.port} - is SITL running?', file=sys.stderr)
        return 1
    print(f'connected to system {link.target_system} on UDP {args.port}')

    failures = 0
    for name, wanted in RANGEFINDER_PARAMS.items():
        before = _read_param(link, name, args.timeout)
        if args.read:
            print(f'  {name:16s} = {before}')
            continue
        value, param_type = _encode(wanted, mavlink)
        link.mav.param_set_send(link.target_system, link.target_component,
                                name.encode('ascii'), value, param_type)
        after = _read_param(link, name, args.timeout)
        ok = after is not None and abs(float(after) - float(wanted)) < 1e-6
        failures += 0 if ok else 1
        print(f'  {name:16s} {before} -> {after}  {"OK" if ok else "FAILED"}')

    if not args.read:
        print('EKF2_HGT_REF is reboot-required: restart PX4 for the height reference to change.')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
