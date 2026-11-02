#!/usr/bin/env bash
# Print launch arguments for a target DIST metres straight ahead of the vehicle
# (and RIGHT metres to its right), keeping the current heading.
#
#   ARGS=$(~/drone_ws/src/drone_control/tools/forward_target.sh 8)      # 8 m ahead
#   ros2 launch drone_control drone_onboard.launch.py mission:=land target_altitude:=4.0 $ARGS
#
# Reads x, y and heading from PX4 (vehicle_local_position_v1), so the uXRCE-DDS
# agent must be running. Diagnostics go to stderr, the arguments to stdout.
set -u
DIST=${1:-8}
RIGHT=${2:-0}
X0=""; Y0=""; H=""; VALID=""
for _ in $(seq 1 15); do
    OUT=$(timeout 10 ros2 topic echo /fmu/out/vehicle_local_position_v1 --once 2>/dev/null)
    X0=$(awk '/^x:/{print $2; exit}' <<<"$OUT")
    Y0=$(awk '/^y:/{print $2; exit}' <<<"$OUT")
    H=$(awk '/^heading:/{print $2; exit}' <<<"$OUT")
    VALID=$(awk '/^xy_valid:/{print $2; exit}' <<<"$OUT")
    [ -n "$X0" ] && [ -n "$Y0" ] && [ -n "$H" ] && break
    echo "waiting for /fmu/out/vehicle_local_position_v1 (is MicroXRCEAgent running?)" >&2
    sleep 1
done
if [ -z "$X0" ] || [ -z "$H" ]; then
    echo "ERROR: no vehicle_local_position_v1 from PX4" >&2; exit 1
fi
if [ "$VALID" != "true" ] || [[ "$H" == *nan* ]]; then
    echo "ERROR: position or heading not valid yet (xy_valid=$VALID heading=$H) - wait for GPS fix" >&2; exit 1
fi
python3 - "$X0" "$Y0" "$H" "$DIST" "$RIGHT" <<'PY'
import math, sys
x, y, h, d, r = map(float, sys.argv[1:])
n = x + d * math.cos(h) - r * math.sin(h)
e = y + d * math.sin(h) + r * math.cos(h)
print(f'now N={x:.2f} E={y:.2f} heading={math.degrees(h):.1f} deg -> target N={n:.2f} E={e:.2f} '
      f'({d:g} m ahead, {r:g} m right)', file=sys.stderr)
print(f'target_x:={n:.2f} target_y:={e:.2f} target_yaw:={math.degrees(h):.1f}')
PY
