"""python examples/show_drone.py RUN_DIR X_METERS Y_METERS"""
import sys
from pathlib import Path
import cv2
import numpy as np

from drone_stitch.coordinates import MapCoordinates

run = Path(sys.argv[1])
xy = [[float(sys.argv[2]), float(sys.argv[3])]]
coords = MapCoordinates(run / "transforms.json")
uv = coords.local_to_pixel(xy)
# For telemetry: coords.ned_to_pixel([[north_m, east_m]], frame_id="flight-001-ekf-origin-0")
coverage = coords.coverage_at(uv)
if not coverage["covered"][0]:
    raise SystemExit("Position falls outside valid map coverage")
image = cv2.imread(str(run / "map.png"))
point = tuple(np.rint(uv[0]).astype(int))
cv2.circle(image, point, 12, (0, 0, 255), -1, cv2.LINE_AA)
cv2.putText(image, "DRONE", (point[0] + 18, point[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, .8, (0, 0, 255), 2)
destination = run / "drone_position.png"
if destination.exists():
    raise SystemExit(f"Already exists: {destination}")
if not cv2.imwrite(str(destination), image):
    raise SystemExit("Image write failed")
print(destination)
