#!/usr/bin/env python3
r"""
Record a mission video from inside the simulation (no screen capture).

Left: the world's third-person `overview_camera` (make_grass_world.py puts
one south-west of the field).  Right: the drone's downward survey camera;
once the ground station reports the session completed, the finished map is
shown there instead.  A status bar carries PX4 mode / arming, the local
position, and the web app's session progress (n/9, scanning, completed).

Both camera streams are read straight from Gazebo with the gz-transport
Python bindings (python3-gz-transport13 / python3-gz-msgs10): over TCP they
arrive complete at full rate, whereas the 2.8-3.7 MB frames are mostly lost
through a ros_gz bridge on the default CycloneDDS socket buffers (README,
"large images over DDS").  PX4 state comes from ROS 2, the session progress
from the web app's REST API.  Frames are composed at 10 Hz and piped to
ffmpeg (H.264, played back at 2x).  Runs with the ROS Python (rclpy, PIL,
requests); needs `ffmpeg` on PATH:

    export GZ_IP=127.0.0.1 ROS_DOMAIN_ID=7 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    python3 tools/record_mission_video.py --output mission.mp4 --max-seconds 480 &
    ros2 launch drone_control mosaic_mission.launch.py rviz:=false after_mosaic:=land

Stop with Ctrl+C or let it stop itself: once the session is completed (or
failed) it records until the vehicle disarms (plus 3 s) or `--after-complete`
seconds have passed, whichever comes first.  Sessions created before the
recorder started are ignored, so a web app left running from an earlier
mission does not end the recording early.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import io
import os
import signal
import subprocess
import sys
import threading
import time
from typing import Optional

from gz.msgs10.image_pb2 import Image as GzImage
from gz.transport13 import Node as GzNode
from PIL import Image as PilImage, ImageDraw, ImageFont
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
import requests

W, H = 1600, 660
LEFT = (16, 60, 1024, 576)      # x, y, w, h
RIGHT = (1056, 60, 528, 396)
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
NAV = {4: 'AUTO_LOITER', 14: 'OFFBOARD', 18: 'AUTO_LAND', 2: 'POSITION', 0: 'MANUAL',
       3: 'AUTO_MISSION'}
RGB_INT8 = 3                    # gz.msgs.PixelFormatType


class Recorder(Node):
    """Keep the latest overview / drone frames, PX4 state and web-app session."""

    def __init__(self, webapp: str, overview_topic: str, drone_topic: str) -> None:
        super().__init__('mission_recorder')
        px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self._lock = threading.Lock()
        self._images: dict = {'overview': None, 'drone': None}
        self.status: Optional[VehicleStatus] = None
        self.pos: Optional[VehicleLocalPosition] = None
        self.session: dict = {}
        self.map_img: Optional[PilImage.Image] = None
        self.webapp = webapp
        self._last_poll = 0.0
        self._started_at = datetime.now(timezone.utc)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v4', self._on_status,
                                 px4_qos)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
                                 self._on_pos, px4_qos)
        # gz-transport callbacks arrive on a non-Python thread: convert there, hand over
        # the finished PIL image under the lock.
        self._gz = GzNode()
        self._gz_topics = []
        for topic, key in ((overview_topic, 'overview'), (drone_topic, 'drone')):
            if not self._gz.subscribe(GzImage, topic, self._gz_handler(key)):
                self.get_logger().error(f'gz subscribe failed: {topic}')
            else:
                self._gz_topics.append(topic)
                self.get_logger().info(f'gz subscribed: {topic} -> {key} pane')

    def close(self) -> None:
        """Unsubscribe before the interpreter shuts down (a live gz callback then aborts)."""
        for topic in self._gz_topics:
            self._gz.unsubscribe(topic)
        self._gz_topics = []
        time.sleep(0.2)
        self._gz = None

    def _gz_handler(self, key: str):
        def handler(msg: GzImage) -> None:
            if msg.pixel_format_type != RGB_INT8:
                return
            img = PilImage.frombytes('RGB', (msg.width, msg.height), bytes(msg.data))
            with self._lock:
                self._images[key] = img
        return handler

    def images(self):
        with self._lock:
            return self._images['overview'], self._images['drone']

    def _is_current(self, session: dict) -> bool:
        """Return False for a session created before this recorder started."""
        created = str(session.get('created_at', ''))
        try:
            created_at = datetime.fromisoformat(created.replace('Z', '+00:00'))
        except ValueError:
            return True
        if created_at.tzinfo is None:
            created_at = created_at.replace(tzinfo=timezone.utc)
        return created_at >= self._started_at

    def armed(self) -> Optional[bool]:
        if self.status is None:
            return None
        return self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def _on_status(self, m):
        self.status = m

    def _on_pos(self, m):
        self.pos = m

    def poll_session(self) -> None:
        now = time.monotonic()
        if now - self._last_poll < 1.0:
            return
        self._last_poll = now
        try:
            r = requests.get(f'{self.webapp}/api/sessions/latest', timeout=1.0)
            if r.status_code == 200 and self._is_current(r.json()):
                self.session = r.json()
                if (self.session.get('status') == 'completed' and self.map_img is None
                        and self.session.get('map_url')):
                    m = requests.get(f"{self.webapp}{self.session['map_url']}", timeout=5.0)
                    if m.status_code == 200:
                        self.map_img = PilImage.open(io.BytesIO(m.content)).convert('RGB')
        except requests.RequestException:
            pass


def fit(img: PilImage.Image, box) -> PilImage.Image:
    x, y, w, h = box
    scale = min(w / img.width, h / img.height)
    return img.resize((max(1, int(img.width * scale)), max(1, int(img.height * scale))),
                      PilImage.BILINEAR)


def compose(rec: Recorder, t: float, fonts, drone_topic: str) -> PilImage.Image:
    canvas = PilImage.new('RGB', (W, H), (18, 20, 24))
    draw = ImageDraw.Draw(canvas)
    big, med, small = fonts
    draw.text((16, 14), 'drone_control  mosaic mission  -  grass field + garbage bags '
              '(PX4 SITL, Gazebo Harmonic)', font=med, fill=(235, 235, 235))
    draw.text((W - 16, 14), f't = {t:6.1f} s   (playback 2x)', font=med, fill=(160, 160, 160),
              anchor='ra')

    def panel(box, img, title):
        x, y, w, h = box
        draw.rectangle([x - 2, y - 2, x + w + 1, y + h + 1], outline=(70, 70, 80))
        if img is not None:
            thumb = fit(img, box)
            canvas.paste(thumb, (x + (w - thumb.width) // 2, y + (h - thumb.height) // 2))
        else:
            draw.text((x + w // 2, y + h // 2), 'waiting for image...', font=med,
                      fill=(120, 120, 120), anchor='mm')
        draw.text((x + 8, y + h - 26), title, font=small, fill=(255, 255, 255))

    overview, drone = rec.images()
    panel(LEFT, overview, 'overview camera (world, 3rd person)')
    if rec.map_img is not None:
        panel(RIGHT, rec.map_img, 'stitched map (web app result)')
    else:
        panel(RIGHT, drone, f'drone downward camera ({drone_topic})')

    # status bar
    y0 = 470
    st, pos = rec.status, rec.pos
    mode = NAV.get(st.nav_state, f'nav_state {st.nav_state}') if st else '-'
    armed = 'ARMED' if (st and st.arming_state == VehicleStatus.ARMING_STATE_ARMED) else 'disarmed'
    col = (120, 230, 120) if armed == 'ARMED' else (200, 200, 200)
    draw.text((1056, y0), f'PX4: {mode}   {armed}', font=big, fill=col)
    if pos is not None and pos.xy_valid:
        draw.text((1056, y0 + 40), f'N {pos.x:6.2f} m   E {pos.y:6.2f} m   alt {-pos.z:5.2f} m'
                  + (f'   AGL {pos.dist_bottom:4.2f} m' if pos.dist_bottom_valid else ''),
                  font=med, fill=(220, 220, 220))
    s = rec.session
    received, total = int(s.get('received', 0) or 0), int(s.get('total', 9) or 9)
    status = s.get('status', '-') if s else 'waiting for session'
    draw.text((1056, y0 + 78), f'frames {received}/{total}   session: {status}',
              font=med, fill=(240, 220, 120))
    bx, by, bw, bh = 1056, y0 + 112, 528, 22
    draw.rectangle([bx, by, bx + bw, by + bh], outline=(120, 120, 120))
    if total:
        draw.rectangle([bx, by, bx + int(bw * received / total), by + bh], fill=(240, 200, 60))
    if status == 'scanning' or status == 'queued':
        draw.text((bx + bw // 2, by + bh // 2), 'MAP SCANNING', font=small, fill=(30, 30, 30),
                  anchor='mm')
    elif status == 'completed':
        draw.text((bx + bw // 2, by + bh // 2), 'MAP COMPLETED', font=small, fill=(30, 30, 30),
                  anchor='mm')
    draw.text((16, 648), 'left: Gazebo overview camera   right: drone camera -> stitched map   '
              'state: /fmu/out/vehicle_status_v4, /fmu/out/vehicle_local_position_v1, '
              'web app /api/sessions/latest', font=small, fill=(110, 110, 110), anchor='ls')
    return canvas


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--poster', help='PNG of the last frame')
    parser.add_argument('--max-seconds', type=float, default=480.0)
    parser.add_argument('--after-complete', type=float, default=45.0,
                        help='max seconds to keep recording after the session completed')
    parser.add_argument('--after-disarm', type=float, default=3.0)
    parser.add_argument('--webapp', default='http://127.0.0.1:8000')
    parser.add_argument('--overview-topic', default='/overview_camera/image')
    parser.add_argument('--drone-topic', default='/survey/camera/image')
    parser.add_argument('--fps', type=float, default=10.0)
    parser.add_argument('--speed', type=float, default=2.0)
    args = parser.parse_args(argv)
    os.environ.setdefault('GZ_IP', '127.0.0.1')   # PX4 starts Gazebo on loopback

    fonts = (ImageFont.truetype(FONT, 26), ImageFont.truetype(FONT, 20),
             ImageFont.truetype(FONT, 15))
    ff = subprocess.Popen(
        ['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
         '-s', f'{W}x{H}', '-framerate', str(args.fps * args.speed), '-i', '-',
         '-c:v', 'libx264', '-preset', 'veryfast', '-crf', '24', '-pix_fmt', 'yuv420p',
         '-movflags', '+faststart', args.output],
        stdin=subprocess.PIPE)
    rclpy.init()
    rec = Recorder(args.webapp, args.overview_topic, args.drone_topic)
    stop = {'flag': False}
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__('flag', True))
    start = time.monotonic()
    completed_at: Optional[float] = None
    disarmed_at: Optional[float] = None
    frames = 0
    last = None
    try:
        while not stop['flag']:
            rclpy.spin_once(rec, timeout_sec=0.02)
            rec.poll_session()
            now = time.monotonic()
            if now - start >= frames / args.fps:
                last = compose(rec, now - start, fonts, args.drone_topic)
                ff.stdin.write(last.tobytes())
                frames += 1
            if rec.session.get('status') in ('completed', 'failed') and completed_at is None:
                completed_at = now
            if completed_at is not None:
                if rec.armed() is False and disarmed_at is None:
                    disarmed_at = now
                if disarmed_at is not None and now - disarmed_at > args.after_disarm:
                    break
                if now - completed_at > args.after_complete:
                    break
            if now - start > args.max_seconds:
                break
    except KeyboardInterrupt:
        pass
    finally:
        try:
            ff.stdin.close()
        except Exception:
            pass
        ff.wait(timeout=60)
        if args.poster and last is not None:
            last.save(args.poster)
        rec.close()
        rec.destroy_node()
        rclpy.shutdown()
    print(f'{args.output}: {frames} frames, {frames / args.fps:.0f} s real time, '
          f'{frames / (args.fps * args.speed):.0f} s video, session {rec.session.get("status")}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
