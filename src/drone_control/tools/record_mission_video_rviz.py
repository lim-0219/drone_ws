#!/usr/bin/env python3
r"""
Record the mosaic mission with an RViz2 view of the drone, as two videos.

Same sources as record_mission_video.py (Gazebo overview camera, downward
survey camera, web-app stitched map, PX4 state) plus a live RViz2 3D view.
RViz2 runs in its own nested X server (Xephyr) so the capture does not
depend on what is on the desktop; frames are grabbed with ffmpeg x11grab.
This recorder also publishes what RViz shows:

* ``/mission_video/path``    - nav_msgs/Path, the flown trajectory (ENU, frame map)
* ``/mission_video/markers`` - the drone (body + rotors), its camera footprint
  and view frustum, every captured cell's footprint (numbered), and the
  ground objects from ``bags.json`` at their true positions.

Outputs:

* ``--rviz-output``  RViz view only (1280x828, header with PX4 state)
* ``--output``       integrated 1920x1080: overview | RViz on top,
  downward camera | stitched map | status underneath

    export GZ_IP=127.0.0.1 ROS_DOMAIN_ID=7 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    python3 tools/record_mission_video_rviz.py --output combined.mp4 \
        --rviz-output rviz.mp4 &
    ros2 launch drone_control mosaic_mission.launch.py rviz:=false after_mosaic:=land

Needs Xephyr (xserver-xephyr) and ffmpeg. Stops like record_mission_video.py.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from typing import List, Optional, Tuple

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from record_mission_video import NAV, Recorder, fit  # noqa: E402

from geometry_msgs.msg import Point, PoseStamped  # noqa: E402
from nav_msgs.msg import Path  # noqa: E402
from PIL import Image as PilImage, ImageDraw, ImageFont  # noqa: E402
from px4_msgs.msg import VehicleStatus  # noqa: E402
from rcl_interfaces.msg import Log  # noqa: E402
import re  # noqa: E402
import rclpy  # noqa: E402
from std_msgs.msg import ColorRGBA  # noqa: E402
from visualization_msgs.msg import Marker, MarkerArray  # noqa: E402

FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
XEPHYR_W, XEPHYR_H = 1330, 880      # Xephyr screen (RViz fills it, no window manager)
RVIZ_CROP = (25, 70, 1305, 842)     # the 3D render panel inside the RViz window
RVIZ_W, RVIZ_H = RVIZ_CROP[2] - RVIZ_CROP[0], RVIZ_CROP[3] - RVIZ_CROP[1]
RVIZ_HEADER = 56
CAM_HFOV = 1.2                      # x500_survey camera (rad), 1280x960
CAM_ASPECT = 960 / 1280
DEFAULT_BAGS = os.path.expanduser(
    '~/PX4-Autopilot/Tools/simulation/gz/models/trash_bags/bags.json')
BAG_RGB = {
    'white': (235, 235, 230), 'translucent_white': (215, 220, 205),
    'yellow': (242, 204, 40), 'blue': (65, 115, 215), 'green': (115, 180, 90),
    'pink': (242, 140, 180), 'orange': (242, 140, 40), 'black': (28, 28, 30),
    'grey': (140, 140, 145),
}

# integrated layout (x, y, w, h)
C_W, C_H = 1920, 1080
C_OVERVIEW = (16, 60, 1024, 576)
C_RVIZ = (1056, 60, 848, 576)
C_DRONE = (16, 668, 496, 372)
C_MAP = (528, 668, 512, 372)
C_STATUS_X = 1072


def quat_z(yaw: float):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class VizRecorder(Recorder):
    """Recorder + the Path / markers that the RViz pane shows."""

    def __init__(self, webapp, overview_topic, drone_topic, bags_file) -> None:
        super().__init__(webapp, overview_topic, drone_topic)
        self._path_pub = self.create_publisher(Path, '/mission_video/path', 10)
        self._marker_pub = self.create_publisher(MarkerArray, '/mission_video/markers', 10)
        self._path = Path()
        self._path.header.frame_id = 'map'
        self._last_xy: Optional[Tuple[float, float, float]] = None
        self._captures: List[Tuple[float, float, float, float, str]] = []  # e,n,yaw,agl,cell
        # the navigator logs 'NAVIGATE -> CAPTURE (photo requested at gridN)' when it
        # holds over a cell: mark the cell at the pose of that moment
        self.create_subscription(Log, '/rosout', self._on_log, 50)
        self._bags = []
        try:
            with open(bags_file) as f:
                self._bags = json.load(f).get('bags', [])
            self.get_logger().info(f'{len(self._bags)} ground objects from {bags_file}')
        except (OSError, ValueError) as exc:
            self.get_logger().warn(f'no ground objects ({exc})')
        self.create_timer(0.1, self._publish_viz_safe)

    def _publish_viz_safe(self) -> None:
        try:
            self._publish_viz()
        except Exception as exc:  # a drawing bug must not stop the recording
            self.get_logger().error(f'viz: {exc!r}', throttle_duration_sec=5.0)

    # pose in ENU: east, north, up, yaw (ENU, ccw from east), AGL
    def _pose(self):
        p = self.pos
        if p is None or not p.xy_valid:
            return None
        agl = p.dist_bottom if p.dist_bottom_valid else -p.z
        return p.y, p.x, -p.z, math.pi / 2.0 - p.heading, max(0.0, agl)

    def _on_log(self, msg: Log) -> None:
        if 'navigator' not in msg.name:
            return
        m = re.search(r'-> CAPTURE \(photo requested at (\w+)\)', msg.msg)
        pose = self._pose()
        if m and pose is not None:
            e, n, _, yaw, agl = pose
            self._captures.append((e, n, yaw, agl, m.group(1)))

    def _marker(self, ns, mid, mtype, rgba, scale=(1.0, 1.0, 1.0)) -> Marker:
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = (float(v) for v in scale)
        m.color = ColorRGBA(r=float(rgba[0]), g=float(rgba[1]), b=float(rgba[2]), a=float(rgba[3]))
        return m

    @staticmethod
    def _footprint(e, n, yaw, agl, z=0.03) -> List[Point]:
        """Ground rectangle seen by the downward camera (image top = heading)."""
        w = 2.0 * agl * math.tan(CAM_HFOV / 2.0)       # across (body right)
        h = w * CAM_ASPECT                               # along heading
        fx, fy = math.cos(yaw), math.sin(yaw)            # forward (ENU)
        rx, ry = math.sin(yaw), -math.cos(yaw)           # right
        pts = []
        for a, b in ((1, 1), (1, -1), (-1, -1), (-1, 1), (1, 1)):
            pts.append(Point(x=e + a * fx * h / 2 + b * rx * w / 2,
                             y=n + a * fy * h / 2 + b * ry * w / 2, z=z))
        return pts

    def _publish_viz(self) -> None:
        now = self.get_clock().now().to_msg()
        markers: List[Marker] = []
        pose = self._pose()
        if pose is not None:
            e, n, u, yaw, agl = pose
            if self._last_xy is None or math.dist(self._last_xy, (e, n, u)) > 0.05:
                ps = PoseStamped()
                ps.header.frame_id = 'map'
                ps.header.stamp = now
                ps.pose.position = Point(x=e, y=n, z=u)
                ps.pose.orientation.w = 1.0
                self._path.poses.append(ps)
                self._last_xy = (e, n, u)
            qx, qy, qz, qw = quat_z(yaw)
            body = self._marker('drone', 0, Marker.CUBE, (0.15, 0.15, 0.18, 1.0),
                                (0.7, 0.4, 0.18))
            body.pose.position = Point(x=e, y=n, z=u)
            body.pose.orientation.x, body.pose.orientation.y = qx, qy
            body.pose.orientation.z, body.pose.orientation.w = qz, qw
            markers.append(body)
            arms = self._marker('drone', 1, Marker.LINE_LIST, (0.3, 0.3, 0.3, 1.0),
                                (0.08, 0, 0))
            rotors = self._marker('drone', 2, Marker.SPHERE_LIST, (0.95, 0.35, 0.2, 0.9),
                                  (0.5, 0.5, 0.08))
            for k in range(4):
                a = yaw + math.pi / 4 + k * math.pi / 2
                p = Point(x=e + 0.6 * math.cos(a), y=n + 0.6 * math.sin(a), z=u + 0.06)
                arms.points += [Point(x=e, y=n, z=u), p]
                rotors.points.append(p)
                rotors.colors.append(ColorRGBA(r=0.95, g=0.35, b=0.2, a=0.9) if k in (0, 3)
                                     else ColorRGBA(r=0.2, g=0.6, b=0.95, a=0.9))
            markers += [arms, rotors]
            # heading arrow
            head = self._marker('drone', 3, Marker.ARROW, (1.0, 0.85, 0.1, 1.0),
                                (0.1, 0.2, 0.25))
            head.points = [Point(x=e, y=n, z=u),
                           Point(x=e + 1.2 * math.cos(yaw), y=n + 1.2 * math.sin(yaw), z=u)]
            markers.append(head)
            if agl > 0.3:
                fp = self._footprint(e, n, yaw, agl)
                ring = self._marker('camera', 0, Marker.LINE_STRIP, (0.2, 1.0, 0.4, 1.0),
                                    (0.06, 0, 0))
                ring.points = fp
                rays = self._marker('camera', 1, Marker.LINE_LIST, (0.2, 1.0, 0.4, 0.45),
                                    (0.025, 0, 0))
                for p in fp[:4]:
                    rays.points += [Point(x=e, y=n, z=u), p]
                markers += [ring, rays]
            else:
                for mid in (0, 1):
                    d = self._marker('camera', mid, Marker.LINE_STRIP, (0, 0, 0, 0))
                    d.action = Marker.DELETE
                    markers.append(d)
            label = self._marker('drone', 4, Marker.TEXT_VIEW_FACING, (1, 1, 1, 1),
                                 (0, 0, 0.45))
            label.pose.position = Point(x=e, y=n, z=u + 0.8)
            label.text = f'N {n:.1f}  E {e:.1f}  AGL {agl:.1f} m'
            markers.append(label)

        for i, (ce, cn, cyaw, cagl, cell_name) in enumerate(self._captures):
            cell = self._marker('captures', i, Marker.LINE_STRIP, (1.0, 0.8, 0.15, 0.9),
                                (0.05, 0, 0))
            cell.points = self._footprint(ce, cn, cyaw, cagl, z=0.02 + 0.005 * i)
            num = self._marker('capture_ids', i, Marker.TEXT_VIEW_FACING,
                               (1.0, 0.85, 0.2, 1.0), (0, 0, 0.7))
            num.pose.position = Point(x=ce, y=cn, z=0.4)
            num.text = f'#{i + 1} ({cell_name})'
            dot = self._marker('capture_pts', i, Marker.SPHERE, (1.0, 0.8, 0.15, 1.0),
                               (0.2, 0.2, 0.2))
            dot.pose.position = Point(x=ce, y=cn, z=0.1)
            markers += [cell, num, dot]

        for b in self._bags:
            m = self._marker('ground_objects', b['id'], Marker.SPHERE,
                             tuple(c / 255 for c in BAG_RGB.get(b['colour'], (200, 200, 200)))
                             + (1.0,),
                             (b['length_m'], b['width_m'], b['height_m']))
            m.pose.position = Point(x=b['east_m'], y=b['north_m'], z=b['height_m'] / 2)
            _, _, m.pose.orientation.z, m.pose.orientation.w = quat_z(-b['yaw_rad'])
            markers.append(m)

        ground = self._marker('ground', 0, Marker.CUBE, (0.22, 0.42, 0.18, 0.55),
                              (30.0, 30.0, 0.01))
        ground.pose.position = Point(x=0.0, y=5.0, z=-0.01)
        markers.append(ground)
        self._path.header.stamp = now
        self._path_pub.publish(self._path)
        self._marker_pub.publish(MarkerArray(markers=markers))


class X11Grabber:
    """Latest frame of an X display, read from ffmpeg x11grab as raw RGB."""

    def __init__(self, display: str, size: Tuple[int, int], fps: float) -> None:
        self.size = size
        self.frame: Optional[PilImage.Image] = None
        self._proc = subprocess.Popen(
            ['ffmpeg', '-loglevel', 'error', '-f', 'x11grab', '-draw_mouse', '0',
             '-framerate', str(fps), '-video_size', f'{size[0]}x{size[1]}', '-i', display,
             '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-'],
            stdout=subprocess.PIPE)
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self) -> None:
        n = self.size[0] * self.size[1] * 3
        while True:
            buf = self._proc.stdout.read(n)
            if len(buf) < n:
                return
            self.frame = PilImage.frombytes('RGB', self.size, buf).crop(RVIZ_CROP)

    def close(self) -> None:
        self._proc.terminate()


def status_lines(rec: VizRecorder):
    st, pos = rec.status, rec.pos
    mode = NAV.get(st.nav_state, f'nav_state {st.nav_state}') if st else '-'
    armed = bool(st and st.arming_state == VehicleStatus.ARMING_STATE_ARMED)
    posline = ''
    if pos is not None and pos.xy_valid:
        posline = (f'N {pos.x:6.2f} m   E {pos.y:6.2f} m   alt {-pos.z:5.2f} m'
                   + (f'   AGL {pos.dist_bottom:4.2f} m' if pos.dist_bottom_valid else ''))
    s = rec.session
    received, total = int(s.get('received', 0) or 0), int(s.get('total', 9) or 9)
    status = s.get('status', '-') if s else 'waiting for session'
    return mode, armed, posline, received, total, status


def panel(canvas, draw, box, img, title, font_t, font_w):
    x, y, w, h = box
    draw.rectangle([x - 2, y - 2, x + w + 1, y + h + 1], outline=(70, 70, 80))
    if img is not None:
        thumb = fit(img, box)
        canvas.paste(thumb, (x + (w - thumb.width) // 2, y + (h - thumb.height) // 2))
    else:
        draw.text((x + w // 2, y + h // 2), 'waiting...', font=font_w, fill=(120, 120, 120),
                  anchor='mm')
    tw = draw.textlength(title, font=font_t)
    draw.rectangle([x, y + h - 30, x + tw + 16, y + h], fill=(0, 0, 0))
    draw.text((x + 8, y + h - 25), title, font=font_t, fill=(255, 255, 255))


def compose_rviz(rec, rviz_img, t, fonts, speed) -> PilImage.Image:
    big, med, small = fonts
    canvas = PilImage.new('RGB', (RVIZ_W, RVIZ_H + RVIZ_HEADER), (18, 20, 24))
    draw = ImageDraw.Draw(canvas)
    if rviz_img is not None:
        canvas.paste(rviz_img, (0, RVIZ_HEADER))
    mode, armed, posline, received, total, status = status_lines(rec)
    draw.text((14, 8), 'RViz2  -  drone position (map frame, ENU)', font=med,
              fill=(235, 235, 235))
    draw.text((14, 32), f'PX4 {mode}  {"ARMED" if armed else "disarmed"}   {posline}',
              font=small, fill=(120, 230, 120) if armed else (200, 200, 200))
    draw.text((RVIZ_W - 14, 8), f't = {t:6.1f} s  ({speed:g}x)', font=med,
              fill=(160, 160, 160), anchor='ra')
    draw.text((RVIZ_W - 14, 32), f'frames {received}/{total}   session: {status}',
              font=small, fill=(240, 220, 120), anchor='ra')
    legend = ('red: flight path   green: camera footprint   yellow #n: captured cells   '
              'ellipsoids: ground objects (true positions)')
    draw.rectangle([0, RVIZ_HEADER + RVIZ_H - 26, RVIZ_W, RVIZ_HEADER + RVIZ_H], fill=(0, 0, 0))
    draw.text((10, RVIZ_HEADER + RVIZ_H - 21), legend, font=small, fill=(190, 190, 190))
    return canvas


def compose_combined(rec, rviz_img, t, fonts, speed, drone_topic) -> PilImage.Image:
    big, med, small = fonts
    canvas = PilImage.new('RGB', (C_W, C_H), (18, 20, 24))
    draw = ImageDraw.Draw(canvas)
    draw.text((16, 16), 'drone_control  mosaic mission  -  grass field  (PX4 SITL, Gazebo '
              'Harmonic, RViz2)', font=med, fill=(235, 235, 235))
    draw.text((C_W - 16, 16), f't = {t:6.1f} s   (playback {speed:g}x)', font=med,
              fill=(160, 160, 160), anchor='ra')
    overview, drone = rec.images()
    panel(canvas, draw, C_OVERVIEW, overview, 'overview camera (Gazebo, 3rd person)', small, med)
    panel(canvas, draw, C_RVIZ, rviz_img, 'RViz2: drone position, path, camera footprint',
          small, med)
    panel(canvas, draw, C_DRONE, drone, f'downward camera ({drone_topic})', small, med)
    if rec.map_img is not None:
        panel(canvas, draw, C_MAP, rec.map_img, 'stitched map (web app result)', small, med)
    else:
        x, y, w, h = C_MAP
        draw.rectangle([x - 2, y - 2, x + w + 1, y + h + 1], outline=(70, 70, 80))
        draw.text((x + w // 2, y + h // 2), 'stitched map: pending', font=med,
                  fill=(120, 120, 120), anchor='mm')
        draw.text((x + 8, y + h - 25), 'stitched map (web app result)', font=small,
                  fill=(255, 255, 255))

    mode, armed, posline, received, total, status = status_lines(rec)
    x0, y0 = C_STATUS_X, 676
    draw.text((x0, y0), f'PX4: {mode}   {"ARMED" if armed else "disarmed"}', font=big,
              fill=(120, 230, 120) if armed else (200, 200, 200))
    draw.text((x0, y0 + 44), posline, font=med, fill=(220, 220, 220))
    draw.text((x0, y0 + 86), f'frames {received}/{total}   session: {status}', font=med,
              fill=(240, 220, 120))
    bx, by, bw, bh = x0, y0 + 124, 820, 26
    draw.rectangle([bx, by, bx + bw, by + bh], outline=(120, 120, 120))
    if total:
        draw.rectangle([bx, by, bx + int(bw * received / total), by + bh], fill=(240, 200, 60))
    if status in ('scanning', 'queued'):
        draw.text((bx + bw // 2, by + bh // 2), 'MAP SCANNING', font=small, fill=(30, 30, 30),
                  anchor='mm')
    elif status == 'completed':
        draw.text((bx + bw // 2, by + bh // 2), 'MAP COMPLETED', font=small, fill=(30, 30, 30),
                  anchor='mm')
    for i, line in enumerate((
            'RViz2: red line = flight path, green = live camera footprint,',
            'yellow #n = captured cells, ellipsoids = ground objects (true pos.)',
            'state: /fmu/out/vehicle_status_v4, /fmu/out/vehicle_local_position_v1',
            'map: web app /api/sessions/latest')):
        draw.text((x0, y0 + 176 + i * 26), line, font=small, fill=(130, 130, 130))
    return canvas


def ffmpeg_writer(path, size, rate):
    return subprocess.Popen(
        ['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
         '-s', f'{size[0]}x{size[1]}', '-framerate', str(rate), '-i', '-',
         '-c:v', 'libx264', '-preset', 'veryfast', '-crf', '23', '-pix_fmt', 'yuv420p',
         '-movflags', '+faststart', path], stdin=subprocess.PIPE)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True, help='integrated video (1920x1080)')
    parser.add_argument('--rviz-output', required=True, help='RViz-only video (1280x828)')
    parser.add_argument('--poster', help='PNG of the last integrated frame')
    parser.add_argument('--rviz-poster', help='PNG of the last RViz frame')
    parser.add_argument('--max-seconds', type=float, default=480.0)
    parser.add_argument('--after-complete', type=float, default=45.0)
    parser.add_argument('--after-disarm', type=float, default=3.0)
    parser.add_argument('--webapp', default='http://127.0.0.1:8000')
    parser.add_argument('--overview-topic', default='/overview_camera/image')
    parser.add_argument('--drone-topic', default='/survey/camera/image')
    parser.add_argument('--bags', default=DEFAULT_BAGS)
    parser.add_argument('--rviz-config', default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'mission_video.rviz'))
    parser.add_argument('--xdisplay', default=':57', help='Xephyr display for RViz2')
    parser.add_argument('--fps', type=float, default=10.0)
    parser.add_argument('--speed', type=float, default=2.0)
    args = parser.parse_args(argv)
    os.environ.setdefault('GZ_IP', '127.0.0.1')

    children = []
    xephyr = subprocess.Popen(['Xephyr', args.xdisplay, '-screen', f'{XEPHYR_W}x{XEPHYR_H}',
                               '-ac', '-br', '-noreset', '-title', 'rviz2 capture'],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    children.append(xephyr)
    time.sleep(2.0)
    rviz = subprocess.Popen(['rviz2', '-d', os.path.abspath(args.rviz_config)],
                            env={**os.environ, 'DISPLAY': args.xdisplay},
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    children.append(rviz)

    fonts = (ImageFont.truetype(FONT, 28), ImageFont.truetype(FONT, 21),
             ImageFont.truetype(FONT, 16))
    rate = args.fps * args.speed
    ff_c = ffmpeg_writer(args.output, (C_W, C_H), rate)
    ff_r = ffmpeg_writer(args.rviz_output, (RVIZ_W, RVIZ_H + RVIZ_HEADER), rate)
    rclpy.init()
    rec = VizRecorder(args.webapp, args.overview_topic, args.drone_topic, args.bags)
    time.sleep(6.0)                       # RViz window up before grabbing
    grab = X11Grabber(args.xdisplay, (XEPHYR_W, XEPHYR_H), args.fps * 2)
    stop = {'flag': False}
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__('flag', True))
    start = time.monotonic()
    completed_at: Optional[float] = None
    disarmed_at: Optional[float] = None
    frames = 0
    last_c = last_r = None
    try:
        while not stop['flag']:
            rclpy.spin_once(rec, timeout_sec=0.02)
            rec.poll_session()
            now = time.monotonic()
            if now - start >= frames / args.fps:
                rv = grab.frame
                last_c = compose_combined(rec, rv, now - start, fonts, args.speed,
                                          args.drone_topic)
                last_r = compose_rviz(rec, rv, now - start, fonts, args.speed)
                ff_c.stdin.write(last_c.tobytes())
                ff_r.stdin.write(last_r.tobytes())
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
        for ff in (ff_c, ff_r):
            try:
                ff.stdin.close()
            except Exception:
                pass
        for ff in (ff_c, ff_r):
            ff.wait(timeout=120)
        if args.poster and last_c is not None:
            last_c.save(args.poster)
        if args.rviz_poster and last_r is not None:
            last_r.save(args.rviz_poster)
        grab.close()
        for p in reversed(children):
            p.terminate()
        rec.close()
        rec.destroy_node()
        rclpy.shutdown()
    print(f'{args.output} + {args.rviz_output}: {frames} frames, {frames / args.fps:.0f} s '
          f'real time, {frames / rate:.0f} s video, session {rec.session.get("status")}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
