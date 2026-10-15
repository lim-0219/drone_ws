# drone_control

PX4 offboard waypoint navigation and a minimal **survey-and-mark** mission for
ROS 2 Humble + PX4 SITL (Gazebo Harmonic, x500).

One node, `waypoint_navigator`, does both jobs:

* **Waypoint mission** - fly a YAML list of waypoints, then hover or land.
* **Survey mission** - fly to one target, hover, take a photo with a downward
  camera, then hold position while drawing three laser beams and the
  equilateral triangle they would project on the ground as RViz markers.

---

## Coordinate conventions

PX4 works in a **local NED** frame, which is the single most common source of
sign errors, so this package is explicit about it:

| Axis | Direction | Notes |
|------|-----------|-------|
| `x`  | **North** (metres) | positive = north of the origin |
| `y`  | **East** (metres)  | positive = east of the origin |
| `z`  | **Down** (metres)  | **negative = above the origin** |

The origin is the EKF local origin: in SITL, the spot where the vehicle sat when
the estimator initialised - i.e. the takeoff pad.

### Altitude is positive-up in the YAML file and in `target_altitude`

Writing `z: -5.0` for "5 metres up" is easy to get wrong, so the YAML file takes
a **positive `altitude`** (and the survey mission a positive `target_altitude`)
and the node flips the sign exactly once:

```
z_ned = -altitude
```

For YAML input that is `_parse_waypoint()`, for the survey target it is
`_read_parameters()`, both in
[`drone_control/waypoint_navigator.py`](drone_control/waypoint_navigator.py).
`altitude: 5.0` becomes `position[2] = -5.0` in the `TrajectorySetpoint`.

A `z:` key is accepted as an alias in the YAML, but **it is still read as a
positive-up altitude**, not as a raw NED `z`. Prefer `altitude:`.

### Yaw

Degrees by default (`yaw_in_degrees:=false` for radians), NED convention:
North `0`, East `90`, South `180`, West `-90` (or `270`, wrapped). Converted to
radians and wrapped into `[-pi, pi]` for PX4.

### RViz is ENU

Markers and the `map -> base_link` transform are published in ENU
(x = East, y = North, z = Up), the ROS convention:
`(E, N, U) = (y_ned, x_ned, -z_ned)`. See `laser_triangle.ned_to_enu()`.

---

## PX4 topic names

Verified against the **live bridge** with `ros2 topic list | grep fmu`, which is
the only reliable source (see the note below):

| Direction | Topic | Message |
|-----------|-------|---------|
| publish   | `/fmu/in/offboard_control_mode` | `OffboardControlMode` |
| publish   | `/fmu/in/trajectory_setpoint`   | `TrajectorySetpoint` |
| publish   | `/fmu/in/vehicle_command`       | `VehicleCommand` |
| publish   | `/fmu/in/distance_sensor`       | `DistanceSensor` (survey range finder) |
| subscribe | `/fmu/out/vehicle_local_position_v1` | `VehicleLocalPosition` |
| subscribe | `/fmu/out/vehicle_status_v4`    | `VehicleStatus` |

> **On message-version suffixes.** PX4 main appends `_v<N>` to the DDS topic
> name of every message that declares a non-zero `MESSAGE_VERSION`, and leaves
> the name bare otherwise:
>
> | Message | `MESSAGE_VERSION` | Topic |
> |---------|-------------------|-------|
> | `VehicleStatus` | 4 | `vehicle_status_v4` |
> | `VehicleLocalPosition` | 1 | `vehicle_local_position_v1` |
> | `TrajectorySetpoint` | 0 | `trajectory_setpoint` |
> | `VehicleCommand` | 0 | `vehicle_command` |
> | `OffboardControlMode` | *(none)* | `offboard_control_mode` |
> | `DistanceSensor` | *(none)* | `distance_sensor` |
>
> **The suffix is added at runtime** inside `create_data_writer()` /
> `create_data_reader()`; `dds_topics.h.em` passes the bare name and the version
> separately. So the suffix does **not** appear in the generated `dds_topics.h`
> nor in `strings build/px4_sitl_default/bin/px4` - those static artifacts will
> mislead you. Check a running bridge. A PX4 update that bumps a version renames
> the topic and the subscription goes silent with no error: re-run
> `ros2 topic list | grep fmu` after upgrading and update the `_TOPIC_*`
> constants at the top of `waypoint_navigator.py`.
>
> `px4_ros_com`'s bundled `offboard_control.py` subscribes to the unsuffixed
> `/fmu/out/vehicle_status`, so on this PX4 revision its status subscription
> never fires.

### QoS

All `/fmu` publishers and subscribers use the profile the working `px4_ros_com`
example uses: `BEST_EFFORT`, `TRANSIENT_LOCAL`, `KEEP_LAST` depth 1. The
Gazebo-bridged sensor topics are subscribed with `qos_profile_sensor_data`
(best effort) for the small range scan, and a RELIABLE profile for the camera
image - see "Large camera images over DDS".

---

## Waypoint file format (waypoint mission)

```yaml
waypoints:
  - {name: 'above_home', x:  0.0, y:  0.0, altitude: 5.0, yaw:    0.0}
  - {name: 'north_east', x: 10.0, y:  0.0, altitude: 5.0, yaw:    0.0}
  - {name: 'corner_ne',  x: 10.0, y: 10.0, altitude: 5.0, yaw:   90.0}
  - {name: 'corner_se',  x:  0.0, y: 10.0, altitude: 5.0, yaw:  180.0}
  - {name: 'back_home',  x:  0.0, y:  0.0, altitude: 5.0, yaw:  -90.0}
```

`x`, `y`, `altitude` required; `yaw` defaults to `0`; `name` is a log label. The
shipped `config/square_waypoints.yaml` is a 10 m square at 5 m, installed to
`install/drone_control/share/drone_control/config/`.

---

## State machine

```
IDLE ──► TAKEOFF ──► NAVIGATE ──┬──► HOLD                         (final_action: hover)
                                ├──► LANDING ──► LANDED           (final_action: land)
                                └──► CAPTURE ──► LASER_HOLD       (final_action: survey)
```

| State | What it does | Exit condition |
|-------|--------------|----------------|
| `IDLE` | Streams `offboard_control_mode` + a hold-station setpoint, then requests OFFBOARD and ARM (in that order), retrying ~1 Hz | `nav_state == OFFBOARD` **and** `arming_state == ARMED` |
| `TAKEOFF` | Climbs **vertically**: keeps the current x/y, targets the first waypoint's altitude and yaw | target held within `position_tolerance` for `reach_hold_time` |
| `NAVIGATE` | Commands the current waypoint; advances on arrival | last waypoint reached |
| `HOLD` | Hovers at the last waypoint | terminal |
| `CAPTURE` | Hovers, waits `capture_settle_time`, saves one camera frame (PNG + JSON) | photo saved, or `capture_timeout` elapsed (warning) |
| `LASER_HOLD` | Hovers and publishes the laser beam / ground triangle markers | terminal |
| `LANDING` | Sends `VEHICLE_CMD_NAV_LAND` once and **stops** the offboard heartbeat so PX4 keeps AUTO_LAND | `arming_state == DISARMED` |
| `LANDED` | Terminal | - |

Every transition is logged as `STATE A -> B (reason)`. A waypoint counts as
reached when the 3D distance stays within `position_tolerance` continuously for
`reach_hold_time` seconds; leaving the sphere resets the timer.

---

## Parameters

| Parameter | Type | Default | Meaning |
|-----------|------|---------|---------|
| `waypoint_file` | string | `''` | Waypoint YAML (required unless the survey target is given) |
| `px4_namespace` | string | `''` | PX4 topic prefix, e.g. `/px4_1` |
| `timer_period` | double | `0.1` | Setpoint period, seconds (0.1 = 10 Hz) |
| `position_tolerance` | double | `0.3` | Arrival radius, metres |
| `reach_hold_time` | double | `1.0` | Dwell inside the tolerance before "reached" |
| `final_action` | string | `hover` | `hover`, `land` or `survey` |
| `setpoint_warmup_count` | int | `20` | Setpoints streamed before OFFBOARD + ARM |
| `yaw_in_degrees` | bool | `true` | Units of YAML `yaw` and `target_yaw` |
| **Survey target** | | | *used when `final_action=survey` and `target_altitude > 0`; otherwise the survey happens at the last YAML waypoint* |
| `target_x`, `target_y` | double | `0.0` | Target North / East, metres (NED) |
| `target_altitude` | double | `0.0` | Target altitude above the origin, metres, **positive-up** |
| `target_yaw` | double | `0.0` | Heading at the target |
| **Sensors** | | | |
| `camera_topic` | string | `/survey/camera/image` | Bridged `sensor_msgs/Image` |
| `range_topic` | string | `/survey/range/scan` | Bridged `sensor_msgs/LaserScan` (1 beam) |
| `range_sensor_down_offset` | double | `0.05` | Sensor height below the body origin, m (= SDF mount = `EKF2_RNG_POS_Z`) |
| **Photo** | | | |
| `photo_dir` | string | `~/drone_survey_photos` | Output directory (created) |
| `capture_settle_time` | double | `2.0` | Hover seconds before the photo |
| `capture_timeout` | double | `30.0` | Give up waiting for an image after this |
| **Laser triangle** | | | |
| `laser_half_angle_deg` | double | `15.0` | Tilt of each beam from straight down |
| `laser_mount_radius` | double | `0.08` | Emitter distance from the body origin, m (= SDF) |
| `laser_mount_down` | double | `0.03` | Emitter height below the body origin, m (= SDF) |
| `marker_topic` | string | `/survey/laser_markers` | `visualization_msgs/MarkerArray` |
| `marker_frame` | string | `map` | Fixed ENU frame for markers and TF |
| `body_frame` | string | `base_link` | Child frame of the broadcast transform |
| `publish_tf` | bool | `true` | Broadcast `map -> base_link` |

All of them are launch arguments of `waypoint_nav.launch.py` (waypoint mission)
or `survey_mission.launch.py` (survey mission).

---

## Survey mission

### 1. Gazebo model `x500_survey`

`~/PX4-Autopilot/Tools/simulation/gz/models/x500_survey/` includes the stock
`x500` (= `x500_base` + motor plugins) and adds:

| Link | Sensor / role | Pose (body, SDF frame: x fwd, y left, z up) |
|------|---------------|---------------------------------------------|
| `survey_camera_link` | `camera`, 1280x960 RGB, 5 Hz, topic `/survey/camera/image` | 6 cm below origin, pitched +90 deg (looks down) |
| `survey_range_link` | `gpu_lidar` 1x1 beam, 0.1-60 m, 30 Hz, topic `/survey/range/scan` | 5 cm below origin, pitched +90 deg |
| `laser_emitter_{0,1,2}_link` | geometry only, red stub visual | radius 8 cm, 3 cm below origin, azimuth k*120 deg, beam (+x) tilted 15 deg from nadir |

Sensor topics are pinned with `<topic>` so nothing depends on the world name or
on the instance name PX4 gives the spawned model (`x500_survey_0`).

The range link is deliberately **not** called `lidar_sensor_link`: that is the
name PX4's built-in Gazebo bridge subscribes to (`GZBridge.cpp`,
`laserScantoLidarSensorCallback`), which would inject a second
`distance_sensor` source. The airframe also sets `SIM_GZ_EN_LIDAR 0`.

Validated with the Harmonic SDF parser
(`GZ_SIM_RESOURCE_PATH=$PX4_GZ_MODELS gz sdf -k model.sdf` -> `Valid.`).

### 2. Airframe + make target

`~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_x500_survey`
(registered in that directory's `CMakeLists.txt`) sources the stock
`4001_gz_x500` and sets the range-finder parameters below. PX4 generates one
make target per `*_gz_*` airframe, so after `make px4_sitl` (already run once)
you get:

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500_survey          # default world; also gz_x500_survey_forest etc.
```

If you pull a new PX4 revision, re-run `make px4_sitl` so the ROMFS and the
target are regenerated.

### 3. ros_gz bridge: **the apt package is Fortress, Gazebo is Harmonic**

Finding: `ros-humble-ros-gz-bridge` 0.244.26 from packages.ros.org links
against `libignition-transport11` / `libignition-msgs8` (Gazebo **Fortress**,
gz-sim 6), while the installed simulator is **Harmonic** 8.15
(`libgz-transport13` / `libgz-msgs10`). Transport 11 and 13 are not
wire-compatible, so that bridge never sees a single Harmonic topic.

What was changed: the `humble` branch of
[gazebosim/ros_gz](https://github.com/gazebosim/ros_gz) is vendored at
`src/ros_gz` and built with `GZ_VERSION=harmonic` (only `ros_gz_interfaces` and
`ros_gz_bridge`; everything else has a `COLCON_IGNORE`). Its
`ros_gz_bridge/CMakeLists.txt` carries a small local patch that defaults
`GZ_VERSION` to `harmonic` when unset, so a plain `colcon build` can never
silently reconfigure against the Fortress libraries that are also installed.
The overlay in `install/` shadows the apt package once `install/setup.bash` is
sourced. Verify with:

```bash
readelf -d ~/drone_ws/install/ros_gz_bridge/lib/libros_gz_bridge.so | grep -E 'gz-|ignition'
# expect libgz-transport13 / libgz-msgs10, no ignition-*
```

Alternative without a source build (needs sudo, replaces `ros-humble-ros-gz*`):
`sudo apt install ros-humble-ros-gzharmonic` from the OSRF repository that is
already configured on this machine (`/etc/apt/sources.list.d/gazebo-stable.list`).

### 4. Range finder -> PX4 -> EKF2

```
Gazebo gpu_lidar (1 beam) --ros_gz--> /survey/range/scan (LaserScan)
   --waypoint_navigator--> /fmu/in/distance_sensor (DistanceSensor, ROTATION_DOWNWARD_FACING)
   --uXRCE-DDS--> uORB distance_sensor --EKF2--> vehicle_local_position.dist_bottom
```

EKF2 selects the first `distance_sensor` instance whose
`orientation == ROTATION_DOWNWARD_FACING` (`EKF2.cpp`, `UpdateRangeSample`);
`device_id` is not required. `DistanceSensor` has no `MESSAGE_VERSION`, so the
topic is unsuffixed.

**PX4 parameters** (definitions read from `src/modules/ekf2/*.yaml`):

| Parameter | Values | Default | Set to | Why |
|-----------|--------|---------|--------|-----|
| `EKF2_RNG_CTRL` | 0 off, 1 conditional, 2 always | 1 | **2** | Conditional mode only fuses below `EKF2_RNG_A_HMAX` (5 m) and slow; the survey hover may be higher |
| `EKF2_HGT_REF` | 0 baro, 1 GPS, 2 range, 3 vision | 1 | **2** | Height estimate converges to height-above-ground. Flat terrain only. **Reboot required** |
| `EKF2_RNG_POS_Z` | metres, body z (down) | 0 | **0.05** | Sensor is 5 cm below the body origin in the SDF |

Two ways to set them:

1. **Boot defaults** - the `4022_gz_x500_survey` airframe (`param set-default`).
   Nothing to do when you launch `gz_x500_survey`.
2. **Runtime** - `ros2 run drone_control rangefinder_params` (`--read` to only
   inspect). It talks MAVLink to PX4's onboard link on UDP 14540, so it works
   while QGroundControl owns 14550. `EKF2_HGT_REF` still needs a PX4 restart.

**Verify that `dist_bottom` becomes valid** (with SITL, the agent and the
survey launch running):

```bash
ros2 run drone_control check_range_fusion --seconds 20
```

It prints scan rate -> `distance_sensor` rate -> `dist_bottom`,
`dist_bottom_valid` and whether the `DIST_BOTTOM_SENSOR_RANGE` bit is set, once
per second, and exits 0 only if `dist_bottom_valid` became true *from the range
sensor*. Expected: ~30 scans/s, ~30 `distance_sensor`/s, `valid=True
range_bit=True`. On the pad the raw scan reads ~0.19 m (the model sits 0.24 m up,
the sensor 5 cm below the origin) and `dist_bottom` ~0.24 m (EKF2 adds the
`EKF2_RNG_POS_Z` offset back); in the hover `dist_bottom` ~= the target altitude.

### 5. Photo capture

On `CAPTURE` the node hovers `capture_settle_time`, then saves the first camera
frame that arrived *after* settling (so the photo shows the vehicle at the
target, however slowly the camera renders) to `photo_dir`:

```
survey_20260920T142305Z_N+5.00_E+0.00_alt5.00.png
survey_20260920T142305Z_N+5.00_E+0.00_alt5.00.json
```

The JSON sidecar holds the UTC time, image stamp / frame / encoding / size,
the NED pose (`x_north_m`, `y_east_m`, `z_down_m`, `altitude_above_origin_m`,
heading), the height above ground and its source (`dist_bottom` or raw range),
and the target. PNG writing uses PIL directly from the `sensor_msgs/Image`
bytes (encodings `rgb8`, `bgr8`, `rgba8`, `bgra8`, `mono8`, row stride
honoured) - see the note on OpenCV below.

### 6. Laser triangle

Derivation and frames are in
[`drone_control/laser_triangle.py`](drone_control/laser_triangle.py). Summary:
with body height above ground `h`, beam half-angle `theta`, emitter mount
radius `r_e` and mount depth `z_e`, every beam meets the ground at horizontal
distance

```
R = r_e + (h - z_e) * tan(theta)
```

from the point under the vehicle, at azimuth `phi_k + yaw` (`phi_k` = 0, -120,
+120 deg in FRD, matching the SDF's 0/120/240 deg in its y-left frame). Equal
tilt and 120 deg spacing give an equilateral triangle of side `sqrt(3) * R`
centred under the vehicle. Assumptions: level hover (roll = pitch = 0; only yaw
is applied) and locally flat ground.

`h` is taken from the **range finder** - `dist_bottom` when valid, else the
raw range reading plus `range_sensor_down_offset` - and never from barometric
or GPS altitude: those are relative to arming pressure or an ellipsoid and know
nothing about where the ground is, whereas the range finder measures exactly the
distance the formula needs.

Markers (`/survey/laser_markers`, frame `map`, ENU, 1 s lifetime): `LINE_LIST`
beams emitter -> ground, `SPHERE_LIST` ground spots, closed `LINE_STRIP`
triangle outline, and a text label with the AGL, its source and the side
length.

### 7. Launch

`survey_mission.launch.py` starts three processes:

| Process | Purpose | Skip with |
|---------|---------|-----------|
| `ros_gz_bridge/parameter_bridge` | `/survey/camera/image` (Image) and `/survey/range/scan` (LaserScan), Gazebo -> ROS. Runs with `GZ_IP=127.0.0.1` (`gz_ip` arg) because PX4 starts Gazebo that way | `bridge:=false` |
| `drone_control/waypoint_navigator` | the mission, `final_action=survey` | - |
| `rviz2 -d config/survey.rviz` | grid, TF, laser markers, camera image, fixed frame `map` | `rviz:=false` |

---

## Build

```bash
cd ~/drone_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select drone_control      # the package only
colcon build                                       # everything, incl. ros_gz_bridge (Harmonic)
source install/setup.bash
```

Re-run after every edit - the build copies files into `install/`.

> **Do not add `--symlink-install` to this workspace.** `px4_msgs` and
> `px4_ros_com` were built without it and CMake caches the choice; switching
> makes `px4_msgs` fail with `failed to create symbolic link ... existing path
> cannot be removed`. Recover with
> `colcon build --packages-select px4_msgs --cmake-args -DAMENT_CMAKE_SYMLINK_INSTALL=0`.

---

## Running the survey mission: terminal order

Every terminal needs the same DDS settings - a mismatch is silent, topics just
never appear:

```bash
export ROS_DOMAIN_ID=7
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# ROS shells (terminals 4 and 5) additionally - see "Large camera images over DDS":
export CYCLONEDDS_URI=file://$HOME/drone_ws/install/drone_control/share/drone_control/config/cyclonedds.xml
```

### Terminal 1 - uXRCE-DDS agent (start before PX4)

```bash
MicroXRCEAgent udp4 -p 8888
```

### Terminal 2 - PX4 SITL with the survey model

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500_survey
```

Wait for `INFO [commander] Ready for takeoff!`. The `4022` airframe already
applies `EKF2_RNG_CTRL 2`, `EKF2_HGT_REF 2`, `EKF2_RNG_POS_Z 0.05`,
`SIM_GZ_EN_LIDAR 0`.

### Terminal 3 - QGroundControl

```bash
~/QGroundControl.AppImage
```

Required: the stock `4001_gz_x500` airframe sets `NAV_DLL_ACT 2`, which makes
"No connection to the GCS" a hard arming failure. Any MAVLink GCS on UDP 14550
clears it; keep QGC open. (Headless alternative: `param set NAV_DLL_ACT 0` in
the PX4 console.)

### Terminal 4 - the mission

```bash
source /opt/ros/humble/setup.bash
source ~/drone_ws/install/setup.bash

ros2 topic list | grep -E 'fmu/out/vehicle_status|fmu/in/distance_sensor'   # bridge alive?

ros2 launch drone_control survey_mission.launch.py \
    target_x:=5.0 target_y:=0.0 target_altitude:=5.0
```

Expected log:

```
[INFO]: Loaded 1 waypoint(s) from target_x/target_y/target_altitude parameters
[INFO]:   - survey_target (N=5.00 E=0.00 alt=5.00m yaw=0.0deg)
[INFO]: survey: camera=/survey/camera/image range=/survey/range/scan photos -> /home/.../drone_survey_photos ...
[INFO]: Range finder online: 0.19 m, forwarding to /fmu/in/distance_sensor
[INFO]: Streamed 20 setpoints; requesting OFFBOARD mode and ARM
[INFO]: PX4 mode -> OFFBOARD
[INFO]: PX4 arming_state -> ARMED
[INFO]: STATE IDLE -> TAKEOFF (offboard engaged and armed)
[INFO]: STATE TAKEOFF -> NAVIGATE (reached takeoff altitude 5.00m)
[INFO]: Reached waypoint 1/1: survey_target (...)
[INFO]: STATE NAVIGATE -> CAPTURE (last waypoint reached, final_action=survey)
[INFO]: Photo saved: /home/.../survey_..._N+5.00_E+0.00_alt5.00.png
[INFO]: STATE CAPTURE -> LASER_HOLD (photo captured)
[INFO]: Laser hold: AGL 5.00 m (dist_bottom), triangle side 2.45 m
```

### Terminal 5 (optional) - check range fusion

```bash
ros2 run drone_control check_range_fusion --seconds 20
```

### Waypoint mission instead

Terminals 1-3 as above but `make px4_sitl gz_x500` (or the survey model, it
flies the same), then:

```bash
ros2 launch drone_control waypoint_nav.launch.py            # square path, hover
ros2 launch drone_control waypoint_nav.launch.py final_action:=land
```

---

## Troubleshooting: `Waiting for a camera frame ...`

The flight works but `CAPTURE` keeps warning. The warning now says which case
you are in; `ros2 run drone_control check_range_fusion` shows the same numbers
(`cam N/s`, `scan N/s`).

| Log says | Cause | Fix |
|----------|-------|-----|
| `Camera online: first image ...` then `1 images received, last one 30.3s ago` (and `scan 30/s`) | **Observed on this machine.** Frames are lost between the bridge and the node: a 1280x960 rgb8 frame is 3.7 MB = ~2,600 UDP fragments per burst, and `net.core.rmem_max` is the Linux default 212992 bytes. CycloneDDS confirms it: `failed to increase socket receive buffer size to at least 10485760 bytes, current is 425984 bytes`. | See *Large camera images over DDS* below. |
| `no image has ever arrived on /survey/camera/image` and `scan 0/s` | The bridge cannot discover Gazebo. PX4's make target starts Gazebo with `GZ_IP=127.0.0.1`; gz-transport only discovers peers on the same interface, so a bridge without `GZ_IP` (it picks the LAN interface) never sees it. Same symptom if `GZ_PARTITION` differs between the two shells. | The launch file now sets `GZ_IP=127.0.0.1` for the bridge (`gz_ip:=` to change). Check from the mission shell: `GZ_IP=127.0.0.1 gz topic -l \| grep survey` must list both topics. |
| `no image has ever arrived` but `scan 30/s` | Discovery is fine; the camera sensor itself produces nothing (the Gazebo sensors system could not create the render engine for it). | Look at the Gazebo terminal for `ogre2` / render errors; try `gz topic -e -t /survey/camera/image -n 1` (with `GZ_IP=127.0.0.1`). |
| `N images received, last one X s ago, ~0.3 Hz` | Discovery works but the 1280x960 camera renders slowly (software GL). The node accepts the first frame taken after settling, so it will succeed - just later; it only gives up after `capture_timeout`. | Lower the resolution in `x500_survey/model.sdf` (`<width>/<height>`), or raise `capture_timeout`. |

Intermittent success ("sometimes it works") is the slow-camera or the
discovery-race case: whichever process advertises first wins the multicast
handshake. Verify the render/discovery state with the commands above rather
than by retrying.

---

## Large camera images over DDS

Three layers, from the proper fix to the mitigations that are already in place:

**1. Kernel receive buffer (needs sudo, once).** This is the fix the ROS 2
"DDS tuning" guide gives for exactly this symptom:

```bash
sudo sysctl -w net.core.rmem_max=2147483647
sudo sysctl -w net.core.rmem_default=2147483647
# make it survive a reboot
printf 'net.core.rmem_max=2147483647\nnet.core.rmem_default=2147483647\n' \
  | sudo tee /etc/sysctl.d/60-ros2-dds.conf
```

**2. Let CycloneDDS use it.** The package ships
`config/cyclonedds.xml` (installed to
`install/drone_control/share/drone_control/config/`) which asks for up to a
10 MB socket buffer. Export it in every ROS shell (terminals 4 and 5):

```bash
export CYCLONEDDS_URI=file://$HOME/drone_ws/install/drone_control/share/drone_control/config/cyclonedds.xml
```

It uses `max=`, so it is safe to set before the sysctl change (CycloneDDS
then just keeps the kernel's limit). Do not change it to `min=`: CycloneDDS
0.10 treats an unsatisfiable minimum as fatal and every node fails with
`rmw_create_node: failed to create domain`.

**3. Already applied in the package (sufficient on its own).** The camera
subscription in the node and the RViz Image display are **RELIABLE** (the
bridge publishes reliable), so lost fragments are resent instead of the frame
being dropped; and the camera renders at **5 Hz** (`update_rate` in the SDF)
instead of 15.

Measured on this machine with the kernel default `rmem_max=212992`, a
reliable publisher sending 1280x960 rgb8 at 5 Hz (what the bridge does):

| Subscriber QoS | Frames received in 12 s |
|----------------|-------------------------|
| best effort (old navigator) | **1** - the "one image, then nothing" symptom |
| reliable (current navigator) | **60** - the full 5 Hz |

So steps 1-2 are optional headroom (they also help best-effort consumers
such as `ros2 topic echo`); step 3 alone makes the capture work.

Verify:

```bash
ros2 topic hz /survey/camera/image            # reliable, like the node
ros2 run drone_control check_range_fusion --seconds 10   # "cam N/s" column
```

---

## Multi-vehicle later

PX4 topic names are stored as **relative** strings and prefixed at runtime by
`px4_namespace`, so a second vehicle needs no code change:

```bash
ros2 launch drone_control waypoint_nav.launch.py \
    px4_namespace:=/px4_1 node_namespace:=/drone1 waypoint_file:=/abs/path/drone1.yaml
```

Multi-vehicle *coordination* is deliberately not implemented; this only keeps
the plumbing ready.

---

## Environment notes

* **OpenCV / numpy.** `python3-opencv` (apt, built against numpy 1.x) fails to
  import next to the numpy 2.2 installed in `~/.local`
  (`numpy.core.multiarray failed to import`), and `cv_bridge` drags it in. The
  node therefore never imports numpy or cv2; images are written with PIL. If you
  need cv2 elsewhere: `pip install "numpy<2"`.
* **QGC is required for arming** with the stock x500 airframes (`NAV_DLL_ACT 2`).
* **Verified offline vs. live.** Everything that does not need a running
  simulator was verified here (SDF parses with the Harmonic parser, PX4 target
  generated, bridge linked against Harmonic and accepts the topic specs,
  state machine / photo / markers exercised with fake telemetry, linters
  clean). `dist_bottom` validity and the live image are verified with
  `check_range_fusion` and RViz once SITL runs the survey model.

## Safety notes

- The node arms and flies as soon as PX4 accepts OFFBOARD. SITL first.
- If PX4 leaves offboard (failsafe, RC override) the node warns and keeps
  streaming setpoints so offboard can be re-engaged; it does **not** re-arm.
- Waypoints and the survey target are absolute local-NED positions relative to
  the EKF origin, not relative to the vehicle.
