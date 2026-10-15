# One photo taken by the drone, bundled with everything the ground station
# needs to place it in a drone_stitch manifest. Sent drone -> ground PC as
# soon as the photo is taken ("send each picture right away").
#
# Frames: ned_m is PX4's local NED (x North, y East, z Down, metres) in the
# EKF session identified by session_id; yaw_rad is the heading in that frame
# (clockwise from North). grid_local_xy_m is the commanded 3x3 grid cell
# position relative to grid cell 5 (X = image right = body right, Y = image
# top = body forward), in metres - the same local frame drone_stitch uses.

std_msgs/Header header        # stamp: capture time; frame_id: session_id

string session_id             # one mission run; doubles as the stitcher's ned_frame_id
uint8 sequence                # 1-based capture index (1..total)
uint8 grid_id                 # 3x3 cell id; capture order is 5,4,1,2,3,6,9,8,7
uint8 total                   # frames in this session (9 for the mosaic)

float64 timestamp_s           # capture time, seconds, same clock for every frame
float64[3] ned_m              # vehicle position at capture (N, E, D)
float64 yaw_rad               # vehicle heading at capture
float64 agl_m                 # height above ground (positive); 0 when agl_valid is false
bool agl_valid
string agl_source             # "dist_bottom", "raw range" or "" when unknown

float64[2] grid_local_xy_m    # commanded cell position (X, Y) relative to cell 5
float64[2] grid_spacing_m     # grid step used for this session (X, Y); 0,0 if n/a

sensor_msgs/Image image       # the photo (rgb8 from the Gazebo camera)
