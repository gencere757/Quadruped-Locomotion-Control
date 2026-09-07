"""
Script: champgait_wave.py
Author: Arda Gencer
Wave (4-beat) gait controller, a variant of champgait.py's approach.
Detailed tuning-history notes for the constants below are in tuning_history/champgait_wave_history.txt
"""

import gz.transport13 as transport
from gz.msgs10.double_pb2 import Double
from gz.msgs10.imu_pb2 import IMU
from gz.msgs10.pose_v_pb2 import Pose_V
import math
import threading
import time
import sys
import os
import shutil
import datetime

_LOG_NAME = "run_log_wave.txt"   # filename for this run's log
_ARCHIVE_DIR = "run_log_archive"   # folder where old logs get copied
try:
    os.makedirs(_ARCHIVE_DIR, exist_ok=True)
    if os.path.exists(_LOG_NAME):   # true if a log file from a previous run is still there
        _ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")   # timestamp for archiving the old log
        shutil.copy2(_LOG_NAME, os.path.join(_ARCHIVE_DIR, f"{_ts}_{_LOG_NAME}"))   # copies the old log into the archive folder
except OSError:
    pass

_log_file = open(_LOG_NAME, "w")   # file handle for this run's log

class _Tee:
    def __init__(self, *streams):   # streams: the output streams to duplicate writes to
        self.streams = streams   # stores the streams to write to
    def write(self, data):   # data: text being written
        for s in self.streams:   # loop over each output stream
            s.write(data)
            s.flush()
    def flush(self):
        for s in self.streams:   # loop over each output stream
            s.flush()

sys.stdout = _Tee(sys.stdout, _log_file)   # now stdout prints go to both console and log file

def log_line(text):   # text: line to write to the log
    print(text, file=_log_file)
    _log_file.flush()


L1 = 0.2   # length of the leg's upper segment, in meters
L2 = 0.2   # length of the leg's lower segment, in meters

def leg_ik(fx, fz, s):
    """fx,fz = desired foot position relative to hip, in body frame (fz negative = below hip).
       s = +1 for front legs, -1 for back legs. Returns (hip, knee) angles."""
    u = s * fx   # foot x offset, flipped for leg side
    w = fz   # foot z offset (just renamed for the math below)
    r2 = u*u + w*w   # squared distance from hip to foot
    r2 = max(r2, 1e-9)   # clamp away from zero to avoid divide by zero
    c = (r2 - L1*L1 - L2*L2) / (2*L1*L2)   # law-of-cosines term for knee angle
    c = max(-1.0, min(1.0, c))   # clamp into valid range for acos
    knee = -math.acos(c)   # knee joint angle
    k1 = L1 + L2*math.cos(knee)   # helper term for hip angle calc
    k2 = L2*math.sin(knee)   # helper term for hip angle calc
    sin_a = (u*k1 + k2*w) / r2   # sine component of hip angle
    cos_a = (k2*u - k1*w) / r2   # cosine component of hip angle
    hip = math.atan2(sin_a, cos_a)   # hip joint angle
    return hip, knee

def rotate(fx, fz, theta):
    """Rotate a foot-target vector by theta (same convention as the leg's own swing rotation)."""
    return (fx*math.cos(theta) - fz*math.sin(theta),
            fx*math.sin(theta) + fz*math.cos(theta))

# 3-DOF leg IK: ABAD abduction plus the existing hip/knee 2-link. Unchanged from champgait.py.
D_ABAD = 0.1   # distance from body centerline to the ABAD pivot, in meters
OY = {"FL": 1, "FR": -1, "BL": 1, "BR": -1}   # +1 for left legs, -1 for right legs

def leg_ik_3d(fx, fy, fz, oy, s):
    """fx,fy,fz = desired foot position relative to the leg's ABAD pivot, in body-frame axes.
       oy = OY[leg] (+1 left, -1 right), s = LEG_SIDE[leg] (+1 front, -1 back). Returns (abad, hip,
       knee). Full geometry derivation is in champgait.py's header - unchanged here."""
    dy = oy * D_ABAD + fy   # lateral offset from hip to foot
    dz = fz   # vertical offset (renamed for the math below)
    r = math.hypot(dy, dz)   # distance from ABAD pivot to foot in the y-z plane
    r = max(r, D_ABAD + 1e-6)   # keep above the pivot offset to avoid a bad acos input
    c = max(-1.0, min(1.0, (oy * D_ABAD) / r))   # clamped ratio for the ABAD angle calc
    base = math.atan2(dz, dy)   # base angle toward the foot
    phi_a = base + math.acos(c)   # one candidate ABAD angle
    phi_b = base - math.acos(c)   # other candidate ABAD angle
    abad = phi_a if abs(phi_a) < abs(phi_b) else phi_b   # pick the smaller-magnitude solution
    w = -dy*math.sin(abad) + dz*math.cos(abad)   # effective z offset after removing the ABAD rotation
    hip, knee = leg_ik(fx, w, s)   # solve hip/knee with the 2-link IK
    return abad, hip, knee

LEG_SIDE = {"FL": 1, "FR": 1, "BL": -1, "BR": -1}   # +1 for front legs, -1 for back legs
legs = ["FL", "FR", "BL", "BR"]   # names of all four legs

node = transport.Node()   # gz-transport node for pub/sub
pubs = {}   # holds one publisher per joint command topic
for leg in legs:   # loop over the 4 legs
    pubs[f"{leg}_ABAD"] = node.advertise(f"/model/my_quadruped/joint/{leg}_ABAD/cmd_pos", Double)
    pubs[f"{leg}_HIP"] = node.advertise(f"/model/my_quadruped/joint/{leg}_HIP/cmd_pos", Double)
    pubs[f"{leg}_KNEE"] = node.advertise(f"/model/my_quadruped/joint/{leg}_KNEE/cmd_pos", Double)

# IMU: orientation (pitch/roll/yaw) plus filtered rates. Unchanged from champgait.py.
latest_pitch = [0.0]   # latest measured pitch, in radians
latest_roll = [0.0]   # latest measured roll, in radians
latest_yaw = [0.0]   # latest measured yaw, in radians

PITCH_RATE_LPF_ALPHA = 0.2   # smoothing factor for the filtered pitch rate, 0 to 1
latest_pitch_rate = [0.0]   # filtered pitch rate, in radians/sec
latest_pitch_rate_raw = [0.0]   # unfiltered pitch rate, in radians/sec
_pitch_rate_source = [None]   # tracks whether pitch rate came from gyro or finite difference
_prev_pitch_for_rate = [None]   # previous pitch value, for finite-difference rate
_prev_pitch_rate_time = [None]   # timestamp of that previous pitch reading
_dumped_imu_fields = [False]   # whether the IMU field names have been logged yet

YAW_RATE_LPF_ALPHA = 0.2   # smoothing factor for the filtered yaw rate, 0 to 1
latest_yaw_rate = [0.0]   # filtered yaw rate, in radians/sec
latest_yaw_rate_raw = [0.0]   # unfiltered yaw rate, in radians/sec
_yaw_rate_source = [None]   # tracks whether yaw rate came from gyro or finite difference
_prev_yaw_for_rate = [None]   # previous yaw value, for finite-difference rate
_prev_yaw_rate_time = [None]   # timestamp of that previous yaw reading

def imu_callback(msg):   # msg: IMU message from the sensor
    if not _dumped_imu_fields[0]:   # only do this once, on the very first IMU message
        _dumped_imu_fields[0] = True
        try:
            log_line(f"DEBUG: IMU message fields: {[f.name for f in msg.DESCRIPTOR.fields]}")
        except Exception as e:   # the introspection error
            log_line(f"DEBUG: could not introspect IMU message fields: {e}")

    q = msg.orientation   # quaternion orientation from the IMU
    sinp = max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))   # clamped sine-of-pitch term
    pitch = math.asin(sinp)   # pitch angle, in radians
    latest_pitch[0] = pitch
    latest_roll[0] = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    latest_yaw[0] = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    raw_rate = None   # pitch rate before filtering, if we get one
    try:
        raw_rate = msg.angular_velocity.y   # pitch rate straight from the gyro
        _pitch_rate_source[0] = "gyro"
    except AttributeError:
        _pitch_rate_source[0] = "fd"
        now = time.time()   # current time, for computing rate by finite difference
        if _prev_pitch_for_rate[0] is not None and _prev_pitch_rate_time[0] is not None:   # only compute a rate if there's a prior pitch reading to compare
            dt = now - _prev_pitch_rate_time[0]   # time since the last pitch reading
            if dt > 1e-4:   # only compute rate if enough time has passed
                raw_rate = (pitch - _prev_pitch_for_rate[0]) / dt   # pitch rate estimated by finite difference
        _prev_pitch_for_rate[0] = pitch
        _prev_pitch_rate_time[0] = now

    if raw_rate is not None:   # only update the filtered rate if a new reading came in
        latest_pitch_rate_raw[0] = raw_rate
        latest_pitch_rate[0] = (PITCH_RATE_LPF_ALPHA * raw_rate
                                 + (1.0 - PITCH_RATE_LPF_ALPHA) * latest_pitch_rate[0])

    yaw_raw_rate = None   # yaw rate before filtering, if we get one
    try:
        yaw_raw_rate = msg.angular_velocity.z   # yaw rate straight from the gyro
        _yaw_rate_source[0] = "gyro"
    except AttributeError:
        _yaw_rate_source[0] = "fd"
        now2 = time.time()   # current time, for the yaw finite-difference calc
        if _prev_yaw_for_rate[0] is not None and _prev_yaw_rate_time[0] is not None:   # only compute a rate if there's a prior yaw reading to compare
            dt2 = now2 - _prev_yaw_rate_time[0]   # time since the last yaw reading
            if dt2 > 1e-4:   # only compute rate if enough time has passed
                yaw_raw_rate = (latest_yaw[0] - _prev_yaw_for_rate[0]) / dt2   # yaw rate estimated by finite difference
        _prev_yaw_for_rate[0] = latest_yaw[0]
        _prev_yaw_rate_time[0] = now2

    if yaw_raw_rate is not None:   # only update the filtered rate if a new reading came in
        latest_yaw_rate_raw[0] = yaw_raw_rate
        latest_yaw_rate[0] = (YAW_RATE_LPF_ALPHA * yaw_raw_rate
                               + (1.0 - YAW_RATE_LPF_ALPHA) * latest_yaw_rate[0])

node.subscribe(IMU, "/model/my_quadruped/imu", imu_callback)

# ground-truth body + per-foot world-frame telemetry. Unchanged from champgait.py.
body_xyz = [None, None, None]   # latest measured body position (x,y,z), world frame
body_vel = [0.0, 0.0, 0.0]   # latest body velocity (x,y,z)
body_accel = [0.0, 0.0, 0.0]   # latest body acceleration (x,y,z)
_prev_body_xyz = [None, None, None]   # body position from the previous pose update
_prev_body_vel = [None, None, None]   # body velocity from the previous pose update
_prev_pose_time = [None]   # timestamp of the previous pose update

link_xyz = {}   # each leg's shank link position, body frame
_dumped_pose_names = [False]   # whether the pose entity names have been logged yet

def _rotate_body_to_world(dx, dy, dz, pitch, roll):   # rotates a body-frame offset into world frame
    cp, sp = math.cos(pitch), math.sin(pitch)   # cos/sin of pitch
    cr, sr = math.cos(roll), math.sin(roll)   # cos/sin of roll
    wx = cp*dx + sp*sr*dy + sp*cr*dz   # world-frame x offset
    wy = cr*dy - sr*dz   # world-frame y offset
    wz = -sp*dx + cp*sr*dy + cp*cr*dz   # world-frame z offset
    return wx, wy, wz

foot_world_xyz = {}   # each leg's actual foot position, world frame
foot_world_vel = {leg: [0.0, 0.0, 0.0] for leg in legs}   # each leg's actual foot velocity, world frame
_prev_foot_world_xyz = {}   # previous foot world positions, keyed by leg
_prev_foot_world_time = {}   # previous foot world timestamps, keyed by leg

def _update_foot_world_positions(now):   # now: current timestamp
    if body_xyz[0] is None:   # bail out if we don't know the body's position yet
        return
    pitch, roll = latest_pitch[0], latest_roll[0]   # current pitch and roll
    for leg_name, (lx, ly, lz) in link_xyz.items():   # loop over each leg's link position
        wx_off, wy_off, wz_off = _rotate_body_to_world(lx, ly, lz, pitch, roll)   # link offset rotated into world frame
        wx, wy, wz = body_xyz[0] + wx_off, body_xyz[1] + wy_off, body_xyz[2] + wz_off   # foot position in world frame
        prev = _prev_foot_world_xyz.get(leg_name)   # this leg's last known world position
        prev_t = _prev_foot_world_time.get(leg_name)   # timestamp of that last known position
        if prev is not None and prev_t is not None:   # only compute velocity if there's a previous reading to compare against
            dt = now - prev_t   # time since the last update for this foot
            if dt > 1e-4:   # only compute rate if enough time has passed
                foot_world_vel[leg_name][0] = (wx - prev[0]) / dt   # computes this foot's velocity from its change in position
                foot_world_vel[leg_name][1] = (wy - prev[1]) / dt
                foot_world_vel[leg_name][2] = (wz - prev[2]) / dt
        foot_world_xyz[leg_name] = (wx, wy, wz)
        _prev_foot_world_xyz[leg_name] = (wx, wy, wz)
        _prev_foot_world_time[leg_name] = now

CMD_HIP_OFFSET = {"FL": (0.15, 0.213), "FR": (0.15, -0.213), "BL": (-0.15, 0.213), "BR": (-0.15, -0.213)}   # each leg's hip position (x, y) relative to body center, in meters

cmd_foot_world_xyz = {}   # each leg's commanded foot position, world frame
cmd_foot_world_vel = {leg: [0.0, 0.0, 0.0] for leg in legs}   # each leg's commanded foot velocity, world frame
_prev_cmd_foot_world_xyz = {}   # previous commanded foot positions, keyed by leg
_prev_cmd_foot_world_time = {}   # previous commanded foot timestamps, keyed by leg

def _update_commanded_foot_world(leg, fx_c, fz_c, now):   # fx_c/fz_c: commanded foot x/z after rotation
    if body_xyz[0] is None:   # bail out if we don't know the body's position yet
        return
    x0, y0 = CMD_HIP_OFFSET[leg]   # this leg's hip offset from body center
    pitch, roll = latest_pitch[0], latest_roll[0]   # current pitch and roll
    wx_off, wy_off, wz_off = _rotate_body_to_world(x0 + fx_c, y0, fz_c, pitch, roll)   # commanded foot offset rotated into world frame
    wx, wy, wz = body_xyz[0] + wx_off, body_xyz[1] + wy_off, body_xyz[2] + wz_off   # commanded foot position in world frame
    prev = _prev_cmd_foot_world_xyz.get(leg)   # this leg's last commanded position
    prev_t = _prev_cmd_foot_world_time.get(leg)   # timestamp of that last commanded position
    if prev is not None and prev_t is not None:   # only compute velocity if there's a previous reading to compare against
        dt = now - prev_t   # time since the last commanded update
        if dt > 1e-4:   # only compute rate if enough time has passed
            cmd_foot_world_vel[leg][0] = (wx - prev[0]) / dt   # computes this leg's commanded velocity from its change in position
            cmd_foot_world_vel[leg][1] = (wy - prev[1]) / dt
            cmd_foot_world_vel[leg][2] = (wz - prev[2]) / dt
    cmd_foot_world_xyz[leg] = (wx, wy, wz)
    _prev_cmd_foot_world_xyz[leg] = (wx, wy, wz)
    _prev_cmd_foot_world_time[leg] = now

def foot_tracking_error(active_leg):   # active_leg: the leg currently swinging (excluded from tracking)
    out = {}   # tracking error per stance leg
    for l in legs:   # loop over all 4 legs
        if l == active_leg:   # skip the leg that's currently swinging, it's not planted
            continue
        if l in cmd_foot_world_xyz and l in foot_world_xyz:   # only compare if both the commanded and actual positions are known
            cx, cy, cz = cmd_foot_world_xyz[l]   # this leg's commanded position
            ax, ay, az = foot_world_xyz[l]   # this leg's actual position
            pos_err = math.sqrt((cx - ax) ** 2 + (cy - ay) ** 2 + (cz - az) ** 2)   # distance between commanded and actual foot position
            cvx, cvy, _cvz = cmd_foot_world_vel[l]   # this leg's commanded velocity
            avx, avy, _avz = foot_world_vel[l]   # this leg's actual velocity
            vel_err = math.hypot(cvx - avx, cvy - avy)   # horizontal velocity tracking error
            out[l] = (pos_err, vel_err)
    return out

def all_foot_world_z():
    return {l: foot_world_xyz[l][2] for l in legs if l in foot_world_xyz}

def pose_callback(msg):   # msg: Pose_V message with all entity poses
    if not _dumped_pose_names[0]:   # only do this once, on the very first pose message
        _dumped_pose_names[0] = True
        log_line(f"DEBUG: pose entity names seen: {sorted(set(p.name for p in msg.pose))}")
    for p in msg.pose:   # loop over every entity's pose in this message
        if p.name == "my_quadruped":   # this pose entry is the robot's main body
            now = time.time()   # current time, for velocity/accel calcs
            if _prev_pose_time[0] is not None:   # only compute velocity if we have a previous timestamp
                dt = now - _prev_pose_time[0]   # time since the last pose update
                if dt > 1e-4:   # only compute rate if enough time has passed
                    new_vx = (p.position.x - _prev_body_xyz[0]) / dt   # body x velocity this tick
                    new_vy = (p.position.y - _prev_body_xyz[1]) / dt   # body y velocity this tick
                    new_vz = (p.position.z - _prev_body_xyz[2]) / dt   # body z velocity this tick
                    if _prev_body_vel[0] is not None:   # only compute acceleration if we have a previous velocity
                        body_accel[0] = (new_vx - _prev_body_vel[0]) / dt   # acceleration = how much velocity changed over time
                        body_accel[1] = (new_vy - _prev_body_vel[1]) / dt
                        body_accel[2] = (new_vz - _prev_body_vel[2]) / dt
                    _prev_body_vel[0] = new_vx
                    _prev_body_vel[1] = new_vy
                    _prev_body_vel[2] = new_vz
                    body_vel[0] = new_vx
                    body_vel[1] = new_vy
                    body_vel[2] = new_vz
            _prev_body_xyz[0] = p.position.x
            _prev_body_xyz[1] = p.position.y
            _prev_body_xyz[2] = p.position.z
            _prev_pose_time[0] = now

            body_xyz[0] = p.position.x
            body_xyz[1] = p.position.y
            body_xyz[2] = p.position.z
        else:
            for leg_name in legs:   # loop over all 4 legs to match this link's name
                if p.name.endswith(f"{leg_name}_shank"):   # this pose entry belongs to this leg's shank link
                    link_xyz[leg_name] = (p.position.x, p.position.y, p.position.z)
                    break
    _update_foot_world_positions(time.time())

node.subscribe(Pose_V, "/world/empty/pose/info", pose_callback)

# stability checks against the support polygon. Unchanged from champgait.py.
def _sign2d(p1, p2, p3):   # p1,p2,p3: 2d points to test winding of
    return (p1[0]-p3[0])*(p2[1]-p3[1]) - (p2[0]-p3[0])*(p1[1]-p3[1])

def _point_in_triangle(pt, v1, v2, v3):   # pt: point to test; v1-v3: triangle corners
    d1 = _sign2d(pt, v1, v2)   # which side of edge v1-v2 the point is on
    d2 = _sign2d(pt, v2, v3)   # which side of edge v2-v3 the point is on
    d3 = _sign2d(pt, v3, v1)   # which side of edge v3-v1 the point is on
    has_neg = (d1 < 0) or (d2 < 0) or (d3 < 0)   # true if the point is outside on any edge (one side)
    has_pos = (d1 > 0) or (d2 > 0) or (d3 > 0)   # true if the point is outside on any edge (other side)
    return not (has_neg and has_pos)

def _polygon_check(point_xy, active_leg):   # point_xy: point to check; active_leg: leg excluded from support
    support_legs = [l for l in legs if l != active_leg]   # the 3 legs currently on the ground
    if any(l not in foot_world_xyz for l in support_legs):   # bail out if any support leg's position isn't known yet
        return None, None
    pts = [(foot_world_xyz[l][0], foot_world_xyz[l][1]) for l in support_legs]   # xy positions of the support legs
    inside = _point_in_triangle(point_xy, pts[0], pts[1], pts[2])   # whether the point is inside the support triangle
    def edge_dist(a, b):   # a,b: endpoints of one support-polygon edge
        ex, ey = b[0]-a[0], b[1]-a[1]   # edge vector
        edge_len = math.hypot(ex, ey)   # length of the edge
        if edge_len < 1e-6:   # avoid dividing by a near-zero-length edge
            return 0.0
        cross = (point_xy[0]-a[0])*ey - (point_xy[1]-a[1])*ex   # cross product, signed distance numerator
        return cross / edge_len
    margins = [edge_dist(pts[0], pts[1]), edge_dist(pts[1], pts[2]), edge_dist(pts[2], pts[0])]   # distance from point to each support edge
    return inside, min(abs(m) for m in margins)

def support_status(active_leg):
    if body_xyz[0] is None:   # bail out if we don't know the body's position yet
        return None, None
    return _polygon_check((body_xyz[0], body_xyz[1]), active_leg)

G = 9.8   # gravity's acceleration, in m/s^2

def compute_zmp():
    if body_xyz[0] is None:   # bail out if we don't know the body's position yet
        return None
    z_com = body_xyz[2]   # body height (center of mass proxy)
    xdd, ydd, zdd = body_accel   # body acceleration components
    denom = zdd + G   # effective vertical acceleration term
    if abs(denom) < 1.0:   # avoid dividing by a near-zero vertical accel term
        denom = G   # fall back to plain gravity if too small
    return (body_xyz[0] - (xdd/denom)*z_com, body_xyz[1] - (ydd/denom)*z_com)

def zmp_status(active_leg):
    zmp = compute_zmp()   # zero moment point estimate
    if zmp is None:   # bail out if the ZMP couldn't be computed
        return None, None, None
    inside, margin = _polygon_check(zmp, active_leg)   # whether ZMP is inside support polygon, and by how much
    return inside, margin, zmp

CAPTURE_HEIGHT = 0.40   # assumed body height used for the capture-point math, in meters
CAPTURE_GAIN = math.sqrt(CAPTURE_HEIGHT / 9.8)   # scaling factor for the capture point calc

def compute_capture_point():
    if body_xyz[0] is None:   # bail out if we don't know the body's position yet
        return None
    return (body_xyz[0] + body_vel[0]*CAPTURE_GAIN, body_xyz[1] + body_vel[1]*CAPTURE_GAIN)

def capture_point_status(active_leg):
    cp = compute_capture_point()   # capture point estimate
    if cp is None:   # bail out if the capture point couldn't be computed
        return None, None, None
    inside, margin = _polygon_check(cp, active_leg)   # whether capture point is inside support polygon, and by how much
    return inside, margin, cp

def stance_foot_velocities(active_leg):
    support_legs = [l for l in legs if l != active_leg]   # the 3 legs currently on the ground
    return {l: tuple(foot_world_vel.get(l, (0.0, 0.0, 0.0))[:2]) for l in support_legs}

def all_foot_fx():
    return {l: foot_target[l][0] for l in legs}

STANCE_FZ = -0.34   # target foot height (z) while standing, in meters
STEP_LENGTH_FRONT = 0.06   # how far a front foot swings forward/back each step, in meters
STEP_LENGTH_BACK = 0.08   # how far a back foot swings forward/back each step, in meters
SWING_HEIGHT_FRONT = 0.06   # how high a front foot lifts during its swing, in meters
SWING_HEIGHT_BACK = 0.08   # how high a back foot lifts during its swing, in meters
FX_LIMIT = 0.09   # largest allowed fore-aft foot offset, in meters

def clamp_fx(v):   # v: fore-aft foot offset to clamp
    return max(-FX_LIMIT, min(FX_LIMIT, v))

def _smoothstep(x):   # x: fraction from 0 to 1 to ease
    x = max(0.0, min(1.0, x))   # clamp input into 0-1 range
    return x * x * (3 - 2 * x)

REACH_BUDGET = 0.39   # leg's max safe reach distance, in meters

def max_safe_fy(fx_c, fz_c, budget=REACH_BUDGET):
    """Unchanged from champgait.py - closed-form reach-aware bound on |fy|. Full derivation is in
       that file's docstring. Still needed here because the continuous sway drives fy through a full
       sine sweep every cycle, so each tick/leg needs the same reach clamp the discrete version used,
       not just assuming a fixed SWAY_AMPLITUDE is safe everywhere."""
    r_max_sq = budget * budget - fx_c * fx_c + D_ABAD * D_ABAD   # max squared reach available for lateral offset
    if r_max_sq <= 0.0:   # no reach left at all here, so no safe lateral offset
        return 0.0
    dy_max_sq = r_max_sq - fz_c * fz_c   # max squared lateral distance at this height
    if dy_max_sq <= 0.0:   # no lateral room left at this height either
        return 0.0
    return max(0.0, math.sqrt(dy_max_sq) - D_ABAD)

def clamp_fy(v, fx_c=0.0, fz_c=None):
    if fz_c is None:   # default to standing height if none was given
        fz_c = STANCE_FZ   # default to standing height if not given
    bound = max_safe_fy(fx_c, fz_c)   # largest safe lateral offset here
    return max(-bound, min(bound, v))

# Continuous gait - new in this fork. Everything below replaces champgait.py's discrete
# shift_com_to / wait_for_safe_lift / do_swing / wait_for_settle state machine.

GAIT_PERIOD = 24.0
SWING_DUTY = 0.15
GAIT_ORDER = ["BR", "FL", "FR", "BL"]   # unchanged from champgait.py - alternates right/left/right/
                                          # left; the sway math below depends on that.
LEG_PHASE_OFFSET = {leg: i / 4.0 for i, leg in enumerate(GAIT_ORDER)}   # each leg's swing start, as a fraction of one gait cycle
N_CYCLES = 3   # how many gait cycles to walk before stopping

def leg_phase_fracs(leg, t):   # t: elapsed gait time in seconds
    global_phase = (t % GAIT_PERIOD) / GAIT_PERIOD   # current position in the gait cycle, 0 to 1
    return (global_phase - LEG_PHASE_OFFSET[leg]) % 1.0

def current_swing_leg(t):
    """None during the quiet (all-planted) windows, otherwise whichever leg is currently swinging."""
    for leg in legs:   # check each leg in turn
        if leg_phase_fracs(leg, t) < SWING_DUTY:   # true while this leg is inside its swing window
            return leg
    return None

def foot_offset_for_leg(leg, t):
    """Continuous fore-aft stance sweep + swing arc, adapted from champ_old.py - this part of the old
       continuous gait was never the problem (forward walking always worked, only the missing lateral
       shift didn't). Swing uses the same smoothstep-fx / sine-fz arc champgait.py's swing_profile
       used, just driven off the shared phase clock instead of its own per-leg timer.

       v8 fix (fz only): three falls in one day, at three different legs (BL, then FL, then BR), all
       with the same signature - a violent gyro/accel spike right as that leg's swing ended (pitch
       rate into the hundreds of deg/s, accel spikes of several m/s^2), theta saturating at
       MAX_CORRECTION_RAD moments later, then a SAFETY ABORT. Same pattern on three unrelated legs
       means it's something about every touchdown, not a per-leg phase bug (already ruled those out
       via the crouch-settle/gait-exit/ease-in/y_com_target fixes). The old height curve,
       fz = STANCE_FZ + height*sin(pi*swing_frac), hits zero height at swing_frac=0 and 1 (fine) but
       its derivative - vertical velocity - is at its max (+-height*pi) at those same points, so every
       touchdown lands with real downward speed: a thud every time. That was fine under champ_old.py's
       soft gains (p_gain=50/d_gain=2) but not under the live gains (p_gain=265/d_gain=10.6, confirmed
       in model.sdf) - a stiffer controller reacting to a hard landing kicks back much harder. Fix:
       swap the half-sine for a raised-cosine height profile, 0.5*(1-cos(2*pi*swing_frac)) - same peak
       height and timing (mid-swing), but its derivative is 0 at both swing_frac=0 and 1 (checked
       numerically: ~3.14 -> ~0.001), so the foot now lifts off and lands at basically zero vertical
       speed instead of a jolt."""
    is_front = leg in ("FL", "FR")   # true if this is a front leg
    step = STEP_LENGTH_FRONT if is_front else STEP_LENGTH_BACK   # how far this leg swings fore-aft
    height = SWING_HEIGHT_FRONT if is_front else SWING_HEIGHT_BACK   # how high this leg lifts during swing
    local_phase = leg_phase_fracs(leg, t)   # this leg's own position in its swing/stance cycle
    if local_phase < SWING_DUTY:   # true while this leg is in its swing phase
        swing_frac = local_phase / SWING_DUTY   # how far through the swing phase, 0 to 1
        s = _smoothstep(swing_frac)   # eased swing progress
        fx = -step / 2.0 + step * s   # fore-aft foot target during swing
        fz = STANCE_FZ + height * 0.5 * (1.0 - math.cos(2 * math.pi * swing_frac))   # foot height during swing
    else:
        stance_frac = (local_phase - SWING_DUTY) / (1.0 - SWING_DUTY)   # how far through the stance phase, 0 to 1
        fx = (step / 2.0) - step * stance_frac   # fore-aft foot target during stance
        fz = STANCE_FZ   # foot stays at standing height during stance
    return clamp_fx(fx), fz

SWAY_AMPLITUDE = 0.06
SWAY_PERIOD_FRAC = 0.5    # fraction of GAIT_PERIOD for one full sway cycle - see the derivation above
SWAY_PHASE_OFFSET_FRAC = 0.0   # v2: peak at swing onset, not mid-swing - see the fix note above

MAX_SHIFT_RATE = 0.0375

def y_com_target(t):
    global_phase = (t % GAIT_PERIOD) / GAIT_PERIOD   # current position in the gait cycle, 0 to 1
    A = SWAY_AMPLITUDE   # shorthand for the sway amplitude
    if global_phase < 0.15:                          # BR swinging (right) - stay right (away from BR)
        return -A
    elif global_phase < 0.25:                         # quad-stance before FL - transition right->left
        frac = (global_phase - 0.15) / 0.10   # progress through this transition window
        return -A + 2 * A * _smoothstep(frac)
    elif global_phase < 0.40:                          # FL swinging (left) - stay left (away from FL)
        return A
    elif global_phase < 0.50:                          # quad-stance before FR - transition left->right
        frac = (global_phase - 0.40) / 0.10   # progress through this transition window
        return A - 2 * A * _smoothstep(frac)
    elif global_phase < 0.65:                          # FR swinging (right) - stay right (away from FR)
        return -A
    elif global_phase < 0.75:                          # quad-stance before BL - transition right->left
        frac = (global_phase - 0.65) / 0.10   # progress through this transition window
        return -A + 2 * A * _smoothstep(frac)
    elif global_phase < 0.90:                          # BL swinging (left) - stay left (away from BL)
        return A
    else:                                                # quad-stance before BR (wrap) - transition left->right
        frac = (global_phase - 0.90) / 0.10   # progress through this transition window
        return A - 2 * A * _smoothstep(frac)

X_LEAN_AMPLITUDE = 0.035

def x_com_target(t):
    global_phase = (t % GAIT_PERIOD) / GAIT_PERIOD   # current position in the gait cycle, 0 to 1
    X = X_LEAN_AMPLITUDE   # shorthand for the fore-aft lean amplitude
    if global_phase < 0.15:                       # BR still swinging or just landed - stay forward
        return -X
    elif global_phase < 0.25:                     # quad-stance before FL - transition forward->backward
        frac = (global_phase - 0.15) / 0.10   # progress through this transition window
        return -X + 2 * X * _smoothstep(frac)
    elif global_phase < 0.65:                      # FL then FR swinging - stay backward
        return X
    elif global_phase < 0.75:                      # quad-stance before BL - transition backward->forward
        frac = (global_phase - 0.65) / 0.10   # progress through this transition window
        return X - 2 * X * _smoothstep(frac)
    else:                                           # BL swinging - stay forward
        return -X

body_shift = [0.0, 0.0]

last_abad = {leg: 0.0 for leg in legs}   # most recently commanded ABAD angle per leg, for logging
foot_target = {leg: (0.0, -0.4) for leg in legs}   # each leg's current target foot position (fx, fz)

PITCH_SIGN = 1.0
CORRECTION_FRACTION = 0.075   # how strongly pitch error gets corrected (P gain)
MAX_CORRECTION_RAD = 0.35   # largest allowed pitch-correction angle, in radians
PITCH_RATE_DAMPING = 0.028  # v16: was 0.15, tuned for p_gain=50/d_gain=2 (see v16 note above) - scaled
                            # by 1/5.3 to match the live p_gain=265/d_gain=10.6 joints.

ROLL_ABAD_FRACTION = 0.3   # how strongly roll error gets corrected via ABAD (P gain)
MAX_ABAD_ROLL_CORR = 0.15   # largest allowed roll-correction angle, in radians

LEG_LR = {"FL": 1, "FR": -1, "BL": 1, "BR": -1}
YAW_RATE_DAMPING = 0.0
YAW_FEEDFORWARD_BIAS = -0.005
YAW_FX_GAIN = 0.09   # v9 value: P gain for yaw correction (superseded below)
MAX_YAW_FX = 0.075
YAW_FX_GAIN = 0.18   # v18 value: P gain for yaw correction (superseded below)
MAX_YAW_FX = 0.15
YAW_FX_GAIN = 0.18   # final P gain for yaw correction (v19 reverted to v18's value)
MAX_YAW_FX = 0.15   # final clamp for yaw correction (v19 reverted to v18's value)
gait_start_yaw = [None]   # yaw recorded at the moment gait engaged
gait_start_time = [None]   # time recorded at the moment gait engaged

running = [True]   # set False to stop the control loop thread

FLIP_LIMIT_DEG = 25.0   # past this tilt angle, treat the robot as falling over
FLIP_LIMIT_RAD = math.radians(FLIP_LIMIT_DEG)   # same limit as above, in radians
aborted = [False]   # becomes True once the safety abort has triggered

gait_active = [False]   # True while the continuous gait is currently running

def check_abort():
    if aborted[0]:   # already aborted, nothing left to check
        return True
    if abs(latest_pitch[0]) > FLIP_LIMIT_RAD or abs(latest_roll[0]) > FLIP_LIMIT_RAD:   # true if the robot has tipped past the safe angle
        aborted[0] = True
        print(f"!!! SAFETY ABORT: |pitch|={math.degrees(latest_pitch[0]):.1f} deg  "
              f"|roll|={math.degrees(latest_roll[0]):.1f} deg exceeded {FLIP_LIMIT_DEG} deg - "
              f"freezing in place, joint commands keep publishing (clamped) !!!")
    return aborted[0]

CONTROL_DT = 0.02   # time between control loop ticks, in seconds

last_theta_terms = {"p": 0.0, "d": 0.0, "raw": 0.0, "clamped": 0.0}   # most recent pitch-correction terms, for logging
last_yaw_terms = {"err": 0.0, "p": 0.0, "d": 0.0, "fx_term": 0.0}   # most recent yaw-correction terms, for logging
last_active_leg = [None]   # which leg is currently swinging, for logging

def control_loop():
    """Unlike champgait.py's control_loop (which only reads foot_target/body_shift, set elsewhere by
       the main thread's discrete state machine), this one computes both itself, every tick, straight
       off the shared gait clock - there's no separate sequencer driving them. During crouch
       (gait_active False), foot_target is instead driven directly by move_feet_manual, same as in
       champgait.py."""
    while running[0]:   # keep looping until the run finishes or aborts
        check_abort()

        active_leg = None   # leg currently swinging this tick, if any
        if gait_active[0] and gait_start_time[0] is not None:   # only update gait targets once the gait has actually started
            t = time.time() - gait_start_time[0]   # elapsed time since gait started
            for leg in legs:   # update every leg's foot target this tick
                foot_target[leg] = foot_offset_for_leg(leg, t)
            active_leg = current_swing_leg(t)
            max_step = MAX_SHIFT_RATE * CONTROL_DT   # largest body_shift change allowed this tick
            target_fx = x_com_target(t)   # desired fore-aft body shift this instant
            body_shift[0] += max(-max_step, min(max_step, target_fx - body_shift[0]))
            target_fy = y_com_target(t)   # desired lateral body shift this instant
            body_shift[1] += max(-max_step, min(max_step, target_fy - body_shift[1]))
        last_active_leg[0] = active_leg

        theta_p = PITCH_SIGN * CORRECTION_FRACTION * latest_pitch[0]   # proportional term of pitch correction
        theta_d = PITCH_SIGN * PITCH_RATE_DAMPING * latest_pitch_rate[0]   # derivative term of pitch correction
        theta_raw = theta_p + theta_d   # combined pitch correction before clamping
        theta = max(-MAX_CORRECTION_RAD, min(MAX_CORRECTION_RAD, theta_raw))   # clamped pitch correction angle
        last_theta_terms["p"] = theta_p
        last_theta_terms["d"] = theta_d
        last_theta_terms["raw"] = theta_raw
        last_theta_terms["clamped"] = theta

        roll_term = ROLL_ABAD_FRACTION * latest_roll[0]   # proportional roll correction
        roll_term = max(-MAX_ABAD_ROLL_CORR, min(MAX_ABAD_ROLL_CORR, roll_term))   # clamp roll correction

        if gait_active[0] and gait_start_yaw[0] is not None:   # only track yaw drift once the gait has started
            yaw_err = latest_yaw[0] - gait_start_yaw[0]   # how far yaw has drifted since gait started
        else:
            yaw_err = 0.0   # no yaw error to correct before gait starts
        yaw_p = YAW_FX_GAIN * yaw_err   # proportional term of yaw correction
        yaw_d = YAW_RATE_DAMPING * latest_yaw_rate[0]
        yaw_correction = max(-MAX_YAW_FX, min(MAX_YAW_FX, yaw_p + yaw_d + YAW_FEEDFORWARD_BIAS))   # clamped yaw correction to steer with
        last_yaw_terms["err"] = yaw_err
        last_yaw_terms["p"] = yaw_p
        last_yaw_terms["d"] = yaw_d
        last_yaw_terms["fx_term"] = yaw_correction

        now = time.time()   # current time, for commanded-foot telemetry
        for leg in legs:   # compute and publish joint commands for each leg
            fx, fz = foot_target[leg]
            if leg == active_leg or active_leg is None:   # full correction for the leg that's swinging, or when none is
                this_yaw_term = yaw_correction   # full yaw correction for the swinging or unblocked leg
            else:
                this_yaw_term = 0.0   # no yaw correction for a blocked stance leg
            fx = clamp_fx(fx + body_shift[0] + LEG_LR[leg] * this_yaw_term)   # final fore-aft target after shift and yaw correction
            fx_c, fz_c = rotate(fx, fz, theta)   # foot target rotated by the pitch correction
            fy = clamp_fy(body_shift[1], fx_c=fx_c, fz_c=fz_c)   # lateral foot target, clamped to a safe reach
            abad_geo, hip, knee = leg_ik_3d(fx_c, fy, fz_c, OY[leg], LEG_SIDE[leg])   # joint angles from inverse kinematics
            _update_commanded_foot_world(leg, fx_c, fz_c, now)

            abad = abad_geo + roll_term   # final ABAD angle including roll correction
            last_abad[leg] = abad

            m0 = Double(); m0.data = abad   # ABAD command message
            pubs[f"{leg}_ABAD"].publish(m0)
            m1 = Double(); m1.data = hip   # hip command message
            pubs[f"{leg}_HIP"].publish(m1)
            m2 = Double(); m2.data = knee   # knee command message
            pubs[f"{leg}_KNEE"].publish(m2)
        time.sleep(CONTROL_DT)

t = threading.Thread(target=control_loop, daemon=True)   # background thread running the control loop

def diagnostic_line(label, active_leg=None):   # label: text tag for this log line
    vx, vy, vz = body_vel   # current body velocity components
    ax, ay, az = body_accel   # current body acceleration components
    speed = math.hypot(vx, vy)   # horizontal body speed
    line = (f"  [{label}] pitch: {math.degrees(latest_pitch[0]):+.2f} deg  "   # formatted diagnostic text
            f"roll: {math.degrees(latest_roll[0]):+.2f} deg  yaw: {math.degrees(latest_yaw[0]):+.2f} deg  "
            f"body_xyz: {body_xyz}  "
            f"vel(x,y,z): ({vx:+.3f},{vy:+.3f},{vz:+.3f})  speed_xy: {speed:.3f}  "
            f"accel(x,y,z): ({ax:+.3f},{ay:+.3f},{az:+.3f})  "
            f"pitch_rate({_pitch_rate_source[0]} raw={math.degrees(latest_pitch_rate_raw[0]):+.2f} "
            f"filt={math.degrees(latest_pitch_rate[0]):+.2f} deg/s)  "
            f"theta(p={math.degrees(last_theta_terms['p']):+.2f} d={math.degrees(last_theta_terms['d']):+.2f} "
            f"-> {math.degrees(last_theta_terms['clamped']):+.2f} deg)  "
            f"body_shift(fx={body_shift[0]:+.4f} fy={body_shift[1]:+.4f})  "
            f"yaw_corr(err={math.degrees(last_yaw_terms['err']):+.2f}deg "
            f"rate({_yaw_rate_source[0]}={math.degrees(latest_yaw_rate[0]):+.2f}deg/s) "
            f"p={last_yaw_terms['p']:+.4f} d={last_yaw_terms['d']:+.4f} "
            f"-> fx_term={last_yaw_terms['fx_term']:+.4f})")
    if active_leg is not None:   # only check stability margins while a leg is swinging
        inside, margin = support_status(active_leg)   # whether CoM is inside support polygon, and margin
        if inside is not None:   # only log this if the support-polygon check succeeded
            line += f"  CoM: inside={inside} margin={margin:+.4f}"
        zmp_in, zmp_margin, _zmp_pt = zmp_status(active_leg)   # whether ZMP is inside support polygon, and margin
        if zmp_in is not None:   # only log this if the ZMP check succeeded
            line += f"  ZMP: inside={zmp_in} margin={zmp_margin:+.4f}"
        cp_in, cp_margin, _cp_pt = capture_point_status(active_leg)   # whether capture point is inside support polygon, and margin
        if cp_in is not None:   # only log this if the capture-point check succeeded
            line += f"  CapturePt: inside={cp_in} margin={cp_margin:+.4f}"
        vels = stance_foot_velocities(active_leg)   # xy velocity of each stance leg
        line += "  stance_foot_vxy: {" + " ".join(f"{l}:({vx2:+.3f},{vy2:+.3f})" for l, (vx2, vy2) in vels.items()) + "}"
    track = foot_tracking_error(active_leg)   # tracking error for each stance leg
    if track:   # only log tracking error if there is any
        line += "  velerr: {" + " ".join(f"{l}:{ve:.3f}" for l, (_pe, ve) in track.items()) + "}"
    foot_z = all_foot_world_z()   # world-frame foot height for each leg
    if foot_z:   # only log foot heights if any are known
        marker = lambda l: "*" if l == active_leg else ""   # marks the swinging leg with an asterisk
        line += "  foot_z: {" + " ".join(f"{l}{marker(l)}:{z:+.3f}" for l, z in foot_z.items()) + "}"
    fx_all = all_foot_fx()   # current fore-aft foot target for each leg
    line += "  fx: {" + " ".join(f"{l}:{v:+.4f}" for l, v in fx_all.items()) + "}"
    line += "  abad: {" + " ".join(f"{l}:{math.degrees(v):+.2f}" for l, v in last_abad.items()) + "}"
    log_line(line)

def move_feet_manual(deltas, duration=1.5, steps=75, label=None):   # deltas: per-leg target foot positions to move to
    """Unchanged from champgait.py - used only OUTSIDE the continuous gait (crouch, final settle)."""
    starts = {leg: foot_target[leg] for leg in deltas}   # each leg's current foot position, before moving
    print_every = max(1, steps // 10)   # how often to print progress
    for i in range(1, steps + 1):   # step through the interpolation
        if check_abort():   # stop the move early if a safety abort fired
            return
        frac = i / steps   # how far through the move, 0 to 1
        for leg, (tx, tz) in deltas.items():   # loop over each leg's target position
            sx, sz = starts[leg]   # this leg's starting position
            foot_target[leg] = (sx + (tx - sx) * frac, sz + (tz - sz) * frac)
        if label and (i % print_every == 0 or i == 1):   # only print progress occasionally, if a label was given
            diagnostic_line(f"{label} {frac*100:3.0f}%")
        time.sleep(duration / steps)
    for leg, tgt in deltas.items():   # snap each leg to its exact final target
        foot_target[leg] = tgt

def ease_into_gait(duration=4.0, steps=200):
    """v6.11-wave fix: without this, turning on the continuous gait meant control_loop's first
       gait-active tick would directly assign foot_offset_for_leg(leg, 0.0) / y_com_target(0.0) to
       foot_target/body_shift[1] with no rate limit at all (champgait.py's shift_com_to at least caps
       at SHIFT_STEP_MAX) - a real position jump, not a smooth start. Concretely: BR's
       LEG_PHASE_OFFSET is 0, so t=0 is right at the start of its swing (fx jumps from crouch's 0 to
       -STEP_LENGTH_BACK/2, ~4cm) at the same instant y_com_target(0) wants ~-3.5cm of lateral sway
       (t=0 is BR's swing onset, not mid-swing where the sway actually peaks) - two simultaneous,
       unlimited jumps on a freshly-settled, zero-velocity crouch. The first real run fell within
       about a second of gait start (pitch already -9.5deg by t=0.52s), matching what stacking those
       two jumps would produce - so the "no discrete transients" idea this rewrite is built on was
       being broken right at the starting gun. Fix: ramp foot_target (all 4 legs) and body_shift[1]
       smoothly from wherever crouch left them to their own t=0 phase-clock values before gait_active
       flips on, so control_loop's takeover at t=0 is seamless. The phase alignment itself (sway peaks
       at each leg's swing onset, see v2 above) doesn't change - only the cold start into it is now
       continuous too.

       v3: default duration/steps raised from 1.0s/50 to 2.0s/100 (same 0.02s per step) because
       target_fy is now the full -SWAY_AMPLITUDE (v2 moved the peak to swing onset, so
       y_com_target(0.0) = -0.06, not -0.0353) - at the old 1.0s that's a 0.06 m/s ramp, above
       MAX_SHIFT_RATE's 0.0375 m/s cap. 2.0s keeps this ramp under that rate too.

       v4: also eases body_shift[0] (fx) to x_com_target(0.0) - BR is a back leg, so t=0 needs the
       full +X_LEAN_AMPLITUDE forward lean already in place before gait_active flips on, same idea as
       the y half.

       v5: duration/steps doubled again, 2.0s/100 -> 4.0s/200 (same 0.02s pacing). After the v4.1 fx
       sign fix, a run had healthy CoM/ZMP/capture-point margins throughout (+0.06 to +0.10) and real
       forward progress (net dx=+0.228m) - the sign fix clearly worked - but pitch still climbed
       steadily positive (forward tip), starting during this ease-in itself (+4.65deg by the time gait
       engaged, before BR even lifted) and never recovered. theta's reactive correction wasn't
       saturating (8deg vs MAX_CORRECTION_RAD's ~20deg), so this reads as the combined fx+fy ramp -
       both axes moving to full magnitude at once - being a bigger/faster disturbance than the
       reactive loop can arrest, not a sign/logic bug. Doubling the duration halves that transient's
       rate without touching any gain.

       v6: the "both axes at once" theory above was never actually tested - only the duration was
       doubled, which changes the rate but not the fact that fx and fy still move together. In one
       day this ease-in caused three separate incidents, all landing at about the same 25-30% mark of
       the ramp regardless of the 4.0s duration: a -21.4deg pitch dip that recovered, a -22.6deg dip
       that didn't (SAFETY ABORT), and a +12.1deg climb that also aborted. Same fraction across
       separate runs, independent of the earlier duration change, points at the ramp's shape (two
       disturbances stacked) rather than its rate. So now actually testing that theory: stagger fy
       (lateral sway) and fx (forward lean) into two back-to-back phases instead of moving both at
       once, each at the same per-axis rate the old combined ramp used (isolating "simultaneous vs
       sequential" as the one changed variable). Costs an extra `duration` seconds of ease-in - trivial
       next to the 72s of gait that follows, worth it if it kills a failure mode that caused 3 of
       today's aborts."""
    starts = {leg: foot_target[leg] for leg in legs}   # each leg's current foot position, before easing
    start_fx = body_shift[0]   # current fore-aft body shift, before easing
    start_fy = body_shift[1]   # current lateral body shift, before easing
    targets = {leg: foot_offset_for_leg(leg, 0.0) for leg in legs}   # each leg's foot position at gait's t=0
    target_fx = x_com_target(0.0)   # fore-aft body shift at gait's t=0
    target_fy = y_com_target(0.0)   # lateral body shift at gait's t=0
    total_steps = steps * 2   # two phases (fy then fx), each `steps` long
    for i in range(1, total_steps + 1):   # step through both easing phases
        if check_abort():   # stop easing early if a safety abort fired
            return
        frac = i / total_steps   # overall progress through both phases, 0 to 1
        for leg in legs:   # update every leg's foot target this step
            sx, sz = starts[leg]   # this leg's starting position
            tx, tz = targets[leg]   # this leg's target position at gait start
            foot_target[leg] = (sx + (tx - sx) * frac, sz + (tz - sz) * frac)
        # phase 1 (first half): ramp fy only, fx held at its start value.
        # phase 2 (second half): fy held at its now-reached target, ramp fx.
        if i <= steps:   # true during the first half of the ease, the fy-only phase
            fy_frac = i / steps   # progress through the fy-only phase
            fx_frac = 0.0   # fx not moving yet during phase 1
        else:
            fy_frac = 1.0   # fy already fully eased in
            fx_frac = (i - steps) / steps   # progress through the fx-only phase
        body_shift[1] = start_fy + (target_fy - start_fy) * fy_frac
        body_shift[0] = start_fx + (target_fx - start_fx) * fx_frac
        if i % 10 == 0 or i == 1:   # only print progress every so often
            diagnostic_line(f"easing into gait {frac*100:3.0f}%")
        time.sleep(duration / steps)
    for leg in legs:   # snap every leg to its exact final target
        foot_target[leg] = targets[leg]
    body_shift[0] = target_fx
    body_shift[1] = target_fy

SPEED_SETTLE_THRESHOLD = 0.06

def ease_out_of_gait(duration=2.0, steps=100, label="post-gait settle"):
    """Mirror image of ease_into_gait, added after trot_demo.py's post-gait-fall debugging. Used to be:
       body_shift[1] snapped to 0.0 in one tick, body_shift[0] was never reset at all, and foot_target
       eased toward neutral separately via move_feet_manual - three different transition rates for
       three things that all feed the same leg IK, exactly the "discrete jump" problem this file's
       header (see ease_into_gait) already points at as the real cause of falls elsewhere. Also matters
       mechanically: control_loop adds body_shift[0] to every leg's fx every tick, so leaving it
       wherever the gait ended biases all 4 feet the same way through "post-gait settle"/"end of loop",
       not neutral standing. Ramp foot_target and both body_shift axes down together, at the same
       MAX_SHIFT_RATE pace ease_into_gait uses to ramp them up.
    """
    starts = {leg: foot_target[leg] for leg in legs}   # each leg's current foot position, before easing out
    targets = {leg: (0.0, STANCE_FZ) for leg in legs}   # neutral standing position for each leg
    start_shift = list(body_shift)   # body shift values before easing out
    print_every = max(1, steps // 10)   # how often to print progress
    for i in range(1, steps + 1):   # step through the interpolation
        if check_abort():   # stop easing early if a safety abort fired
            return
        frac = i / steps   # how far through the move, 0 to 1
        for leg in legs:   # update every leg's foot target this step
            sx, sz = starts[leg]   # this leg's starting position
            tx, tz = targets[leg]   # this leg's neutral target position
            foot_target[leg] = (sx + (tx - sx) * frac, sz + (tz - sz) * frac)
        body_shift[0] = start_shift[0] * (1.0 - frac)
        body_shift[1] = start_shift[1] * (1.0 - frac)
        if label and (i % print_every == 0 or i == 1):   # only print progress occasionally, if a label was given
            diagnostic_line(f"{label} {frac*100:3.0f}%")
        time.sleep(duration / steps)
    for leg in legs:   # snap every leg to its exact neutral target
        foot_target[leg] = targets[leg]
    body_shift[0] = 0.0
    body_shift[1] = 0.0

print("waiting for the drop to settle...")
time.sleep(5.0)
print(f"landed, pitch = {math.degrees(latest_pitch[0]):.2f} deg  roll = {math.degrees(latest_roll[0]):.2f} deg")
print(f"body position: {body_xyz}")
yaw_at_settle = latest_yaw[0]   # yaw right after the drop settled
print(f"yaw at settle (before control_loop has published a single command): {math.degrees(yaw_at_settle):+.2f} deg")

t.start()

print("--- crouch (hip/knee only, all ABAD at 0) ---")
move_feet_manual({leg: (0.0, STANCE_FZ) for leg in legs}, duration=1.5, steps=75, label="crouch")

CROUCH_SETTLE_MIN = 1.0   # shortest time to wait for the crouch to settle, in seconds
CROUCH_SETTLE_MAX = 4.0   # longest time to wait before giving up on settling, in seconds
CROUCH_ANGLE_SETTLE_DEG = 2.0   # pitch/roll must be under this to count as settled, in degrees
_crouch_settle_start = time.time()   # when the crouch settle wait began
while True:   # loop until the crouch settles or we give up waiting
    if check_abort():   # stop waiting early if a safety abort fired
        break
    diagnostic_line("crouch settle")
    _elapsed_cs = time.time() - _crouch_settle_start   # how long we've waited for the crouch to settle
    _speed_cs = math.hypot(body_vel[0], body_vel[1])   # current horizontal body speed
    _pitch_cs = abs(math.degrees(latest_pitch[0]))   # current pitch magnitude, in degrees
    _roll_cs = abs(math.degrees(latest_roll[0]))   # current roll magnitude, in degrees
    settled = (_speed_cs <= SPEED_SETTLE_THRESHOLD and _pitch_cs <= CROUCH_ANGLE_SETTLE_DEG
               and _roll_cs <= CROUCH_ANGLE_SETTLE_DEG)   # true once speed and tilt are both within limits
    if (_elapsed_cs >= CROUCH_SETTLE_MIN and settled) or _elapsed_cs >= CROUCH_SETTLE_MAX:   # done once settled for the minimum time, or waited too long
        break
    time.sleep(0.1)

start_xyz = list(body_xyz)   # body position at the start of the run
start_yaw = latest_yaw[0]
print(f"start_yaw (reset baseline, absolute): {math.degrees(start_yaw):+.2f} deg")

# the continuous gait itself: no shift/lift/settle state machine, just let the phase clock and sine
# sway run, watched by diagnostic_line + the safety-abort watchdog.
if not aborted[0]:   # only proceed if nothing has aborted yet
    print("--- easing into gait's t=0 pose (avoids a discrete jump at gait start) ---")
    ease_into_gait()
if not aborted[0]:   # only start the gait if nothing has aborted yet
    gait_start_yaw[0] = latest_yaw[0]
    gait_start_time[0] = time.time()
    gait_active[0] = True
    print(f"--- continuous gait engaged: period={GAIT_PERIOD}s  swing_duty={SWING_DUTY}  "
          f"sway_amplitude={SWAY_AMPLITUDE}  cycles={N_CYCLES} ---")
    total_duration = N_CYCLES * GAIT_PERIOD
    GAIT_EXIT_ANGLE_SETTLE_DEG = 5.0   # pitch/roll must be under this to exit the gait, in degrees
    MAX_EXTRA_GAIT_TIME = 3.0   # longest extra time to wait past the cutoff, in seconds
    run_start = time.time()   # when the gait actually started walking
    printed_leg = [None]   # last leg name printed, to avoid repeat prints
    tick = 0   # counts control-loop ticks during the gait
    while True:   # loop until the gait finishes or aborts
        if check_abort():   # stop the gait loop early if a safety abort fired
            break
        elapsed = time.time() - run_start   # how long the gait has been running
        if elapsed >= total_duration:   # true once we've walked the planned number of cycles
            pitch_now = abs(math.degrees(latest_pitch[0]))   # current pitch magnitude, in degrees
            roll_now = abs(math.degrees(latest_roll[0]))   # current roll magnitude, in degrees
            settled = pitch_now <= GAIT_EXIT_ANGLE_SETTLE_DEG and roll_now <= GAIT_EXIT_ANGLE_SETTLE_DEG   # true once tilt is calm enough to exit
            if settled or elapsed >= total_duration + MAX_EXTRA_GAIT_TIME:   # exit once calm enough, or we've waited too long past the cutoff
                break
        cur_leg = last_active_leg[0]   # which leg is swinging right now
        if cur_leg != printed_leg[0]:   # only print when the swinging leg actually changes
            if cur_leg is not None:   # don't print anything during an all-feet-down window
                print(f"  stepping {cur_leg}...")
            printed_leg[0] = cur_leg
        if tick % 25 == 0:   # ~2x/sec at CONTROL_DT=0.02
            diagnostic_line(f"gait t={elapsed:.2f}", active_leg=cur_leg)
        tick += 1
        time.sleep(CONTROL_DT)

    if not aborted[0]:   # only ease out of the gait if nothing has aborted
        gait_active[0] = False
        print("--- continuous gait complete, returning to a stable stance ---")
        ease_out_of_gait()

end_xyz = list(body_xyz)   # body position at the end of the run
dx = end_xyz[0] - start_xyz[0]   # how far the body moved in x
dy = end_xyz[1] - start_xyz[1]   # how far the body moved in y
dist = math.sqrt(dx*dx + dy*dy)   # total straight-line distance traveled
dyaw_deg = math.degrees(math.atan2(math.sin(latest_yaw[0] - start_yaw), math.cos(latest_yaw[0] - start_yaw)))   # net yaw turned, wrapped to +-180 deg
print(f"net displacement: dx={dx:.3f} dy={dy:.3f}  total distance={dist:.3f} m  net yaw turned={dyaw_deg:+.1f} deg")
print(f"final body z: {end_xyz[2]:.3f}  (collapsed if well below ~0.35)")
END_WAIT_MIN, END_WAIT_MAX = 2.0, 6.0   # shortest/longest wait at the very end, in seconds
_end_wait_start = time.time()   # when the final wait began
while True:   # loop until the body settles or we give up waiting
    if check_abort():   # stop waiting early if a safety abort fired
        break
    diagnostic_line("end of loop")
    _elapsed_end = time.time() - _end_wait_start   # how long we've waited at the end
    _speed_now = math.hypot(body_vel[0], body_vel[1])   # current horizontal body speed
    if (_elapsed_end >= END_WAIT_MIN and _speed_now <= SPEED_SETTLE_THRESHOLD) or _elapsed_end >= END_WAIT_MAX:   # done once stopped for the minimum time, or waited too long
        break
    time.sleep(0.1)

print("sequence stopped early (safety abort)" if aborted[0] else "sequence complete")
running[0] = False
