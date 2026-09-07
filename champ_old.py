"""
Script: champ_old.py
Author: Arda Gencer
Older/legacy version of the gait controller, kept for reference.
Detailed tuning-history notes for the constants below are in tuning_history/champ_old_history.txt
"""

import gz.transport13 as transport
from gz.msgs10.double_pb2 import Double
from gz.msgs10.imu_pb2 import IMU
from gz.msgs10.pose_v_pb2 import Pose_V
import math
import threading
import time
import sys

# mirror print() output into a log file too, so we can read it back without copy/pasting.
# always the same filename, gets overwritten each run
_log_file = open("run_log.txt", "w")

class _Tee:
    def __init__(self, *streams):   # streams = output streams to mirror to
        self.streams = streams   # remember the streams to write to
    def write(self, data):
        for s in self.streams:   # loop over each output stream
            s.write(data)
            s.flush()
    def flush(self):
        for s in self.streams:   # loop over each output stream
            s.flush()

sys.stdout = _Tee(sys.stdout, _log_file)   # send stdout to both console and log file

L1 = 0.2   # length of the leg's upper segment, in meters
L2 = 0.2   # length of the leg's lower segment, in meters

def leg_ik(fx, fz, s):
    """fx, fz = target foot position relative to the hip, body frame (negative fz = below hip).
       s = +1 for front legs, -1 for back legs. Returns (hip, knee) angles."""
    u = s * fx   # foot x offset, mirrored for left/right side
    w = fz   # foot z offset (height)
    r2 = u*u + w*w   # squared distance from hip to foot
    r2 = max(r2, 1e-9)   # avoid divide by zero later
    c = (r2 - L1*L1 - L2*L2) / (2*L1*L2)   # cosine of knee angle, law of cosines
    c = max(-1.0, min(1.0, c))   # clamp to valid range for acos
    knee = -math.acos(c)   # knee joint angle
    k1 = L1 + L2*math.cos(knee)   # helper term for hip angle calc
    k2 = L2*math.sin(knee)   # helper term for hip angle calc
    sin_a = (u*k1 + k2*w) / r2   # sine component of hip angle
    cos_a = (k2*u - k1*w) / r2   # cosine component of hip angle
    hip = math.atan2(sin_a, cos_a)   # hip joint angle
    return hip, knee

def rotate(fx, fz, theta):
    """Rotates a foot-target vector by theta (same sign convention as the leg's swing rotation)."""
    return (fx*math.cos(theta) - fz*math.sin(theta),
            fx*math.sin(theta) + fz*math.cos(theta))

LEG_SIDE = {"FL": 1, "FR": 1, "BL": -1, "BR": -1}   # +1 for front legs, -1 for back legs
legs = ["FL", "FR", "BL", "BR"]   # the four leg names

node = transport.Node()   # gz-transport node for pub/sub
pubs = {}   # joint command publishers, keyed by name
for leg in legs:   # set up publishers for each leg
    pubs[f"{leg}_ABAD"] = node.advertise(f"/model/my_quadruped/joint/{leg}_ABAD/cmd_pos", Double)   # publisher for this leg's ABAD joint
    pubs[f"{leg}_HIP"] = node.advertise(f"/model/my_quadruped/joint/{leg}_HIP/cmd_pos", Double)   # publisher for this leg's hip joint
    pubs[f"{leg}_KNEE"] = node.advertise(f"/model/my_quadruped/joint/{leg}_KNEE/cmd_pos", Double)   # publisher for this leg's knee joint

# IMU: orientation (pitch/roll) plus pitch rate, for the D-term further down
latest_pitch = [0.0]   # most recent pitch reading, radians
latest_roll = [0.0]   # most recent roll reading, radians
latest_yaw = [0.0]   # not used for correction, just here to help debug drift/turning issues

PITCH_RATE_LPF_ALPHA = 0.2   # low-pass filter coefficient (0-1), lower = smoother
latest_pitch_rate = [0.0]   # filtered pitch rate, rad/s
latest_pitch_rate_raw = [0.0]   # unfiltered pitch rate, rad/s
_pitch_rate_source = [None]     # "gyro" or "fd"
_prev_pitch_for_rate = [None]   # last pitch value, for finite-diff rate
_prev_pitch_rate_time = [None]   # timestamp of that last pitch value
_dumped_imu_fields = [False]   # flag so we only log IMU fields once

def imu_callback(msg):
    if not _dumped_imu_fields[0]:   # only log the fields the first time we see one
        _dumped_imu_fields[0] = True   # mark that we've logged the fields
        try:
            print(f"DEBUG: IMU message fields: {[f.name for f in msg.DESCRIPTOR.fields]}")
        except Exception as e:   # e = whatever error introspection raised
            print(f"DEBUG: could not introspect IMU message fields: {e}")

    q = msg.orientation   # the IMU's orientation quaternion
    sinp = max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))   # sine of pitch, clamped for asin
    pitch = math.asin(sinp)   # pitch angle from quaternion
    latest_pitch[0] = pitch   # store the new pitch reading
    latest_roll[0] = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))   # compute and store roll angle
    latest_yaw[0] = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))   # compute and store yaw angle

    raw_rate = None   # pitch rate for this update, if any
    try:
        raw_rate = msg.angular_velocity.y   # gyro-reported pitch rate
        _pitch_rate_source[0] = "gyro"   # remember we're using the gyro
    except AttributeError:
        _pitch_rate_source[0] = "fd"   # remember we're using finite differences
        now = time.time()   # current time for rate calc
        if _prev_pitch_for_rate[0] is not None and _prev_pitch_rate_time[0] is not None:   # checks if we have a previous pitch reading to diff against
            dt = now - _prev_pitch_rate_time[0]   # time since last pitch sample
            if dt > 1e-4:   # only compute a rate if enough time has actually passed
                raw_rate = (pitch - _prev_pitch_for_rate[0]) / dt   # estimate rate from change in pitch
        _prev_pitch_for_rate[0] = pitch   # save pitch for next time's diff
        _prev_pitch_rate_time[0] = now   # save timestamp for next time's diff

    if raw_rate is not None:   # checks if we actually got a pitch rate this update
        latest_pitch_rate_raw[0] = raw_rate   # store the unfiltered rate
        latest_pitch_rate[0] = (PITCH_RATE_LPF_ALPHA * raw_rate   # low-pass filter the raw pitch rate
                                 + (1.0 - PITCH_RATE_LPF_ALPHA) * latest_pitch_rate[0])

node.subscribe(IMU, "/model/my_quadruped/imu", imu_callback)

# ground-truth body + per-foot world-frame telemetry, same as crawlgait.py
body_xyz = [None, None, None]   # body position in world frame
body_vel = [0.0, 0.0, 0.0]   # body velocity in world frame
body_accel = [0.0, 0.0, 0.0]   # body acceleration in world frame
_prev_body_xyz = [None, None, None]   # body position from the last pose update
_prev_body_vel = [None, None, None]   # body velocity from the last pose update
_prev_pose_time = [None]   # timestamp of the last pose update

link_xyz = {}   # each leg's shank position, body frame
_dumped_pose_names = [False]   # flag so we only log pose names once

def _rotate_body_to_world(dx, dy, dz, pitch, roll):
    """Rotates a vector from body frame into world frame. Assumes yaw=0 - no yaw sensor,
       and this gait never yaws anyway."""
    cp, sp = math.cos(pitch), math.sin(pitch)   # cos/sin of pitch for rotation
    cr, sr = math.cos(roll), math.sin(roll)   # cos/sin of roll for rotation
    wx = cp*dx + sp*sr*dy + sp*cr*dz   # rotated x component
    wy = cr*dy - sr*dz   # rotated y component
    wz = -sp*dx + cp*sr*dy + cp*cr*dz   # rotated z component
    return wx, wy, wz

foot_world_xyz = {}   # each foot's measured position, world frame
foot_world_vel = {leg: [0.0, 0.0, 0.0] for leg in legs}   # each foot's measured velocity, world frame
_prev_foot_world_xyz = {}   # each foot's last world position
_prev_foot_world_time = {}   # timestamp of each foot's last update

def _update_foot_world_positions(now):
    if body_xyz[0] is None:   # bail out if we don't know the body's position yet
        return
    pitch, roll = latest_pitch[0], latest_roll[0]   # current body tilt
    for leg_name, (lx, ly, lz) in link_xyz.items():   # loop over each leg's local foot offset
        wx_off, wy_off, wz_off = _rotate_body_to_world(lx, ly, lz, pitch, roll)   # foot offset rotated into world frame
        wx, wy, wz = body_xyz[0] + wx_off, body_xyz[1] + wy_off, body_xyz[2] + wz_off   # foot position in world frame
        prev = _prev_foot_world_xyz.get(leg_name)   # this foot's previous world position
        prev_t = _prev_foot_world_time.get(leg_name)   # timestamp of that previous position
        if prev is not None and prev_t is not None:   # checks if there's a previous position to compare against
            dt = now - prev_t   # time since that previous position
            if dt > 1e-4:   # only compute velocity if enough time has passed
                foot_world_vel[leg_name][0] = (wx - prev[0]) / dt   # foot's x velocity
                foot_world_vel[leg_name][1] = (wy - prev[1]) / dt   # foot's y velocity
                foot_world_vel[leg_name][2] = (wz - prev[2]) / dt   # foot's z velocity
        foot_world_xyz[leg_name] = (wx, wy, wz)   # save this foot's new world position
        _prev_foot_world_xyz[leg_name] = (wx, wy, wz)   # remember position for next velocity calc
        _prev_foot_world_time[leg_name] = now   # remember timestamp for next velocity calc

CMD_HIP_OFFSET = {"FL": (0.15, 0.213), "FR": (0.15, -0.213), "BL": (-0.15, 0.213), "BR": (-0.15, -0.213)}   # hip (x, y) offset from body center, meters

cmd_foot_world_xyz = {}   # each foot's commanded position, world frame
cmd_foot_world_vel = {leg: [0.0, 0.0, 0.0] for leg in legs}   # each foot's commanded velocity, world frame
_prev_cmd_foot_world_xyz = {}   # each foot's last commanded position
_prev_cmd_foot_world_time = {}   # timestamp of each foot's last command

def _update_commanded_foot_world(leg, fx_c, fz_c, now):   # fx_c/fz_c = commanded foot x/z for this leg
    if body_xyz[0] is None:   # bail out if we don't know the body's position yet
        return
    x0, y0 = CMD_HIP_OFFSET[leg]   # this leg's hip offset from body center
    pitch, roll = latest_pitch[0], latest_roll[0]   # current body tilt
    wx_off, wy_off, wz_off = _rotate_body_to_world(x0 + fx_c, y0, fz_c, pitch, roll)   # commanded foot offset in world frame
    wx, wy, wz = body_xyz[0] + wx_off, body_xyz[1] + wy_off, body_xyz[2] + wz_off   # commanded foot position in world frame
    prev = _prev_cmd_foot_world_xyz.get(leg)   # this foot's previous commanded position
    prev_t = _prev_cmd_foot_world_time.get(leg)   # timestamp of that previous command
    if prev is not None and prev_t is not None:   # checks if there's a previous command to compare against
        dt = now - prev_t   # time since that previous command
        if dt > 1e-4:   # only compute velocity if enough time has passed
            cmd_foot_world_vel[leg][0] = (wx - prev[0]) / dt   # commanded x velocity
            cmd_foot_world_vel[leg][1] = (wy - prev[1]) / dt   # commanded y velocity
            cmd_foot_world_vel[leg][2] = (wz - prev[2]) / dt   # commanded z velocity
    cmd_foot_world_xyz[leg] = (wx, wy, wz)   # save this foot's new commanded position
    _prev_cmd_foot_world_xyz[leg] = (wx, wy, wz)   # remember for next velocity calc
    _prev_cmd_foot_world_time[leg] = now   # remember timestamp for next velocity calc

def foot_tracking_error(active_leg):
    out = {}   # tracking error per leg, to return
    for l in legs:   # check every leg except the active one
        if l == active_leg:   # skip the leg that's currently swinging
            continue
        if l in cmd_foot_world_xyz and l in foot_world_xyz:   # only if we've got both commanded and actual data for this leg
            cx, cy, cz = cmd_foot_world_xyz[l]   # this leg's commanded position
            ax, ay, az = foot_world_xyz[l]   # this leg's actual position
            pos_err = math.sqrt((cx - ax) ** 2 + (cy - ay) ** 2 + (cz - az) ** 2)   # distance between commanded and actual
            cvx, cvy, _cvz = cmd_foot_world_vel[l]   # this leg's commanded velocity
            avx, avy, _avz = foot_world_vel[l]   # this leg's actual velocity
            vel_err = math.hypot(cvx - avx, cvy - avy)   # velocity mismatch, commanded vs actual
            out[l] = (pos_err, vel_err)   # record this leg's tracking error
    return out

def all_foot_world_z():
    """{leg: z} measured world-frame height of each leg's shank-origin point (foot_world_xyz).
       Not the actual foot tip (see the frame note above), but it's a fixed, consistent offset
       from it for a given leg orientation - so relative motion (does it rise during swing?) is
       trustworthy even though the absolute value isn't real ground-contact height."""
    return {l: foot_world_xyz[l][2] for l in legs if l in foot_world_xyz}

def pose_callback(msg):
    if not _dumped_pose_names[0]:   # only log the pose entity names the first time
        _dumped_pose_names[0] = True   # mark that we've logged the pose names
        print(f"DEBUG: pose entity names seen: {sorted(set(p.name for p in msg.pose))}")
    for p in msg.pose:   # loop over every entity in the pose message
        if p.name == "my_quadruped":   # checks if this pose entry is the robot's main body
            now = time.time()   # timestamp of this pose update
            if _prev_pose_time[0] is not None:   # checks if we have a previous timestamp to measure from
                dt = now - _prev_pose_time[0]   # time since the last pose update
                if dt > 1e-4:   # only compute velocity if enough time has passed
                    new_vx = (p.position.x - _prev_body_xyz[0]) / dt   # body x velocity from position change
                    new_vy = (p.position.y - _prev_body_xyz[1]) / dt   # body y velocity from position change
                    new_vz = (p.position.z - _prev_body_xyz[2]) / dt   # body z velocity from position change
                    if _prev_body_vel[0] is not None:   # checks if we have a previous velocity to diff against
                        body_accel[0] = (new_vx - _prev_body_vel[0]) / dt   # body x acceleration
                        body_accel[1] = (new_vy - _prev_body_vel[1]) / dt   # body y acceleration
                        body_accel[2] = (new_vz - _prev_body_vel[2]) / dt   # body z acceleration
                    _prev_body_vel[0] = new_vx   # remember velocity for next accel calc
                    _prev_body_vel[1] = new_vy   # remember velocity for next accel calc
                    _prev_body_vel[2] = new_vz   # remember velocity for next accel calc
                    body_vel[0] = new_vx   # store the new x velocity
                    body_vel[1] = new_vy   # store the new y velocity
                    body_vel[2] = new_vz   # store the new z velocity
            _prev_body_xyz[0] = p.position.x   # remember position for next velocity calc
            _prev_body_xyz[1] = p.position.y   # remember position for next velocity calc
            _prev_body_xyz[2] = p.position.z   # remember position for next velocity calc
            _prev_pose_time[0] = now   # remember timestamp for next update

            body_xyz[0] = p.position.x   # store the new body x position
            body_xyz[1] = p.position.y   # store the new body y position
            body_xyz[2] = p.position.z   # store the new body z position
        else:
            for leg_name in legs:   # find which leg this shank belongs to
                if p.name.endswith(f"{leg_name}_shank"):   # checks if this pose entry belongs to this leg's shank
                    link_xyz[leg_name] = (p.position.x, p.position.y, p.position.z)   # store this leg's shank position
                    break
    _update_foot_world_positions(time.time())

node.subscribe(Pose_V, "/world/empty/pose/info", pose_callback)

# stability checks against the support polygon, same math as crawlgait.py
def _sign2d(p1, p2, p3):
    return (p1[0]-p3[0])*(p2[1]-p3[1]) - (p2[0]-p3[0])*(p1[1]-p3[1])

def _point_in_triangle(pt, v1, v2, v3):
    d1 = _sign2d(pt, v1, v2)   # which side of edge v1-v2 the point is on
    d2 = _sign2d(pt, v2, v3)   # which side of edge v2-v3 the point is on
    d3 = _sign2d(pt, v3, v1)   # which side of edge v3-v1 the point is on
    has_neg = (d1 < 0) or (d2 < 0) or (d3 < 0)   # point is outside at least one edge
    has_pos = (d1 > 0) or (d2 > 0) or (d3 > 0)   # point is inside at least one edge
    return not (has_neg and has_pos)

def _polygon_check(point_xy, active_leg):
    """(inside, margin) for point_xy against the triangle formed by the three legs that
       aren't active_leg. Returns (None, None) if we haven't seen one of those feet yet."""
    support_legs = [l for l in legs if l != active_leg]   # the three planted legs
    if any(l not in foot_world_xyz for l in support_legs):   # bail out if we haven't seen all three support legs yet
        return None, None
    pts = [(foot_world_xyz[l][0], foot_world_xyz[l][1]) for l in support_legs]   # xy positions of the support legs
    inside = _point_in_triangle(point_xy, pts[0], pts[1], pts[2])   # is the point inside the support triangle
    def edge_dist(a, b):
        ex, ey = b[0]-a[0], b[1]-a[1]   # edge vector from a to b
        edge_len = math.hypot(ex, ey)   # length of that edge
        if edge_len < 1e-6:   # avoid dividing by zero on a degenerate edge
            return 0.0
        cross = (point_xy[0]-a[0])*ey - (point_xy[1]-a[1])*ex   # signed distance of point from the edge
        return cross / edge_len
    margins = [edge_dist(pts[0], pts[1]), edge_dist(pts[1], pts[2]), edge_dist(pts[2], pts[0])]   # distance from each triangle edge
    return inside, min(abs(m) for m in margins)

def support_status(active_leg):
    """Static CoM-vs-support-triangle check. Weak, quasi-static only - kept around to compare
       against ZMP/capture-point, not as the main stability signal."""
    if body_xyz[0] is None:   # bail out if we don't know the body's position yet
        return None, None
    return _polygon_check((body_xyz[0], body_xyz[1]), active_leg)

G = 9.8   # gravity, in meters per second squared

def compute_zmp():
    """x_zmp = x_com - (xdd/(zdd+g))*z_com. Drops the angular-momentum-rate term - a fine
       first-order approximation since body mass is ~15x leg mass on this robot."""
    if body_xyz[0] is None:   # bail out if we don't know the body's position yet
        return None
    z_com = body_xyz[2]   # center of mass height
    xdd, ydd, zdd = body_accel   # body acceleration components
    denom = zdd + G   # denominator for the ZMP formula
    if abs(denom) < 1.0:   # avoid dividing by a near-zero denominator
        denom = G   # fall back to gravity if near zero
    return (body_xyz[0] - (xdd/denom)*z_com, body_xyz[1] - (ydd/denom)*z_com)

def zmp_status(active_leg):
    zmp = compute_zmp()   # zero moment point, if we can compute it
    if zmp is None:   # bail out if the ZMP couldn't be computed
        return None, None, None
    inside, margin = _polygon_check(zmp, active_leg)   # is the ZMP inside the support triangle
    return inside, margin, zmp

CAPTURE_HEIGHT = 0.40   # assumed body height for the capture-point formula, meters
CAPTURE_GAIN = math.sqrt(CAPTURE_HEIGHT / 9.8)   # ~0.202

def compute_capture_point():
    """x_cp = x_com + vx*sqrt(z0/g) (Pratt et al.) - where the CoM would end up if velocity
       dropped to zero right now."""
    if body_xyz[0] is None:   # bail out if we don't know the body's position yet
        return None
    return (body_xyz[0] + body_vel[0]*CAPTURE_GAIN, body_xyz[1] + body_vel[1]*CAPTURE_GAIN)

def capture_point_status(active_leg):
    cp = compute_capture_point()   # capture point, if we can compute it
    if cp is None:   # bail out if the capture point couldn't be computed
        return None, None, None
    inside, margin = _polygon_check(cp, active_leg)   # is the capture point inside the support triangle
    return inside, margin, cp

def stance_foot_velocities(active_leg):
    """{leg: (vx, vy)} world-frame velocity of each currently planted (non-active) leg.
       A genuinely planted foot should read ~0; a sustained nonzero value means that leg's
       servos are slipping under load."""
    support_legs = [l for l in legs if l != active_leg]   # the three planted legs
    return {l: tuple(foot_world_vel.get(l, (0.0, 0.0, 0.0))[:2]) for l in support_legs}

def all_foot_fx():
    """{leg: current body-frame fx target} for all four legs."""
    return {l: foot_target[l][0] for l in legs}

# gait constants
STANCE_FZ = -0.384   # settled-crouch depth that worked in crawlgait.py


STEP_LENGTH_FRONT = 0.08   # how far the front feet step each stride, meters
STEP_LENGTH_BACK = 0.08   # how far the back feet step each stride, meters


SWING_HEIGHT_FRONT = 0.08   # how high the front foot lifts while swinging, meters
SWING_HEIGHT_BACK = 0.08   # how high the back foot lifts while swinging, meters

FX_LIMIT = 0.09   # hard clamp - both step lengths above stay comfortably inside this

def clamp_fx(v):
    return max(-FX_LIMIT, min(FX_LIMIT, v))


GAIT_PERIOD = 6.0     # seconds per full 4-leg cycle


SWING_DUTY = 0.15   # fraction of the gait cycle each leg spends swinging

GAIT_ORDER = ["BR", "FL", "FR", "BL"]   # order the legs take turns swinging in
LEG_PHASE_OFFSET = {leg: i / 4.0 for i, leg in enumerate(GAIT_ORDER)}   # each leg's start point in the cycle
N_CYCLES = 3


SHIFT_MAG_FRONT = 0.03   # how far to shift the body when a front leg swings, meters
SHIFT_MAG_BACK = 0.03   # how far to shift the body when a back leg swings, meters
SHIFT_RAMP_FRAC = 0.1   # fraction of the swing spent easing the shift in and out

def _shift_envelope(swing_frac):
    """0 at liftoff/touchdown, 1 through the middle ~60% of the swing - a trapezoid shape so
       the bias is at full strength for nearly the whole time the leg's actually off the
       ground. A peaked/sine shape would be weakest exactly when it's needed most."""
    if swing_frac < SHIFT_RAMP_FRAC:   # still ramping the shift in near liftoff
        return _smoothstep(swing_frac / SHIFT_RAMP_FRAC)
    if swing_frac > 1.0 - SHIFT_RAMP_FRAC:   # ramping the shift back out near touchdown
        return _smoothstep((1.0 - swing_frac) / SHIFT_RAMP_FRAC)
    return 1.0

last_shift_bias = {"leg": None, "bias": 0.0}   # most recent body-shift bias, for logging

foot_target = {leg: (0.0, -0.4) for leg in legs}   # each leg's current target foot position
abad_cmd = {leg: 0.0 for leg in legs}   # each leg's commanded ABAD angle

PITCH_SIGN = 1.0   # flips pitch correction direction if it's backwards
CORRECTION_FRACTION = 0.4   # how much of the pitch error to correct each cycle
MAX_CORRECTION_RAD = 0.35   # largest pitch correction allowed, in radians
PITCH_RATE_DAMPING = 0.15    # uses the low-pass-filtered pitch rate now, not raw gyro

ROLL_ABAD_FRACTION = 0.3   # how much of the roll error to correct each cycle
MAX_ABAD_ROLL_CORR = 0.15   # largest roll correction allowed, in radians

LEG_LR = {"FL": 1, "FR": -1, "BL": 1, "BR": -1}   # +1 for left legs, -1 for right legs
YAW_FX_GAIN = 0.03     # rad of accumulated yaw error -> meters of fx differential
MAX_YAW_FX = 0.025   # largest fx nudge allowed from yaw correction, meters
gait_start_yaw = [None]   # yaw reading when the gait started

running = [True]   # controls whether the control loop keeps going

FLIP_LIMIT_DEG = 25.0   # pitch/roll angle that triggers a safety abort, degrees
FLIP_LIMIT_RAD = math.radians(FLIP_LIMIT_DEG)   # flip limit converted to radians
aborted = [False]   # flag set when a safety abort triggers
abort_freeze_t = [None]   # gait-clock time, frozen the moment a safety abort happens

gait_active = [False]   # whether the continuous gait is currently running
gait_start_time = [None]   # time when the gait clock started

def leg_phase_fracs(leg, t):
    global_phase = (t % GAIT_PERIOD) / GAIT_PERIOD   # 0-1 position within the gait cycle
    return (global_phase - LEG_PHASE_OFFSET[leg]) % 1.0

def current_swing_leg(t):
    for leg in legs:   # find whichever leg is currently swinging
        if leg_phase_fracs(leg, t) < SWING_DUTY:   # checks if this leg is inside its swing window right now
            return leg
    return None   # shouldn't happen when SWING_DUTY*4==1.0, just being defensive

def _smoothstep(x):   # x = progress fraction, 0 to 1
    x = max(0.0, min(1.0, x))   # clamp input to 0-1 range
    return x * x * (3 - 2 * x)

def foot_offset_for_leg(leg, t):
    is_front = leg in ("FL", "FR")   # true for front legs
    step = STEP_LENGTH_FRONT if is_front else STEP_LENGTH_BACK   # stride length for this leg
    height = SWING_HEIGHT_FRONT if is_front else SWING_HEIGHT_BACK   # swing height for this leg
    local_phase = leg_phase_fracs(leg, t)   # this leg's position in its own cycle
    if local_phase < SWING_DUTY:   # checks if the leg is currently swinging rather than planted
        swing_frac = local_phase / SWING_DUTY   # progress through the swing phase, 0-1
        s = _smoothstep(swing_frac)   # eased swing progress
        fx = -step / 2.0 + step * s   # foot x target during swing
        fz = STANCE_FZ + height * math.sin(math.pi * swing_frac)   # foot z target, arcing up during swing
    else:
        stance_frac = (local_phase - SWING_DUTY) / (1.0 - SWING_DUTY)   # progress through the stance phase, 0-1
        fx = (step / 2.0) - step * stance_frac   # foot x target during stance, sliding back
        fz = STANCE_FZ   # foot stays at stance depth
    return clamp_fx(fx), fz

def check_abort():
    if aborted[0]:   # already aborted, nothing more to check
        return True
    if abs(latest_pitch[0]) > FLIP_LIMIT_RAD or abs(latest_roll[0]) > FLIP_LIMIT_RAD:   # checks if the robot has tipped past the safety limit
        aborted[0] = True   # trip the safety abort flag
        if gait_active[0] and gait_start_time[0] is not None:   # checks if the gait clock is actually running
            abort_freeze_t[0] = time.time() - gait_start_time[0]   # freeze the gait clock at this moment
        print(f"!!! SAFETY ABORT: |pitch|={math.degrees(latest_pitch[0]):.1f} deg  "
              f"|roll|={math.degrees(latest_roll[0]):.1f} deg exceeded {FLIP_LIMIT_DEG} deg - "
              f"freezing gait clock, joint commands keep publishing (clamped) !!!")
    return aborted[0]

CONTROL_DT = 0.02   # 50Hz, matches the sleep() at the bottom of this loop

last_theta_terms = {"p": 0.0, "d": 0.0, "raw": 0.0, "clamped": 0.0}   # pitch-correction terms, for logging
last_yaw_terms = {"err": 0.0, "fx_term": 0.0}   # yaw-correction terms, for logging

def control_loop():
    while running[0]:   # keep looping until told to stop
        check_abort()

        if gait_active[0]:   # checks if the continuous gait is currently engaged
            t = abort_freeze_t[0] if aborted[0] else (time.time() - gait_start_time[0])   # current time in the gait cycle
            for leg in legs:   # update every leg's foot target
                foot_target[leg] = foot_offset_for_leg(leg, t)   # this leg's target for the current time

            active_leg = current_swing_leg(t)   # whichever leg is swinging right now
            bias = 0.0   # body-shift bias, default none
            if active_leg is not None:   # checks if some leg is currently swinging
                swing_frac = leg_phase_fracs(active_leg, t) / SWING_DUTY   # progress through the swing, 0-1
                is_front = active_leg in ("FL", "FR")   # true if the swinging leg is a front leg
                mag = SHIFT_MAG_FRONT if is_front else SHIFT_MAG_BACK   # how far to shift the body
                sign = 1.0 if is_front else -1.0   # shift direction, front vs back
                bias = sign * mag * _shift_envelope(swing_frac)   # final body-shift amount this instant
            last_shift_bias["leg"] = active_leg   # record which leg is being shifted for
            last_shift_bias["bias"] = bias   # record the shift amount
            if active_leg is not None:   # checks if some leg is currently swinging
                for leg in legs:   # apply the shift bias to the stance legs
                    if leg == active_leg:   # skip the leg that's already being handled as the active one
                        continue
                    fx, fz = foot_target[leg]   # this leg's current foot target
                    foot_target[leg] = (clamp_fx(fx + bias), fz)   # shift this stance leg to help balance
        # else: move_feet_manual() is driving foot_target directly (crouch / pre-gait ramp)

        theta_p = PITCH_SIGN * CORRECTION_FRACTION * latest_pitch[0]   # proportional pitch-correction term
        theta_d = PITCH_SIGN * PITCH_RATE_DAMPING * latest_pitch_rate[0]   # derivative (damping) pitch-correction term
        theta_raw = theta_p + theta_d   # combined pitch correction, unclamped
        theta = max(-MAX_CORRECTION_RAD, min(MAX_CORRECTION_RAD, theta_raw))   # clamped pitch correction to apply
        last_theta_terms["p"] = theta_p   # record proportional term for logging
        last_theta_terms["d"] = theta_d   # record derivative term for logging
        last_theta_terms["raw"] = theta_raw   # record unclamped correction for logging
        last_theta_terms["clamped"] = theta   # record clamped correction for logging

        roll_term = ROLL_ABAD_FRACTION * latest_roll[0]   # roll-correction term for the ABAD joints
        roll_term = max(-MAX_ABAD_ROLL_CORR, min(MAX_ABAD_ROLL_CORR, roll_term))   # clamp the roll correction

        if gait_active[0] and gait_start_yaw[0] is not None:   # checks if we're mid-gait with a yaw baseline set
            yaw_err = latest_yaw[0] - gait_start_yaw[0]   # how far we've drifted in yaw since gait start
        else:
            yaw_err = 0.0   # no yaw correction outside continuous gait
        yaw_fx_term = max(-MAX_YAW_FX, min(MAX_YAW_FX, YAW_FX_GAIN * yaw_err))   # fx nudge to correct yaw drift
        last_yaw_terms["err"] = yaw_err   # record yaw error for logging
        last_yaw_terms["fx_term"] = yaw_fx_term   # record fx correction for logging

        now = time.time()   # timestamp for this control tick
        for leg in legs:   # compute and send commands for each leg
            fx, fz = foot_target[leg]   # this leg's current foot target
            fx = clamp_fx(fx + LEG_LR[leg] * yaw_fx_term)   # apply yaw correction, left/right mirrored
            # only apply pitch correction (theta) to stance legs
            if gait_active[0] and leg == current_swing_leg(t):   # true only for the leg currently in swing
                fx_c, fz_c = fx, fz   # swing leg gets no pitch correction
            else:
                fx_c, fz_c = rotate(fx, fz, theta)   # stance leg gets rotated for pitch correction
            hip, knee = leg_ik(fx_c, fz_c, LEG_SIDE[leg])   # solve joint angles for this foot target
            _update_commanded_foot_world(leg, fx_c, fz_c, now)

            abad = abad_cmd[leg] + roll_term   # final ABAD angle with roll correction

            m0 = Double(); m0.data = abad   # message carrying the ABAD command
            pubs[f"{leg}_ABAD"].publish(m0)
            m1 = Double(); m1.data = hip   # message carrying the hip command
            pubs[f"{leg}_HIP"].publish(m1)
            m2 = Double(); m2.data = knee   # message carrying the knee command
            pubs[f"{leg}_KNEE"].publish(m2)
        time.sleep(CONTROL_DT)

t = threading.Thread(target=control_loop, daemon=True)   # background thread running the control loop
t.start()

def diagnostic_line(label, active_leg=None):
    vx, vy, vz = body_vel   # current body velocity components
    ax, ay, az = body_accel   # current body acceleration components
    speed = math.hypot(vx, vy)   # horizontal speed
    line = (f"  [{label}] pitch: {math.degrees(latest_pitch[0]):+.2f} deg  "   # start building the status line
            f"roll: {math.degrees(latest_roll[0]):+.2f} deg  yaw: {math.degrees(latest_yaw[0]):+.2f} deg  "
            f"body_xyz: {body_xyz}  "
            f"vel(x,y,z): ({vx:+.3f},{vy:+.3f},{vz:+.3f})  speed_xy: {speed:.3f}  "
            f"accel(x,y,z): ({ax:+.3f},{ay:+.3f},{az:+.3f})  "
            f"pitch_rate({_pitch_rate_source[0]} raw={math.degrees(latest_pitch_rate_raw[0]):+.2f} "
            f"filt={math.degrees(latest_pitch_rate[0]):+.2f} deg/s)  "
            f"theta(p={math.degrees(last_theta_terms['p']):+.2f} d={math.degrees(last_theta_terms['d']):+.2f} "
            f"-> {math.degrees(last_theta_terms['clamped']):+.2f} deg)  "
            f"shift_bias(leg={last_shift_bias['leg']} val={last_shift_bias['bias']:+.4f})  "
            f"yaw_corr(err={math.degrees(last_yaw_terms['err']):+.2f}deg fx_term={last_yaw_terms['fx_term']:+.4f})")
    if active_leg is not None:   # only show these stability stats while a leg is swinging
        inside, margin = support_status(active_leg)   # CoM-vs-support-triangle check
        if inside is not None:   # only append if the CoM check actually produced a result
            line += f"  CoM: inside={inside} margin={margin:+.4f}"
        zmp_in, zmp_margin, _zmp_pt = zmp_status(active_leg)   # ZMP-vs-support-triangle check
        if zmp_in is not None:   # only append if the ZMP check actually produced a result
            line += f"  ZMP: inside={zmp_in} margin={zmp_margin:+.4f}"
        cp_in, cp_margin, _cp_pt = capture_point_status(active_leg)   # capture-point-vs-support-triangle check
        if cp_in is not None:   # only append if the capture-point check actually produced a result
            line += f"  CapturePt: inside={cp_in} margin={cp_margin:+.4f}"
        vels = stance_foot_velocities(active_leg)   # velocity of each planted foot
        line += "  stance_foot_vxy: {" + " ".join(f"{l}:({vx2:+.3f},{vy2:+.3f})" for l, (vx2, vy2) in vels.items()) + "}"
    track = foot_tracking_error(active_leg)   # how well each foot tracks its target
    if track:   # only append if there's tracking data to show
        line += "  velerr: {" + " ".join(f"{l}:{ve:.3f}" for l, (_pe, ve) in track.items()) + "}"
    foot_z = all_foot_world_z()   # world-frame height of each foot
    if foot_z:   # only append if there's foot height data to show
        marker = lambda l: "*" if l == active_leg else ""   # flags the active leg in the printed line
        line += "  foot_z: {" + " ".join(f"{l}{marker(l)}:{z:+.3f}" for l, z in foot_z.items()) + "}"
    fx_all = all_foot_fx()   # current fx target of each leg
    line += "  fx: {" + " ".join(f"{l}:{v:+.4f}" for l, v in fx_all.items()) + "}"
    print(line)

def move_feet_manual(deltas, duration=1.5, steps=75, label=None):   # deltas = per-leg (x, z) targets to move to
    starts = {leg: foot_target[leg] for leg in deltas}   # each leg's starting foot position
    print_every = max(1, steps // 10)   # how often to print progress
    for i in range(1, steps + 1):   # step counter through the move
        if check_abort():   # stop the move early if a safety abort triggered
            return
        frac = i / steps   # fraction of the move completed
        for leg, (tx, tz) in deltas.items():   # target x/z for each leg being moved
            sx, sz = starts[leg]   # this leg's starting x/z
            foot_target[leg] = (sx + (tx - sx) * frac, sz + (tz - sz) * frac)   # interpolate toward the target
        if label and (i % print_every == 0 or i == 1):   # checks if it's time to print a progress update
            diagnostic_line(f"{label} {frac*100:3.0f}%")
        time.sleep(duration / steps)
    for leg, tgt in deltas.items():   # snap each leg to its final target
        foot_target[leg] = tgt   # set the exact final position

print("waiting for the drop to settle...")
time.sleep(5.0)
print(f"landed, pitch = {math.degrees(latest_pitch[0]):.2f} deg  roll = {math.degrees(latest_roll[0]):.2f} deg")
print(f"body position: {body_xyz}")

print("--- crouch (hip/knee only, all ABAD at 0) ---")
move_feet_manual({leg: (0.0, STANCE_FZ) for leg in legs}, duration=1.5, steps=75, label="crouch")
for _ in range(20):   # settle for a couple seconds, printing status
    if check_abort():   # stop early if a safety abort triggered
        break
    diagnostic_line("crouch settle")
    time.sleep(0.1)

start_xyz = list(body_xyz)   # body position at the start of the gait
start_yaw = latest_yaw[0]   # yaw at the start of the gait

# ramp from the flat crouch pose to the gait clock's own t=0 targets, so turning on the
# continuous gait doesn't start with a sudden, uncommanded jump in every foot's target
if not aborted[0]:   # only ramp in if the run hasn't been aborted
    print("--- pre-gait ramp (to phase-clock t=0 pose) ---")
    t0_targets = {leg: foot_offset_for_leg(leg, 0.0) for leg in legs}   # each leg's target at gait time zero
    move_feet_manual(t0_targets, duration=1.5, steps=75, label="pre-gait ramp")

if not aborted[0]:   # only start the gait if the run hasn't been aborted
    gait_start_time[0] = time.time()   # mark when the gait clock begins
    gait_start_yaw[0] = latest_yaw[0]   # baseline yaw for drift correction
    gait_active[0] = True   # switch the control loop into gait mode
    print(f"--- continuous gait engaged: period={GAIT_PERIOD}s  swing_duty={SWING_DUTY}  "
          f"order={GAIT_ORDER}  step_length(front/back)=({STEP_LENGTH_FRONT}/{STEP_LENGTH_BACK}) ---")

    total_duration = N_CYCLES * GAIT_PERIOD   # how long the full gait should run
    print_interval = 0.25   # how often to print status, seconds
    t = 0.0   # gait clock time, updated each loop
    while t < total_duration:   # keep going until the full gait duration has elapsed
        if check_abort():   # stop the gait early on a safety abort
            break
        t = time.time() - gait_start_time[0]   # elapsed time since gait started
        active_leg = current_swing_leg(t)   # whichever leg is swinging right now
        diagnostic_line(f"gait t={t:5.2f}s leg={active_leg}", active_leg=active_leg)
        time.sleep(print_interval)


    if not aborted[0]:   # only settle back down if the run wasn't aborted
        gait_active[0] = False   # switch the control loop out of gait mode
        print("--- gait clock stopped, returning to a stable stance before finishing ---")
        move_feet_manual({leg: (0.0, STANCE_FZ) for leg in legs}, duration=1.5, steps=75,
                          label="post-gait settle")

end_xyz = list(body_xyz)   # body position at the end of the run
dx = end_xyz[0] - start_xyz[0]   # net x displacement over the run
dy = end_xyz[1] - start_xyz[1]   # net y displacement over the run
dist = math.sqrt(dx*dx + dy*dy)   # total straight-line distance traveled
dyaw_deg = math.degrees(math.atan2(math.sin(latest_yaw[0] - start_yaw), math.cos(latest_yaw[0] - start_yaw)))   # net yaw turned, in degrees
print(f"net displacement: dx={dx:.3f} dy={dy:.3f}  total distance={dist:.3f} m  net yaw turned={dyaw_deg:+.1f} deg")
print(f"final body z: {end_xyz[2]:.3f}  (collapsed if well below ~0.35)")
for _ in range(20):   # settle for a couple seconds, printing status
    if check_abort():   # stop early if a safety abort triggered
        break
    diagnostic_line("end of loop")
    time.sleep(0.1)

print("sequence stopped early (safety abort)" if aborted[0] else "sequence complete")
running[0] = False   # tell the control loop thread to stop
