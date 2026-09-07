"""
Script: champgait.py
Author: Arda Gencer
Full-featured walk/wave gait controller with CoM shifting and ZMP/capture-point stability checks.
Detailed tuning-history notes for the constants below are in tuning_history/champgait_history.txt
"""

import gz.transport13 as transport
from gz.msgs10.double_pb2 import Double
from gz.msgs10.imu_pb2 import IMU
from gz.msgs10.pose_v_pb2 import Pose_V
import math
import threading
import time
import sys

# tee print() to a log file too, not just the console - same filename every run, overwritten
# each time, so I can read it back later without copy-pasting from the terminal
_log_file = open("run_log.txt", "w")   # log file, overwritten each run

class _Tee:
    def __init__(self, *streams):   # streams to duplicate writes to
        self.streams = streams   # save the streams for write/flush
    def write(self, data):   # text being written
        for s in self.streams:   # each output stream
            s.write(data)
            s.flush()
    def flush(self):
        for s in self.streams:   # each output stream
            s.flush()

sys.stdout = _Tee(sys.stdout, _log_file)   # redirect stdout through the tee

def log_line(text):   # text line to log
    print(text, file=_log_file)
    _log_file.flush()


L1 = 0.2   # length of the leg's upper segment, in meters
L2 = 0.2   # length of the leg's lower segment, in meters

def leg_ik(fx, fz, s):
    """fx, fz = desired foot spot relative to the hip, in body frame (fz negative = below the
       hip). s = +1 for front legs, -1 for back. Returns (hip, knee) angles."""
    u = s * fx   # fx flipped for back legs
    w = fz   # just fz, renamed for the formula
    r2 = u*u + w*w   # squared distance to the target
    r2 = max(r2, 1e-9)   # avoid divide-by-zero later
    c = (r2 - L1*L1 - L2*L2) / (2*L1*L2)   # law-of-cosines term for knee angle
    c = max(-1.0, min(1.0, c))   # clamp for acos safety
    knee = -math.acos(c)   # knee joint angle
    k1 = L1 + L2*math.cos(knee)   # helper term for hip angle
    k2 = L2*math.sin(knee)   # helper term for hip angle
    sin_a = (u*k1 + k2*w) / r2   # sine component of hip angle
    cos_a = (k2*u - k1*w) / r2   # cosine component of hip angle
    hip = math.atan2(sin_a, cos_a)   # hip joint angle
    return hip, knee

def rotate(fx, fz, theta):
    """Rotates a foot-target vector by theta (same convention the leg's swing rotation uses)."""
    return (fx*math.cos(theta) - fz*math.sin(theta),
            fx*math.sin(theta) + fz*math.cos(theta))

D_ABAD = 0.1   # fixed arm length from abad pivot to hip pivot

OY = {"FL": 1, "FR": -1, "BL": 1, "BR": -1}   # outward direction (left/right) for each leg

def leg_ik_3d(fx, fy, fz, oy, s):
    """fx, fy, fz = desired foot position relative to the leg's ABAD pivot, in true body-frame
       axes. fy=0 gives exactly the old 2-link-only target (foot at the nominal zero-abduction
       offset); fy != 0 tells the ABAD joint to lean the whole leg sideways by whatever angle
       puts the foot at that extra Y offset. oy = OY[leg] (+1 left, -1 right), s = LEG_SIDE[leg]
       (+1 front, -1 back, same convention as leg_ik). Returns (abad, hip, knee).
       How it's derived: the ABAD joint only rotates in the body's Y-Z plane (its axis is the
       body's own X axis - checked against model.sdf's joint definitions), so the HIP pivot has
       to sit somewhere on a circle of radius D_ABAD around the ABAD axis in that plane. Solving
       for where on that circle the rest of the leg can reach the target's Y-Z projection gives
       the abad angle, and the rest just falls back to the existing, unchanged 2-link leg_ik()
       in whichever plane that leaves."""
    dy = oy * D_ABAD + fy   # target y offset from the abad axis
    dz = fz   # target z, renamed for the formula
    r = math.hypot(dy, dz)   # distance from abad axis to target
    r = max(r, D_ABAD + 1e-6)   # can't be closer to the ABAD axis than the arm length itself -
                                # clamp instead of crashing on an unreachable target
    c = max(-1.0, min(1.0, (oy * D_ABAD) / r))   # clamped ratio for the angle solve
    base = math.atan2(dz, dy)   # base angle to the target
    phi_a = base + math.acos(c)   # one candidate abad angle
    phi_b = base - math.acos(c)   # other candidate abad angle
    abad = phi_a if abs(phi_a) < abs(phi_b) else phi_b   # whichever needs less abduction
    w = -dy*math.sin(abad) + dz*math.cos(abad)           # effective fz inside the tilted 2-link plane
    hip, knee = leg_ik(fx, w, s)   # solve the 2-link plane for hip/knee
    return abad, hip, knee

LEG_SIDE = {"FL": 1, "FR": 1, "BL": -1, "BR": -1}   # +1 front legs, -1 back legs, for the leg IK
legs = ["FL", "FR", "BL", "BR"]   # the four leg names: front/back, left/right

node = transport.Node()   # gz-transport node for pub/sub
pubs = {}   # publishers keyed by leg and joint name
for leg in legs:   # loop over each of the 4 legs
    pubs[f"{leg}_ABAD"] = node.advertise(f"/model/my_quadruped/joint/{leg}_ABAD/cmd_pos", Double)   # publisher for this leg's abad joint
    pubs[f"{leg}_HIP"] = node.advertise(f"/model/my_quadruped/joint/{leg}_HIP/cmd_pos", Double)   # publisher for this leg's hip joint
    pubs[f"{leg}_KNEE"] = node.advertise(f"/model/my_quadruped/joint/{leg}_KNEE/cmd_pos", Double)   # publisher for this leg's knee joint

# IMU: orientation (pitch/roll) plus pitch rate, which feeds the D-term below
latest_pitch = [0.0]   # most recent pitch reading, in radians
latest_roll = [0.0]   # most recent roll reading, in radians
latest_yaw = [0.0]

PITCH_RATE_LPF_ALPHA = 0.2   # low-pass filter coefficient, 0-1, lower = smoother
latest_pitch_rate = [0.0]   # filtered pitch rate used by the D-term
latest_pitch_rate_raw = [0.0]   # unfiltered pitch rate, for comparison
_pitch_rate_source = [None]     # "gyro" or "fd"
_prev_pitch_for_rate = [None]   # last pitch value, for finite difference
_prev_pitch_rate_time = [None]   # timestamp of that last pitch value
_dumped_imu_fields = [False]   # whether we've logged the IMU field names yet

YAW_RATE_LPF_ALPHA = 0.2   # low-pass filter coefficient, 0-1, lower = smoother
latest_yaw_rate = [0.0]   # filtered yaw rate used by the D-term
latest_yaw_rate_raw = [0.0]   # unfiltered yaw rate, for comparison
_yaw_rate_source = [None]   # "gyro" or "fd"
_prev_yaw_for_rate = [None]   # last yaw value, for finite difference
_prev_yaw_rate_time = [None]   # timestamp of that last yaw value

def imu_callback(msg):
    if not _dumped_imu_fields[0]:   # only log the imu field names once
        _dumped_imu_fields[0] = True   # mark that we've logged the fields once
        try:
            log_line(f"DEBUG: IMU message fields: {[f.name for f in msg.DESCRIPTOR.fields]}")   # dump the IMU message's field names once
        except Exception as e:   # couldn't introspect the message
            log_line(f"DEBUG: could not introspect IMU message fields: {e}")

    q = msg.orientation   # quaternion orientation from the IMU
    sinp = max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))   # sine of pitch, clamped for asin
    pitch = math.asin(sinp)   # pitch angle from the quaternion
    latest_pitch[0] = pitch   # store the new pitch reading
    latest_roll[0] = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))   # store the new roll reading
    latest_yaw[0] = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))   # store the new yaw reading

    raw_rate = None   # pitch rate before filtering
    try:
        raw_rate = msg.angular_velocity.y   # pitch rate straight from the gyro
        _pitch_rate_source[0] = "gyro"   # remember we're using the gyro
    except AttributeError:
        _pitch_rate_source[0] = "fd"   # remember we're using finite difference
        now = time.time()   # current time for the rate calc
        if _prev_pitch_for_rate[0] is not None and _prev_pitch_rate_time[0] is not None:   # only compute rate if we have a previous sample
            dt = now - _prev_pitch_rate_time[0]   # time since the last pitch sample
            if dt > 1e-4:   # only compute rate if enough time has passed
                raw_rate = (pitch - _prev_pitch_for_rate[0]) / dt   # pitch rate via finite difference
        _prev_pitch_for_rate[0] = pitch   # remember this pitch for next time
        _prev_pitch_rate_time[0] = now   # remember this timestamp for next time

    if raw_rate is not None:   # only update if we actually got a rate reading
        latest_pitch_rate_raw[0] = raw_rate   # store the unfiltered rate
        latest_pitch_rate[0] = (PITCH_RATE_LPF_ALPHA * raw_rate
                                 + (1.0 - PITCH_RATE_LPF_ALPHA) * latest_pitch_rate[0])   # blend into the low-pass filtered rate

    yaw_raw_rate = None   # yaw rate before filtering
    try:
        yaw_raw_rate = msg.angular_velocity.z   # yaw rate straight from the gyro
        _yaw_rate_source[0] = "gyro"   # remember we're using the gyro
    except AttributeError:
        _yaw_rate_source[0] = "fd"   # remember we're using finite difference
        now2 = time.time()   # current time for the yaw rate calc
        if _prev_yaw_for_rate[0] is not None and _prev_yaw_rate_time[0] is not None:   # only compute rate if we have a previous sample
            dt2 = now2 - _prev_yaw_rate_time[0]   # time since the last yaw sample
            if dt2 > 1e-4:   # only compute rate if enough time has passed
                yaw_raw_rate = (latest_yaw[0] - _prev_yaw_for_rate[0]) / dt2   # yaw rate via finite difference
        _prev_yaw_for_rate[0] = latest_yaw[0]   # remember this yaw for next time
        _prev_yaw_rate_time[0] = now2   # remember this timestamp for next time

    if yaw_raw_rate is not None:   # only update if we actually got a new rate value
        latest_yaw_rate_raw[0] = yaw_raw_rate   # store the unfiltered rate
        latest_yaw_rate[0] = (YAW_RATE_LPF_ALPHA * yaw_raw_rate
                               + (1.0 - YAW_RATE_LPF_ALPHA) * latest_yaw_rate[0])   # blend into the low-pass filtered rate

node.subscribe(IMU, "/model/my_quadruped/imu", imu_callback)

# ground-truth body + per-foot world-frame telemetry - unchanged from crawlgait.py
body_xyz = [None, None, None]   # measured body position, world frame
body_vel = [0.0, 0.0, 0.0]   # measured body velocity, world frame
body_accel = [0.0, 0.0, 0.0]   # measured body acceleration, world frame
_prev_body_xyz = [None, None, None]   # body position from the last sample
_prev_body_vel = [None, None, None]   # body velocity from the last sample
_prev_pose_time = [None]   # timestamp of the last pose sample

link_xyz = {}   # each leg's shank link position, body frame
_dumped_pose_names = [False]   # whether we've logged pose entity names yet

def _rotate_body_to_world(dx, dy, dz, pitch, roll):
    """Rotates a vector from the model's local/body frame into world frame (assumes yaw=0 - no
       yaw sensor here, and this gait never yaws)."""
    cp, sp = math.cos(pitch), math.sin(pitch)   # cos/sin of pitch for the rotation
    cr, sr = math.cos(roll), math.sin(roll)   # cos/sin of roll for the rotation
    wx = cp*dx + sp*sr*dy + sp*cr*dz   # rotated x component
    wy = cr*dy - sr*dz   # rotated y component
    wz = -sp*dx + cp*sr*dy + cp*cr*dz   # rotated z component
    return wx, wy, wz

foot_world_xyz = {}   # each foot's measured world position
foot_world_vel = {leg: [0.0, 0.0, 0.0] for leg in legs}   # each foot's measured world velocity
_prev_foot_world_xyz = {}   # each foot's world position last sample
_prev_foot_world_time = {}   # timestamp of each foot's last sample

def _update_foot_world_positions(now):
    if body_xyz[0] is None:   # skip until we have a body position reading
        return
    pitch, roll = latest_pitch[0], latest_roll[0]   # current body pitch and roll
    for leg_name, (lx, ly, lz) in link_xyz.items():   # each leg's local link offset
        wx_off, wy_off, wz_off = _rotate_body_to_world(lx, ly, lz, pitch, roll)   # that offset rotated into world frame
        wx, wy, wz = body_xyz[0] + wx_off, body_xyz[1] + wy_off, body_xyz[2] + wz_off   # foot's absolute world position
        prev = _prev_foot_world_xyz.get(leg_name)   # this foot's previous world position
        prev_t = _prev_foot_world_time.get(leg_name)   # timestamp of that previous position
        if prev is not None and prev_t is not None:   # only compute velocity once we have a previous sample
            dt = now - prev_t   # time since the last sample
            if dt > 1e-4:   # avoid dividing by a near-zero time step
                foot_world_vel[leg_name][0] = (wx - prev[0]) / dt   # x velocity component
                foot_world_vel[leg_name][1] = (wy - prev[1]) / dt   # y velocity component
                foot_world_vel[leg_name][2] = (wz - prev[2]) / dt   # z velocity component
        foot_world_xyz[leg_name] = (wx, wy, wz)   # store the new world position
        _prev_foot_world_xyz[leg_name] = (wx, wy, wz)   # remember it for next tick
        _prev_foot_world_time[leg_name] = now   # remember this timestamp for next tick

CMD_HIP_OFFSET = {"FL": (0.15, 0.213), "FR": (0.15, -0.213), "BL": (-0.15, 0.213), "BR": (-0.15, -0.213)}   # each leg's nominal hip offset from body origin

cmd_foot_world_xyz = {}   # each foot's commanded world position
cmd_foot_world_vel = {leg: [0.0, 0.0, 0.0] for leg in legs}   # each foot's commanded world velocity
_prev_cmd_foot_world_xyz = {}   # commanded position last sample
_prev_cmd_foot_world_time = {}   # timestamp of that last sample

def _update_commanded_foot_world(leg, fx_c, fz_c, now):
    if body_xyz[0] is None:   # skip until we have a body position reading
        return
    x0, y0 = CMD_HIP_OFFSET[leg]   # this leg's nominal hip offset
    pitch, roll = latest_pitch[0], latest_roll[0]   # current body pitch and roll
    wx_off, wy_off, wz_off = _rotate_body_to_world(x0 + fx_c, y0, fz_c, pitch, roll)   # commanded offset rotated into world frame
    wx, wy, wz = body_xyz[0] + wx_off, body_xyz[1] + wy_off, body_xyz[2] + wz_off   # commanded foot's absolute world position
    prev = _prev_cmd_foot_world_xyz.get(leg)   # previous commanded world position
    prev_t = _prev_cmd_foot_world_time.get(leg)   # timestamp of that previous position
    if prev is not None and prev_t is not None:   # only compute velocity once we have a previous sample
        dt = now - prev_t   # time since last sample
        if dt > 1e-4:   # avoid dividing by a near-zero time step
            cmd_foot_world_vel[leg][0] = (wx - prev[0]) / dt   # x velocity component
            cmd_foot_world_vel[leg][1] = (wy - prev[1]) / dt   # y velocity component
            cmd_foot_world_vel[leg][2] = (wz - prev[2]) / dt   # z velocity component
    cmd_foot_world_xyz[leg] = (wx, wy, wz)   # store the new commanded position
    _prev_cmd_foot_world_xyz[leg] = (wx, wy, wz)   # remember it for next tick
    _prev_cmd_foot_world_time[leg] = now   # remember this timestamp for next tick

def foot_tracking_error(active_leg):
    """Heads up: pos_err isn't a trustworthy absolute number. cmd_foot_world_xyz is built at the
       true foot tip (leg_ik's convention), but foot_world_xyz/link_xyz has always sampled the
       *_shank link's own origin (basically the knee), which sits ~L2=0.2m above the real tip.
       That fixed offset shows up as a near-constant ~0.19-0.22m "error" on every leg, every
       tick, even standing still - it swamps any real signal. vel_err is fine (differencing
       cancels out most of a constant offset), but don't trust pos_err. Use all_foot_world_z()
       below instead for a clean check of whether a foot is actually lifting off the ground
       during its swing."""
    out = {}   # collects tracking error per leg
    for l in legs:   # loop over each leg
        if l == active_leg:   # skip the leg that's currently swinging
            continue
        if l in cmd_foot_world_xyz and l in foot_world_xyz:   # only compare if we have both readings for this leg
            cx, cy, cz = cmd_foot_world_xyz[l]   # this leg's commanded position
            ax, ay, az = foot_world_xyz[l]   # this leg's actual position
            pos_err = math.sqrt((cx - ax) ** 2 + (cy - ay) ** 2 + (cz - az) ** 2)   # distance between commanded and actual
            cvx, cvy, _cvz = cmd_foot_world_vel[l]   # commanded velocity, x/y (z unused)
            avx, avy, _avz = foot_world_vel[l]   # actual velocity, x/y (z unused)
            vel_err = math.hypot(cvx - avx, cvy - avy)   # how far actual velocity is off
            out[l] = (pos_err, vel_err)   # save this leg's error pair
    return out

def all_foot_world_z():
    """{leg: z} - the measured world-frame height at each leg's shank-origin sample point
       (foot_world_xyz). Not the true foot tip (see the note above), but a fixed, consistent
       offset from it for a given leg orientation - so relative motion (does it rise during the
       swing?) is trustworthy even though the absolute value isn't the real ground-contact height."""
    return {l: foot_world_xyz[l][2] for l in legs if l in foot_world_xyz}   # leg name to its measured world z height

def pose_callback(msg):
    if not _dumped_pose_names[0]:   # only log pose entity names the first time we see them
        _dumped_pose_names[0] = True   # mark that we've logged pose names once
        log_line(f"DEBUG: pose entity names seen: {sorted(set(p.name for p in msg.pose))}")   # log all distinct pose entity names seen
    for p in msg.pose:   # each entity's pose in this message
        if p.name == "my_quadruped":   # check if this pose entry is the robot body itself
            now = time.time()   # timestamp of this pose sample
            if _prev_pose_time[0] is not None:   # only compute velocity once we've seen a previous pose
                dt = now - _prev_pose_time[0]   # time since the last pose sample
                if dt > 1e-4:   # avoid dividing by a near-zero time step
                    new_vx = (p.position.x - _prev_body_xyz[0]) / dt   # body velocity x from position change
                    new_vy = (p.position.y - _prev_body_xyz[1]) / dt   # body velocity y from position change
                    new_vz = (p.position.z - _prev_body_xyz[2]) / dt   # body velocity z from position change
                    if _prev_body_vel[0] is not None:   # only compute acceleration once we have a previous velocity
                        body_accel[0] = (new_vx - _prev_body_vel[0]) / dt   # x acceleration
                        body_accel[1] = (new_vy - _prev_body_vel[1]) / dt   # y acceleration
                        body_accel[2] = (new_vz - _prev_body_vel[2]) / dt   # z acceleration
                    _prev_body_vel[0] = new_vx   # remember x velocity for next tick
                    _prev_body_vel[1] = new_vy   # remember y velocity for next tick
                    _prev_body_vel[2] = new_vz   # remember z velocity for next tick
                    body_vel[0] = new_vx   # store new x velocity
                    body_vel[1] = new_vy   # store new y velocity
                    body_vel[2] = new_vz   # store new z velocity
            _prev_body_xyz[0] = p.position.x   # remember x position for next tick
            _prev_body_xyz[1] = p.position.y   # remember y position for next tick
            _prev_body_xyz[2] = p.position.z   # remember z position for next tick
            _prev_pose_time[0] = now   # remember this timestamp for next tick

            body_xyz[0] = p.position.x   # store new x position
            body_xyz[1] = p.position.y   # store new y position
            body_xyz[2] = p.position.z   # store new z position
        else:
            for leg_name in legs:   # check which leg this link belongs to
                if p.name.endswith(f"{leg_name}_shank"):   # check if this pose entry is this leg's shank link
                    link_xyz[leg_name] = (p.position.x, p.position.y, p.position.z)   # store this leg's shank position
                    break
    _update_foot_world_positions(time.time())

node.subscribe(Pose_V, "/world/empty/pose/info", pose_callback)

# stability checks against the support polygon - unchanged math from crawlgait.py
def _sign2d(p1, p2, p3):
    return (p1[0]-p3[0])*(p2[1]-p3[1]) - (p2[0]-p3[0])*(p1[1]-p3[1])

def _point_in_triangle(pt, v1, v2, v3):
    d1 = _sign2d(pt, v1, v2)   # which side of edge v1-v2 the point is on
    d2 = _sign2d(pt, v2, v3)   # which side of edge v2-v3 the point is on
    d3 = _sign2d(pt, v3, v1)   # which side of edge v3-v1 the point is on
    has_neg = (d1 < 0) or (d2 < 0) or (d3 < 0)   # point is outside on at least one edge
    has_pos = (d1 > 0) or (d2 > 0) or (d3 > 0)   # point is outside on at least one edge, other side
    return not (has_neg and has_pos)

def _polygon_check(point_xy, active_leg):
    """(inside, margin) for point_xy against the triangle of the three legs other than
       active_leg. Returns (None, None) if we haven't seen one of those foot positions yet."""
    support_legs = [l for l in legs if l != active_leg]   # the three legs still planted
    if any(l not in foot_world_xyz for l in support_legs):   # bail out if we're missing any support leg's position
        return None, None
    pts = [(foot_world_xyz[l][0], foot_world_xyz[l][1]) for l in support_legs]   # xy positions of the support legs
    inside = _point_in_triangle(point_xy, pts[0], pts[1], pts[2])   # whether point is inside the support triangle
    def edge_dist(a, b):
        ex, ey = b[0]-a[0], b[1]-a[1]   # edge vector from a to b
        edge_len = math.hypot(ex, ey)   # length of that edge
        if edge_len < 1e-6:   # avoid dividing by a zero-length edge
            return 0.0
        cross = (point_xy[0]-a[0])*ey - (point_xy[1]-a[1])*ex   # signed distance numerator
        return cross / edge_len
    margins = [edge_dist(pts[0], pts[1]), edge_dist(pts[1], pts[2]), edge_dist(pts[2], pts[0])]   # signed distance to each edge
    return inside, min(abs(m) for m in margins)

def support_status(active_leg):
    """Static CoM-vs-support-triangle check. Weak - only quasi-static - kept around to compare
       against ZMP/capture-point, not as the main stability signal."""
    if body_xyz[0] is None:   # skip until we have a body position reading
        return None, None
    return _polygon_check((body_xyz[0], body_xyz[1]), active_leg)

G = 9.8   # gravitational acceleration, in m/s^2

def compute_zmp():
    """x_zmp = x_com - (xdd/(zdd+g))*z_com. Drops the angular-momentum-rate term - a first-order
       approximation, fine since body mass outweighs leg mass ~15:1 on this robot."""
    if body_xyz[0] is None:   # skip until we have a body position reading
        return None
    z_com = body_xyz[2]   # body height, used as CoM height
    xdd, ydd, zdd = body_accel   # body acceleration components
    denom = zdd + G   # vertical accel plus gravity
    if abs(denom) < 1.0:   # avoid dividing by a near-zero denominator
        denom = G   # fall back to gravity alone if denom too small
    return (body_xyz[0] - (xdd/denom)*z_com, body_xyz[1] - (ydd/denom)*z_com)

def zmp_status(active_leg):
    zmp = compute_zmp()   # the current ZMP point
    if zmp is None:   # bail out if ZMP couldn't be computed
        return None, None, None
    inside, margin = _polygon_check(zmp, active_leg)   # whether ZMP is inside the support triangle
    return inside, margin, zmp

CAPTURE_HEIGHT = 0.40   # assumed CoM height, in meters, for the capture point formula
CAPTURE_GAIN = math.sqrt(CAPTURE_HEIGHT / 9.8)   # ~0.202

def compute_capture_point():
    """x_cp = x_com + vx*sqrt(z0/g) (Pratt et al.) - where the CoM would land if all velocity
       vanished right now."""
    if body_xyz[0] is None:   # skip until we have a body position reading
        return None
    return (body_xyz[0] + body_vel[0]*CAPTURE_GAIN, body_xyz[1] + body_vel[1]*CAPTURE_GAIN)

def capture_point_status(active_leg):
    cp = compute_capture_point()   # the current capture point
    if cp is None:   # bail out if the capture point couldn't be computed
        return None, None, None
    inside, margin = _polygon_check(cp, active_leg)   # whether capture point is inside the support triangle
    return inside, margin, cp

def stance_foot_velocities(active_leg):
    """{leg: (vx, vy)} signed world-frame velocity for each currently-planted (non-active) leg.
       A properly planted foot should read ~0 - a sustained nonzero value means that leg's
       joints are losing the fight with the ground under load."""
    support_legs = [l for l in legs if l != active_leg]   # the currently planted legs
    return {l: tuple(foot_world_vel.get(l, (0.0, 0.0, 0.0))[:2]) for l in support_legs}   # xy velocity for each planted leg

def all_foot_fx():
    """{leg: current body-frame fx target} for all four legs."""
    return {l: foot_target[l][0] for l in legs}   # each leg's current fx target

# gait constants
STANCE_FZ = -0.34   # shallower crouch depth, frees up lateral reach

STEP_LENGTH_FRONT = 0.06   # how far the front feet step each stride, in meters
STEP_LENGTH_BACK = 0.08   # how far the back feet step each stride, in meters

SWING_HEIGHT_FRONT = 0.06   # how high front feet lift off the ground, in meters
SWING_HEIGHT_BACK = 0.08   # how high back feet lift off the ground, in meters

FX_LIMIT = 0.09   # hard clamp - both step lengths above stay comfortably inside this

def clamp_fx(v):
    return max(-FX_LIMIT, min(FX_LIMIT, v))

GAIT_ORDER = ["BR", "FL", "FR", "BL"]   # order legs lift in, one per step
N_CYCLES = 3   # how many times to repeat the full leg order

def _smoothstep(x):
    x = max(0.0, min(1.0, x))
    return x * x * (3 - 2 * x)

def swing_profile(leg, frac):
    """Same swing arc shape as the very first version of this file - smoothstep fore-aft + sine
       vertical lift, zero velocity at both ends. Now it's a standalone, fixed-duration motion
       for one leg (via do_swing() below) instead of being read off a shared phase clock. frac
       runs 0 (liftoff, at -step/2) to 1 (touchdown, at +step/2)."""
    is_front = leg in ("FL", "FR")   # is this a front leg?
    step = STEP_LENGTH_FRONT if is_front else STEP_LENGTH_BACK   # step length for this leg
    height = SWING_HEIGHT_FRONT if is_front else SWING_HEIGHT_BACK   # swing height for this leg
    s = _smoothstep(frac)   # smoothed progress through the swing, 0 to 1
    fx = -step / 2.0 + step * s   # fore-aft foot target for this point in the swing
    fz = STANCE_FZ + height * math.sin(math.pi * frac)   # vertical foot target for this point in the swing
    return clamp_fx(fx), fz

SWING_DURATION = 0.9   # seconds - same pacing as the old SWING_DUTY(0.15) * GAIT_PERIOD(6.0)
SWING_STEPS = 45       # ~50Hz of profile updates across SWING_DURATION

body_shift = [0.0, 0.0]   # shared fx/fy offset applied to every leg

REACH_BUDGET = 0.39   # safety margin under the leg's max reach

def max_safe_fy(fx_c, fz_c, budget=REACH_BUDGET):
    """Closed-form bound on |fy| - not searched - replacing the old flat MAX_FY_SHIFT (v6.1 fix,
       see the file header). Derivation: leg_ik_3d's geometry gives, for either abduction side,
       total reach = hypot(fx_c, w) where w = sqrt(r^2 - D_ABAD^2) and r = hypot(oy*D_ABAD + fy,
       fz_c). Requiring that reach <= budget for both oy=+1 and oy=-1 at once (fy is one shared
       value applied to every leg, left and right) and solving for the biggest common |fy|, the
       D_ABAD terms cancel between the two sides and you get:
           dy_max = sqrt((budget^2 - fx_c^2 + D_ABAD^2) - fz_c^2)
           max |fy| = max(0, dy_max - D_ABAD)
       Checked against an earlier binary-search reach table, matched to 4 decimal places at
       every sampled (fx, STANCE_FZ) point. Uses this tick's actual fx_c/fz_c (after body_shift,
       yaw correction, and the theta rotation) rather than one fixed value, so it stays correct
       as those other corrections move the leg's real operating point around."""
    r_max_sq = budget * budget - fx_c * fx_c + D_ABAD * D_ABAD   # squared max radius from the abad axis
    if r_max_sq <= 0.0:   # no safe lateral room left at this depth
        return 0.0
    dy_max_sq = r_max_sq - fz_c * fz_c   # squared max lateral offset
    if dy_max_sq <= 0.0:   # no safe lateral room left at this depth
        return 0.0
    return max(0.0, math.sqrt(dy_max_sq) - D_ABAD)

SHIFT_KP = 0.125
SHIFT_STEP_MAX = 0.00075   # meters/tick cap on how fast body_shift can move (~0.0375 m/s
                           # equivalent - halved again in v6.8, same reasoning as SHIFT_KP above)
SHIFT_TOLERANCE = 0.006  # meters - "close enough" to call the CoM aligned with the support centroid
SHIFT_VEL_TOLERANCE = 0.02
SHIFT_TIMEOUT = 6.0
SHIFT_STALL_TICKS = 80
SHIFT_STALL_EPS = 0.001  # improvement smaller than this doesn't reset the stall counter
SHIFT_VEL_WEIGHT = 0.1

FY_LOADED_FX_THRESHOLD = 0.015
FY_RATE_LOADED_SCALE = 0.35      # once any leg is "loaded" (above), fy's step cap drops to
                                  # ~1/3 of SHIFT_STEP_MAX - fx and cycle-1 shifts aren't touched.

def fy_rate_scale():
    """Returns 1.0 (full SHIFT_STEP_MAX) unless some leg already carries a post-swing fx offset,
       in which case fy - and only fy - gets throttled to FY_RATE_LOADED_SCALE. See the v6.11
       note above for why this replaced an earlier reach-margin version the actual failing run's
       numbers ruled out."""
    if any(abs(foot_target[leg][0]) >= FY_LOADED_FX_THRESHOLD for leg in legs):   # check if any leg is already carrying a post-swing offset
        return FY_RATE_LOADED_SCALE
    return 1.0

def clamp_fy(v, fx_c=0.0, fz_c=None):
    """fx_c/fz_c default to a rough, untilted body-shift-only estimate (fz_c=STANCE_FZ) for
       callers like shift_com_to that are clamping the shared accumulator itself, not one leg's
       exact post-rotation target. control_loop instead passes this tick's real per-leg fx_c/fz_c."""
    if fz_c is None:   # default to the standing depth if none was given
        fz_c = STANCE_FZ   # default to the standing depth
    bound = max_safe_fy(fx_c, fz_c)   # max safe lateral shift for this leg
    return max(-bound, min(bound, v))

def support_polygon_centroid(support_legs):
    """Plain geometric centroid (average) of the given legs' current world-frame foot positions
       - not the robot's true center of mass, same approximation support_status()/zmp_status()
       already make (using body_xyz as the CoM stand-in). This is the target we move that
       stand-in onto."""
    xs = [foot_world_xyz[l][0] for l in support_legs]   # x positions of the support legs
    ys = [foot_world_xyz[l][1] for l in support_legs]   # y positions of the support legs
    return (sum(xs) / len(xs), sum(ys) / len(ys))


def shift_com_to(target_xy, about_to_lift=None):
    """Closed-loop: nudges body_shift toward target_xy every tick, checking the real body_xyz
       each time, until within SHIFT_TOLERANCE (or SHIFT_TIMEOUT runs out). Runs in the main
       thread (like move_feet_manual) - control_loop just keeps applying whatever body_shift
       currently is to every leg, every tick, same as everything else here.

       v6.2 sign fix: body_shift is a body-frame foot-target offset applied identically to
       every planted leg. A planted foot is fixed in the world, so asking for a bigger offset
       in some direction doesn't move the foot - it moves the body the opposite way to keep
       that foot at the new relative spot. That's the same thing the old v5 continuous
       stance-sweep relied on (fx = (step/2)*(1-2*phase), decreasing over stance to drive the
       body forward) - which means the sign here has to be the error's negative, not the error
       itself. The first version of this function got that backwards (step = +KP*error), which
       made body_shift grow in exactly the direction that pushes the body away from target_xy -
       confirmed on a real run where body_xyz walked steadily further from target_xy (error
       0.12 -> 0.31) on both axes until fx hit FX_LIMIT. Fixed below: step = -KP*error on both
       axes (fy is a uniform Y offset across all four legs exactly like fx is - see
       max_safe_fy's docstring - so the same argument applies to both)."""
    start_t = time.time()   # when this shift started
    tick = 0   # counts loop iterations
    best_metric = None   # best (lowest) settle metric seen so far
    best_tick = 0   # tick number when best_metric last improved
    while True:   # keep looping until the shift converges, stalls, or times out
        if check_abort():   # stop immediately if a safety abort fired
            return
        if body_xyz[0] is None:   # wait until we have a body position reading
            time.sleep(CONTROL_DT)
            continue
        ex = target_xy[0] - body_xyz[0]   # x error to the target
        ey = target_xy[1] - body_xyz[1]   # y error to the target
        err = math.hypot(ex, ey)   # total distance to the target
        speed_xy = math.hypot(body_vel[0], body_vel[1])   # how fast the body is currently moving
        metric = err + SHIFT_VEL_WEIGHT * speed_xy   # v6.3 fix - see SHIFT_VEL_WEIGHT's comment above
        if tick % 12 == 0:   # ~4x/sec at CONTROL_DT=0.02
            diagnostic_line(f"shift err={err:.4f} speed={speed_xy:.3f}", active_leg=about_to_lift)
        if abs(ex) < SHIFT_TOLERANCE and abs(ey) < SHIFT_TOLERANCE and speed_xy < SHIFT_VEL_TOLERANCE:   # done once position and speed are both within tolerance
            break
        elapsed = time.time() - start_t   # how long this shift has been running
        if elapsed > SHIFT_TIMEOUT:   # give up waiting once the timeout is hit
            log_line(f"  [shift] timed out after {SHIFT_TIMEOUT}s with err={err:.4f} speed={speed_xy:.3f} "
                     f"- lifting anyway")
            break
        if best_metric is None or metric < best_metric - SHIFT_STALL_EPS:   # check if the error is still meaningfully improving
            best_metric = metric   # update the best metric seen
            best_tick = tick   # remember when it improved
        elif tick - best_tick > SHIFT_STALL_TICKS:   # give up if progress has stalled too long
            log_line(f"  [shift] stalled at err={err:.4f} speed={speed_xy:.3f} (no improvement in "
                     f"{SHIFT_STALL_TICKS} ticks) - lifting anyway")
            break
        if abs(ex) >= SHIFT_TOLERANCE:   # only adjust x if it hasn't converged yet
            # v6.2 fix: -SHIFT_KP*ex, not +SHIFT_KP*ex - see the sign-fix docstring above.
            step_x = max(-SHIFT_STEP_MAX, min(SHIFT_STEP_MAX, -SHIFT_KP * ex))   # how much to nudge the x shift this tick
            body_shift[0] = clamp_fx(body_shift[0] + step_x)   # apply the x nudge, clamped
        if abs(ey) >= SHIFT_TOLERANCE:   # only adjust y if it hasn't converged yet
            # v6.11: fy's step cap is throttled by fy_rate_scale() whenever some leg is already
            # carrying a post-swing fx offset (cycle 2+) - see FY_LOADED_FX_THRESHOLD's comment.
            scale = fy_rate_scale()   # throttle factor for fy's step size
            step_y_max = SHIFT_STEP_MAX * scale   # throttled max step for fy
            step_y = max(-step_y_max, min(step_y_max, -SHIFT_KP * ey))   # how much to nudge the y shift this tick
            if tick % 12 == 0 and scale < 1.0:   # log occasionally, only while throttling is active
                log_line(f"  [shift] fy throttled to {scale:.2f}x (leg already carrying post-swing fx)")
            body_shift[1] = clamp_fy(body_shift[1] + step_y, fx_c=body_shift[0], fz_c=STANCE_FZ)   # apply the y nudge, clamped
        tick += 1   # advance the loop counter
        time.sleep(CONTROL_DT)

CAPTURE_MARGIN_MIN = 0.01    # meters - min capture-point margin needed inside the support
                             # triangle that's left once `leg` lifts
CAPTURE_WAIT_MAX = 1.0       # seconds - bounded extra time to let momentum die down if the
                             # capture point isn't safely inside yet when shift_com_to returns

def wait_for_safe_lift(leg, max_wait=CAPTURE_WAIT_MAX):
    """Runs after shift_com_to returns (converged, stalled, or timed out) and before
       do_swing(leg). Doesn't move body_shift at all - just holds position and rechecks, giving
       any leftover momentum a bounded chance to die down before actually lifting `leg` out from
       under the robot. Bounded by CAPTURE_WAIT_MAX so a genuinely marginal step doesn't hang
       forever - if it's still not comfortably inside after that, logs it and moves on instead
       of stalling the whole sequence."""
    start_t = time.time()   # when this wait started
    tick = 0   # counts loop iterations
    while True:   # keep checking until it's safe to lift or we give up
        if check_abort():   # stop immediately if a safety abort fired
            return
        inside, margin, _cp_pt = capture_point_status(leg)   # is the capture point safely inside
        if tick % 12 == 0:   # print progress occasionally, not every tick
            diagnostic_line(f"pre-lift check {leg}", active_leg=leg)
        if inside is None or (inside and margin >= CAPTURE_MARGIN_MIN):   # safe to lift once the margin is comfortable
            return
        if time.time() - start_t > max_wait:   # give up waiting once the max wait is hit
            log_line(f"  [pre-lift] capture point still not safely inside {leg}'s support triangle "
                     f"after {max_wait}s (inside={inside} margin={margin}) - lifting anyway")
            return
        tick += 1   # advance the loop counter
        time.sleep(CONTROL_DT)

SETTLE_WAIT_MAX = 1.0   # seconds - bounded, same reasoning as CAPTURE_WAIT_MAX

def wait_for_settle(max_wait=SETTLE_WAIT_MAX, vel_tol=SHIFT_VEL_TOLERANCE):
    """Runs right after do_swing lands a leg, before the next shift_com_to starts centering
       toward the following leg. Doesn't touch body_shift - just waits for body_vel to actually
       settle (same SHIFT_VEL_TOLERANCE threshold shift_com_to uses to call a move "done"), so
       the next shift starts from a genuinely still robot instead of one still unwinding the
       last swing."""
    start_t = time.time()   # when this wait started
    tick = 0   # counts loop iterations
    while True:   # keep waiting until the body actually stops moving
        if check_abort():   # stop immediately if a safety abort fired
            return
        speed_xy = math.hypot(body_vel[0], body_vel[1])   # how fast the body is currently moving
        if tick % 12 == 0:   # print progress occasionally, not every tick
            diagnostic_line(f"post-swing settle speed={speed_xy:.3f}")
        if speed_xy < vel_tol:   # done once the body has settled
            return
        if time.time() - start_t > max_wait:   # give up waiting once the max wait is hit
            log_line(f"  [settle] still moving (speed={speed_xy:.3f}) after {max_wait}s - proceeding anyway")
            return
        tick += 1   # advance the loop counter
        time.sleep(CONTROL_DT)

def ramp_body_shift_to_zero(duration=1.0, steps=50):
    """Same linear-ramp pattern as move_feet_manual, just for the two body_shift scalars - used
       once at the end of the sequence so the final settle doesn't get left holding a nonzero lean."""
    start = list(body_shift)   # body_shift values at the start of the ramp
    for i in range(1, steps + 1):   # loop over each ramp step
        if check_abort():   # stop immediately if a safety abort fired
            return
        frac = i / steps   # fraction of the ramp completed
        body_shift[0] = start[0] * (1 - frac)   # ramp x shift toward zero
        body_shift[1] = start[1] * (1 - frac)   # ramp y shift toward zero
        time.sleep(duration / steps)
    body_shift[0] = 0.0   # make sure it lands exactly at zero
    body_shift[1] = 0.0   # make sure it lands exactly at zero

last_abad = {leg: 0.0 for leg in legs}   # each leg's last commanded ABAD angle, for logging

foot_target = {leg: (0.0, -0.4) for leg in legs}   # each leg's current fx/fz target

PITCH_SIGN = 1.0   # +1 or -1, flips pitch correction if backwards
CORRECTION_FRACTION = 0.4   # how much of the measured pitch to correct, 0-1
MAX_CORRECTION_RAD = 0.35   # largest allowed pitch-correction angle, in radians
PITCH_RATE_DAMPING = 0.15    # now applied to the low-pass-filtered pitch rate, not the raw gyro

ROLL_ABAD_FRACTION = 0.3   # how much of the measured roll to correct, 0-1
MAX_ABAD_ROLL_CORR = 0.15   # largest allowed roll-correction angle, in radians

LEG_LR = {"FL": 1, "FR": -1, "BL": 1, "BR": -1}   # left(+1)/right(-1) map for each leg
YAW_FX_GAIN = 0.03     # rad of accumulated yaw error -> meters of fx differential
YAW_RATE_DAMPING = 0.02   # first guess, unvalidated - same status as the YAW_FX_GAIN sign note above
MAX_YAW_FX = 0.025   # largest allowed fx nudge from yaw correction, in meters
gait_start_yaw = [None]   # the yaw heading when the gait sequence begins

running = [True]   # flips to False to stop the control loop

FLIP_LIMIT_DEG = 25.0   # pitch/roll angle, in degrees, that triggers a safety abort
FLIP_LIMIT_RAD = math.radians(FLIP_LIMIT_DEG)   # same limit, in radians, for the math below
aborted = [False]   # flips to True once a safety abort fires

gait_active = [False]   # gates the yaw-hold correction - only active while actually stepping

swinging_now = [False]   # true while a leg is mid-swing

def check_abort():
    if aborted[0]:   # already aborted, so stay aborted
        return True
    if abs(latest_pitch[0]) > FLIP_LIMIT_RAD or abs(latest_roll[0]) > FLIP_LIMIT_RAD:   # check if the robot has tipped past the safety limit
        aborted[0] = True   # latch the abort so it stays tripped
        print(f"!!! SAFETY ABORT: |pitch|={math.degrees(latest_pitch[0]):.1f} deg  "
              f"|roll|={math.degrees(latest_roll[0]):.1f} deg exceeded {FLIP_LIMIT_DEG} deg - "
              f"freezing in place, joint commands keep publishing (clamped) !!!")
    return aborted[0]

CONTROL_DT = 0.02   # 50Hz, matches the sleep() at the bottom of this loop

last_theta_terms = {"p": 0.0, "d": 0.0, "raw": 0.0, "clamped": 0.0}   # last tick's pitch-correction terms, kept for logging
last_yaw_terms = {"err": 0.0, "p": 0.0, "d": 0.0, "fx_term": 0.0}   # last tick's yaw-correction terms, kept for logging

def control_loop():
    """No phase clock anymore - foot_target[leg] and body_shift are just read every tick,
       whatever the main thread (move_feet_manual / shift_com_to / do_swing) last set them to.
       If the main thread stops advancing anything (abort, or between explicit steps), this
       loop just keeps republishing the same values - which is exactly the "freeze in place"
       behavior we want, with no separate freeze logic needed."""
    while running[0]:   # keep looping until the run finishes or aborts
        check_abort()

        theta_p = PITCH_SIGN * CORRECTION_FRACTION * latest_pitch[0]   # proportional pitch correction term
        theta_d = PITCH_SIGN * PITCH_RATE_DAMPING * latest_pitch_rate[0]   # derivative pitch correction term
        theta_raw = theta_p + theta_d   # combined pitch correction before clamping
        theta = max(-MAX_CORRECTION_RAD, min(MAX_CORRECTION_RAD, theta_raw))   # clamped pitch correction angle
        last_theta_terms["p"] = theta_p   # save p term for logging
        last_theta_terms["d"] = theta_d   # save d term for logging
        last_theta_terms["raw"] = theta_raw   # save raw term for logging
        last_theta_terms["clamped"] = theta   # save clamped term for logging

        roll_term = ROLL_ABAD_FRACTION * latest_roll[0]   # proportional roll correction term
        roll_term = max(-MAX_ABAD_ROLL_CORR, min(MAX_ABAD_ROLL_CORR, roll_term))   # clamped roll correction angle

        if gait_active[0] and gait_start_yaw[0] is not None:   # only correct yaw while the gait is actively running
            yaw_err = latest_yaw[0] - gait_start_yaw[0]   # how far yaw has drifted from the start
        else:
            yaw_err = 0.0   # no yaw correction when not gait-active
        yaw_p = YAW_FX_GAIN * yaw_err   # proportional yaw correction term
        yaw_d = YAW_RATE_DAMPING * latest_yaw_rate[0]   # v6.6 fix - see YAW_RATE_DAMPING's comment
        if swinging_now[0]:   # freeze yaw correction while a leg is mid-swing
            yaw_fx_term = 0.0   # v6.7 fix - see swinging_now's comment: no fx nudge for any leg
        else:                   # while a leg's actually mid-swing (includes stance legs)
            yaw_fx_term = max(-MAX_YAW_FX, min(MAX_YAW_FX, yaw_p + yaw_d))   # clamped fx nudge from yaw correction
        last_yaw_terms["err"] = yaw_err   # save err term for logging
        last_yaw_terms["p"] = yaw_p   # save p term for logging
        last_yaw_terms["d"] = yaw_d   # save d term for logging
        last_yaw_terms["fx_term"] = yaw_fx_term   # save fx_term for logging

        now = time.time()   # timestamp for this control tick
        for leg in legs:   # update every leg this tick
            fx, fz = foot_target[leg]   # this leg's current target, before corrections
            fx = clamp_fx(fx + body_shift[0] + LEG_LR[leg] * yaw_fx_term)   # add shift and yaw nudge, clamped
            fx_c, fz_c = rotate(fx, fz, theta)   # target rotated by the pitch correction
            fy = clamp_fy(body_shift[1], fx_c=fx_c, fz_c=fz_c)   # lateral shift, clamped to this leg's reach
            abad_geo, hip, knee = leg_ik_3d(fx_c, fy, fz_c, OY[leg], LEG_SIDE[leg])   # solved joint angles for this leg
            _update_commanded_foot_world(leg, fx_c, fz_c, now)

            abad = abad_geo + roll_term   # final abad angle with roll correction added
            last_abad[leg] = abad   # remember for logging

            m0 = Double(); m0.data = abad   # abad command message
            pubs[f"{leg}_ABAD"].publish(m0)
            m1 = Double(); m1.data = hip   # hip command message
            pubs[f"{leg}_HIP"].publish(m1)
            m2 = Double(); m2.data = knee   # knee command message
            pubs[f"{leg}_KNEE"].publish(m2)
        time.sleep(CONTROL_DT)

t = threading.Thread(target=control_loop, daemon=True)   # background thread running the control loop
t.start()

def diagnostic_line(label, active_leg=None):
    vx, vy, vz = body_vel   # body velocity components
    ax, ay, az = body_accel   # body acceleration components
    speed = math.hypot(vx, vy)   # horizontal speed
    line = (f"  [{label}] pitch: {math.degrees(latest_pitch[0]):+.2f} deg  "   # build the telemetry log line
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
    if active_leg is not None:   # only add support-triangle info once we know the active leg
        inside, margin = support_status(active_leg)   # CoM inside the support triangle?
        if inside is not None:   # only append if the support-triangle check succeeded
            line += f"  CoM: inside={inside} margin={margin:+.4f}"
        zmp_in, zmp_margin, _zmp_pt = zmp_status(active_leg)   # ZMP inside the support triangle?
        if zmp_in is not None:   # only append if the ZMP check succeeded
            line += f"  ZMP: inside={zmp_in} margin={zmp_margin:+.4f}"
        cp_in, cp_margin, _cp_pt = capture_point_status(active_leg)   # capture point inside the support triangle?
        if cp_in is not None:   # only append if the capture-point check succeeded
            line += f"  CapturePt: inside={cp_in} margin={cp_margin:+.4f}"
        vels = stance_foot_velocities(active_leg)   # xy velocity for each planted leg
        line += "  stance_foot_vxy: {" + " ".join(f"{l}:({vx2:+.3f},{vy2:+.3f})" for l, (vx2, vy2) in vels.items()) + "}"
    track = foot_tracking_error(active_leg)   # commanded vs actual foot tracking error
    if track:   # only append if we have tracking-error data
        line += "  velerr: {" + " ".join(f"{l}:{ve:.3f}" for l, (_pe, ve) in track.items()) + "}"
    foot_z = all_foot_world_z()   # measured world height for each foot
    if foot_z:   # only append if we have foot height data
        marker = lambda l: "*" if l == active_leg else ""   # flags the currently active leg
        line += "  foot_z: {" + " ".join(f"{l}{marker(l)}:{z:+.3f}" for l, z in foot_z.items()) + "}"
    fx_all = all_foot_fx()   # each leg's current fx target
    line += "  fx: {" + " ".join(f"{l}:{v:+.4f}" for l, v in fx_all.items()) + "}"
    line += "  abad: {" + " ".join(f"{l}:{math.degrees(v):+.2f}" for l, v in last_abad.items()) + "}"
    log_line(line)   # v6.7 fix - full telemetry goes to run_log.txt only, console stays step-level

def move_feet_manual(deltas, duration=1.5, steps=75, label=None):
    """Linear ramp of foot_target for the given legs - used outside the step sequence (crouch,
       final settle). During a step, do_swing() owns the swinging leg's foot_target instead."""
    starts = {leg: foot_target[leg] for leg in deltas}   # each leg's starting position before the ramp
    print_every = max(1, steps // 10)   # how often to print progress
    for i in range(1, steps + 1):   # loop over each ramp step
        if check_abort():   # stop immediately if a safety abort fired
            return
        frac = i / steps   # fraction of the ramp completed
        for leg, (tx, tz) in deltas.items():   # target fx/fz for each leg
            sx, sz = starts[leg]   # this leg's starting fx/fz
            foot_target[leg] = (sx + (tx - sx) * frac, sz + (tz - sz) * frac)   # interpolate toward the target
        if label and (i % print_every == 0 or i == 1):   # print progress occasionally, only if a label was given
            diagnostic_line(f"{label} {frac*100:3.0f}%")
        time.sleep(duration / steps)
    for leg, tgt in deltas.items():   # make sure every leg lands exactly on target
        foot_target[leg] = tgt   # snap to the final target

def do_swing(leg, duration=SWING_DURATION, steps=SWING_STEPS):
    """Lifts, arcs, and places one leg - a standalone timed motion (main thread owns
       foot_target[leg] the whole time), not read off any shared clock. Lands at fx=+step/2,
       matching swing_profile's frac=1.0 endpoint."""
    print(f"  stepping {leg}...")
    swinging_now[0] = True   # v6.7 fix - see swinging_now's comment; freezes the yaw correction
    try:
        print_every = max(1, steps // 6)   # how often to print progress
        for i in range(1, steps + 1):   # loop over each swing step
            if check_abort():   # stop immediately if a safety abort fired
                return
            frac = i / steps   # fraction of the swing completed
            foot_target[leg] = swing_profile(leg, frac)   # this leg's fx/fz for this point in the swing
            if i % print_every == 0 or i == 1:   # print progress periodically, or on the very first step
                diagnostic_line(f"swing {leg} {frac*100:3.0f}%", active_leg=leg)
            time.sleep(duration / steps)
    finally:
        swinging_now[0] = False   # swing done, let yaw correction run again

print("waiting for the drop to settle...")
time.sleep(5.0)
print(f"landed, pitch = {math.degrees(latest_pitch[0]):.2f} deg  roll = {math.degrees(latest_roll[0]):.2f} deg")
print(f"body position: {body_xyz}")

print("--- crouch (hip/knee only, all ABAD at 0) ---")
move_feet_manual({leg: (0.0, STANCE_FZ) for leg in legs}, duration=1.5, steps=75, label="crouch")   # crouch every leg down to standing depth
for _ in range(20):   # settle for 20 ticks before starting
    if check_abort():   # stop early if a safety abort fired
        break
    diagnostic_line("crouch settle")
    time.sleep(0.1)

start_xyz = list(body_xyz)   # body position before the gait starts
start_yaw = latest_yaw[0]   # body yaw before the gait starts

if not aborted[0]:   # only run the gait if nothing has aborted yet
    gait_start_yaw[0] = latest_yaw[0]   # remember heading to hold during the gait
    gait_active[0] = True   # turn on the yaw-hold correction
    print(f"--- step sequence engaged: order={GAIT_ORDER}  cycles={N_CYCLES}  "
          f"swing_duration={SWING_DURATION}s ---")

    for cycle in range(N_CYCLES):   # repeat the full leg order N_CYCLES times
        if check_abort():   # stop early if a safety abort fired
            break
        for leg in GAIT_ORDER:   # step through the legs in gait order
            if check_abort():   # stop early if a safety abort fired
                break
            support_legs = [l for l in legs if l != leg]   # the three legs that'll stay planted
            target_xy = support_polygon_centroid(support_legs)   # pure centroid - see the v6.5 revert note above
            print(f"--- cycle {cycle+1}/{N_CYCLES}: centering CoM over {support_legs} before "
                  f"lifting {leg} (target={target_xy[0]:+.3f},{target_xy[1]:+.3f}) ---")
            shift_com_to(target_xy, about_to_lift=leg)
            if check_abort():   # stop early if a safety abort fired
                break
            wait_for_safe_lift(leg)   # v6.3 fix - a real capture-point gate, not just a position check
            if check_abort():   # stop early if a safety abort fired
                break
            do_swing(leg)
            if check_abort():   # stop early if a safety abort fired
                break
            wait_for_settle()   # v6.4 fix - let the swing's disturbance decay before the next
                                # shift starts centering toward a different leg's polygon

    if not aborted[0]:   # only settle back to stance if nothing aborted
        gait_active[0] = False   # gait done, turn off yaw-hold correction
        print("--- step sequence stopped, returning to a stable stance ---")
        ramp_body_shift_to_zero(duration=1.0, steps=50)
        move_feet_manual({leg: (0.0, STANCE_FZ) for leg in legs}, duration=1.5, steps=75,   # settle back to standing depth
                          label="post-gait settle")

end_xyz = list(body_xyz)   # body position after the gait finishes
dx = end_xyz[0] - start_xyz[0]   # net x displacement
dy = end_xyz[1] - start_xyz[1]   # net y displacement
dist = math.sqrt(dx*dx + dy*dy)   # total straight-line distance moved
dyaw_deg = math.degrees(math.atan2(math.sin(latest_yaw[0] - start_yaw), math.cos(latest_yaw[0] - start_yaw)))   # net yaw turned, in degrees
print(f"net displacement: dx={dx:.3f} dy={dy:.3f}  total distance={dist:.3f} m  net yaw turned={dyaw_deg:+.1f} deg")
print(f"final body z: {end_xyz[2]:.3f}  (collapsed if well below ~0.35)")
for _ in range(20):   # hold position for 20 more ticks
    if check_abort():   # stop early if a safety abort fired
        break
    diagnostic_line("end of loop")
    time.sleep(0.1)

print("sequence stopped early (safety abort)" if aborted[0] else "sequence complete")
running[0] = False   # stop the control loop thread
