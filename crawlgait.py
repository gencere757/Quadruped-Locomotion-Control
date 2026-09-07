"""
Script: crawlgait.py
Author: Arda Gencer
Crawl gait controller with per-leg CoM shifting for extra stability.
Detailed tuning-history notes for the constants below are in tuning_history/crawlgait_history.txt
"""
import gz.transport13 as transport
from gz.msgs10.double_pb2 import Double
from gz.msgs10.imu_pb2 import IMU
from gz.msgs10.pose_v_pb2 import Pose_V
import math
import threading
import time
import sys

# also send print() output to a log file, same filename every run, overwritten each time -
# so it can be read back without having to copy/paste from the console
_log_file = open("run_log.txt", "w")  # file handle for the run log

class _Tee:
    def __init__(self, *streams):  # streams: output streams to tee writes to
        self.streams = streams  # save the list of streams to write to
    def write(self, data):  # data: text being written
        for s in self.streams:  # loop over each output stream
            s.write(data)
            s.flush()
    def flush(self):
        for s in self.streams:  # loop over each output stream
            s.flush()

sys.stdout = _Tee(sys.stdout, _log_file)  # redirect stdout to print to console and log file


L1 = 0.2   # length of the leg's upper segment, in meters
L2 = 0.2   # length of the leg's lower segment, in meters

def leg_ik(fx, fz, s):
    """fx,fz = desired foot position relative to hip, in body frame (fz negative = below hip).
       s = +1 for front legs, -1 for back legs. Returns (hip, knee) angles."""
    u = s * fx  # foot x offset, flipped for leg side
    w = fz  # foot z offset (height)
    r2 = u*u + w*w  # squared distance from hip to foot
    r2 = max(r2, 1e-9)  # avoid divide-by-zero later
    c = (r2 - L1*L1 - L2*L2) / (2*L1*L2)  # law-of-cosines term for knee angle
    c = max(-1.0, min(1.0, c))  # clamp for float rounding safety
    knee = -math.acos(c)  # knee joint angle
    k1 = L1 + L2*math.cos(knee)  # helper term for hip angle calc
    k2 = L2*math.sin(knee)  # helper term for hip angle calc
    sin_a = (u*k1 + k2*w) / r2  # sine component of hip angle
    cos_a = (k2*u - k1*w) / r2  # cosine component of hip angle
    hip = math.atan2(sin_a, cos_a)  # hip joint angle
    return hip, knee

def rotate(fx, fz, theta):
    """Rotate a foot-target vector by theta (same convention as the leg's own swing rotation)."""
    return (fx*math.cos(theta) - fz*math.sin(theta),
            fx*math.sin(theta) + fz*math.cos(theta))

LEG_SIDE = {"FL": 1, "FR": 1, "BL": -1, "BR": -1}  # +1 for front legs, -1 for back legs, used in leg_ik
legs = ["FL", "FR", "BL", "BR"]  # names of the four legs
LEFT_LEGS = {"FL", "BL"}   # Y+ side per the SDF

node = transport.Node()  # the gz-transport node for pub/sub
pubs = {}  # publishers for each leg's joint commands
for leg in legs:  # loop over each leg name
    pubs[f"{leg}_ABAD"] = node.advertise(f"/model/my_quadruped/joint/{leg}_ABAD/cmd_pos", Double)  # publisher for this leg's ABAD joint
    pubs[f"{leg}_HIP"] = node.advertise(f"/model/my_quadruped/joint/{leg}_HIP/cmd_pos", Double)  # publisher for this leg's HIP joint
    pubs[f"{leg}_KNEE"] = node.advertise(f"/model/my_quadruped/joint/{leg}_KNEE/cmd_pos", Double)  # publisher for this leg's KNEE joint

latest_pitch = [0.0]  # most recent pitch angle from the IMU, in radians
latest_roll = [0.0]  # most recent roll angle from the IMU, in radians

latest_pitch_rate = [0.0]  # most recent pitch rate, rad/s
_pitch_rate_source = [None]     # "gyro" or "fd", set once we know which one is working
_prev_pitch_for_rate = [None]  # last pitch value, for fd fallback
_prev_pitch_rate_time = [None]  # timestamp of last pitch sample
_dumped_imu_fields = [False]  # whether we've printed the imu field names yet

def imu_callback(msg):
    if not _dumped_imu_fields[0]:  # only run this debug dump once
        _dumped_imu_fields[0] = True  # mark that we've dumped the fields
        try:
            print(f"DEBUG: IMU message fields: {[f.name for f in msg.DESCRIPTOR.fields]}")  # print the imu field names for debugging
        except Exception as e:  # the introspection error
            print(f"DEBUG: could not introspect IMU message fields: {e}")

    q = msg.orientation  # quaternion orientation from imu
    sinp = max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))  # sine of pitch, clamped
    pitch = math.asin(sinp)  # pitch angle in radians
    latest_pitch[0] = pitch  # store latest pitch globally
    latest_roll[0] = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))  # store latest roll globally

    got_gyro = False  # whether the imu actually has gyro data
    try:
        latest_pitch_rate[0] = msg.angular_velocity.y  # pitch rate straight from gyro
        got_gyro = True  # mark gyro data as available
    except AttributeError:
        pass
    if got_gyro:  # check if the imu actually gave us gyro data
        _pitch_rate_source[0] = "gyro"  # record that we're using the gyro
    else:
        _pitch_rate_source[0] = "fd"  # record that we're using finite difference
        now = time.time()  # current time for finite-difference calc
        if _prev_pitch_for_rate[0] is not None and _prev_pitch_rate_time[0] is not None:  # only compute rate once we have a prior sample
            dt = now - _prev_pitch_rate_time[0]  # time since last pitch sample
            if dt > 1e-4:  # only compute rate if enough time has passed
                latest_pitch_rate[0] = (pitch - _prev_pitch_for_rate[0]) / dt  # pitch rate via finite difference
        _prev_pitch_for_rate[0] = pitch  # save pitch for next fd calc
        _prev_pitch_rate_time[0] = now  # save time for next fd calc

node.subscribe(IMU, "/model/my_quadruped/imu", imu_callback)

body_xyz = [None, None, None]  # current body position (x, y, z) in world frame

body_vel = [0.0, 0.0, 0.0]  # current body velocity (x, y, z)
body_accel = [0.0, 0.0, 0.0]  # current body acceleration (x, y, z), from differencing velocity
_prev_body_xyz = [None, None, None]  # body position from previous pose sample
_prev_body_vel = [None, None, None]  # body velocity from previous pose sample
_prev_pose_time = [None]  # timestamp of previous pose sample

link_xyz = {}  # leg name -> shank link position
_dumped_pose_names = [False]  # whether we've printed pose entity names yet

def _rotate_body_to_world(dx, dy, dz, pitch, roll):
    """Rotates a vector from the model's local/body frame into world frame, using the same
       Tait-Bryan ZYX (yaw-pitch-roll) convention imu_callback uses to extract pitch/roll,
       with yaw=0."""
    cp, sp = math.cos(pitch), math.sin(pitch)  # cos/sin of pitch for rotation
    cr, sr = math.cos(roll), math.sin(roll)  # cos/sin of roll for rotation
    wx = cp*dx + sp*sr*dy + sp*cr*dz  # x component in world frame
    wy = cr*dy - sr*dz  # y component in world frame
    wz = -sp*dx + cp*sr*dy + cp*cr*dz  # z component in world frame
    return wx, wy, wz

foot_world_xyz = {}                                    # leg -> (wx, wy, wz), most recent
foot_world_vel = {leg: [0.0, 0.0, 0.0] for leg in legs} # leg -> [vx, vy, vz]
_prev_foot_world_xyz = {}  # previous foot world positions, for velocity calc
_prev_foot_world_time = {}  # previous foot position timestamps

def _update_foot_world_positions(now):
    if body_xyz[0] is None:  # skip until we've received a body position
        return
    pitch, roll = latest_pitch[0], latest_roll[0]  # current pitch/roll, cached locally
    for leg_name, (lx, ly, lz) in link_xyz.items():  # loop over each leg's local foot position
        wx_off, wy_off, wz_off = _rotate_body_to_world(lx, ly, lz, pitch, roll)  # foot offset rotated into world frame
        wx, wy, wz = body_xyz[0] + wx_off, body_xyz[1] + wy_off, body_xyz[2] + wz_off  # foot position in world frame
        prev = _prev_foot_world_xyz.get(leg_name)  # this foot's previous world position
        prev_t = _prev_foot_world_time.get(leg_name)  # this foot's previous timestamp
        if prev is not None and prev_t is not None:  # only compute velocity once we have a prior sample
            dt = now - prev_t  # time since last sample for this foot
            if dt > 1e-4:   # guard against a duplicate/zero-interval callback firing
                foot_world_vel[leg_name][0] = (wx - prev[0]) / dt  # x velocity of this foot
                foot_world_vel[leg_name][1] = (wy - prev[1]) / dt  # y velocity of this foot
                foot_world_vel[leg_name][2] = (wz - prev[2]) / dt  # z velocity of this foot
        foot_world_xyz[leg_name] = (wx, wy, wz)  # store this foot's current world position
        _prev_foot_world_xyz[leg_name] = (wx, wy, wz)  # save for next velocity calc
        _prev_foot_world_time[leg_name] = now  # save timestamp for next velocity calc

def pose_callback(msg):
    if not _dumped_pose_names[0]:  # only print this once
        _dumped_pose_names[0] = True  # mark that we've printed pose names
        print(f"DEBUG: pose entity names seen: {sorted(set(p.name for p in msg.pose))}")  # print the entity names for debugging
    for p in msg.pose:  # loop over each entity's pose in the message
        if p.name == "my_quadruped":  # check if this entry is the robot's body
            now = time.time()  # current time for this pose sample
            if _prev_pose_time[0] is not None:  # only compute velocity once we have a prior sample
                dt = now - _prev_pose_time[0]  # time since last pose sample
                if dt > 1e-4:   # guard against a duplicate/zero-interval callback firing
                    new_vx = (p.position.x - _prev_body_xyz[0]) / dt  # body x velocity this sample
                    new_vy = (p.position.y - _prev_body_xyz[1]) / dt  # body y velocity this sample
                    new_vz = (p.position.z - _prev_body_xyz[2]) / dt  # body z velocity this sample
                    if _prev_body_vel[0] is not None:  # only compute acceleration once we have a prior velocity
                        body_accel[0] = (new_vx - _prev_body_vel[0]) / dt  # body x acceleration
                        body_accel[1] = (new_vy - _prev_body_vel[1]) / dt  # body y acceleration
                        body_accel[2] = (new_vz - _prev_body_vel[2]) / dt  # body z acceleration
                    _prev_body_vel[0] = new_vx  # save vx for next accel calc
                    _prev_body_vel[1] = new_vy  # save vy for next accel calc
                    _prev_body_vel[2] = new_vz  # save vz for next accel calc
                    body_vel[0] = new_vx  # update global body x velocity
                    body_vel[1] = new_vy  # update global body y velocity
                    body_vel[2] = new_vz  # update global body z velocity
            _prev_body_xyz[0] = p.position.x  # save x for next velocity calc
            _prev_body_xyz[1] = p.position.y  # save y for next velocity calc
            _prev_body_xyz[2] = p.position.z  # save z for next velocity calc
            _prev_pose_time[0] = now  # save timestamp for next velocity calc

            body_xyz[0] = p.position.x  # update global body x position
            body_xyz[1] = p.position.y  # update global body y position
            body_xyz[2] = p.position.z  # update global body z position
        else:
            for leg_name in legs:  # check which leg this link belongs to
                if p.name.endswith(f"{leg_name}_shank"):  # check if this link belongs to this leg
                    link_xyz[leg_name] = (p.position.x, p.position.y, p.position.z)  # store this leg's shank position
                    break
    _update_foot_world_positions(time.time())

node.subscribe(Pose_V, "/world/empty/pose/info", pose_callback)

def _sign2d(p1, p2, p3):  # p1,p2,p3: three 2d points
    return (p1[0]-p3[0])*(p2[1]-p3[1]) - (p2[0]-p3[0])*(p1[1]-p3[1])

def _point_in_triangle(pt, v1, v2, v3):  # pt: point to test; v1-v3: triangle corners
    d1 = _sign2d(pt, v1, v2)  # sign relative to edge pt-v1-v2
    d2 = _sign2d(pt, v2, v3)  # sign relative to edge pt-v2-v3
    d3 = _sign2d(pt, v3, v1)  # sign relative to edge pt-v3-v1
    has_neg = (d1 < 0) or (d2 < 0) or (d3 < 0)  # true if any sign is negative
    has_pos = (d1 > 0) or (d2 > 0) or (d3 > 0)  # true if any sign is positive
    return not (has_neg and has_pos)

def _polygon_check(point_xy, active_leg):
    """Returns (inside, margin) for a point against the triangle of the three
       legs other than active_leg, using the already-computed world-frame foot
       positions (foot_world_xyz). Shared by the CoM/ZMP/capture-point checks -
       only the point differs. `margin` is a rough (not exact perpendicular)
       distance to the nearest edge in meters, useful for trend even though its
       sign isn't guaranteed to match `inside` (triangle winding varies leg to
       leg). Returns (None, None) if a needed foot position hasn't been seen
       yet."""
    support_legs = [l for l in legs if l != active_leg]  # the three legs currently planted
    if any(l not in foot_world_xyz for l in support_legs):  # bail out if a support leg's position isn't known yet
        return None, None
    pts = [(foot_world_xyz[l][0], foot_world_xyz[l][1]) for l in support_legs]  # xy positions of the support legs
    inside = _point_in_triangle(point_xy, pts[0], pts[1], pts[2])  # whether point is inside support triangle
    def edge_dist(a, b):  # a,b: two triangle corners
        ex, ey = b[0]-a[0], b[1]-a[1]  # edge vector components
        edge_len = math.hypot(ex, ey)  # length of the edge
        if edge_len < 1e-6:  # avoid dividing by zero for a degenerate edge
            return 0.0
        cross = (point_xy[0]-a[0])*ey - (point_xy[1]-a[1])*ex  # cross product for signed distance
        return cross / edge_len
    margins = [edge_dist(pts[0], pts[1]), edge_dist(pts[1], pts[2]), edge_dist(pts[2], pts[0])]  # distance to each of the 3 edges
    margin = min(abs(m) for m in margins)  # closest distance to any edge
    return inside, margin

def support_status(active_leg):
    """Returns (inside, margin) for the body's static (x,y) position vs. the
       support triangle - see the caveat above. Kept for comparison against
       the ZMP/capture-point checks, not as the main stability signal
       anymore."""
    if body_xyz[0] is None:  # skip until we know the body's position
        return None, None
    return _polygon_check((body_xyz[0], body_xyz[1]), active_leg)

G = 9.8  # gravitational acceleration, in m/s^2

def compute_zmp():
    if body_xyz[0] is None:  # skip until we know the body's position
        return None
    z_com = body_xyz[2]  # body height, used as center of mass height
    xdd, ydd, zdd = body_accel  # body acceleration components
    denom = zdd + G  # denominator for zmp equation
    if abs(denom) < 1.0:   # guard against a near-zero/negative denominator from noisy zdd spikes
        denom = G  # fall back to plain gravitational value
    return (body_xyz[0] - (xdd/denom)*z_com, body_xyz[1] - (ydd/denom)*z_com)

def zmp_status(active_leg):
    """Returns (inside, margin, (x_zmp, y_zmp)). If this goes False before
       support_status() does (or before pitch visibly runs away), that
       confirms it's a dynamic/inertial tipping problem - one the static CoM
       check could never catch."""
    zmp = compute_zmp()  # zero moment point position
    if zmp is None:  # skip if the zmp couldn't be computed yet
        return None, None, None
    inside, margin = _polygon_check(zmp, active_leg)  # whether zmp is inside support triangle, and margin
    return inside, margin, zmp

def compute_capture_point():
    if body_xyz[0] is None:  # skip until we know the body's position
        return None
    return (body_xyz[0] + body_vel[0]*CAPTURE_GAIN, body_xyz[1] + body_vel[1]*CAPTURE_GAIN)

def capture_point_status(active_leg):  # active_leg: leg currently swinging, excluded from the triangle
    cp = compute_capture_point()  # where body's momentum would carry it to rest
    if cp is None:  # skip if the capture point couldn't be computed yet
        return None, None, None
    inside, margin = _polygon_check(cp, active_leg)  # whether capture point is inside support triangle
    return inside, margin, cp

def stance_foot_speeds(active_leg):
    """Returns {leg: horizontal world-frame speed (m/s)} for each currently
       planted (non-active) leg. A genuinely planted foot should read ~0 -
       it's not supposed to move relative to the ground while supporting the
       body. A spike here (especially lining up with pitch running away)
       points to that leg's position-servo joints failing to hold their
       angle under load (losing grip/slipping), not a gait-geometry
       problem."""
    support_legs = [l for l in legs if l != active_leg]  # the currently planted legs
    out = {}  # leg -> speed result dict
    for l in support_legs:  # loop over each planted leg
        vx, vy, _vz = foot_world_vel.get(l, (0.0, 0.0, 0.0))  # this leg's world-frame velocity
        out[l] = math.hypot(vx, vy)  # horizontal speed of this foot
    return out

def stance_foot_velocities(active_leg):
    """Returns {leg: (vx, vy)} signed world-frame velocity for each currently
       planted (non-active) leg. Unlike stance_foot_speeds() (magnitude only),
       this lets us check whether front and back stance legs slide in the
       same or opposite direction during a shift/push - the concrete,
       checkable version of "front/back joint axis convention fights
       itself". If that were true, front and back stance-leg vx would show
       consistently opposite sign while sliding; if LEG_SIDE in leg_ik is
       correct, there's no reason to expect a systematic sign split."""
    support_legs = [l for l in legs if l != active_leg]  # the currently planted legs
    out = {}  # leg -> (vx, vy) result dict
    for l in support_legs:  # loop over each planted leg
        vx, vy, _vz = foot_world_vel.get(l, (0.0, 0.0, 0.0))  # this leg's world-frame velocity
        out[l] = (vx, vy)  # store signed x/y velocity for this foot
    return out

def all_foot_fx():
    """Returns {leg: current body-frame fx target} for all four legs. Checks
       the "leg near full extension" concern around FX_LIMIT: rotate()'s
       pitch/roll correction can't change a foot's reach (it's a pure
       rotation, so r=sqrt(fx^2+fz^2) doesn't change), but repeated
       same-direction shifts could walk fx toward the +-FX_LIMIT=0.09 clamp,
       where r=0.394m and cos(knee)=0.945 - notably closer to the 0.40m
       singularity than the nominal cos(knee)=0.843 at fx=0. Logging fx
       directly checks whether that's actually happening instead of
       guessing."""
    return {l: foot_target[l][0] for l in legs}

# foot_target[leg] = (fx, fz) in the leg's own nominal (untilted) body frame - drives HIP/KNEE via leg_ik
foot_target = {leg: (0.0, -0.4) for leg in legs}  # leg -> current foot xz target, all legs start the same
# abad_cmd[leg] = direct ABAD joint angle command (radians) - the base lateral trim for that leg,
# separate from the auto roll-correction term added in control_loop()
abad_cmd = {leg: 0.0 for leg in legs}  # leg -> current abad angle command

PITCH_SIGN = 1.0  # flips the sign of the pitch correction, if it's backwards
CORRECTION_FRACTION = 0.4
MAX_CORRECTION_RAD = 0.35  # largest pitch correction allowed, in radians

PITCH_RATE_DAMPING = 0.15  # gain for the pitch-rate (D-term) correction

ROLL_ABAD_FRACTION = 0.3   # matches the pitch controller's gain as a starting point
MAX_ABAD_ROLL_CORR = 0.15  # radians


running = [True]  # whether the control loop should keep running

FLIP_LIMIT_DEG = 25.0  # tip angle, in degrees, that counts as a flip
FLIP_LIMIT_RAD = math.radians(FLIP_LIMIT_DEG)  # flip limit angle, in radians
aborted = [False]  # whether the safety abort has triggered

def check_abort():
    if aborted[0]:  # already aborted, so nothing more to check
        return True
    if abs(latest_pitch[0]) > FLIP_LIMIT_RAD or abs(latest_roll[0]) > FLIP_LIMIT_RAD:  # check if the robot has tipped past the safety limit
        aborted[0] = True  # mark that the abort has triggered
        print(f"!!! SAFETY ABORT: |pitch|={math.degrees(latest_pitch[0]):.1f} deg  "
              f"|roll|={math.degrees(latest_roll[0]):.1f} deg exceeded {FLIP_LIMIT_DEG} deg - "
              f"halting gait sequence (joint commands keep publishing, clamped) !!!")
    return aborted[0]

CONTROL_DT = 0.02   # matches the sleep() at the bottom of this loop

last_theta_terms = {"p": 0.0, "d": 0.0, "raw": 0.0, "clamped": 0.0}  # latest pitch-correction terms, for logging

def control_loop():
    while running[0]:  # keep looping as long as the control loop is active
        theta_p = PITCH_SIGN * CORRECTION_FRACTION * latest_pitch[0]  # proportional pitch correction term
        theta_d = PITCH_SIGN * PITCH_RATE_DAMPING * latest_pitch_rate[0]  # derivative pitch correction term
        theta_raw = theta_p + theta_d  # combined correction before clamping
        theta = max(-MAX_CORRECTION_RAD, min(MAX_CORRECTION_RAD, theta_raw))  # clamped pitch correction angle
        last_theta_terms["p"] = theta_p  # log the p term
        last_theta_terms["d"] = theta_d  # log the d term
        last_theta_terms["raw"] = theta_raw  # log the raw combined term
        last_theta_terms["clamped"] = theta  # log the clamped final term

        roll_term = ROLL_ABAD_FRACTION * latest_roll[0]  # proportional roll correction term
        roll_term = max(-MAX_ABAD_ROLL_CORR, min(MAX_ABAD_ROLL_CORR, roll_term))  # clamp roll correction

        for leg in legs:  # loop over each of the 4 legs
            fx, fz = foot_target[leg]  # this leg's target foot position
            fx_c, fz_c = rotate(fx, fz, theta)  # foot target rotated by pitch correction
            hip, knee = leg_ik(fx_c, fz_c, LEG_SIDE[leg])  # resulting hip/knee joint angles

            abad = abad_cmd[leg] + roll_term  # final abad command for this leg

            m0 = Double(); m0.data = abad  # joint command message for abad
            pubs[f"{leg}_ABAD"].publish(m0)
            m1 = Double(); m1.data = hip  # joint command message for hip
            pubs[f"{leg}_HIP"].publish(m1)
            m2 = Double(); m2.data = knee  # joint command message for knee
            pubs[f"{leg}_KNEE"].publish(m2)
        time.sleep(CONTROL_DT)

t = threading.Thread(target=control_loop, daemon=True)  # background thread running the control loop
t.start()

def move_feet(deltas, duration=1.5, steps=75, label=None, active_leg=None):
    """deltas: {leg: (target_fx, target_fz)}. Ramps foot_target for those legs
       together. If label is given, prints pitch/roll periodically during the
       ramp, not just after. active_leg: if given, also prints support-polygon
       status during the ramp - matters because a safety abort often fires
       mid-ramp (check_abort() returns before step_leg reaches its own
       watch() call), so this is the only place that catches the support
       polygon at the actual moment of collapse."""
    starts = {leg: foot_target[leg] for leg in deltas}  # starting foot positions before the ramp
    print_every = max(1, steps // 15)  # how often to print progress during the ramp
    for i in range(1, steps + 1):  # loop over each step of the ramp
        if check_abort():  # stop this ramp early if the robot already tipped over
            return
        frac = i / steps  # fraction of the ramp completed
        for leg, (tx, tz) in deltas.items():  # loop over each leg being moved
            sx, sz = starts[leg]  # this leg's starting position
            foot_target[leg] = (sx + (tx-sx)*frac, sz + (tz-sz)*frac)  # interpolated foot position this step
        if label and (i % print_every == 0 or i == 1):  # only log at intervals, or on the first step
            fr_z = link_xyz.get("FR", (None, None, None))[2]  # FR foot height, for logging
            line = (f"    ({label} {frac*100:3.0f}%) pitch: {math.degrees(latest_pitch[0]):+.2f} deg  roll: {math.degrees(latest_roll[0]):+.2f} deg  FR_shank_z: {fr_z}"  # start of the per-leg diagnostic log line
                    f"  pitch_rate({_pitch_rate_source[0]}): {math.degrees(latest_pitch_rate[0]):+.2f} deg/s"
                    f"  theta(p={math.degrees(last_theta_terms['p']):+.2f} d={math.degrees(last_theta_terms['d']):+.2f} -> {math.degrees(last_theta_terms['clamped']):+.2f} deg)")
            if active_leg is not None:  # only check support-triangle status when a leg is given
                inside, margin = support_status(active_leg)  # whether CoM is inside support triangle, and margin
                if inside is not None:  # only log if the com check produced a result
                    line += f"  CoM: inside={inside} margin={margin:+.4f}"
                zmp_in, zmp_margin, _zmp_pt = zmp_status(active_leg)  # whether zmp is inside support triangle, and margin
                if zmp_in is not None:  # only log if the zmp check produced a result
                    line += f"  ZMP: inside={zmp_in} margin={zmp_margin:+.4f}"
                cp_in, cp_margin, _cp_pt = capture_point_status(active_leg)  # whether capture point is inside triangle, and margin
                if cp_in is not None:  # only log if the capture-point check produced a result
                    line += f"  CapturePt: inside={cp_in} margin={cp_margin:+.4f}"
                speeds = stance_foot_speeds(active_leg)  # horizontal speed of each planted foot
                line += "  stance_foot_speed: {" + " ".join(f"{l}:{s:.3f}" for l, s in speeds.items()) + "}"
                vels = stance_foot_velocities(active_leg)  # signed x/y velocity of each planted foot
                line += "  stance_foot_vxy: {" + " ".join(f"{l}:({vx:+.3f},{vy:+.3f})" for l, (vx, vy) in vels.items()) + "}"
            fx_all = all_foot_fx()  # current fx target for every leg
            line += "  fx: {" + " ".join(f"{l}:{v:+.4f}" for l, v in fx_all.items()) + "}"
            print(line)
        time.sleep(duration / steps)
    for leg, tgt in deltas.items():  # loop to set final exact target position
        foot_target[leg] = tgt  # snap foot target to exact final value

def move_abad(deltas, duration=2.0, steps=100, label=None):
    """deltas: {leg: target_abad_angle_rad}. Ramps abad_cmd for those legs together."""
    starts = {leg: abad_cmd[leg] for leg in deltas}  # starting abad angles before the ramp
    print_every = max(1, steps // 15)  # how often to print progress during the ramp
    for i in range(1, steps + 1):  # loop over each step of the ramp
        if check_abort():  # stop this ramp early if the robot already tipped over
            return
        frac = i / steps  # fraction of the ramp completed
        for leg, target in deltas.items():  # loop over each leg being moved
            s = starts[leg]  # this leg's starting abad angle
            abad_cmd[leg] = s + (target - s) * frac  # interpolated abad angle this step
        if label and (i % print_every == 0 or i == 1):  # only log at intervals, or on the first step
            print(f"    ({label} {frac*100:3.0f}%) pitch: {math.degrees(latest_pitch[0]):+.2f} deg  roll: {math.degrees(latest_roll[0]):+.2f} deg")
        time.sleep(duration / steps)
    for leg, target in deltas.items():  # loop to set final exact abad angles
        abad_cmd[leg] = target  # snap abad angle to exact final value

def watch(seconds, label, active_leg=None):
    """active_leg: if given, also checks/prints whether the body is inside the support triangle
       formed by the other three (planted) legs' feet - see support_status() above."""
    for _ in range(int(seconds / 0.1)):  # loop once per 0.1s tick for the watch duration
        if check_abort():  # stop watching early if the robot already tipped over
            return
        fr_z = link_xyz.get("FR", (None, None, None))[2]  # FR foot height, for logging
        vx, vy, vz = body_vel  # current body velocity components
        ax, ay, az = body_accel  # current body acceleration components
        speed = math.sqrt(vx*vx + vy*vy)  # horizontal body speed
        line = (f"  [{label}] pitch: {math.degrees(latest_pitch[0]):+.2f} deg  roll: {math.degrees(latest_roll[0]):+.2f} deg  "  # start of the watch diagnostic log line
                f"FR_shank_z: {fr_z}  body_xyz: {body_xyz}  vel(x,y,z): ({vx:+.3f},{vy:+.3f},{vz:+.3f})  speed_xy: {speed:.3f}  "
                f"accel(x,y,z): ({ax:+.3f},{ay:+.3f},{az:+.3f})"
                f"  pitch_rate({_pitch_rate_source[0]}): {math.degrees(latest_pitch_rate[0]):+.2f} deg/s"
                f"  theta(p={math.degrees(last_theta_terms['p']):+.2f} d={math.degrees(last_theta_terms['d']):+.2f} -> {math.degrees(last_theta_terms['clamped']):+.2f} deg)")
        if active_leg is not None:  # only check support-triangle status when a leg is given
            inside, margin = support_status(active_leg)  # whether CoM is inside support triangle, and margin
            if inside is not None:  # only log if the com check produced a result
                line += f"  CoM: inside={inside} margin={margin:+.4f}"
            zmp_in, zmp_margin, _zmp_pt = zmp_status(active_leg)  # whether zmp is inside support triangle, and margin
            if zmp_in is not None:  # only log if the zmp check produced a result
                line += f"  ZMP: inside={zmp_in} margin={zmp_margin:+.4f}"
            cp_in, cp_margin, _cp_pt = capture_point_status(active_leg)  # whether capture point is inside triangle, and margin
            if cp_in is not None:  # only log if the capture-point check produced a result
                line += f"  CapturePt: inside={cp_in} margin={cp_margin:+.4f}"
            speeds = stance_foot_speeds(active_leg)  # horizontal speed of each planted foot
            line += "  stance_foot_speed: {" + " ".join(f"{l}:{s:.3f}" for l, s in speeds.items()) + "}"
            vels = stance_foot_velocities(active_leg)  # signed x/y velocity of each planted foot
            line += "  stance_foot_vxy: {" + " ".join(f"{l}:({vx:+.3f},{vy:+.3f})" for l, (vx, vy) in vels.items()) + "}"
        fx_all = all_foot_fx()  # current fx target for every leg
        line += "  fx: {" + " ".join(f"{l}:{v:+.4f}" for l, v in fx_all.items()) + "}"
        print(line)
        time.sleep(0.1)

print("waiting for the drop to settle...")
time.sleep(5.0)
print(f"landed, pitch = {math.degrees(latest_pitch[0]):.2f} deg  roll = {math.degrees(latest_roll[0]):.2f} deg")
print(f"body position: {body_xyz}")

STANCE_FZ = -0.384  # foot height while a leg is planted and bearing weight
LIFT_FZ = -0.304  # foot height a back leg lifts to during its swing
SHIFT_MAG_FRONT = 0.05  # how far front legs shift sideways to rebalance, in meters
SHIFT_MAG_BACK = 0.03  # how far back legs shift sideways to rebalance, in meters
FX_LIMIT = 0.09  # max fore-aft foot reach allowed, in meters
SHIFT_DURATION_FRONT = 2.0  # how long a front leg's weight shift takes, in seconds
LIFT_FZ_FRONT = -0.34  # foot height a front leg lifts to during its swing
LIFT_DURATION_FRONT = 1.5  # how long a front leg's lift takes, in seconds

SWING_MAG_FRONT = 0.04       # was 0.06 shared
SWING_MAG_BACK = 0.06  # how far back legs reach forward during swing, in meters
PUSH_MAG_FRONT = 0.01        # was 0.02, before that 0.035 shared
PUSH_MAG_BACK = 0.035  # how far back legs drag back during push, in meters
PLACE_DURATION_FRONT = 3.0   # was 2.5
PUSH_DURATION_FRONT = 3.0    # was 2.5

CAPTURE_HEIGHT = 0.40  # settled crouch height, in meters
CAPTURE_GAIN = math.sqrt(CAPTURE_HEIGHT / 9.8)   # ~0.202

CAPTURE_SIGN = 1.0  # sign of the capture-point correction, flip if it's backwards
MAX_CAPTURE_CORRECTION = 0.03   # clamp, same units as swing_mag (m)

def capture_point_correction():
    """Sampled the moment a leg commits to its swing target - returns a
       bounded fore-aft foot-offset correction based on the body's current
       (finite-differenced, ground-truth) velocity. See CAPTURE_GAIN/
       CAPTURE_SIGN above."""
    vx = body_vel[0]  # current body x velocity
    corr = CAPTURE_SIGN * CAPTURE_GAIN * vx  # raw capture-point correction
    return max(-MAX_CAPTURE_CORRECTION, min(MAX_CAPTURE_CORRECTION, corr))

def clamp(v):  # v: value to clamp within the fx limit
    return max(-FX_LIMIT, min(FX_LIMIT, v))

def step_leg(leg, settle=0.3):  # leg: which leg is stepping; settle: pause length after each phase
    """One full step cycle (shift/lift/swing/place/push) for a single leg.
       ABAD roll correction keeps running independently in the background
       control_loop the whole time. Checks check_abort() between every phase
       so a flip stops the sequence instead of grinding through the rest of
       the gait (and padding the log) on a robot that's already down."""
    others = [l for l in legs if l != leg]  # the three legs not currently stepping
    shift_sign = -1.0 if leg in ("BL", "BR") else 1.0  # which direction to shift weight
    is_front = leg in ("FL", "FR")  # whether this is a front leg
    shift_mag = SHIFT_MAG_FRONT if is_front else SHIFT_MAG_BACK  # how far to shift weight sideways
    shift_duration = SHIFT_DURATION_FRONT if is_front else 1.0  # how long the weight shift takes
    lift_fz = LIFT_FZ_FRONT if is_front else LIFT_FZ  # foot height during lift
    lift_duration = LIFT_DURATION_FRONT if is_front else 0.5  # how long the lift takes
    swing_duration = 1.0 if is_front else 0.5  # how long the swing takes
    place_duration = PLACE_DURATION_FRONT if is_front else 0.5  # how long placing the foot takes
    push_duration = PUSH_DURATION_FRONT if is_front else 0.5  # how long the push takes
    swing_mag_nominal = SWING_MAG_FRONT if is_front else SWING_MAG_BACK  # default forward reach during swing
    push_mag = PUSH_MAG_FRONT if is_front else PUSH_MAG_BACK  # how far to drag the foot back during push

    if check_abort(): return  # stop this step early if the robot already tipped over
    print(f"--- {leg}: shift ---")
    move_feet({l: (clamp(foot_target[l][0] + shift_sign * shift_mag), STANCE_FZ) for l in others}, duration=shift_duration, label=f"{leg} shift")
    watch(settle, f"{leg} shift", active_leg=leg)

    if check_abort(): return  # stop this step early if the robot already tipped over
    print(f"--- {leg}: lift ---")
    move_feet({leg: (foot_target[leg][0], lift_fz)}, duration=lift_duration, label=f"{leg} lift", active_leg=leg)
    watch(settle, f"{leg} lift", active_leg=leg)

    if check_abort(): return  # stop this step early if the robot already tipped over
    capture_corr = capture_point_correction()  # fore-aft correction from body momentum
    swing_mag = swing_mag_nominal + capture_corr  # actual swing reach after correction
    print(f"--- {leg}: swing (v_x={body_vel[0]:+.3f}  capture_corr={capture_corr:+.4f}  "
          f"swing_mag {swing_mag_nominal:.3f} -> {swing_mag:.3f}) ---")
    move_feet({leg: (swing_mag, lift_fz)}, duration=swing_duration, label=f"{leg} swing", active_leg=leg)
    watch(settle, f"{leg} swing", active_leg=leg)

    if check_abort(): return  # stop this step early if the robot already tipped over
    print(f"--- {leg}: place ---")
    move_feet({leg: (swing_mag, STANCE_FZ)}, duration=place_duration, label=f"{leg} place", active_leg=leg)
    watch(settle, f"{leg} place", active_leg=leg)

    if check_abort(): return  # stop this step early if the robot already tipped over
    print(f"--- {leg}: push ---")
    move_feet({leg: (swing_mag - push_mag, STANCE_FZ)}, duration=push_duration, label=f"{leg} push", active_leg=leg)
    watch(settle, f"{leg} push", active_leg=leg)

print("--- crouch (hip/knee only, all ABAD at 0) ---")
move_feet({leg: (0.0, STANCE_FZ) for leg in legs}, duration=1.5)
watch(2.0, "crouch")

start_xyz = list(body_xyz)  # body position before the gait loop starts
GAIT_ORDER = ["BR", "FR", "BL", "FL"]  # order the legs take turns stepping in
N_CYCLES = 2  # how many times to repeat the full gait cycle
INTER_LEG_SETTLE = 1.5
for cycle in range(N_CYCLES):  # loop over each full gait cycle
    if aborted[0]:  # stop the whole gait early if a safety abort fired
        print(f"--- gait loop stopped early before cycle {cycle+1}: safety abort triggered ---")
        break
    print(f"=== cycle {cycle+1} ===")
    for leg in GAIT_ORDER:  # loop over each leg in stepping order
        if aborted[0]:  # skip the rest of this cycle's legs if aborted
            break
        step_leg(leg)
        watch(INTER_LEG_SETTLE, f"settle after {leg}")

end_xyz = list(body_xyz)  # body position after the gait loop finishes
dx = end_xyz[0] - start_xyz[0]   # how far the body moved in x
dy = end_xyz[1] - start_xyz[1]  # how far the body moved in y
dist = math.sqrt(dx*dx + dy*dy)  # total straight-line distance moved
print(f"net displacement: dx={dx:.3f} dy={dy:.3f}  total distance={dist:.3f} m")
print(f"final body z: {end_xyz[2]:.3f}  (collapsed if well below ~0.35)")
watch(2.0, "end of loop")

print("sequence stopped early (safety abort)" if aborted[0] else "sequence complete")
running[0] = False  # signal the control loop thread to stop
