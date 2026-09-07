"""
Script: gallop_demo.py
Author: Arda Gencer
Experimental bound/half-bound gait - a first step toward a real gallop, not yet a true rotary gallop.
Detailed tuning-history notes for the constants below are in tuning_history/gallop_demo_history.txt
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

# Logging: same tee-to-file setup as trot_demo.py/manual_control.py, just its own log
# file so it doesn't collide with either.
_LOG_NAME = "run_log_gallop.txt"   # filename this run's log gets written to
_ARCHIVE_DIR = "run_log_archive"   # folder where old log files get moved before a new run
try:
    os.makedirs(_ARCHIVE_DIR, exist_ok=True)
    if os.path.exists(_LOG_NAME):   # only archive if a log from a previous run exists
        _ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")   # timestamp for the archived filename
        shutil.copy2(_LOG_NAME, os.path.join(_ARCHIVE_DIR, f"{_ts}_{_LOG_NAME}"))   # move the old log into the archive folder
except OSError:
    pass

_log_file = open(_LOG_NAME, "w")   # file handle for this run's log output

class _Tee:
    def __init__(self, *streams):
        self.streams = streams   # the output streams to duplicate writes to
    def write(self, data):
        for s in self.streams:   # each output stream (console, log file)
            s.write(data)
            s.flush()
    def flush(self):
        for s in self.streams:   # each output stream (console, log file)
            s.flush()

sys.stdout = _Tee(sys.stdout, _log_file)   # send prints to console and log file both

def log_line(text):
    print(text, file=_log_file)
    _log_file.flush()

# Leg IK - same as trot_demo.py, this is just robot geometry, nothing gait-specific.
L1 = 0.2   # length of the leg's upper segment, in meters
L2 = 0.2   # length of the leg's lower segment, in meters

def leg_ik(fx, fz, s):
    """fx, fz = desired foot position relative to the hip, in body frame (fz negative = below hip).
       s = +1 for front legs, -1 for back. Returns (hip, knee) angles."""
    u = s * fx   # foot x position, mirrored for back legs
    w = fz   # foot z position (same as fz here)
    r2 = u*u + w*w   # squared distance from hip to foot
    r2 = max(r2, 1e-9)   # avoid divide-by-zero on a zero-length target
    c = (r2 - L1*L1 - L2*L2) / (2*L1*L2)   # law-of-cosines term for the knee angle
    c = max(-1.0, min(1.0, c))   # clamp into the valid range for acos
    knee = -math.acos(c)   # knee joint angle
    k1 = L1 + L2*math.cos(knee)   # helper term for solving the hip angle
    k2 = L2*math.sin(knee)   # helper term for solving the hip angle
    sin_a = (u*k1 + k2*w) / r2   # sine component of the hip angle
    cos_a = (k2*u - k1*w) / r2   # cosine component of the hip angle
    hip = math.atan2(sin_a, cos_a)   # hip joint angle
    return hip, knee

def rotate(fx, fz, theta):
    """Rotate a foot-target vector by theta (same convention as the leg's own swing rotation)."""
    return (fx*math.cos(theta) - fz*math.sin(theta),
            fx*math.sin(theta) + fz*math.cos(theta))

# 3-DOF leg IK (ABAD abduction + the existing HIP/KNEE 2-link) - same as champgait.py.
D_ABAD = 0.1
OY = {"FL": 1, "FR": -1, "BL": 1, "BR": -1}
FRONT_BACK = {"FL": 1, "FR": 1, "BL": -1, "BR": -1}   # used by the fy-based yaw steering below,
                                                        # +1 front, -1 back, regardless of side.

def leg_ik_3d(fx, fy, fz, oy, s):
    """fx, fy, fz = desired foot position relative to the leg's ABAD pivot, in body-frame axes.
       oy = OY[leg] (+1 left, -1 right), s = LEG_SIDE[leg] (+1 front, -1 back). Returns
       (abad, hip, knee). Full geometric derivation is in champgait.py's header note - unchanged here."""
    dy = oy * D_ABAD + fy   # target y position relative to the ABAD pivot
    dz = fz   # target z position relative to the ABAD pivot
    r = math.hypot(dy, dz)   # distance from the ABAD axis to the target
    r = max(r, D_ABAD + 1e-6)   # can't be closer to the ABAD axis than the arm length itself -
                                # clamp instead of crashing on an unreachable target
    c = max(-1.0, min(1.0, (oy * D_ABAD) / r))   # clamped ratio used to solve the abduction angle
    base = math.atan2(dz, dy)   # base angle toward the target
    phi_a = base + math.acos(c)   # one candidate abduction solution
    phi_b = base - math.acos(c)   # the other candidate abduction solution
    abad = phi_a if abs(phi_a) < abs(phi_b) else phi_b   # take the smaller-abduction solution
    w = -dy*math.sin(abad) + dz*math.cos(abad)           # effective fz inside the tilted 2-link plane
    hip, knee = leg_ik(fx, w, s)   # solve the remaining 2-link hip/knee angles
    return abad, hip, knee

LEG_SIDE = {"FL": 1, "FR": 1, "BL": -1, "BR": -1}   # which legs are front (+1) vs back (-1)
LEG_LR = {"FL": 1, "FR": -1, "BL": 1, "BR": -1}   # which legs are left (+1) vs right (-1)
legs = ["FL", "FR", "BL", "BR"]   # the four leg names used throughout this file

node = transport.Node()   # gz-transport node used to publish/subscribe
pubs = {}   # joint command publishers, keyed by "<leg>_<joint>"
for leg in legs:   # set up publishers for each of the 4 legs
    pubs[f"{leg}_ABAD"] = node.advertise(f"/model/my_quadruped/joint/{leg}_ABAD/cmd_pos", Double)
    pubs[f"{leg}_HIP"] = node.advertise(f"/model/my_quadruped/joint/{leg}_HIP/cmd_pos", Double)
    pubs[f"{leg}_KNEE"] = node.advertise(f"/model/my_quadruped/joint/{leg}_KNEE/cmd_pos", Double)

# IMU: orientation + filtered rates, same as trot_demo.py's validated version.
latest_pitch = [0.0]   # current pitch angle, in a 1-item box so threads share it
latest_roll = [0.0]   # current roll angle, shared across threads
latest_yaw = [0.0]   # current yaw angle, shared across threads

PITCH_RATE_LPF_ALPHA = 0.10   # how much new data vs old data shapes the smoothed pitch rate
latest_pitch_rate = [0.0]   # smoothed pitch rate, shared across threads
_pitch_rate_source = [None]   # whether pitch rate came from the gyro or finite difference
_prev_pitch_for_rate = [None]   # last pitch value, used to compute rate by finite difference
_prev_pitch_rate_time = [None]   # timestamp of that last pitch value

YAW_RATE_LPF_ALPHA = 0.2   # how much new data vs old data shapes the smoothed yaw rate
latest_yaw_rate = [0.0]   # smoothed yaw rate, shared across threads
_yaw_rate_source = [None]   # whether yaw rate came from the gyro or finite difference
_prev_yaw_for_rate = [None]   # last yaw value, used to compute rate by finite difference
_prev_yaw_rate_time = [None]   # timestamp of that last yaw value

def imu_callback(msg):
    q = msg.orientation   # quaternion orientation from the IMU message
    sinp = max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))   # clamped sine-of-pitch term
    latest_pitch[0] = math.asin(sinp)
    latest_roll[0] = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    latest_yaw[0] = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    raw_rate = None   # pitch rate before smoothing, filled in below
    try:
        raw_rate = msg.angular_velocity.y   # pitch rate straight from the gyro
        _pitch_rate_source[0] = "gyro"
    except AttributeError:
        _pitch_rate_source[0] = "fd"
        now = time.time()   # current time, for the finite-difference fallback
        if _prev_pitch_for_rate[0] is not None and _prev_pitch_rate_time[0] is not None:   # only compute rate once we have a prior sample
            dt = now - _prev_pitch_rate_time[0]   # time since the last pitch sample
            if dt > 1e-4:   # skip if barely any time has passed, to avoid a divide-by-zero
                raw_rate = (latest_pitch[0] - _prev_pitch_for_rate[0]) / dt   # pitch rate by finite difference
        _prev_pitch_for_rate[0] = latest_pitch[0]
        _prev_pitch_rate_time[0] = now
    if raw_rate is not None:   # only update the smoothed rate if we actually got a new reading
        latest_pitch_rate[0] = (PITCH_RATE_LPF_ALPHA * raw_rate   # blend the new rate into the smoothed value
                                 + (1.0 - PITCH_RATE_LPF_ALPHA) * latest_pitch_rate[0])

    yaw_raw_rate = None   # yaw rate before smoothing, filled in below
    try:
        yaw_raw_rate = msg.angular_velocity.z   # yaw rate straight from the gyro
        _yaw_rate_source[0] = "gyro"
    except AttributeError:
        _yaw_rate_source[0] = "fd"
        now2 = time.time()   # current time, for the finite-difference fallback
        if _prev_yaw_for_rate[0] is not None and _prev_yaw_rate_time[0] is not None:   # only compute rate once we have a prior sample
            dt2 = now2 - _prev_yaw_rate_time[0]   # time since the last yaw sample
            if dt2 > 1e-4:   # skip if barely any time has passed, to avoid a divide-by-zero
                yaw_raw_rate = (latest_yaw[0] - _prev_yaw_for_rate[0]) / dt2   # yaw rate by finite difference
        _prev_yaw_for_rate[0] = latest_yaw[0]
        _prev_yaw_rate_time[0] = now2
    if yaw_raw_rate is not None:   # only update the smoothed rate if we actually got a new reading
        latest_yaw_rate[0] = (YAW_RATE_LPF_ALPHA * yaw_raw_rate   # blend the new rate into the smoothed value
                               + (1.0 - YAW_RATE_LPF_ALPHA) * latest_yaw_rate[0])

node.subscribe(IMU, "/model/my_quadruped/imu", imu_callback)

# Ground-truth body pose + per-foot world z, for diagnostics only.
body_xyz = [None, None, None]   # latest body position, x/y/z
body_vel = [0.0, 0.0, 0.0]   # latest body velocity, x/y/z
_prev_body_xyz = [None, None, None]   # previous body position, for finite-difference velocity
_prev_pose_time = [None]   # timestamp of that previous body position
link_z = {}   # latest world z height for each foot, keyed by leg name

def pose_callback(msg):
    for p in msg.pose:   # each object's pose in the message
        if p.name == "my_quadruped":   # check if this pose entry is the robot's own body
            now = time.time()   # current time, for computing velocity
            if _prev_pose_time[0] is not None:   # skip velocity calc on the very first sample
                dt = now - _prev_pose_time[0]   # time since the last pose sample
                if dt > 1e-4:   # skip if barely any time has passed, to avoid a divide-by-zero
                    body_vel[0] = (p.position.x - _prev_body_xyz[0]) / dt   # velocity by finite difference
                    body_vel[1] = (p.position.y - _prev_body_xyz[1]) / dt
                    body_vel[2] = (p.position.z - _prev_body_xyz[2]) / dt
            _prev_body_xyz[0], _prev_body_xyz[1], _prev_body_xyz[2] = p.position.x, p.position.y, p.position.z
            _prev_pose_time[0] = now
            body_xyz[0], body_xyz[1], body_xyz[2] = p.position.x, p.position.y, p.position.z
        else:
            for leg_name in legs:   # check this pose against each leg's shank link
                if p.name.endswith(f"{leg_name}_shank"):   # check if this pose belongs to this leg's foot link
                    link_z[leg_name] = p.position.z
                    break

node.subscribe(Pose_V, "/world/empty/pose/info", pose_callback)

# Gait timing config - this is where bound actually differs from trot.
STANCE_FZ = -0.34   # foot height while standing on the ground, in meters below the hip
SWING_HEIGHT = 0.05   # how high a foot lifts during its swing, in meters
TROT_FX_LIMIT = 0.12   # max forward/backward foot reach, in meters

GALLOP_PERIOD = 1.0

STANCE_DUTY = 0.6
SWING_DUTY = 1.0 - STANCE_DUTY   # fraction of each cycle a leg spends swinging

GALLOP_PHASE_LAG = 0.15

DESIRED_VX = 0.10

def clamp_fx(v):
    return max(-TROT_FX_LIMIT, min(TROT_FX_LIMIT, v))

def _smoothstep(x):
    x = max(0.0, min(1.0, x))   # clamp input into 0..1 before shaping
    return x * x * (3 - 2 * x)

# Front/back pairing instead of trot's diagonal pairing - this is the actual gait change.
# FRONT_PAIR's legs swing together, then (after GALLOP_PHASE_LAG) BACK_PAIR's do.
FRONT_PAIR = ["FL", "FR"]
BACK_PAIR = ["BL", "BR"]
PAIR_PHASE_OFFSET = {}   # how far behind (in cycle fraction) each leg's pair starts, filled in below
for leg in FRONT_PAIR:   # front pair starts at phase 0 (no lag)
    PAIR_PHASE_OFFSET[leg] = 0.0
for leg in BACK_PAIR:   # back pair starts lagged behind the front pair
    PAIR_PHASE_OFFSET[leg] = GALLOP_PHASE_LAG

def leg_phase_frac(leg, t):
    global_phase = (t % GALLOP_PERIOD) / GALLOP_PERIOD   # fraction of the way through the current cycle
    return (global_phase - PAIR_PHASE_OFFSET[leg]) % 1.0

def leg_is_swinging(leg, t):
    return leg_phase_frac(leg, t) < SWING_DUTY

def current_swing_pair(t):
    if leg_is_swinging(FRONT_PAIR[0], t):   # check if the front pair is currently swinging
        return "FRONT"
    if leg_is_swinging(BACK_PAIR[0], t):   # check if the back pair is currently swinging
        return "BACK"
    return None

def foot_target_for_leg(leg, t, vx_meas):   # vx_meas = measured forward speed, used to size the stride
    local_phase = leg_phase_frac(leg, t)   # this leg's own position within its cycle
    A = 0.5 * vx_meas * STANCE_DUTY * GALLOP_PERIOD   # half the fore-aft stride length

    if local_phase < SWING_DUTY:   # check if this leg is currently in its swing phase
        swing_frac = local_phase / SWING_DUTY   # progress through the swing phase, 0 to 1
        s = _smoothstep(swing_frac)   # eased swing progress
        fx = -A + 2 * A * s   # foot x target while swinging
        fz = STANCE_FZ + SWING_HEIGHT * math.sin(math.pi * swing_frac)   # foot z target while swinging (arced up)
    else:
        stance_frac = (local_phase - SWING_DUTY) / STANCE_DUTY   # progress through the stance phase, 0 to 1
        fx = A - 2 * A * stance_frac   # foot x target while planted (sweeps backward)
        fz = STANCE_FZ   # foot stays at ground height while planted
    return clamp_fx(fx), fz

STRIDE_HALF_AMPLITUDE = 0.5 * DESIRED_VX * STANCE_DUTY * GALLOP_PERIOD   # half the fore-aft stride length, in meters

PITCH_SIGN = 1.0   # flips the direction of the pitch correction if needed
CORRECTION_FRACTION = 0.25   # how strongly pitch angle gets corrected
MAX_CORRECTION_RAD = 0.35   # biggest pitch correction allowed, in radians
PITCH_RATE_DAMPING = 0.075   # how strongly pitch rate (speed of tilting) gets corrected
IDLE_CORRECTION_FRACTION = 0.047   # pitch correction strength while standing still, not gaiting
IDLE_PITCH_RATE_DAMPING = 0.0142   # pitch rate damping while standing still, not gaiting
ROLL_ABAD_FRACTION = 0.0   # how strongly roll angle gets corrected via the ABAD joints
MAX_ABAD_ROLL_CORR = 0.15   # biggest roll correction allowed, in radians

FEEDFORWARD_PITCH_BIAS_RAD = math.radians(3.0)

# Yaw correction - same structure as trot_demo.py, applied only to the pair currently swinging.
YAW_FX_GAIN = 0.15   # how strongly yaw error steers the forward/back foot position
MAX_YAW_FX = 0.06   # biggest forward/back foot shift allowed for yaw correction, in meters
YAW_FY_GAIN = -0.07   # how strongly yaw error steers the side-to-side foot position
MAX_YAW_FY = 0.08   # biggest side-to-side foot shift allowed for yaw correction, in meters
YAW_KI = 0.3   # how strongly the accumulated yaw error gets corrected
YAW_INTEGRAL_LIMIT = 0.3   # cap on the accumulated yaw error, to avoid runaway correction
yaw_integral = [0.0]   # accumulated (integral) yaw error

gait_start_yaw = [None]   # yaw angle recorded when the gait started
gait_start_time = [None]   # timestamp when the gait started
coast_to_stop = [False]   # whether the robot is winding down to a stop

foot_target = {leg: (0.0, STANCE_FZ) for leg in legs}   # current (fx, fz) target per leg

running = [True]   # whether the control loop should keep running
FLIP_LIMIT_DEG = 25.0   # pitch/roll angle, in degrees, considered a flip/fall
FLIP_LIMIT_RAD = math.radians(FLIP_LIMIT_DEG)   # same flip limit, in radians
aborted = [False]   # whether a safety abort has been triggered
gait_active = [False]   # whether the gait is currently being commanded

def check_abort():
    if aborted[0]:   # already aborted, nothing more to check
        return True
    if abs(latest_pitch[0]) > FLIP_LIMIT_RAD or abs(latest_roll[0]) > FLIP_LIMIT_RAD:   # check if the robot has tipped past a safe angle
        aborted[0] = True   # trigger the safety abort
        print(f"!!! SAFETY ABORT: |pitch|={math.degrees(latest_pitch[0]):.1f} deg  "
              f"|roll|={math.degrees(latest_roll[0]):.1f} deg exceeded {FLIP_LIMIT_DEG} deg !!!")
    return aborted[0]

SPEED_SETTLE_THRESHOLD = 0.06   # speed, in m/s, below which the robot counts as stopped

CONTROL_DT = 0.02   # time between control loop updates, in seconds
last_theta_terms = {"p": 0.0, "d": 0.0, "ff": 0.0, "clamped": 0.0, "saturated": False}   # latest pitch-correction terms, for logging
last_yaw_terms = {"err": 0.0, "fx_term": 0.0, "fy_term": 0.0}   # latest yaw-correction terms, for logging
last_active_pair = [None]   # which leg pair (FRONT/BACK) was swinging last tick

def control_loop():
    while running[0]:   # keep looping until the run is told to stop
        check_abort()

        active_pair = None   # which pair is swinging this tick, if any
        if gait_active[0] and gait_start_time[0] is not None:   # only move feet once the gait has actually started
            if coast_to_stop[0]:   # check if we're winding down instead of actively gaiting
                for leg in legs:   # plant every leg flat while coasting to a stop
                    foot_target[leg] = (0.0, STANCE_FZ)
            else:
                t = time.time() - gait_start_time[0]   # elapsed time since the gait started
                active_pair = current_swing_pair(t)
                for leg in legs:   # update each leg's foot target for this tick
                    foot_target[leg] = foot_target_for_leg(leg, t, body_vel[0])
        last_active_pair[0] = active_pair

        is_actively_moving = gait_active[0] and not coast_to_stop[0]   # true while actually gaiting forward
        pitch_gain = CORRECTION_FRACTION if is_actively_moving else IDLE_CORRECTION_FRACTION   # active pitch-correction gain
        pitch_damping = PITCH_RATE_DAMPING if is_actively_moving else IDLE_PITCH_RATE_DAMPING   # active pitch-rate damping gain
        theta_p = PITCH_SIGN * pitch_gain * latest_pitch[0]   # proportional pitch correction term
        theta_d = PITCH_SIGN * pitch_damping * latest_pitch_rate[0]   # derivative (rate) pitch correction term

        theta_ff = 0.0   # feedforward pitch term, stays 0 unless a pair is swinging below
        if is_actively_moving and active_pair is not None:   # only apply feedforward while actually gaiting
            swing_leg = FRONT_PAIR[0] if active_pair == "FRONT" else BACK_PAIR[0]   # representative leg of the swinging pair
            swing_frac = leg_phase_frac(swing_leg, t) / SWING_DUTY   # progress through that leg's swing
            theta_ff = -PITCH_SIGN * FEEDFORWARD_PITCH_BIAS_RAD * math.sin(math.pi * swing_frac)

        theta = max(-MAX_CORRECTION_RAD, min(MAX_CORRECTION_RAD, theta_p + theta_d + theta_ff))   # total clamped pitch correction
        last_theta_terms["p"] = theta_p
        last_theta_terms["d"] = theta_d
        last_theta_terms["ff"] = theta_ff
        last_theta_terms["clamped"] = theta
        last_theta_terms["saturated"] = abs(theta_p + theta_d + theta_ff) > MAX_CORRECTION_RAD

        roll_term = ROLL_ABAD_FRACTION * latest_roll[0]   # raw roll correction term
        roll_term = max(-MAX_ABAD_ROLL_CORR, min(MAX_ABAD_ROLL_CORR, roll_term))   # clamped roll correction term

        if gait_active[0] and not coast_to_stop[0] and gait_start_yaw[0] is not None:   # only track yaw drift while actively gaiting
            yaw_err = latest_yaw[0] - gait_start_yaw[0]   # how far yaw has drifted since gait start
            yaw_integral[0] = max(-YAW_INTEGRAL_LIMIT, min(YAW_INTEGRAL_LIMIT,
                                                             yaw_integral[0] + yaw_err * CONTROL_DT))
        else:
            yaw_err = 0.0   # no yaw correction when not actively gaiting
            yaw_integral[0] = 0.0
        yaw_signal = yaw_err + YAW_KI * yaw_integral[0]   # combined proportional+integral yaw signal
        yaw_correction = max(-MAX_YAW_FX, min(MAX_YAW_FX, YAW_FX_GAIN * yaw_signal))   # clamped fore-aft yaw correction
        fy_correction = max(-MAX_YAW_FY, min(MAX_YAW_FY, YAW_FY_GAIN * yaw_signal))   # clamped side-to-side yaw correction
        last_yaw_terms["err"] = yaw_err
        last_yaw_terms["fx_term"] = yaw_correction
        last_yaw_terms["fy_term"] = fy_correction

        swinging_legs = FRONT_PAIR if active_pair == "FRONT" else (BACK_PAIR if active_pair == "BACK" else [])   # legs to steer this tick

        for leg in legs:   # compute and publish this tick's joint angles for each leg
            fx, fz = foot_target[leg]   # this leg's current (fx, fz) target
            this_yaw_term = yaw_correction if leg in swinging_legs else 0.0   # yaw steering, only while this leg swings
            fx = clamp_fx(fx + LEG_LR[leg] * this_yaw_term)   # fx with yaw steering applied
            fx_c, fz_c = rotate(fx, fz, theta)   # foot target rotated by the pitch correction
            fy_c = FRONT_BACK[leg] * fy_correction   # side-to-side yaw correction for this leg
            abad_geo, hip, knee = leg_ik_3d(fx_c, fy_c, fz_c, OY[leg], LEG_SIDE[leg])   # joint angles from geometry alone
            abad = abad_geo + roll_term   # final abad angle with roll correction added

            m0 = Double(); m0.data = abad   # ABAD joint command message
            pubs[f"{leg}_ABAD"].publish(m0)
            m1 = Double(); m1.data = hip   # HIP joint command message
            pubs[f"{leg}_HIP"].publish(m1)
            m2 = Double(); m2.data = knee   # KNEE joint command message
            pubs[f"{leg}_KNEE"].publish(m2)
        time.sleep(CONTROL_DT)

t = threading.Thread(target=control_loop, daemon=True)   # background thread running the control loop

def diagnostic_line(label):
    vx, vy, vz = body_vel   # body velocity components
    speed = math.hypot(vx, vy)   # horizontal speed magnitude
    states = {l: ("SW" if leg_is_swinging(l, (time.time() - gait_start_time[0]) if gait_start_time[0] else 0.0) else "st")   # per-leg swing/stance label for this log line
              for l in legs} if gait_active[0] else {l: "-" for l in legs}
    foot_z_str = " ".join(f"{l}:{link_z.get(l, float('nan')):+.3f}" for l in legs)   # formatted per-foot height string
    line = (f"  [{label}] pitch: {math.degrees(latest_pitch[0]):+.2f} deg  "   # the full diagnostic text to log
            f"roll: {math.degrees(latest_roll[0]):+.2f} deg  yaw: {math.degrees(latest_yaw[0]):+.2f} deg  "
            f"body_xyz: {body_xyz}  speed_xy: {speed:.3f} (target {DESIRED_VX:.3f})  "
            f"pitch_rate: {math.degrees(latest_pitch_rate[0]):+.1f} deg/s  "
            f"theta(p={math.degrees(last_theta_terms['p']):+.2f} d={math.degrees(last_theta_terms['d']):+.2f} "
            f"ff={math.degrees(last_theta_terms['ff']):+.2f} "
            f"clamped={math.degrees(last_theta_terms['clamped']):+.2f}"
            f"{'*SAT*' if last_theta_terms['saturated'] else ''}) deg  "
            f"yaw_corr(err={math.degrees(last_yaw_terms['err']):+.2f}deg I={yaw_integral[0]:+.3f} -> fx_term={last_yaw_terms['fx_term']:+.4f} fy_term={last_yaw_terms['fy_term']:+.4f})  "
            f"legs: {states}  active_pair: {last_active_pair[0]}  "
            f"foot_z: {{{foot_z_str}}}")
    log_line(line)

def move_feet_manual(deltas, duration=1.5, steps=75, label=None):   # deltas: dict of leg -> target (fx, fz) to move to
    starts = {leg: foot_target[leg] for leg in deltas}   # each leg's starting (fx, fz) before this move
    print_every = max(1, steps // 10)   # log a diagnostic line roughly every 10% of the move
    for i in range(1, steps + 1):   # step counter through the interpolation
        if check_abort():   # stop early if a safety abort has triggered
            return
        frac = i / steps   # how far through the move, 0 to 1
        for leg, (tx, tz) in deltas.items():   # leg name and its (tx, tz) target for this move
            sx, sz = starts[leg]   # this leg's starting (fx, fz)
            foot_target[leg] = (sx + (tx - sx) * frac, sz + (tz - sz) * frac)
        if label and (i % print_every == 0 or i == 1):   # only log if a label was given, and only periodically
            diagnostic_line(f"{label} {frac*100:3.0f}%")
        time.sleep(duration / steps)
    for leg, tgt in deltas.items():   # leg name and its final target
        foot_target[leg] = tgt

def ease_into_gallop(duration=3.0, steps=150):
    """Ramp foot_target from the crouch stance point to each leg's t=0 gait target before
       gait_active turns on - avoids a hard position jump at gait start."""
    starts = {leg: foot_target[leg] for leg in legs}   # each leg's starting (fx, fz) before easing in
    targets = {leg: foot_target_for_leg(leg, 0.0, 0.0) for leg in legs}   # each leg's t=0 gait target
    for i in range(1, steps + 1):   # step counter through the interpolation
        if check_abort():   # stop early if a safety abort has triggered
            return
        frac = i / steps   # how far through the ease-in, 0 to 1
        for leg in legs:   # interpolate each leg's foot target
            sx, sz = starts[leg]   # this leg's starting (fx, fz)
            tx, tz = targets[leg]   # this leg's t=0 gait target (fx, fz)
            foot_target[leg] = (sx + (tx - sx) * frac, sz + (tz - sz) * frac)
        if i % 15 == 0 or i == 1:   # log progress every 15 steps, plus the very first
            diagnostic_line(f"easing into gallop {frac*100:3.0f}%")
        time.sleep(duration / steps)
    for leg in legs:   # snap exactly to the final targets once done
        foot_target[leg] = targets[leg]

N_CYCLES = 4   # Fewer than trot_demo.py's 6 on purpose - this is a short, cautious first test of
               # an unvalidated gait, not an endurance run. Extend once it clears cleanly.

print("EXPERIMENTAL GALLOP-FAMILY (bound) GAIT - see this file's header comment for what's proven")
print("vs. what's a first guess. Logging this run to run_log_gallop.txt (same folder)")
print("waiting for the drop to settle...")
time.sleep(5.0)
print(f"landed, pitch = {math.degrees(latest_pitch[0]):.2f} deg  roll = {math.degrees(latest_roll[0]):.2f} deg")
print(f"body position: {body_xyz}")
print(f"yaw at settle (before control_loop has published a single command): {math.degrees(latest_yaw[0]):+.2f} deg")

t.start()

print("--- crouch (hip/knee only, all ABAD at 0) ---")
move_feet_manual({leg: (0.0, STANCE_FZ) for leg in legs}, duration=1.5, steps=75, label="crouch")
for _ in range(20):   # settle briefly in the crouch before starting the gait
    if check_abort():   # stop early if a safety abort has triggered
        break
    diagnostic_line("crouch settle")
    time.sleep(0.1)

start_xyz = list(body_xyz)   # body position at the start of the run, for measuring displacement later
start_yaw = latest_yaw[0]   # yaw angle at the start of the run, for measuring turn later
print(f"start_yaw (reset baseline, absolute): {math.degrees(start_yaw):+.2f} deg")
print(f"GALLOP_PERIOD={GALLOP_PERIOD}s  STANCE_DUTY={STANCE_DUTY}  GALLOP_PHASE_LAG={GALLOP_PHASE_LAG}  "
      f"DESIRED_VX={DESIRED_VX} m/s  STRIDE_HALF_AMPLITUDE={STRIDE_HALF_AMPLITUDE:.4f} m")

if not aborted[0]:   # skip if a safety abort already happened
    print("--- easing into gait's t=0 pose ---")
    ease_into_gallop()
if not aborted[0]:   # only start the gait if nothing aborted the run so far
    gait_start_yaw[0] = latest_yaw[0]
    gait_start_time[0] = time.time()
    gait_active[0] = True
    print(f"--- gallop (bound) engaged: period={GALLOP_PERIOD}s  target_vx={DESIRED_VX} m/s  cycles={N_CYCLES} ---")
    total_duration = N_CYCLES * GALLOP_PERIOD   # planned total run time, in seconds
    DECEL_DURATION = 2.5   # how long the robot takes to slow down at the end, in seconds
    MAX_EXTRA_DECEL = 5.0   # extra time allowed to wait for the robot to slow down, in seconds
    decel_start = max(0.0, total_duration - DECEL_DURATION)   # when to start ramping speed down
    run_start = time.time()   # timestamp when this run phase started
    tick = 0   # counts control-loop iterations in this run phase
    current_target_vx = [DESIRED_VX]   # current commanded forward speed (ramps down near the end)
    while True:   # run the gait loop until it's told to stop below
        if check_abort():   # stop early if a safety abort has triggered
            break
        elapsed = time.time() - run_start   # time elapsed since this run phase started
        if elapsed >= decel_start:   # check if it's time to start ramping speed down
            frac = min(1.0, (elapsed - decel_start) / DECEL_DURATION)   # how far through deceleration, 0 to 1
            current_target_vx[0] = DESIRED_VX * (1.0 - frac)
        if elapsed >= total_duration:   # check if the planned run duration is up
            if not coast_to_stop[0]:   # only trigger the stop sequence once
                coast_to_stop[0] = True
                print(f"--- run duration elapsed (t={elapsed:.2f}s); planting all 4 feet flat, "
                      f"waiting for speed < {SPEED_SETTLE_THRESHOLD:.2f} m/s ---")
            speed_now = math.hypot(body_vel[0], body_vel[1])   # current horizontal speed
            if speed_now <= SPEED_SETTLE_THRESHOLD or elapsed >= total_duration + MAX_EXTRA_DECEL:   # check if the robot has slowed enough or we've waited too long
                break
        if tick % 5 == 0:
            tag = " (decelerating)" if elapsed >= decel_start else ""   # extra label for the log line
            if elapsed >= total_duration:   # check if we're past the planned run duration
                tag += f" (waiting for speed<{SPEED_SETTLE_THRESHOLD:.2f}, now {math.hypot(body_vel[0], body_vel[1]):.3f})"
            diagnostic_line(f"gallop t={elapsed:.2f}{tag}")
        tick += 1
        time.sleep(CONTROL_DT)

    if not aborted[0]:   # only wrap up normally if nothing aborted the run
        gait_active[0] = False
        print("--- gallop complete, returning to a stable stance ---")
        move_feet_manual({leg: (0.0, STANCE_FZ) for leg in legs}, duration=1.5, steps=75,
                          label="post-gait settle")

end_xyz = list(body_xyz)   # body position at the end of the run, for measuring displacement
dx = end_xyz[0] - start_xyz[0]   # how far the body moved in x
dy = end_xyz[1] - start_xyz[1]   # how far the body moved in y
dist = math.sqrt(dx*dx + dy*dy)   # total straight-line distance traveled
elapsed_total = N_CYCLES * GALLOP_PERIOD   # nominal total run duration, for average speed
dyaw_deg = math.degrees(math.atan2(math.sin(latest_yaw[0] - start_yaw), math.cos(latest_yaw[0] - start_yaw)))   # net yaw turned, wrapped to +/-180
print(f"net displacement: dx={dx:.3f} dy={dy:.3f}  total distance={dist:.3f} m  "
      f"net yaw turned={dyaw_deg:+.1f} deg")
print(f"avg speed achieved: {dist/elapsed_total:.4f} m/s (target was {DESIRED_VX:.3f} m/s)")
print(f"final body z: {end_xyz[2]:.3f}  (collapsed if well below ~0.35)")

END_WAIT_MIN = 2.0   # minimum time to wait at the end before stopping, in seconds
END_WAIT_MAX = 6.0   # maximum time to wait at the end before stopping, in seconds
_end_wait_start = time.time()   # timestamp when this final wait began
while True:   # keep checking until the robot settles or the max wait is reached
    if check_abort():   # stop early if a safety abort has triggered
        break
    diagnostic_line("end of loop")
    _elapsed_end = time.time() - _end_wait_start   # time elapsed since the final wait began
    _speed_now = math.hypot(body_vel[0], body_vel[1])   # current horizontal speed
    if (_elapsed_end >= END_WAIT_MIN and _speed_now <= SPEED_SETTLE_THRESHOLD) or _elapsed_end >= END_WAIT_MAX:   # check if the robot has settled or we've waited long enough
        break
    time.sleep(0.1)

print("sequence stopped early (safety abort)" if aborted[0] else "sequence complete")
running[0] = False
