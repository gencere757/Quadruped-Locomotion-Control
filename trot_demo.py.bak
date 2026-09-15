"""
Script: trot_demo.py
Author: Arda Gencer
Runs the diagonal trot gait with reactive pitch/yaw correction and a safety abort.
Detailed tuning-history notes for the constants below are in tuning_history/trot_demo_history.txt
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

_LOG_NAME = "run_log_trot.txt"   # filename for this run's log
_ARCHIVE_DIR = "run_log_archive"   # folder where old logs get copied
try:
    os.makedirs(_ARCHIVE_DIR, exist_ok=True)
    if os.path.exists(_LOG_NAME):   # only copy the old log if one exists yet
        _ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")   # timestamp for the archived filename
        shutil.copy2(_LOG_NAME, os.path.join(_ARCHIVE_DIR, f"{_ts}_{_LOG_NAME}"))
except OSError:
    pass

_log_file = open(_LOG_NAME, "w")   # file handle for the run log

class _Tee:
    def __init__(self, *streams):   # streams to duplicate all writes to
        self.streams = streams   # save the list of streams to write to
    def write(self, data):
        for s in self.streams:   # loop over each output stream
            s.write(data)
            s.flush()
    def flush(self):
        for s in self.streams:   # loop over each output stream
            s.flush()

sys.stdout = _Tee(sys.stdout, _log_file)   # redirect stdout to print to console and log file

def log_line(text):
    print(text, file=_log_file)
    _log_file.flush()

L1 = 0.2   # length of the leg's upper segment, in meters
L2 = 0.2   # length of the leg's lower segment, in meters

def leg_ik(fx, fz, s):
    """fx, fz = foot position relative to the hip, body frame (fz negative = below hip).
       s = +1 front leg, -1 back leg. Returns (hip, knee) angles."""
    u = s * fx   # foot x position, side-corrected
    w = fz   # foot z position (height)
    r2 = u*u + w*w   # squared distance from hip to foot
    r2 = max(r2, 1e-9)   # avoid divide by zero later
    c = (r2 - L1*L1 - L2*L2) / (2*L1*L2)   # law of cosines term for knee angle
    c = max(-1.0, min(1.0, c))   # clamp for safe acos input
    knee = -math.acos(c)   # knee joint angle
    k1 = L1 + L2*math.cos(knee)   # helper term for hip angle calc
    k2 = L2*math.sin(knee)   # helper term for hip angle calc
    sin_a = (u*k1 + k2*w) / r2   # sine component of hip angle
    cos_a = (k2*u - k1*w) / r2   # cosine component of hip angle
    hip = math.atan2(sin_a, cos_a)   # hip joint angle
    return hip, knee

def rotate(fx, fz, theta):
    """Rotates a foot-target vector by theta (same convention the leg's own swing rotation uses)."""
    return (fx*math.cos(theta) - fz*math.sin(theta),
            fx*math.sin(theta) + fz*math.cos(theta))

D_ABAD = 0.1
OY = {"FL": 1, "FR": -1, "BL": 1, "BR": -1}
FRONT_BACK = {"FL": 1, "FR": 1, "BL": -1, "BR": -1}   # used by the fy yaw steering below - +1
                                                        # front, -1 back, independent of left/right.

def leg_ik_3d(fx, fy, fz, oy, s):
    """fx, fy, fz = foot position relative to the leg's ABAD pivot, in true body-frame axes.
       oy = OY[leg] (+1 left, -1 right), s = LEG_SIDE[leg] (+1 front, -1 back). Returns (abad,
       hip, knee). Full geometric derivation is in champgait.py's header - unchanged here."""
    dy = oy * D_ABAD + fy   # foot's y offset from the ABAD pivot
    dz = fz   # foot's z offset (height)
    r = math.hypot(dy, dz)   # distance from ABAD axis to foot
    r = max(r, D_ABAD + 1e-6)   # distance from the ABAD axis can't be less than the arm length -
                                # clamp instead of crashing on an unreachable target
    c = max(-1.0, min(1.0, (oy * D_ABAD) / r))   # clamped ratio for abduction angle
    base = math.atan2(dz, dy)   # base angle toward the foot
    phi_a = base + math.acos(c)   # one candidate abduction angle
    phi_b = base - math.acos(c)   # other candidate abduction angle
    abad = phi_a if abs(phi_a) < abs(phi_b) else phi_b   # pick the smaller-abduction solution
    w = -dy*math.sin(abad) + dz*math.cos(abad)           # effective fz within the tilted 2-link plane
    hip, knee = leg_ik(fx, w, s)   # solve 2-link hip/knee angles
    return abad, hip, knee

LEG_SIDE = {"FL": 1, "FR": 1, "BL": -1, "BR": -1}   # which legs are front (+1) vs back (-1)
LEG_LR = {"FL": 1, "FR": -1, "BL": 1, "BR": -1}   # which legs are left (+1) vs right (-1)
legs = ["FL", "FR", "BL", "BR"]   # short names for the four legs

node = transport.Node()   # gz-transport node for pub/sub
pubs = {}   # dict of joint command publishers
for leg in legs:   # set up publishers for each leg
    pubs[f"{leg}_ABAD"] = node.advertise(f"/model/my_quadruped/joint/{leg}_ABAD/cmd_pos", Double)
    pubs[f"{leg}_HIP"] = node.advertise(f"/model/my_quadruped/joint/{leg}_HIP/cmd_pos", Double)
    pubs[f"{leg}_KNEE"] = node.advertise(f"/model/my_quadruped/joint/{leg}_KNEE/cmd_pos", Double)

# IMU: orientation plus filtered rates
latest_pitch = [0.0]   # current pitch angle, radians
latest_roll = [0.0]   # current roll angle, radians
latest_yaw = [0.0]   # current yaw angle, radians

PITCH_RATE_LPF_ALPHA = 0.10
latest_pitch_rate = [0.0]   # filtered pitch rate, rad/s
_pitch_rate_source = [None]   # tracks whether pitch rate came from gyro or estimate
_prev_pitch_for_rate = [None]   # previous pitch value, for finite-difference rate
_prev_pitch_rate_time = [None]   # timestamp of previous pitch rate sample

YAW_RATE_LPF_ALPHA = 0.2   # smoothing factor for the filtered yaw rate
latest_yaw_rate = [0.0]   # filtered yaw rate, rad/s
_yaw_rate_source = [None]   # tracks whether yaw rate came from gyro or estimate
_prev_yaw_for_rate = [None]   # previous yaw value, for finite-difference rate
_prev_yaw_rate_time = [None]   # timestamp of previous yaw rate sample

def imu_callback(msg):
    q = msg.orientation   # quaternion orientation from the IMU
    sinp = max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))   # clamped sine-of-pitch term
    latest_pitch[0] = math.asin(sinp)
    latest_roll[0] = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    latest_yaw[0] = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    raw_rate = None   # holds this tick's raw pitch rate value
    try:
        raw_rate = msg.angular_velocity.y   # pitch rate straight from the gyro
        _pitch_rate_source[0] = "gyro"
    except AttributeError:
        _pitch_rate_source[0] = "fd"
        now = time.time()   # current time, for rate calc
        if _prev_pitch_for_rate[0] is not None and _prev_pitch_rate_time[0] is not None:   # do we have a previous sample to compare against?
            dt = now - _prev_pitch_rate_time[0]   # time since last pitch sample
            if dt > 1e-4:   # only compute the rate if enough time has passed
                raw_rate = (latest_pitch[0] - _prev_pitch_for_rate[0]) / dt   # pitch rate via finite difference
        _prev_pitch_for_rate[0] = latest_pitch[0]
        _prev_pitch_rate_time[0] = now
    if raw_rate is not None:   # only update the filter if we actually got a rate this tick
        latest_pitch_rate[0] = (PITCH_RATE_LPF_ALPHA * raw_rate
                                 + (1.0 - PITCH_RATE_LPF_ALPHA) * latest_pitch_rate[0])

    yaw_raw_rate = None   # holds this tick's raw yaw rate value
    try:
        yaw_raw_rate = msg.angular_velocity.z   # yaw rate straight from the gyro
        _yaw_rate_source[0] = "gyro"
    except AttributeError:
        _yaw_rate_source[0] = "fd"
        now2 = time.time()   # current time, for yaw rate calc
        if _prev_yaw_for_rate[0] is not None and _prev_yaw_rate_time[0] is not None:   # do we have a previous yaw sample to compare against?
            dt2 = now2 - _prev_yaw_rate_time[0]   # time since last yaw sample
            if dt2 > 1e-4:   # only compute the rate if enough time has passed
                yaw_raw_rate = (latest_yaw[0] - _prev_yaw_for_rate[0]) / dt2   # yaw rate via finite difference
        _prev_yaw_for_rate[0] = latest_yaw[0]
        _prev_yaw_rate_time[0] = now2
    if yaw_raw_rate is not None:   # only update the filter if we actually got a rate this tick
        latest_yaw_rate[0] = (YAW_RATE_LPF_ALPHA * yaw_raw_rate
                               + (1.0 - YAW_RATE_LPF_ALPHA) * latest_yaw_rate[0])

node.subscribe(IMU, "/model/my_quadruped/imu", imu_callback)

# Ground-truth body pose and per-foot world z, for diagnostics only - no support-polygon math here
body_xyz = [None, None, None]   # latest body x,y,z position
body_vel = [0.0, 0.0, 0.0]   # latest body velocity x,y,z
_prev_body_xyz = [None, None, None]   # previous body position, for velocity calc
_prev_pose_time = [None]   # timestamp of previous pose sample
link_z = {}   # per-leg foot height, world frame

def pose_callback(msg):
    for p in msg.pose:   # loop over each body/link pose in the message
        if p.name == "my_quadruped":   # is this pose entry for the robot's main body?
            now = time.time()   # current time, for velocity calc
            if _prev_pose_time[0] is not None:   # skip the velocity calc on the very first sample
                dt = now - _prev_pose_time[0]   # time since last pose sample
                if dt > 1e-4:   # only compute velocity if enough time has passed
                    body_vel[0] = (p.position.x - _prev_body_xyz[0]) / dt
                    body_vel[1] = (p.position.y - _prev_body_xyz[1]) / dt
                    body_vel[2] = (p.position.z - _prev_body_xyz[2]) / dt
            _prev_body_xyz[0], _prev_body_xyz[1], _prev_body_xyz[2] = p.position.x, p.position.y, p.position.z   # save current position for next velocity calc
            _prev_pose_time[0] = now
            body_xyz[0], body_xyz[1], body_xyz[2] = p.position.x, p.position.y, p.position.z   # update latest known body position
        else:
            for leg_name in legs:   # check each leg name for a match
                if p.name.endswith(f"{leg_name}_shank"):   # is this pose entry for this leg's shank link?
                    link_z[leg_name] = p.position.z
                    break

node.subscribe(Pose_V, "/world/empty/pose/info", pose_callback)

# trot gait config
STANCE_FZ = -0.34         # crouch/stance depth - validated for this robot's leg geometry
SWING_HEIGHT = 0.05   # how high a foot lifts off the ground during swing, in meters
TROT_FX_LIMIT = 0.12

TROT_PERIOD = 1.0
STANCE_DUTY = 0.6

SWING_DUTY = 1.0 - STANCE_DUTY   # fraction of the cycle each leg spends swinging (airborne)

DESIRED_VX = 0.17

STRIDE_HALF_AMPLITUDE = 0.5 * DESIRED_VX * STANCE_DUTY * TROT_PERIOD

current_target_vx = [DESIRED_VX]

ENABLE_RAIBERT_FEEDBACK = False
KV_RAIBERT = 0.03                 # meters of touchdown-position shift per (m/s) of velocity error.
                                   # Starting small deliberately - this is unvalidated.


def _smoothstep(x):
    x = max(0.0, min(1.0, x))   # clamp x into 0 to 1 range
    return x * x * (3 - 2 * x)

def clamp_fx(v):
    return max(-TROT_FX_LIMIT, min(TROT_FX_LIMIT, v))

PAIR_A = ["FL", "BR"]   # first diagonal leg pair
PAIR_B = ["FR", "BL"]   # second diagonal leg pair
PAIR_PHASE_OFFSET = {}   # each leg's phase offset in the gait cycle
for leg in PAIR_A:   # pair A starts swinging at phase 0
    PAIR_PHASE_OFFSET[leg] = 0.0
for leg in PAIR_B:   # pair B starts swinging half a cycle later
    PAIR_PHASE_OFFSET[leg] = 0.5

def leg_phase_frac(leg, t):
    global_phase = (t % TROT_PERIOD) / TROT_PERIOD   # how far through the overall gait cycle, 0 to 1
    return (global_phase - PAIR_PHASE_OFFSET[leg]) % 1.0

def leg_is_swinging(leg, t):
    return leg_phase_frac(leg, t) < SWING_DUTY

def current_swing_pair(t):
    if leg_is_swinging(PAIR_A[0], t):   # is pair A currently in its swing phase?
        return "A"   # pair A is the one swinging right now
    if leg_is_swinging(PAIR_B[0], t):   # is pair B currently in its swing phase?
        return "B"   # pair B is the one swinging right now
    return None

def foot_target_for_leg(leg, t, vx_meas):   # vx_meas is measured forward speed, m/s
    local_phase = leg_phase_frac(leg, t)   # this leg's position in its own gait cycle
    if ENABLE_RAIBERT_FEEDBACK:   # use velocity-based stride correction if that's turned on
        A_dyn = 0.5 * vx_meas * STANCE_DUTY * TROT_PERIOD + KV_RAIBERT * (vx_meas - DESIRED_VX)   # Raibert-style dynamic stride amplitude
        A = 0.5 * A_dyn + 0.5 * STRIDE_HALF_AMPLITUDE   # blended, not raw, to damp noisy vx_meas
    else:
        A = 0.5 * current_target_vx[0] * STANCE_DUTY * TROT_PERIOD   # stride half-amplitude for this tick

    if local_phase < SWING_DUTY:   # is this leg currently in its swing phase?
        swing_frac = local_phase / SWING_DUTY   # progress through the swing phase, 0 to 1
        s = _smoothstep(swing_frac)   # smoothed swing progress
        fx = -A + 2 * A * s   # foot x target during swing
        fz = STANCE_FZ + SWING_HEIGHT * math.sin(math.pi * swing_frac)   # foot z target, lifted during swing
    else:
        stance_frac = (local_phase - SWING_DUTY) / STANCE_DUTY   # progress through the stance phase, 0 to 1
        fx = A - 2 * A * stance_frac   # foot x target during stance
        fz = STANCE_FZ   # foot stays at stance depth
    return clamp_fx(fx), fz

# reactive pitch/roll correction
PITCH_SIGN = 1.0   # flips the direction of pitch correction if needed
CORRECTION_FRACTION = 0.25
MAX_CORRECTION_RAD = 0.35   # clamp on how much pitch correction can tilt the feet, in radians
PITCH_RATE_DAMPING = 0.075
IDLE_CORRECTION_FRACTION = 0.047
IDLE_PITCH_RATE_DAMPING = 0.0142
ROLL_ABAD_FRACTION = 0.0
MAX_ABAD_ROLL_CORR = 0.15   # clamp on how much roll correction can adjust the ABAD joint

YAW_FX_GAIN = 0.15
MAX_YAW_FX = 0.06

YAW_FY_GAIN = -0.07        # negative: increasing fy on a stance foot pushes the body the opposite way.
MAX_YAW_FY = 0.08   # clamp on the sideways foot correction used for yaw steering

YAW_KI = 0.3   # gain applied to the yaw integral term
YAW_INTEGRAL_LIMIT = 0.3
yaw_integral = [0.0]   # running integral of yaw error

gait_start_yaw = [None]   # yaw heading when the gait started
gait_start_time = [None]   # timestamp when the gait started
coast_to_stop = [False]

foot_target = {leg: (0.0, STANCE_FZ) for leg in legs}   # current commanded (fx, fz) target for each leg

running = [True]   # flag that keeps the control loop thread alive
FLIP_LIMIT_DEG = 25.0   # pitch/roll angle, in degrees, that triggers a safety abort
FLIP_LIMIT_RAD = math.radians(FLIP_LIMIT_DEG)   # same limit, converted to radians
aborted = [False]   # flag set once a safety abort has triggered
gait_active = [False]   # flag: True while the trot gait is running

def check_abort():
    if aborted[0]:   # already aborted, no need to check again
        return True
    if abs(latest_pitch[0]) > FLIP_LIMIT_RAD or abs(latest_roll[0]) > FLIP_LIMIT_RAD:   # has the robot tipped past the safety limit?
        aborted[0] = True
        print(f"!!! SAFETY ABORT: |pitch|={math.degrees(latest_pitch[0]):.1f} deg  "
              f"|roll|={math.degrees(latest_roll[0]):.1f} deg exceeded {FLIP_LIMIT_DEG} deg !!!")
    return aborted[0]

SPEED_SETTLE_THRESHOLD = 0.06

CONTROL_DT = 0.02   # seconds between control loop updates
last_theta_terms = {"p": 0.0, "d": 0.0, "ff": 0.0, "clamped": 0.0, "saturated": False}   # most recent pitch correction terms, for logging
last_yaw_terms = {"err": 0.0, "fx_term": 0.0, "fy_term": 0.0}   # most recent yaw correction terms, for logging
last_active_pair = [None]   # which leg pair is currently swinging

def control_loop():
    while running[0]:   # keep looping until the run finishes or aborts
        check_abort()

        active_pair = None   # which pair is swinging this tick, if any
        if gait_active[0] and gait_start_time[0] is not None:   # only move the feet if the trot gait has actually started
            if coast_to_stop[0]:   # are we coasting to a stop instead of actively trotting?
                # Plant all 4 feet flat immediately instead of continuing to cycle swing/stance at
                # zero amplitude - see coast_to_stop's comment above.
                for leg in legs:   # loop over all four legs
                    foot_target[leg] = (0.0, STANCE_FZ)   # plant this foot flat on the ground
            else:
                t = time.time() - gait_start_time[0]   # elapsed time since gait started
                active_pair = current_swing_pair(t)   # which pair is swinging right now
                for leg in legs:   # loop over all four legs
                    foot_target[leg] = foot_target_for_leg(leg, t, body_vel[0])   # compute this leg's trot target for right now
        last_active_pair[0] = active_pair

        is_actively_walking = gait_active[0] and not coast_to_stop[0]   # True only while actively trotting, not idle/coasting
        pitch_gain = CORRECTION_FRACTION if is_actively_walking else IDLE_CORRECTION_FRACTION   # pick active or idle pitch gain
        pitch_damping = PITCH_RATE_DAMPING if is_actively_walking else IDLE_PITCH_RATE_DAMPING   # pick active or idle pitch damping
        theta_p = PITCH_SIGN * pitch_gain * latest_pitch[0]   # proportional pitch correction term
        theta_d = PITCH_SIGN * pitch_damping * latest_pitch_rate[0]   # derivative (damping) pitch correction term

        FEEDFORWARD_PITCH_BIAS_RAD = math.radians(7.0)
        theta_ff = 0.0   # feedforward pitch bias, default off
        if is_actively_walking and active_pair is not None:   # only add the feedforward bias while actively trotting mid-swing
            swing_leg = PAIR_A[0] if active_pair == "A" else PAIR_B[0]   # representative leg of the swinging pair
            swing_frac = leg_phase_frac(swing_leg, t) / SWING_DUTY   # 0..1 across this pair's swing
            theta_ff = -PITCH_SIGN * FEEDFORWARD_PITCH_BIAS_RAD * math.sin(math.pi * swing_frac)

        theta = max(-MAX_CORRECTION_RAD, min(MAX_CORRECTION_RAD, theta_p + theta_d + theta_ff))   # total pitch correction, clamped
        last_theta_terms["p"] = theta_p
        last_theta_terms["d"] = theta_d
        last_theta_terms["ff"] = theta_ff
        last_theta_terms["clamped"] = theta
        last_theta_terms["saturated"] = abs(theta_p + theta_d + theta_ff) > MAX_CORRECTION_RAD

        roll_term = ROLL_ABAD_FRACTION * latest_roll[0]   # roll correction before clamping
        roll_term = max(-MAX_ABAD_ROLL_CORR, min(MAX_ABAD_ROLL_CORR, roll_term))   # roll correction, clamped

        if gait_active[0] and not coast_to_stop[0] and gait_start_yaw[0] is not None:   # only track yaw drift while actively walking
            yaw_err = latest_yaw[0] - gait_start_yaw[0]   # how far yaw has drifted from gait start
            yaw_integral[0] = max(-YAW_INTEGRAL_LIMIT, min(YAW_INTEGRAL_LIMIT,
                                                             yaw_integral[0] + yaw_err * CONTROL_DT))
        else:
            yaw_err = 0.0   # no yaw error tracked while idle
            yaw_integral[0] = 0.0
        yaw_signal = yaw_err + YAW_KI * yaw_integral[0]   # combined proportional + integral yaw signal
        yaw_correction = max(-MAX_YAW_FX, min(MAX_YAW_FX, YAW_FX_GAIN * yaw_signal))   # fore-aft foot correction for yaw, clamped
        fy_correction = max(-MAX_YAW_FY, min(MAX_YAW_FY, YAW_FY_GAIN * yaw_signal))   # sideways foot correction for yaw, clamped
        last_yaw_terms["err"] = yaw_err
        last_yaw_terms["fx_term"] = yaw_correction
        last_yaw_terms["fy_term"] = fy_correction

        swinging_legs = PAIR_A if active_pair == "A" else (PAIR_B if active_pair == "B" else [])   # which pair is currently swinging, if any

        for leg in legs:   # apply corrections and publish commands per leg
            fx, fz = foot_target[leg]   # this leg's current target x,z
            this_yaw_term = yaw_correction if leg in swinging_legs else 0.0   # yaw correction only for swinging legs
            fx = clamp_fx(fx + LEG_LR[leg] * this_yaw_term)   # apply yaw correction to foot x
            fx_c, fz_c = rotate(fx, fz, theta)   # foot target rotated for pitch correction
            fy_c = FRONT_BACK[leg] * fy_correction   # sideways steering, all 4 legs, always on
            abad_geo, hip, knee = leg_ik_3d(fx_c, fy_c, fz_c, OY[leg], LEG_SIDE[leg])   # solve joint angles for this foot target
            abad = abad_geo + roll_term   # final abad angle with roll correction

            m0 = Double(); m0.data = abad   # message to publish the abad command
            pubs[f"{leg}_ABAD"].publish(m0)
            m1 = Double(); m1.data = hip   # message to publish the hip command
            pubs[f"{leg}_HIP"].publish(m1)
            m2 = Double(); m2.data = knee   # message to publish the knee command
            pubs[f"{leg}_KNEE"].publish(m2)
        time.sleep(CONTROL_DT)

# Thread's created here but not started until after the drop settles, so no joint commands go out
# during the chaotic drop/landing.
t = threading.Thread(target=control_loop, daemon=True)

def diagnostic_line(label):
    vx, vy, vz = body_vel   # unpack body velocity components
    speed = math.hypot(vx, vy)   # horizontal speed magnitude
    states = {l: ("SW" if leg_is_swinging(l, (time.time() - gait_start_time[0]) if gait_start_time[0] else 0.0) else "st")
              for l in legs} if gait_active[0] else {l: "-" for l in legs}   # swing/stance state label per leg, for logging
    foot_z_str = " ".join(f"{l}:{link_z.get(l, float('nan')):+.3f}" for l in legs)   # formatted per-leg foot height string, for logging
    line = (f"  [{label}] pitch: {math.degrees(latest_pitch[0]):+.2f} deg  "
            f"roll: {math.degrees(latest_roll[0]):+.2f} deg  yaw: {math.degrees(latest_yaw[0]):+.2f} deg  "
            f"body_xyz: {body_xyz}  speed_xy: {speed:.3f} (target {DESIRED_VX:.3f})  "
            f"pitch_rate: {math.degrees(latest_pitch_rate[0]):+.1f} deg/s  "
            f"theta(p={math.degrees(last_theta_terms['p']):+.2f} d={math.degrees(last_theta_terms['d']):+.2f} "
            f"ff={math.degrees(last_theta_terms['ff']):+.2f} "
            f"clamped={math.degrees(last_theta_terms['clamped']):+.2f}"
            f"{'*SAT*' if last_theta_terms['saturated'] else ''}) deg  "
            f"yaw_corr(err={math.degrees(last_yaw_terms['err']):+.2f}deg I={yaw_integral[0]:+.3f} -> fx_term={last_yaw_terms['fx_term']:+.4f} fy_term={last_yaw_terms['fy_term']:+.4f})  "
            f"legs: {states}  active_pair: {last_active_pair[0]}  "
            f"foot_z: {{{foot_z_str}}}")   # full diagnostic line to print
    log_line(line)

def move_feet_manual(deltas, duration=1.5, steps=75, label=None):   # deltas: {leg: target (fx,fz)} to move to
    starts = {leg: foot_target[leg] for leg in deltas}   # starting foot position for each leg being moved
    print_every = max(1, steps // 10)   # how often to log progress
    for i in range(1, steps + 1):   # loop over each interpolation step
        if check_abort():   # stop moving the feet if a safety abort happened
            return
        frac = i / steps   # progress through the move, 0 to 1
        for leg, (tx, tz) in deltas.items():   # loop over each leg's target x,z
            sx, sz = starts[leg]   # this leg's starting x,z
            foot_target[leg] = (sx + (tx - sx) * frac, sz + (tz - sz) * frac)
        if label and (i % print_every == 0 or i == 1):   # only log progress occasionally, and on the first step
            diagnostic_line(f"{label} {frac*100:3.0f}%")
        time.sleep(duration / steps)
    for leg, tgt in deltas.items():   # final snap to exact target per leg
        foot_target[leg] = tgt

def ease_into_trot(duration=3.0, steps=150):
    """Ramps foot_target from the crouch stance to each leg's own t=0 trot target before
       gait_active flips on, so there's no hard position jump at gait start."""
    starts = {leg: foot_target[leg] for leg in legs}   # current foot position for each leg
    targets = {leg: foot_target_for_leg(leg, 0.0, 0.0) for leg in legs}   # each leg's trot starting target
    for i in range(1, steps + 1):   # loop over each interpolation step
        if check_abort():   # stop easing in if a safety abort happened
            return
        frac = i / steps   # progress through the ease-in, 0 to 1
        for leg in legs:   # loop over all four legs
            sx, sz = starts[leg]   # this leg's starting x,z
            tx, tz = targets[leg]   # this leg's trot target x,z
            foot_target[leg] = (sx + (tx - sx) * frac, sz + (tz - sz) * frac)
        if i % 15 == 0 or i == 1:   # log progress periodically, and on the first step
            diagnostic_line(f"easing into trot {frac*100:3.0f}%")
        time.sleep(duration / steps)
    for leg in legs:   # snap all legs to their trot target
        foot_target[leg] = targets[leg]

N_CYCLES = 6   # how many trot cycles to run before stopping

print("waiting for the drop to settle...")
time.sleep(5.0)
print(f"landed, pitch = {math.degrees(latest_pitch[0]):.2f} deg  roll = {math.degrees(latest_roll[0]):.2f} deg")
print(f"body position: {body_xyz}")
print(f"yaw at settle (before control_loop has published a single command): {math.degrees(latest_yaw[0]):+.2f} deg")

t.start()

print("--- crouch (hip/knee only, all ABAD at 0) ---")
move_feet_manual({leg: (0.0, STANCE_FZ) for leg in legs}, duration=1.5, steps=75, label="crouch")
for _ in range(20):   # repeat a few times to sample settle
    if check_abort():   # stop sampling if a safety abort happened
        break
    diagnostic_line("crouch settle")
    time.sleep(0.1)

start_xyz = list(body_xyz)   # body position at the start of the run
start_yaw = latest_yaw[0]   # yaw heading at the start of the run
print(f"start_yaw (reset baseline, absolute): {math.degrees(start_yaw):+.2f} deg")
print(f"TROT_PERIOD={TROT_PERIOD}s  STANCE_DUTY={STANCE_DUTY}  DESIRED_VX={DESIRED_VX} m/s  "
      f"STRIDE_HALF_AMPLITUDE={STRIDE_HALF_AMPLITUDE:.4f} m  RAIBERT={ENABLE_RAIBERT_FEEDBACK}")

if not aborted[0]:   # only ease into the trot pose if we haven't aborted
    print("--- easing into trot's t=0 pose ---")
    ease_into_trot()
if not aborted[0]:   # only start the actual trot if we haven't aborted
    gait_start_yaw[0] = latest_yaw[0]
    gait_start_time[0] = time.time()
    gait_active[0] = True
    print(f"--- trot engaged: period={TROT_PERIOD}s  target_vx={DESIRED_VX} m/s  cycles={N_CYCLES} ---")
    total_duration = N_CYCLES * TROT_PERIOD   # total planned run time, seconds
    DECEL_DURATION = 2.5
    MAX_EXTRA_DECEL = 5.0
    decel_start = max(0.0, total_duration - DECEL_DURATION)   # when the deceleration ramp begins
    run_start = time.time()   # timestamp when the trot run began
    tick = 0   # counts control-loop iterations this run
    while True:   # run the trot loop until it's time to stop
        if check_abort():   # stop the trot loop if a safety abort happened
            break
        elapsed = time.time() - run_start   # seconds since the trot run began
        if elapsed >= decel_start:   # is it time to start ramping speed down?
            frac = min(1.0, (elapsed - decel_start) / DECEL_DURATION)   # progress through the decel ramp, 0 to 1
            current_target_vx[0] = DESIRED_VX * (1.0 - frac)
        if elapsed >= total_duration:   # has the planned run time elapsed?
            if not coast_to_stop[0]:   # only announce the coast-to-stop once
                coast_to_stop[0] = True
                print(f"--- run duration elapsed (t={elapsed:.2f}s); planting all 4 feet flat, "
                      f"waiting for speed < {SPEED_SETTLE_THRESHOLD:.2f} m/s ---")
            speed_now = math.hypot(body_vel[0], body_vel[1])   # current horizontal speed
            if speed_now <= SPEED_SETTLE_THRESHOLD or elapsed >= total_duration + MAX_EXTRA_DECEL:   # has the robot slowed enough, or did grace time run out?
                break
        if tick % 3 == 0:
            tag = " (decelerating)" if elapsed >= decel_start else ""   # label to show in the log line
            if elapsed >= total_duration:   # are we in the waiting-to-settle phase?
                tag += f" (waiting for speed<{SPEED_SETTLE_THRESHOLD:.2f}, now {math.hypot(body_vel[0], body_vel[1]):.3f})"
            diagnostic_line(f"trot t={elapsed:.2f}{tag}")
        tick += 1
        time.sleep(CONTROL_DT)
    current_target_vx[0] = 0.0   # in case the loop exited (abort) before the ramp finished

    if not aborted[0]:   # only settle into stance if we haven't aborted
        gait_active[0] = False
        print("--- trot complete, returning to a stable stance ---")
        move_feet_manual({leg: (0.0, STANCE_FZ) for leg in legs}, duration=1.5, steps=75,
                          label="post-gait settle")

end_xyz = list(body_xyz)   # body position at the end of the run
dx = end_xyz[0] - start_xyz[0]   # how far the body moved in x
dy = end_xyz[1] - start_xyz[1]   # how far the body moved in y
dist = math.sqrt(dx*dx + dy*dy)   # total straight-line distance traveled
elapsed_total = N_CYCLES * TROT_PERIOD   # total time the gait ran, seconds
dyaw_deg = math.degrees(math.atan2(math.sin(latest_yaw[0] - start_yaw), math.cos(latest_yaw[0] - start_yaw)))   # net yaw turned, wrapped to +-180 deg
print(f"net displacement: dx={dx:.3f} dy={dy:.3f}  total distance={dist:.3f} m  "
      f"net yaw turned={dyaw_deg:+.1f} deg")
print(f"avg speed achieved: {dist/elapsed_total:.4f} m/s (target was {DESIRED_VX:.3f} m/s)")
print(f"final body z: {end_xyz[2]:.3f}  (collapsed if well below ~0.35)")
END_WAIT_MIN = 2.0    # seconds - always sample at least this long, even if speed happens to read
                       # low immediately (avoids calling a fluke instant "settled").
END_WAIT_MAX = 6.0     # seconds - hard cap.
_end_wait_start = time.time()   # timestamp when the final wait began
while True:   # keep sampling until settled or the timeout hits
    if check_abort():   # stop waiting if a safety abort happened
        break
    diagnostic_line("end of loop")
    _elapsed_end = time.time() - _end_wait_start   # seconds spent in this final wait
    _speed_now = math.hypot(body_vel[0], body_vel[1])   # current horizontal speed
    if (_elapsed_end >= END_WAIT_MIN and _speed_now <= SPEED_SETTLE_THRESHOLD) or _elapsed_end >= END_WAIT_MAX:   # has it settled long enough, or hit the max wait?
        break
    time.sleep(0.1)

print("sequence stopped early (safety abort)" if aborted[0] else "sequence complete")
running[0] = False
