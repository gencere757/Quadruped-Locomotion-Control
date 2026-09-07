"""
Script: manual_control.py
Author: Arda Gencer
Lets you drive the quadruped's trot gait live from the keyboard (WASD to move/turn, space or esc to stop).
Detailed tuning-history notes for the constants below are in tuning_history/manual_control_history.txt
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

_LOG_NAME = "run_log_manual.txt"  # file name this run's log is written to
_ARCHIVE_DIR = "run_log_archive"  # folder where old log files get archived
try:
    os.makedirs(_ARCHIVE_DIR, exist_ok=True)
    if os.path.exists(_LOG_NAME):  # is there already a log file from a previous run?
        _ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")  # timestamp used to name the archived log file
        shutil.copy2(_LOG_NAME, os.path.join(_ARCHIVE_DIR, f"{_ts}_{_LOG_NAME}"))
except OSError:
    pass

_log_file = open(_LOG_NAME, "w")  # file handle for this run's log

class _Tee:
    def __init__(self, *streams):
        self.streams = streams  # the output streams to write to (console + log file)
    def write(self, data):
        for s in self.streams:  # each stream this data gets written to
            s.write(data)
            s.flush()
    def flush(self):
        for s in self.streams:  # each stream to flush
            s.flush()

sys.stdout = _Tee(sys.stdout, _log_file)  # redirect stdout to print to console and log file both

def log_line(text):
    print(text, file=_log_file)
    _log_file.flush()

L1 = 0.2   # length of the leg's upper segment, in meters
L2 = 0.2   # length of the leg's lower segment, in meters

def leg_ik(fx, fz, s):  # fx/fz = target foot position, s = left/right sign flip
    u = s * fx  # foot x position, mirrored for left/right
    w = fz  # foot z position (renamed for the math below)
    r2 = u*u + w*w  # squared distance from hip to foot
    r2 = max(r2, 1e-9)  # avoid divide-by-zero if foot is right at the hip
    c = (r2 - L1*L1 - L2*L2) / (2*L1*L2)  # law-of-cosines term for the knee angle
    c = max(-1.0, min(1.0, c))  # clamp so acos doesn't get an invalid input
    knee = -math.acos(c)  # knee joint angle
    k1 = L1 + L2*math.cos(knee)  # helper term for solving the hip angle
    k2 = L2*math.sin(knee)  # helper term for solving the hip angle
    sin_a = (u*k1 + k2*w) / r2  # sine component of the hip angle
    cos_a = (k2*u - k1*w) / r2  # cosine component of the hip angle
    hip = math.atan2(sin_a, cos_a)  # hip joint angle
    return hip, knee

def rotate(fx, fz, theta):
    return (fx*math.cos(theta) - fz*math.sin(theta),
            fx*math.sin(theta) + fz*math.cos(theta))

D_ABAD = 0.1  # how far the hip joint sticks out sideways from center, in meters
OY = {"FL": 1, "FR": -1, "BL": 1, "BR": -1}  # sideways offset direction for each leg
FRONT_BACK = {"FL": 1, "FR": 1, "BL": -1, "BR": -1}  # +1 for front legs, -1 for back legs

def leg_ik_3d(fx, fy, fz, oy, s):  # oy = sideways offset direction, s = leg-side sign flip
    dy = oy * D_ABAD + fy  # sideways foot offset from the hip
    dz = fz  # vertical foot offset (renamed for the math below)
    r = math.hypot(dy, dz)  # distance from hip to foot in the sideways plane
    r = max(r, D_ABAD + 1e-6)  # keep r from collapsing to a degenerate value
    c = max(-1.0, min(1.0, (oy * D_ABAD) / r))  # clamped ratio used to solve the abad angle
    base = math.atan2(dz, dy)  # base angle toward the foot in the sideways plane
    phi_a = base + math.acos(c)  # one candidate solution for the abad angle
    phi_b = base - math.acos(c)  # the other candidate solution for the abad angle
    abad = phi_a if abs(phi_a) < abs(phi_b) else phi_b  # pick the smaller-magnitude, more natural solution
    w = -dy*math.sin(abad) + dz*math.cos(abad)  # foot position in the leg's swing plane, for the 2D solver
    hip, knee = leg_ik(fx, w, s)  # solve hip/knee angles in the leg's own swing plane
    return abad, hip, knee

LEG_SIDE = {"FL": 1, "FR": 1, "BL": -1, "BR": -1}  # mirrors the leg IK for front vs back legs
LEG_LR = {"FL": 1, "FR": -1, "BL": 1, "BR": -1}  # +1 for left legs, -1 for right legs
legs = ["FL", "FR", "BL", "BR"]  # short names for the four legs

node = transport.Node()  # gz-transport node used to publish/subscribe
pubs = {}  # publisher handle for each leg joint, filled in below
for leg in legs:  # loop over each of the 4 legs
    pubs[f"{leg}_ABAD"] = node.advertise(f"/model/my_quadruped/joint/{leg}_ABAD/cmd_pos", Double)  # publisher for this leg's abad joint
    pubs[f"{leg}_HIP"] = node.advertise(f"/model/my_quadruped/joint/{leg}_HIP/cmd_pos", Double)  # publisher for this leg's hip joint
    pubs[f"{leg}_KNEE"] = node.advertise(f"/model/my_quadruped/joint/{leg}_KNEE/cmd_pos", Double)  # publisher for this leg's knee joint

# IMU: orientation + filtered rates
latest_pitch = [0.0]  # most recent pitch angle from the IMU, in radians
latest_roll = [0.0]  # most recent roll angle from the IMU, in radians
latest_yaw = [0.0]  # most recent yaw (heading) angle from the IMU, in radians

PITCH_RATE_LPF_ALPHA = 0.10
latest_pitch_rate = [0.0]  # smoothed rate of pitch change, in radians per second
_prev_pitch_for_rate = [None]  # last pitch value, used to estimate the rate by hand
_prev_pitch_rate_time = [None]  # timestamp of that last pitch reading

def imu_callback(msg):
    q = msg.orientation  # quaternion orientation from the IMU message
    sinp = max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))  # clamped sine-of-pitch term from the quaternion
    latest_pitch[0] = math.asin(sinp)  # pitch angle from the quaternion
    latest_roll[0] = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))  # roll angle from the quaternion
    latest_yaw[0] = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))  # yaw angle from the quaternion

    raw_rate = None  # pitch rate for this update, filled in below
    try:
        raw_rate = msg.angular_velocity.y  # pitch rate straight from the IMU, if it has one
    except AttributeError:
        now = time.time()  # current time, used to estimate the rate by hand
        if _prev_pitch_for_rate[0] is not None and _prev_pitch_rate_time[0] is not None:  # do we have a prior reading to compare against?
            dt = now - _prev_pitch_rate_time[0]  # time since the last pitch reading
            if dt > 1e-4:   # only compute rate if enough time has passed
                raw_rate = (latest_pitch[0] - _prev_pitch_for_rate[0]) / dt  # pitch rate estimated from the change in pitch
        _prev_pitch_for_rate[0] = latest_pitch[0]  # remember this pitch for next time's rate estimate
        _prev_pitch_rate_time[0] = now  # remember this timestamp for next time's rate estimate
    if raw_rate is not None:  # did we get a usable pitch rate this update?
        latest_pitch_rate[0] = (PITCH_RATE_LPF_ALPHA * raw_rate  # smooth the pitch rate with a low-pass filter
                                 + (1.0 - PITCH_RATE_LPF_ALPHA) * latest_pitch_rate[0])

node.subscribe(IMU, "/model/my_quadruped/imu", imu_callback)

# ground-truth body pose, diagnostics only
body_xyz = [None, None, None]  # robot's real (x, y, z) position, for diagnostics only
body_vel = [0.0, 0.0, 0.0]  # robot's real (x, y, z) velocity, computed from position changes
_prev_body_xyz = [None, None, None]  # previous position, used to compute body_vel
_prev_pose_time = [None]  # timestamp of the previous position reading
link_z = {}  # height (z) of each leg's shank, filled in as pose updates arrive

def pose_callback(msg):
    for p in msg.pose:  # each body/link pose in this update
        if p.name == "my_quadruped":  # is this pose update for the robot body itself?
            now = time.time()  # current time, used to compute velocity
            if _prev_pose_time[0] is not None:  # do we have a prior timestamp to measure velocity from?
                dt = now - _prev_pose_time[0]  # time since the last position reading
                if dt > 1e-4:   # only compute velocity if enough time has passed
                    body_vel[0] = (p.position.x - _prev_body_xyz[0]) / dt  # x velocity from the change in position
                    body_vel[1] = (p.position.y - _prev_body_xyz[1]) / dt  # y velocity from the change in position
                    body_vel[2] = (p.position.z - _prev_body_xyz[2]) / dt  # z velocity from the change in position
            _prev_body_xyz[0], _prev_body_xyz[1], _prev_body_xyz[2] = p.position.x, p.position.y, p.position.z  # remember this position for next tick's velocity calc
            _prev_pose_time[0] = now  # remember this timestamp for next tick's velocity calc
            body_xyz[0], body_xyz[1], body_xyz[2] = p.position.x, p.position.y, p.position.z  # store the latest real body position
        else:
            for leg_name in legs:  # check each leg to see if this pose belongs to it
                if p.name.endswith(f"{leg_name}_shank"):  # does this pose belong to this leg's shank?
                    link_z[leg_name] = p.position.z  # store this leg's shank height
                    break

node.subscribe(Pose_V, "/world/empty/pose/info", pose_callback)

# gait timing config
STANCE_FZ = -0.34  # foot height while standing on the ground, in meters
SWING_HEIGHT = 0.05  # how high the foot lifts during its swing, in meters
TROT_FX_LIMIT = 0.12  # max forward/back distance a foot can move, in meters
TROT_PERIOD = 1.0
STANCE_DUTY = 0.6
SWING_DUTY = 1.0 - STANCE_DUTY  # fraction of the stride each foot spends swinging in the air

def clamp_fx(v):
    return max(-TROT_FX_LIMIT, min(TROT_FX_LIMIT, v))

def _smoothstep(x):
    x = max(0.0, min(1.0, x))  # clamp input to 0..1 before smoothing
    return x * x * (3 - 2 * x)

PAIR_A = ["FL", "BR"]  # one diagonal pair of legs that move together
PAIR_B = ["FR", "BL"]  # the other diagonal pair, opposite phase from PAIR_A
PAIR_PHASE_OFFSET = {}  # phase offset per leg within the trot cycle, filled in below
for leg in PAIR_A:  # give this pair phase offset 0
    PAIR_PHASE_OFFSET[leg] = 0.0  # pair A starts the cycle at phase 0
for leg in PAIR_B:  # give this pair the opposite phase offset
    PAIR_PHASE_OFFSET[leg] = 0.5  # pair B starts half a cycle later

def leg_phase_frac(leg, t):
    global_phase = (t % TROT_PERIOD) / TROT_PERIOD  # where we are in the overall trot cycle, 0..1
    return (global_phase - PAIR_PHASE_OFFSET[leg]) % 1.0

# live control targets - keyboard thread writes these, control_loop reads them
MAX_VX = 0.17
MAX_TURN = 1.0
MAX_VX_ACCEL = 0.20         # m/s^2 - caps how fast target_vx can ramp, so a key tap doesn't slam a step
                            # change into the gait. ~0.75s to go from 0 to MAX_VX.
MAX_TURN_ACCEL = 2.0        # steering units/s, same idea as MAX_VX_ACCEL.

TURN_VX_GAIN = 0.2 * MAX_VX

TURN_VX_GAIN_PURE = 0.4 * MAX_VX

target_vx = [0.0]           # current, rate-limited forward-speed target (m/s)
target_turn = [0.0]         # current, rate-limited steering target (-1..+1)
desired_vx = [0.0]          # what the keys are currently asking for (before rate limiting)
desired_turn = [0.0]        # steering the keys are currently asking for, before rate limiting

def foot_target_for_leg(leg, t, vx):
    local_phase = leg_phase_frac(leg, t)  # this leg's position within the trot cycle, 0..1
    A = 0.5 * abs(vx) * STANCE_DUTY * TROT_PERIOD  # stride amplitude, how far the foot swings
    direction = 1.0 if vx >= 0.0 else -1.0  # which way the robot is trying to move
    if local_phase < SWING_DUTY:  # is this leg currently in its swing phase?
        swing_frac = local_phase / SWING_DUTY  # progress through the swing phase, 0..1
        s = _smoothstep(swing_frac)  # smoothed swing progress, eases in/out
        fx = direction * (-A + 2 * A * s)  # foot's forward/back target during swing
        fz = STANCE_FZ + SWING_HEIGHT * math.sin(math.pi * swing_frac)  # foot's height target during swing
    else:
        stance_frac = (local_phase - SWING_DUTY) / STANCE_DUTY  # progress through the stance phase, 0..1
        fx = direction * (A - 2 * A * stance_frac)  # foot's forward/back target during stance
        fz = STANCE_FZ  # foot stays on the ground during stance
    return clamp_fx(fx), fz

# reactive pitch/roll balance correction
PITCH_SIGN = 1.0  # flips pitch-correction direction if the IMU sign convention is backwards
CORRECTION_FRACTION = 0.25
MAX_CORRECTION_RAD = 0.35
PITCH_RATE_DAMPING = 0.075
IDLE_CORRECTION_FRACTION = 0.047
IDLE_PITCH_RATE_DAMPING = 0.0142
FEEDFORWARD_PITCH_BIAS_RAD = math.radians(5.0)
ROLL_ABAD_FRACTION = 0.0
MAX_ABAD_ROLL_CORR = 0.15  # clamp on how much roll correction can adjust the hip-spread angle

YAW_HOLD_KP = 0.0
MAX_YAW_HOLD_FX = 0.06  # clamp on how strong the yaw-hold correction can push
YAW_HOLD_KI = 0.3  # integral gain for the yaw-hold controller
YAW_HOLD_INTEGRAL_LIMIT = 0.3  # clamp on the yaw-hold integral term, to stop windup
yaw_hold_target = [None]  # heading to hold, set once steering stops
yaw_hold_integral = [0.0]  # running integral term for the yaw-hold controller

SPEED_SETTLE_THRESHOLD = 0.06

running = [True]  # master flag, set False to stop every thread and loop
FLIP_LIMIT_DEG = 25.0  # pitch/roll angle, in degrees, that triggers a safety abort
FLIP_LIMIT_RAD = math.radians(FLIP_LIMIT_DEG)  # same abort limit as FLIP_LIMIT_DEG, in radians
aborted = [False]  # set True once a safety abort has happened
gait_active = [False]  # whether the trot gait is currently supposed to run

def check_abort():
    if aborted[0]:  # has a safety abort already happened?
        return True
    if abs(latest_pitch[0]) > FLIP_LIMIT_RAD or abs(latest_roll[0]) > FLIP_LIMIT_RAD:   # has the robot tipped past the safety limit?
        aborted[0] = True  # trip the abort flag, pitch or roll went too far
        print(f"!!! SAFETY ABORT: |pitch|={math.degrees(latest_pitch[0]):.1f} deg  "
              f"|roll|={math.degrees(latest_roll[0]):.1f} deg exceeded {FLIP_LIMIT_DEG} deg !!!")
    return aborted[0]

CONTROL_DT = 0.02  # seconds between control loop ticks
foot_target = {leg: (0.0, STANCE_FZ) for leg in legs}  # current commanded (fx, fz) foot position per leg
gait_start_time = [None]  # time.time() when the current gait started, for phase timing
last_theta_terms = {"clamped": 0.0, "p": 0.0, "d": 0.0, "ff": 0.0, "saturated": False}  # latest pitch-correction terms, for the status printout
last_status = {"roll_term": 0.0, "active_pair": None, "yaw_hold_err": 0.0, "yaw_hold_fx": 0.0}
last_abad = {leg: 0.0 for leg in legs}   # final commanded ABAD angle per leg, to watch stance width
                                          # narrow/widen live (same roll_term investigation).

MOVE_EPS_VX = 0.005     # m/s - below this, forward/back input counts as "released"
MOVE_EPS_TURN = 0.02    # steering units - below this, turn input counts as "released"

FOOT_RATE_LIMIT = 1.0   # m/s ceiling on how fast a commanded foot position may move

def _rate_limit(current, target, max_rate, dt):
    step = max_rate * dt  # max amount the value is allowed to change this tick
    return current + max(-step, min(step, target - current))

def control_loop():
    last_t = time.time()  # timestamp of the previous loop tick
    while running[0]:   # keep looping until told to stop
        now = time.time()  # timestamp of this loop tick
        dt = max(1e-4, now - last_t)  # time elapsed since the last tick
        last_t = now  # remember this tick's time for the next one

        if check_abort():  # stop the control loop if a safety abort has triggered
            break

        # rate-limit the live targets toward whatever the keyboard thread is currently asking for
        target_vx[0] = _rate_limit(target_vx[0], desired_vx[0], MAX_VX_ACCEL, dt)  # ease the forward-speed target toward what's requested
        target_turn[0] = _rate_limit(target_turn[0], desired_turn[0], MAX_TURN_ACCEL, dt)  # ease the steering target toward what's requested

        is_moving = (abs(target_vx[0]) > MOVE_EPS_VX) or (abs(target_turn[0]) > MOVE_EPS_TURN)  # whether any real forward/turn input is active
        is_turning = abs(target_turn[0]) > MOVE_EPS_TURN

        if gait_active[0] and is_moving:   # is the gait turned on and actually being asked to move?
            if gait_start_time[0] is None:  # is this the first tick of a new gait phase?
                gait_start_time[0] = now  # mark when this gait phase began
            t = now - gait_start_time[0]  # elapsed time since the gait started, drives the phase
            turn_gain = TURN_VX_GAIN_PURE if abs(target_vx[0]) <= MOVE_EPS_VX else TURN_VX_GAIN  # stronger gain for in-place turning, weaker while walking
            turn_vx = turn_gain * target_turn[0]  # per-side speed offset from steering
            for leg in legs:  # update the foot target for each leg
                leg_vx = target_vx[0] + LEG_LR[leg] * turn_vx  # this leg's effective forward speed
                raw_fx, raw_fz = foot_target_for_leg(leg, t, leg_vx)  # this leg's ideal foot position right now
                cur_fx, cur_fz = foot_target[leg]  # this leg's current (not yet rate-limited) foot position
                fx = _rate_limit(cur_fx, raw_fx, FOOT_RATE_LIMIT, dt)  # smoothed forward/back foot position
                fz = _rate_limit(cur_fz, raw_fz, FOOT_RATE_LIMIT, dt)  # smoothed vertical foot position
                foot_target[leg] = (fx, fz)  # store this leg's updated foot target
            a_phase = leg_phase_frac(PAIR_A[0], t)  # where pair A is in its swing/stance cycle
            b_phase = leg_phase_frac(PAIR_B[0], t)  # where pair B is in its swing/stance cycle
            if a_phase < SWING_DUTY:  # is pair A currently swinging?
                last_status["active_pair"] = "A"  # pair A is currently swinging
            elif b_phase < SWING_DUTY:  # otherwise, is pair B currently swinging?
                last_status["active_pair"] = "B"  # pair B is currently swinging
            else:
                last_status["active_pair"] = None  # neither pair is mid-swing right now
        else:
            gait_start_time[0] = None   # so the next move starts the phase clock fresh, at t=0
            last_status["active_pair"] = None  # not gaiting, so no pair is swinging
            for leg in legs:  # ease every leg back to a flat standing stance
                cur_fx, cur_fz = foot_target[leg]  # this leg's current foot position
                fx = _rate_limit(cur_fx, 0.0, FOOT_RATE_LIMIT, dt)  # ease forward/back position back to centered
                fz = _rate_limit(cur_fz, STANCE_FZ, FOOT_RATE_LIMIT, dt)  # ease height back to standing height
                foot_target[leg] = (fx, fz)  # store this leg's updated stance target

        # ported fix: full-strength gain while actually moving, weaker idle gain otherwise - see
        # IDLE_CORRECTION_FRACTION's comment above.
        is_actively_moving = gait_active[0] and is_moving  # whether the robot is actually walking right now
        pitch_gain = CORRECTION_FRACTION if is_actively_moving else IDLE_CORRECTION_FRACTION  # use the full gain while walking, weaker while idle
        pitch_damping = PITCH_RATE_DAMPING if is_actively_moving else IDLE_PITCH_RATE_DAMPING  # same idea for the pitch-rate damping gain
        theta_p = PITCH_SIGN * pitch_gain * latest_pitch[0]  # proportional pitch-correction term
        theta_d = PITCH_SIGN * pitch_damping * latest_pitch_rate[0]  # derivative (rate) pitch-correction term
        theta_ff = 0.0  # feedforward pitch bias, defaults to none
        if is_actively_moving and abs(target_vx[0]) > MOVE_EPS_VX and last_status["active_pair"] is not None:  # only bias while actually walking forward/back
            swing_leg = PAIR_A[0] if last_status["active_pair"] == "A" else PAIR_B[0]  # a representative leg from whichever pair is swinging
            swing_frac = leg_phase_frac(swing_leg, t) / SWING_DUTY  # progress through this swing, 0..1
            vx_direction = 1.0 if target_vx[0] >= 0.0 else -1.0  # which way we're walking, to mirror the bias
            theta_ff = -PITCH_SIGN * vx_direction * FEEDFORWARD_PITCH_BIAS_RAD * math.sin(math.pi * swing_frac)  # anticipatory nose-up bias, peaks mid-swing
        theta = max(-MAX_CORRECTION_RAD, min(MAX_CORRECTION_RAD, theta_p + theta_d + theta_ff))  # total pitch correction, clamped to a safe range
        last_theta_terms["p"] = theta_p  # record the proportional term for the status printout
        last_theta_terms["d"] = theta_d  # record the derivative term for the status printout
        last_theta_terms["ff"] = theta_ff  # record the feedforward term for the status printout
        last_theta_terms["clamped"] = theta  # record the final clamped correction
        last_theta_terms["saturated"] = abs(theta_p + theta_d + theta_ff) > MAX_CORRECTION_RAD  # whether the clamp is actively limiting the correction

        roll_term = ROLL_ABAD_FRACTION * latest_roll[0]  # roll-correction amount before clamping
        roll_term = max(-MAX_ABAD_ROLL_CORR, min(MAX_ABAD_ROLL_CORR, roll_term))  # clamp roll correction to a safe range
        last_status["roll_term"] = roll_term  # record for the status printout

        if is_turning:  # is the robot actively being steered right now?
            yaw_hold_target[0] = None  # no fixed heading to hold while actively steering
            yaw_hold_integral[0] = 0.0  # reset the integral term while steering
            yaw_hold_err = 0.0  # no heading error to report while steering
            yaw_hold_correction = 0.0  # no yaw-hold correction applied while steering
        else:
            if yaw_hold_target[0] is None:  # no heading locked in yet, so lock in the current one
                yaw_hold_target[0] = latest_yaw[0]
            raw_err = latest_yaw[0] - yaw_hold_target[0]  # raw difference between current and held heading
            yaw_hold_err = math.atan2(math.sin(raw_err), math.cos(raw_err))  # wrap to [-pi, +pi]
            yaw_hold_integral[0] = max(-YAW_HOLD_INTEGRAL_LIMIT, min(YAW_HOLD_INTEGRAL_LIMIT,  # accumulate heading error over time, clamped
                                                                       yaw_hold_integral[0] + yaw_hold_err * dt))
            yaw_hold_signal = yaw_hold_err + YAW_HOLD_KI * yaw_hold_integral[0]  # combined proportional+integral heading error
            yaw_hold_correction = max(-MAX_YAW_HOLD_FX, min(MAX_YAW_HOLD_FX, YAW_HOLD_KP * yaw_hold_signal))  # clamped yaw-hold correction to apply
        last_status["yaw_hold_err"] = yaw_hold_err  # record for the status printout
        last_status["yaw_hold_fx"] = yaw_hold_correction  # record for the status printout

        swinging_legs = (PAIR_A if last_status["active_pair"] == "A"  # which pair is currently swinging (gets the yaw correction)
                          else (PAIR_B if last_status["active_pair"] == "B" else []))

        for leg in legs:  # compute and send the final joint commands for each leg
            fx, fz = foot_target[leg]  # this leg's commanded foot position
            this_yaw_term = yaw_hold_correction if leg in swinging_legs else 0.0  # yaw correction, only for the swinging pair
            fx = clamp_fx(fx + LEG_LR[leg] * this_yaw_term)  # foot x position with yaw correction applied
            fx_c, fz_c = rotate(fx, fz, theta)  # foot position rotated by the pitch correction
            abad_geo, hip, knee = leg_ik_3d(fx_c, 0.0, fz_c, OY[leg], LEG_SIDE[leg])  # geometric joint angles for this foot position
            abad = abad_geo + OY[leg] * roll_term  # final abad angle, geometric plus mirrored roll correction
            last_abad[leg] = abad  # record for the status printout

            m0 = Double(); m0.data = abad  # message carrying this leg's abad command
            pubs[f"{leg}_ABAD"].publish(m0)
            m1 = Double(); m1.data = hip  # message carrying this leg's hip command
            pubs[f"{leg}_HIP"].publish(m1)
            m2 = Double(); m2.data = knee  # message carrying this leg's knee command
            pubs[f"{leg}_KNEE"].publish(m2)
        time.sleep(CONTROL_DT)

t = threading.Thread(target=control_loop, daemon=True)  # background thread running the control loop

def move_feet_manual(deltas, duration=1.5, steps=75):  # deltas = target (fx, fz) per leg to ease toward
    starts = {leg: foot_target[leg] for leg in deltas}  # each leg's foot position before the move starts
    for i in range(1, steps + 1):  # step counter through the interpolation
        if check_abort():  # stop this move early if a safety abort has triggered
            return
        frac = i / steps  # how far through the move we are, 0..1
        for leg, (tx, tz) in deltas.items():  # this leg and its target (x, z) position
            sx, sz = starts[leg]  # this leg's starting (x, z) position
            foot_target[leg] = (sx + (tx - sx) * frac, sz + (tz - sz) * frac)  # interpolated position for this step
        time.sleep(duration / steps)
    for leg, tgt in deltas.items():  # snap each leg to its exact final target
        foot_target[leg] = tgt  # make sure we land exactly on the target, no drift

if not sys.platform.startswith("win"):  # bail out if this isn't running on Windows
    print("manual_control.py needs Windows (uses GetAsyncKeyState for keyboard input) - see the "
          "comment above this check for a porting note.")
    sys.exit(1)

import ctypes

VK_W, VK_A, VK_S, VK_D = 0x57, 0x41, 0x53, 0x44  # Windows virtual-key codes for W/A/S/D
VK_SPACE, VK_Q, VK_ESCAPE = 0x20, 0x51, 0x1B  # Windows virtual-key codes for space, Q, and escape

def _key_down(vk):
    # High bit of GetAsyncKeyState's return is set iff the key is down right now - a direct
    # hardware check per key, no repeat-timing involved.
    return bool(ctypes.windll.user32.GetAsyncKeyState(vk) & 0x8000)

def keyboard_thread():
    while running[0]:   # keep polling keys until told to stop
        if _key_down(VK_Q) or _key_down(VK_ESCAPE):  # did the user press quit or escape?
            running[0] = False  # quit key pressed, signal every thread to stop
            continue
        if _key_down(VK_SPACE):  # did the user press the space bar to stop?
            desired_vx[0] = 0.0  # space bar pressed, request an immediate stop
            desired_turn[0] = 0.0  # space bar pressed, stop turning too
        else:
            w, a, s, d = _key_down(VK_W), _key_down(VK_A), _key_down(VK_S), _key_down(VK_D)  # whether each movement key is currently held
            desired_vx[0] = MAX_VX if (w and not s) else (-MAX_VX if (s and not w) else 0.0)  # requested forward/back speed from W/S keys
            desired_turn[0] = MAX_TURN if (d and not a) else (-MAX_TURN if (a and not d) else 0.0)  # requested steering from A/D keys
        time.sleep(0.01)

def status_line():
    vx, vy, vz = body_vel  # unpack the body's real velocity components
    speed = math.hypot(vx, vy)  # horizontal ground speed
    abad_str = " ".join(f"{leg}:{math.degrees(last_abad[leg]):+.1f}" for leg in legs)  # formatted abad angles for each leg, for printing
    print(f"  vx_target={target_vx[0]:+.3f}  turn_target={target_turn[0]:+.2f}  "
          f"speed_xy={speed:.3f}  body_xyz={body_xyz}  "
          f"pitch={math.degrees(latest_pitch[0]):+.1f}deg  "
          f"pitch_rate={math.degrees(latest_pitch_rate[0]):+.1f}deg/s  "
          f"theta(p={math.degrees(last_theta_terms['p']):+.1f} "
          f"d={math.degrees(last_theta_terms['d']):+.1f} "
          f"ff={math.degrees(last_theta_terms['ff']):+.2f} "
          f"clamped={math.degrees(last_theta_terms['clamped']):+.1f}"
          f"{'*SAT*' if last_theta_terms['saturated'] else ''})deg  "
          f"roll={math.degrees(latest_roll[0]):+.1f}deg  "
          f"roll_term={math.degrees(last_status['roll_term']):+.2f}deg  "
          f"yaw={math.degrees(latest_yaw[0]):+.1f}deg  "
          f"yaw_hold(err={math.degrees(last_status['yaw_hold_err']):+.1f}deg "
          f"fx={last_status['yaw_hold_fx']:+.4f})  "
          f"active_pair={last_status['active_pair']}  "
          f"abad={{{abad_str}}}")

# main sequence
print("logging this run to run_log_manual.txt (same folder)")
print("waiting for the drop to settle...")
time.sleep(5.0)
print(f"landed, pitch = {math.degrees(latest_pitch[0]):.2f} deg  roll = {math.degrees(latest_roll[0]):.2f} deg")

t.start()   # start after the drop-settle wait, so the balance correction doesn't fight the drop

print("--- crouch ---")
move_feet_manual({leg: (0.0, STANCE_FZ) for leg in legs}, duration=1.5, steps=75)  # flat neutral stance target for every leg

gait_start_time[0] = time.time()  # mark the start of the first gait phase
gait_active[0] = True  # allow the trot gait to start running

kb_thread = threading.Thread(target=keyboard_thread, daemon=True)  # background thread polling the keyboard
kb_thread.start()

print("=" * 60)
print(" W/S = forward/back   A/D = turn left/right   SPACE = stop")
print(" Q or ESC = quit (ramps to a stop, then a safe stance)")
print("=" * 60)

last_print = 0.0  # timestamp of the last status printout
PRINT_INTERVAL = 0.05
while running[0]:   # keep looping and printing status until told to stop
    if check_abort():  # stop the main loop if a safety abort has triggered
        break
    now = time.time()  # current time, to check if it's time to print status
    if now - last_print > PRINT_INTERVAL:  # is it time to print another status update?
        status_line()
        last_print = now  # remember when we last printed
    time.sleep(0.05)

print("--- stopping: ramping targets to zero ---")
desired_vx[0] = 0.0  # ask the gait to ramp forward/back speed to zero
desired_turn[0] = 0.0  # ask the gait to ramp steering to zero
stop_deadline = time.time() + 2.0  # give the ramp-down at most 2 seconds
while time.time() < stop_deadline and not aborted[0]:   # wait for the ramp-down to finish or time out
    if abs(target_vx[0]) < 0.005 and abs(target_turn[0]) < 0.02:  # have the speed/turn targets basically reached zero?
        break
    time.sleep(0.05)

gait_active[0] = False  # stop the trot gait now that speed has ramped down
if not aborted[0]:  # only settle into a stance if we didn't just crash
    print("--- returning to a neutral stance ---")
    move_feet_manual({leg: (0.0, STANCE_FZ) for leg in legs}, duration=1.0, steps=50)  # flat neutral stance target for every leg

if not aborted[0]:  # only wait to settle if there wasn't a safety abort
    print(f"--- waiting for speed < {SPEED_SETTLE_THRESHOLD:.2f} m/s before stopping ---")
    END_WAIT_MIN, END_WAIT_MAX = 1.0, 5.0  # min/max seconds to wait for the robot to settle before stopping
    _wait_start = time.time()  # when we started waiting for speed to settle
    while not aborted[0]:   # keep waiting for the robot to settle down
        if check_abort():  # stop waiting if a safety abort triggers
            break
        _elapsed_wait = time.time() - _wait_start  # how long we've been waiting so far
        speed_now = math.hypot(body_vel[0], body_vel[1])  # current real ground speed
        if (_elapsed_wait >= END_WAIT_MIN and speed_now <= SPEED_SETTLE_THRESHOLD) or _elapsed_wait >= END_WAIT_MAX:  # settled, or waited long enough?
            break
        time.sleep(0.1)

time.sleep(0.2)
running[0] = False  # final shutdown of every thread and loop
print("sequence stopped early (safety abort)" if aborted[0] else "manual_control.py exiting")
