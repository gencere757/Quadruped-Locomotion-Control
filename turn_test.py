"""
Script: turn_test.py
Author: Arda Gencer
Scripted test sequence for evaluating the robot's in-place turning.
Detailed tuning-history notes for the constants below are in tuning_history/turn_test_history.txt
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

_LOG_NAME = "run_log_turn_test.txt"  # file this run's log gets written to
_ARCHIVE_DIR = "run_log_archive"  # folder old logs get copied into before being overwritten
try:
    os.makedirs(_ARCHIVE_DIR, exist_ok=True)
    if os.path.exists(_LOG_NAME):  # checks if a log from the last run is still there
        _ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")  # timestamp for the archived filename
        shutil.copy2(_LOG_NAME, os.path.join(_ARCHIVE_DIR, f"{_ts}_{_LOG_NAME}"))  # copy the old log file into the archive folder
except OSError:
    pass  # best-effort - don't block a run if archiving fails

_log_file = open(_LOG_NAME, "w")  # file handle for this run's log

class _Tee:
    def __init__(self, *streams):
        self.streams = streams  # remember the streams to write to
    def write(self, data):
        for s in self.streams:  # write to every stream
            s.write(data)
            s.flush()
    def flush(self):
        for s in self.streams:  # flush every stream
            s.flush()

sys.stdout = _Tee(sys.stdout, _log_file)  # send stdout to console and the log file

def log_line(text):
    print(text, file=_log_file)
    _log_file.flush()

L1 = 0.2   # length of the leg's upper segment, in meters
L2 = 0.2   # length of the leg's lower segment, in meters

def leg_ik(fx, fz, s):  # 2D leg IK: fx/fz = foot offset, s = side sign
    u = s * fx  # foot x offset mirrored for this leg's side
    w = fz  # foot z offset (height)
    r2 = u*u + w*w  # squared distance from hip to foot
    r2 = max(r2, 1e-9)  # avoid divide-by-zero below
    c = (r2 - L1*L1 - L2*L2) / (2*L1*L2)  # law-of-cosines term for knee angle
    c = max(-1.0, min(1.0, c))  # clamp for safe acos
    knee = -math.acos(c)  # knee joint angle
    k1 = L1 + L2*math.cos(knee)  # helper term for hip angle
    k2 = L2*math.sin(knee)  # helper term for hip angle
    sin_a = (u*k1 + k2*w) / r2  # sine component of hip angle
    cos_a = (k2*u - k1*w) / r2  # cosine component of hip angle
    hip = math.atan2(sin_a, cos_a)  # hip joint angle
    return hip, knee

def rotate(fx, fz, theta):  # rotate a foot point by angle theta
    return (fx*math.cos(theta) - fz*math.sin(theta),
            fx*math.sin(theta) + fz*math.cos(theta))

D_ABAD = 0.1  # distance from the body's centerline out to the hip joint, in meters
OY = {"FL": 1, "FR": -1, "BL": 1, "BR": -1}  # which side of the body each leg is on, left/right
FRONT_BACK = {"FL": 1, "FR": 1, "BL": -1, "BR": -1}  # which end of the body each leg is on, front/back

def leg_ik_3d(fx, fy, fz, oy, s):  # 3D leg IK adding hip-width offset oy
    dy = oy * D_ABAD + fy  # sideways foot offset from body center
    dz = fz  # vertical foot offset
    r = math.hypot(dy, dz)  # distance from abad joint to foot, in the y-z plane
    r = max(r, D_ABAD + 1e-6)  # keep r just outside the abad offset to avoid bad math
    c = max(-1.0, min(1.0, (oy * D_ABAD) / r))  # clamped ratio for abad angle
    base = math.atan2(dz, dy)  # base angle toward the foot
    phi_a = base + math.acos(c)  # one candidate abad angle
    phi_b = base - math.acos(c)  # other candidate abad angle
    abad = phi_a if abs(phi_a) < abs(phi_b) else phi_b  # pick the smaller-magnitude abad solution
    w = -dy*math.sin(abad) + dz*math.cos(abad)  # foot height in the rotated (post-abad) plane
    hip, knee = leg_ik(fx, w, s)  # solve hip/knee in that rotated plane
    return abad, hip, knee

LEG_SIDE = {"FL": 1, "FR": 1, "BL": -1, "BR": -1}  # flips the leg IK math for front vs back legs
LEG_LR = {"FL": 1, "FR": -1, "BL": 1, "BR": -1}  # left/right sign, used so sides step oppositely when turning
legs = ["FL", "FR", "BL", "BR"]  # the 4 leg names used everywhere below

node = transport.Node()  # gz-transport node for pub/sub
pubs = {}  # joint command publishers, keyed by name
for leg in legs:  # set up publishers for each leg
    pubs[f"{leg}_ABAD"] = node.advertise(f"/model/my_quadruped/joint/{leg}_ABAD/cmd_pos", Double)  # publisher for this leg's abad joint
    pubs[f"{leg}_HIP"] = node.advertise(f"/model/my_quadruped/joint/{leg}_HIP/cmd_pos", Double)  # publisher for this leg's hip joint
    pubs[f"{leg}_KNEE"] = node.advertise(f"/model/my_quadruped/joint/{leg}_KNEE/cmd_pos", Double)  # publisher for this leg's knee joint

# IMU: orientation + filtered pitch rate
latest_pitch = [0.0]  # current pitch estimate, radians (list = mutable box)
latest_roll = [0.0]  # current roll estimate, radians
latest_yaw = [0.0]  # current yaw estimate, radians

PITCH_RATE_LPF_ALPHA = 0.2  # how much the pitch-rate filter trusts new readings vs old ones
latest_pitch_rate = [0.0]  # filtered pitch rate estimate
_prev_pitch_for_rate = [None]  # last pitch value, for computing rate by finite difference
_prev_pitch_rate_time = [None]  # timestamp of that last pitch reading

def imu_callback(msg):
    q = msg.orientation  # quaternion orientation from the IMU
    sinp = max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))  # sine of pitch, clamped for asin
    latest_pitch[0] = math.asin(sinp)  # update pitch estimate from quaternion
    latest_roll[0] = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))  # update roll estimate from quaternion
    latest_yaw[0] = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))  # update yaw estimate from quaternion

    raw_rate = None  # pitch rate for this update, if we can compute one
    try:
        raw_rate = msg.angular_velocity.y  # use the sim's actual angular velocity if available
    except AttributeError:
        now = time.time()  # current time, for finite-difference rate calc
        if _prev_pitch_for_rate[0] is not None and _prev_pitch_rate_time[0] is not None:  # checks if we have a prior reading to measure a rate from
            dt = now - _prev_pitch_rate_time[0]  # time since last pitch reading
            if dt > 1e-4:  # only compute rate if enough time has passed
                raw_rate = (latest_pitch[0] - _prev_pitch_for_rate[0]) / dt  # pitch rate via finite difference
        _prev_pitch_for_rate[0] = latest_pitch[0]  # remember this pitch for next time
        _prev_pitch_rate_time[0] = now  # remember this timestamp for next time
    if raw_rate is not None:  # only update the filter if we actually got a rate value
        latest_pitch_rate[0] = (PITCH_RATE_LPF_ALPHA * raw_rate
                                 + (1.0 - PITCH_RATE_LPF_ALPHA) * latest_pitch_rate[0])  # blend new rate into the filtered estimate

node.subscribe(IMU, "/model/my_quadruped/imu", imu_callback)

# ground-truth body pose - used for diagnostics and for measuring displacement/yaw per phase
body_xyz = [None, None, None]  # latest measured body position, x/y/z
body_vel = [0.0, 0.0, 0.0]  # latest measured body velocity, x/y/z
_prev_body_xyz = [None, None, None]  # previous body position, for velocity calc
_prev_pose_time = [None]  # timestamp of the previous pose reading
link_z = {}  # height of each leg's shank link, keyed by leg name

def pose_callback(msg):
    for p in msg.pose:  # go through each body/link pose in the message
        if p.name == "my_quadruped":  # checks if this entry is the robot's main body
            now = time.time()  # current time, for velocity calc
            if _prev_pose_time[0] is not None:  # checks if there's a previous reading to measure velocity from
                dt = now - _prev_pose_time[0]  # time since last pose reading
                if dt > 1e-4:  # only compute velocity if enough time has passed
                    body_vel[0] = (p.position.x - _prev_body_xyz[0]) / dt  # body velocity in x
                    body_vel[1] = (p.position.y - _prev_body_xyz[1]) / dt  # body velocity in y
                    body_vel[2] = (p.position.z - _prev_body_xyz[2]) / dt  # body velocity in z
            _prev_body_xyz[0], _prev_body_xyz[1], _prev_body_xyz[2] = p.position.x, p.position.y, p.position.z  # store this position for next velocity calc
            _prev_pose_time[0] = now  # store this timestamp for next velocity calc
            body_xyz[0], body_xyz[1], body_xyz[2] = p.position.x, p.position.y, p.position.z  # update latest body position
        else:
            for leg_name in legs:  # check if this pose belongs to one of the legs
                if p.name.endswith(f"{leg_name}_shank"):  # checks if this pose belongs to this leg's shank
                    link_z[leg_name] = p.position.z  # record this leg's shank height
                    break

node.subscribe(Pose_V, "/world/empty/pose/info", pose_callback)

# gait timing config
STANCE_FZ = -0.34  # foot height while planted on the ground, in meters
SWING_HEIGHT = 0.05  # how high a foot lifts during its swing phase, in meters
TROT_FX_LIMIT = 0.12  # max forward/back foot travel allowed per stride, in meters
TROT_PERIOD = 1.0
STANCE_DUTY = 0.5  # fraction of each gait cycle a leg spends planted on the ground
SWING_DUTY = 1.0 - STANCE_DUTY  # fraction of each gait cycle a leg spends swinging

def clamp_fx(v):
    return max(-TROT_FX_LIMIT, min(TROT_FX_LIMIT, v))

def _smoothstep(x):
    x = max(0.0, min(1.0, x))  # clamp x to 0-1 range
    return x * x * (3 - 2 * x)

PAIR_A = ["FL", "BR"]  # one diagonal pair of legs that move together in the trot gait
PAIR_B = ["FR", "BL"]  # the other diagonal pair, moving opposite to PAIR_A
PAIR_PHASE_OFFSET = {}  # gait-cycle phase offset for each leg, filled in below
for leg in PAIR_A:  # assign phase 0 to this pair
    PAIR_PHASE_OFFSET[leg] = 0.0  # this pair starts its gait cycle at phase 0
for leg in PAIR_B:  # assign opposite phase to the other pair
    PAIR_PHASE_OFFSET[leg] = 0.5  # this pair starts half a cycle later

def leg_phase_frac(leg, t):
    global_phase = (t % TROT_PERIOD) / TROT_PERIOD  # where we are in the shared gait cycle, 0 to 1
    return (global_phase - PAIR_PHASE_OFFSET[leg]) % 1.0

# live control targets - same shape as manual_control.py's, but driven by the scripted phase runner
# below instead of a keyboard thread
MAX_VX = 0.15
MAX_TURN = 1.0  # top turning speed used in the test, in rad/s
MAX_VX_ACCEL = 0.20  # fastest forward speed is allowed to ramp up or down, in m/s^2
MAX_TURN_ACCEL = 2.0  # fastest turning speed is allowed to ramp up or down, in rad/s^2

TURN_VX_GAIN = 0.2 * MAX_VX

target_vx = [0.0]  # current rate-limited forward speed command
target_turn = [0.0]  # current rate-limited turn speed command
desired_vx = [0.0]  # forward speed we want to ramp toward
desired_turn = [0.0]  # turn speed we want to ramp toward

def foot_target_for_leg(leg, t, vx):
    local_phase = leg_phase_frac(leg, t)  # this leg's position in its own gait cycle
    A = 0.5 * abs(vx) * STANCE_DUTY * TROT_PERIOD  # stride amplitude (half the foot's travel distance)
    direction = 1.0 if vx >= 0.0 else -1.0  # which way the foot sweeps, forward or backward
    if local_phase < SWING_DUTY:  # checks if this leg is in its swing part of the cycle
        swing_frac = local_phase / SWING_DUTY  # progress through the swing part of the cycle, 0 to 1
        s = _smoothstep(swing_frac)  # eased swing progress for a smooth foot path
        fx = direction * (-A + 2 * A * s)  # foot's forward/back target position
        fz = STANCE_FZ + SWING_HEIGHT * math.sin(math.pi * swing_frac)  # foot height, arcing up mid-swing
    else:
        stance_frac = (local_phase - SWING_DUTY) / STANCE_DUTY  # progress through the stance part of the cycle, 0 to 1
        fx = direction * (A - 2 * A * stance_frac)  # foot sliding back under the body while planted
        fz = STANCE_FZ  # foot stays at ground height while planted
    return clamp_fx(fx), fz

# reactive pitch/roll balance correction
PITCH_SIGN = 1.0  # flips the pitch correction direction if the robot leans the wrong way
CORRECTION_FRACTION = 0.25
MAX_CORRECTION_RAD = 0.35
PITCH_RATE_DAMPING = 0.15
ROLL_ABAD_FRACTION = 0.3
MAX_ABAD_ROLL_CORR = 0.15  # largest roll correction allowed on the abad joint, in radians

SPEED_SETTLE_THRESHOLD = 0.06

running = [True]  # whether the control loop thread should keep running
FLIP_LIMIT_DEG = 25.0  # pitch/roll angle, in degrees, that counts as a fall and stops the test
FLIP_LIMIT_RAD = math.radians(FLIP_LIMIT_DEG)  # same limit, in radians, for the math functions
aborted = [False]  # set true once a safety abort has triggered
gait_active = [False]  # whether the walking gait is currently engaged

def check_abort():
    if aborted[0]:  # checks if we've already aborted
        return True
    if abs(latest_pitch[0]) > FLIP_LIMIT_RAD or abs(latest_roll[0]) > FLIP_LIMIT_RAD:  # checks if the robot has tipped too far and counts as a fall
        aborted[0] = True  # flag the abort so everything else stops
        print(f"!!! SAFETY ABORT: |pitch|={math.degrees(latest_pitch[0]):.1f} deg  "
              f"|roll|={math.degrees(latest_roll[0]):.1f} deg exceeded {FLIP_LIMIT_DEG} deg !!!")
    return aborted[0]

CONTROL_DT = 0.02  # how often the control loop updates, in seconds
foot_target = {leg: (0.0, STANCE_FZ) for leg in legs}  # each leg's current target foot position, starts planted
gait_start_time = [None]  # wall-clock time the gait started, for phase timing
last_theta_terms = {"clamped": 0.0}  # last computed pitch-correction rotation angle
last_status = {"roll_term": 0.0}  # last computed roll-correction term, for status printing
last_abad = {leg: 0.0 for leg in legs}  # each leg's last commanded abad angle, for status printing

MOVE_EPS_VX = 0.005  # below this speed, forward motion counts as basically stopped
MOVE_EPS_TURN = 0.02  # below this rate, turning counts as basically stopped

FOOT_RATE_LIMIT = 1.0  # how fast a foot's target position can change, per second

def _rate_limit(current, target, max_rate, dt):  # ramps current toward target, capped by max_rate*dt
    step = max_rate * dt  # max amount allowed to change this tick
    return current + max(-step, min(step, target - current))

def control_loop():
    last_t = time.time()  # timestamp of the previous loop iteration
    while running[0]:  # keep looping until the run finishes or aborts
        now = time.time()  # timestamp of this loop iteration
        dt = max(1e-4, now - last_t)  # time elapsed since last iteration
        last_t = now  # remember this time for next iteration

        if check_abort():  # stop the loop if a safety abort happened
            break

        target_vx[0] = _rate_limit(target_vx[0], desired_vx[0], MAX_VX_ACCEL, dt)  # ramp the forward-speed target toward desired_vx
        target_turn[0] = _rate_limit(target_turn[0], desired_turn[0], MAX_TURN_ACCEL, dt)  # ramp the turn target toward desired_turn

        is_moving = (abs(target_vx[0]) > MOVE_EPS_VX) or (abs(target_turn[0]) > MOVE_EPS_TURN)  # whether we're commanding any real motion right now

        if gait_active[0] and is_moving:  # checks if the gait is on and actually commanded to move
            if gait_start_time[0] is None:  # checks if this is the first tick of a fresh gait run
                gait_start_time[0] = now  # mark when this gait run started
            t = now - gait_start_time[0]  # elapsed time since the gait started
            turn_vx = TURN_VX_GAIN * target_turn[0]  # extra fore/aft speed added per side to turn
            for leg in legs:  # update each leg's foot target
                leg_vx = target_vx[0] + LEG_LR[leg] * turn_vx  # this leg's effective forward speed, turning included
                raw_fx, raw_fz = foot_target_for_leg(leg, t, leg_vx)  # ideal foot target before rate limiting
                cur_fx, cur_fz = foot_target[leg]  # this leg's current foot position
                fx = _rate_limit(cur_fx, raw_fx, FOOT_RATE_LIMIT, dt)  # smoothed foot x, capped rate of change
                fz = _rate_limit(cur_fz, raw_fz, FOOT_RATE_LIMIT, dt)  # smoothed foot z, capped rate of change
                foot_target[leg] = (fx, fz)  # store this leg's updated foot target
        else:
            gait_start_time[0] = None  # gait not running, clear its start time
            for leg in legs:  # ease every foot back to a neutral stance
                cur_fx, cur_fz = foot_target[leg]  # this leg's current foot position
                fx = _rate_limit(cur_fx, 0.0, FOOT_RATE_LIMIT, dt)  # ease foot x back toward centered
                fz = _rate_limit(cur_fz, STANCE_FZ, FOOT_RATE_LIMIT, dt)  # ease foot z back toward standing height
                foot_target[leg] = (fx, fz)  # store this leg's updated foot target

        theta_p = PITCH_SIGN * CORRECTION_FRACTION * latest_pitch[0]  # proportional pitch-correction term
        theta_d = PITCH_SIGN * PITCH_RATE_DAMPING * latest_pitch_rate[0]  # derivative (damping) pitch-correction term
        theta = max(-MAX_CORRECTION_RAD, min(MAX_CORRECTION_RAD, theta_p + theta_d))  # combined, clamped pitch-correction angle
        last_theta_terms["clamped"] = theta  # save for status/debug reporting

        roll_term = ROLL_ABAD_FRACTION * latest_roll[0]  # raw roll-correction amount
        roll_term = max(-MAX_ABAD_ROLL_CORR, min(MAX_ABAD_ROLL_CORR, roll_term))  # clamp roll correction to a safe range
        last_status["roll_term"] = roll_term  # save for status/debug reporting

        for leg in legs:  # compute and send joint commands for each leg
            fx, fz = foot_target[leg]  # this leg's target foot position
            fx_c, fz_c = rotate(fx, fz, theta)  # foot position rotated for pitch correction
            abad_geo, hip, knee = leg_ik_3d(fx_c, 0.0, fz_c, OY[leg], LEG_SIDE[leg])  # joint angles from inverse kinematics
            abad = abad_geo + roll_term  # final abad angle with roll correction added
            last_abad[leg] = abad  # save for status/debug reporting

            m0 = Double(); m0.data = abad  # abad joint command message
            pubs[f"{leg}_ABAD"].publish(m0)
            m1 = Double(); m1.data = hip  # hip joint command message
            pubs[f"{leg}_HIP"].publish(m1)
            m2 = Double(); m2.data = knee  # knee joint command message
            pubs[f"{leg}_KNEE"].publish(m2)
        time.sleep(CONTROL_DT)

t = threading.Thread(target=control_loop, daemon=True)  # background thread running the control loop

def move_feet_manual(deltas, duration=1.5, steps=75):  # deltas: {leg: target foot pos} to move smoothly to
    starts = {leg: foot_target[leg] for leg in deltas}  # each leg's starting foot position
    for i in range(1, steps + 1):  # step through the move in small increments
        if check_abort():  # stop moving early if a safety abort happened
            return
        frac = i / steps  # how far through the move we are, 0 to 1
        for leg, (tx, tz) in deltas.items():  # target foot position for this leg
            sx, sz = starts[leg]  # this leg's starting foot position
            foot_target[leg] = (sx + (tx - sx) * frac, sz + (tz - sz) * frac)  # interpolate toward the target this step
        time.sleep(duration / steps)
    for leg, tgt in deltas.items():  # snap each leg to its exact final target
        foot_target[leg] = tgt  # make sure we land exactly on target

def status_line():
    vx, vy, vz = body_vel  # current body velocity components
    speed = math.hypot(vx, vy)  # horizontal speed, ignoring vertical
    abad_str = " ".join(f"{leg}:{math.degrees(last_abad[leg]):+.1f}" for leg in legs)  # formatted abad angles for all legs
    print(f"  vx_target={target_vx[0]:+.3f}  turn_target={target_turn[0]:+.2f}  "
          f"speed_xy={speed:.3f}  body_xyz={body_xyz}  "
          f"pitch={math.degrees(latest_pitch[0]):+.1f}deg  "
          f"roll={math.degrees(latest_roll[0]):+.1f}deg  "
          f"roll_term={math.degrees(last_status['roll_term']):+.2f}deg  "
          f"yaw={math.degrees(latest_yaw[0]):+.1f}deg  "
          f"abad={{{abad_str}}}")

PHASE_DURATION = 4 * TROT_PERIOD   # ~6.4s of active movement per phase - long enough for a few full
                                    # gait cycles of steady-state behavior, not just a transient.
PHASE_SETTLE = 1.5

failed_phase = [None]  # name of the phase that failed, if any

def run_phase(label, vx, turn, duration=PHASE_DURATION):  # runs one phase: commands vx/turn and measures the result
    print(f"--- phase: {label}  (vx={vx:+.3f} m/s, turn={turn:+.2f}) ---")
    log_line(f"PHASE START: {label} vx={vx:+.3f} turn={turn:+.2f}")
    start_xyz = list(body_xyz)  # body position at the start of this phase
    start_yaw = latest_yaw[0]  # body yaw at the start of this phase
    desired_vx[0] = vx  # command the forward speed for this phase
    desired_turn[0] = turn  # command the turn speed for this phase

    phase_start = time.time()  # when this phase began
    last_print = 0.0  # last time we printed a status line
    while True:  # loop until the phase's time is up or it aborts
        if check_abort():  # stop the phase early if a safety abort happened
            break
        elapsed = time.time() - phase_start  # how long this phase has been running
        if elapsed >= duration:  # checks if this phase's time is up
            break
        now = time.time()  # current time, for status-print throttling
        if now - last_print > 0.3:  # checks if it's time to print another status update
            status_line()
            last_print = now  # remember when we last printed status
        time.sleep(0.05)

    elapsed_actual = max(1e-3, time.time() - phase_start)  # real time this phase actually took
    end_xyz = list(body_xyz)  # body position at the end of this phase
    dx = end_xyz[0] - start_xyz[0]  # how far the body moved in x
    dy = end_xyz[1] - start_xyz[1]  # how far the body moved in y
    dist = math.hypot(dx, dy)  # straight-line distance traveled
    dyaw_deg = math.degrees(math.atan2(math.sin(latest_yaw[0] - start_yaw),
                                        math.cos(latest_yaw[0] - start_yaw)))  # net yaw change during this phase, in degrees
    summary = (f"    [{label}] dx={dx:+.3f} dy={dy:+.3f} dist={dist:.3f}m  "
               f"net_yaw={dyaw_deg:+.1f}deg  avg_speed={dist/elapsed_actual:.3f} m/s  "
               f"avg_yaw_rate={dyaw_deg/elapsed_actual:+.1f} deg/s"
               + ("  *** ABORTED (fell/flipped during this phase) ***" if aborted[0] else ""))  # text summarizing this phase's results
    print(summary)
    log_line(f"PHASE END: {label}")
    if aborted[0]:  # checks if this phase ended in a safety abort
        failed_phase[0] = label  # record which phase failed
        return True
    return False

def settle(duration=PHASE_SETTLE):
    desired_vx[0] = 0.0  # command zero forward speed to settle
    desired_turn[0] = 0.0  # command zero turn to settle
    deadline = time.time() + duration  # when the settle period ends
    while time.time() < deadline and not aborted[0]:  # keep waiting until the settle time is up or it aborts
        check_abort()
        time.sleep(0.05)
    if not aborted[0]:  # only run the extra wait if nothing has aborted
        EXTRA_MAX = 3.0  # longest extra time to wait for speed to drop, in seconds
        _extra_start = time.time()  # when this extra settle wait began
        while not aborted[0]:  # keep checking until it settles, times out, or aborts
            if check_abort():  # stop waiting early if a safety abort happened
                break
            speed_now = math.hypot(body_vel[0], body_vel[1])  # current horizontal speed
            if speed_now <= SPEED_SETTLE_THRESHOLD or time.time() - _extra_start >= EXTRA_MAX:  # checks if it's slowed down enough or we've waited long enough
                break
            time.sleep(0.05)

# main sequence
print("logging this run to run_log_turn_test.txt (same folder)")
print("waiting for the drop to settle...")
time.sleep(5.0)
print(f"landed, pitch = {math.degrees(latest_pitch[0]):.2f} deg  roll = {math.degrees(latest_roll[0]):.2f} deg")

t.start()

print("--- crouch ---")
move_feet_manual({leg: (0.0, STANCE_FZ) for leg in legs}, duration=1.5, steps=75)

gait_start_time[0] = time.time()  # mark the gait's start time
gait_active[0] = True  # turn on the walking gait

PHASES = [
    ("forward walk",              MAX_VX,  0.0),
    ("in-place rotation",         0.0,     MAX_TURN),
    ("back walk",                -MAX_VX,  0.0),
    ("combined linear+angular",   MAX_VX,  MAX_TURN),
]  # the sequence of test phases: (name, forward speed, turn speed)

print("=" * 60)
print(" scripted test: " + " -> ".join(label for label, _, _ in PHASES))
print("=" * 60)

if not aborted[0]:  # only run the test phases if nothing has aborted yet
    for label, vx, turn in PHASES:  # run each phase in order
        if run_phase(label, vx, turn):  # checks if that phase ended in a safety abort
            break
        settle()

print("--- stopping: ramping targets to zero ---")
desired_vx[0] = 0.0  # command zero forward speed to stop
desired_turn[0] = 0.0  # command zero turn to stop
stop_deadline = time.time() + 2.0  # give the ramp-down at most 2 seconds
while time.time() < stop_deadline and not aborted[0]:  # keep ramping down until it stops, times out, or aborts
    if abs(target_vx[0]) < 0.005 and abs(target_turn[0]) < 0.02:  # checks if the robot has basically come to a stop
        break
    time.sleep(0.05)

gait_active[0] = False  # turn off the walking gait
if not aborted[0]:  # only reset to standing if nothing has aborted
    print("--- returning to a neutral stance ---")
    move_feet_manual({leg: (0.0, STANCE_FZ) for leg in legs}, duration=1.0, steps=50)

if not aborted[0]:  # only wait to settle if nothing has aborted
    print(f"--- waiting for speed < {SPEED_SETTLE_THRESHOLD:.2f} m/s before stopping ---")
    END_WAIT_MIN, END_WAIT_MAX = 1.0, 5.0  # shortest and longest time to wait for the robot to settle
    _wait_start = time.time()  # when this final wait began
    while not aborted[0]:  # keep waiting until it settles, times out, or aborts
        if check_abort():  # stop waiting early if a safety abort happened
            break
        _elapsed_wait = time.time() - _wait_start  # how long we've been waiting so far
        speed_now = math.hypot(body_vel[0], body_vel[1])  # current horizontal speed
        if (_elapsed_wait >= END_WAIT_MIN and speed_now <= SPEED_SETTLE_THRESHOLD) or _elapsed_wait >= END_WAIT_MAX:  # checks if it's slowed enough after the minimum wait, or hit the max wait
            break
        time.sleep(0.1)

time.sleep(0.2)
running[0] = False  # tell the control loop thread to stop
if aborted[0]:  # checks if the run ended early from a safety abort
    print(f"sequence stopped early (safety abort) during phase: {failed_phase[0]}")
else:
    print("all phases completed without a safety abort")
print("turn_test.py exiting")
