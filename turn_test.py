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

# --- logging: tee everything printed to both the console and a timestamped log file, same
# pattern as trot_demo.py's run_log_trot.txt / manual_control.py's run_log_manual.txt. -------------
#
# Archive the PREVIOUS run's log before truncating it, so the auto-ping loop (which runs this
# script back-to-back with nothing in between) doesn't silently erase history every single run -
# that was a real problem: several runs' worth of results were only ever visible for the few
# seconds between one run finishing and the next one starting, and got lost if nobody read the
# file in that window. Copies (not moves) go in run_log_archive/, timestamped, so run_log_turn_test.txt
# itself still always means "the live/most recent run" for anything that reads it (the ping/grep
# workflow, run_turn_test_once.bat, etc.) - nothing about the live path's meaning changes.
_LOG_NAME = "run_log_turn_test.txt"
_ARCHIVE_DIR = "run_log_archive"
try:
    os.makedirs(_ARCHIVE_DIR, exist_ok=True)
    if os.path.exists(_LOG_NAME):
        _ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        shutil.copy2(_LOG_NAME, os.path.join(_ARCHIVE_DIR, f"{_ts}_{_LOG_NAME}"))
except OSError:
    pass  # archiving is best-effort - never block a run over it

_log_file = open(_LOG_NAME, "w")

class _Tee:
    def __init__(self, *streams):
        self.streams = streams
    def write(self, data):
        for s in self.streams:
            s.write(data)
            s.flush()
    def flush(self):
        for s in self.streams:
            s.flush()

sys.stdout = _Tee(sys.stdout, _log_file)

def log_line(text):
    print(text, file=_log_file)
    _log_file.flush()

L1 = 0.2
L2 = 0.2

def leg_ik(fx, fz, s):
    u = s * fx
    w = fz
    r2 = u*u + w*w
    r2 = max(r2, 1e-9)
    c = (r2 - L1*L1 - L2*L2) / (2*L1*L2)
    c = max(-1.0, min(1.0, c))
    knee = -math.acos(c)
    k1 = L1 + L2*math.cos(knee)
    k2 = L2*math.sin(knee)
    sin_a = (u*k1 + k2*w) / r2
    cos_a = (k2*u - k1*w) / r2
    hip = math.atan2(sin_a, cos_a)
    return hip, knee

def rotate(fx, fz, theta):
    return (fx*math.cos(theta) - fz*math.sin(theta),
            fx*math.sin(theta) + fz*math.cos(theta))

D_ABAD = 0.1
OY = {"FL": 1, "FR": -1, "BL": 1, "BR": -1}
FRONT_BACK = {"FL": 1, "FR": 1, "BL": -1, "BR": -1}

def leg_ik_3d(fx, fy, fz, oy, s):
    dy = oy * D_ABAD + fy
    dz = fz
    r = math.hypot(dy, dz)
    r = max(r, D_ABAD + 1e-6)
    c = max(-1.0, min(1.0, (oy * D_ABAD) / r))
    base = math.atan2(dz, dy)
    phi_a = base + math.acos(c)
    phi_b = base - math.acos(c)
    abad = phi_a if abs(phi_a) < abs(phi_b) else phi_b
    w = -dy*math.sin(abad) + dz*math.cos(abad)
    hip, knee = leg_ik(fx, w, s)
    return abad, hip, knee

LEG_SIDE = {"FL": 1, "FR": 1, "BL": -1, "BR": -1}
LEG_LR = {"FL": 1, "FR": -1, "BL": 1, "BR": -1}
legs = ["FL", "FR", "BL", "BR"]

node = transport.Node()
pubs = {}
for leg in legs:
    pubs[f"{leg}_ABAD"] = node.advertise(f"/model/my_quadruped/joint/{leg}_ABAD/cmd_pos", Double)
    pubs[f"{leg}_HIP"] = node.advertise(f"/model/my_quadruped/joint/{leg}_HIP/cmd_pos", Double)
    pubs[f"{leg}_KNEE"] = node.advertise(f"/model/my_quadruped/joint/{leg}_KNEE/cmd_pos", Double)

# --- IMU: orientation + filtered rates --------------------------------------------------------------
latest_pitch = [0.0]
latest_roll = [0.0]
latest_yaw = [0.0]

PITCH_RATE_LPF_ALPHA = 0.2
latest_pitch_rate = [0.0]
_prev_pitch_for_rate = [None]
_prev_pitch_rate_time = [None]

def imu_callback(msg):
    q = msg.orientation
    sinp = max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))
    latest_pitch[0] = math.asin(sinp)
    latest_roll[0] = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    latest_yaw[0] = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    raw_rate = None
    try:
        raw_rate = msg.angular_velocity.y
    except AttributeError:
        now = time.time()
        if _prev_pitch_for_rate[0] is not None and _prev_pitch_rate_time[0] is not None:
            dt = now - _prev_pitch_rate_time[0]
            if dt > 1e-4:
                raw_rate = (latest_pitch[0] - _prev_pitch_for_rate[0]) / dt
        _prev_pitch_for_rate[0] = latest_pitch[0]
        _prev_pitch_rate_time[0] = now
    if raw_rate is not None:
        latest_pitch_rate[0] = (PITCH_RATE_LPF_ALPHA * raw_rate
                                 + (1.0 - PITCH_RATE_LPF_ALPHA) * latest_pitch_rate[0])

node.subscribe(IMU, "/model/my_quadruped/imu", imu_callback)

# --- ground-truth body pose (diagnostics + per-phase displacement/yaw measurement) -------------------
body_xyz = [None, None, None]
body_vel = [0.0, 0.0, 0.0]
_prev_body_xyz = [None, None, None]
_prev_pose_time = [None]
link_z = {}

def pose_callback(msg):
    for p in msg.pose:
        if p.name == "my_quadruped":
            now = time.time()
            if _prev_pose_time[0] is not None:
                dt = now - _prev_pose_time[0]
                if dt > 1e-4:
                    body_vel[0] = (p.position.x - _prev_body_xyz[0]) / dt
                    body_vel[1] = (p.position.y - _prev_body_xyz[1]) / dt
                    body_vel[2] = (p.position.z - _prev_body_xyz[2]) / dt
            _prev_body_xyz[0], _prev_body_xyz[1], _prev_body_xyz[2] = p.position.x, p.position.y, p.position.z
            _prev_pose_time[0] = now
            body_xyz[0], body_xyz[1], body_xyz[2] = p.position.x, p.position.y, p.position.z
        else:
            for leg_name in legs:
                if p.name.endswith(f"{leg_name}_shank"):
                    link_z[leg_name] = p.position.z
                    break

node.subscribe(Pose_V, "/world/empty/pose/info", pose_callback)

# --- gait timing config ------------------------------------------------------------------------------
STANCE_FZ = -0.34
SWING_HEIGHT = 0.05
TROT_FX_LIMIT = 0.12
TROT_PERIOD = 1.0            # see manual_control.py's comment on this constant (kept in sync) - cut from
                            # 1.6s to shorten/speed up strides for the same commanded vx, since the
                            # recurring falls across every p_gain/d_gain tried were in plain walking
                            # (turn=0.00), pointing at per-stride disturbance from the now much heavier
                            # legs rather than a turn-logic or joint-tracking-gain problem.
STANCE_DUTY = 0.5
SWING_DUTY = 1.0 - STANCE_DUTY

def clamp_fx(v):
    return max(-TROT_FX_LIMIT, min(TROT_FX_LIMIT, v))

def _smoothstep(x):
    x = max(0.0, min(1.0, x))
    return x * x * (3 - 2 * x)

PAIR_A = ["FL", "BR"]
PAIR_B = ["FR", "BL"]
PAIR_PHASE_OFFSET = {}
for leg in PAIR_A:
    PAIR_PHASE_OFFSET[leg] = 0.0
for leg in PAIR_B:
    PAIR_PHASE_OFFSET[leg] = 0.5

def leg_phase_frac(leg, t):
    global_phase = (t % TROT_PERIOD) / TROT_PERIOD
    return (global_phase - PAIR_PHASE_OFFSET[leg]) % 1.0

# --- live control targets - same shape as manual_control.py's, but driven by the scripted phase
# runner below instead of a keyboard thread ------------------------------------------------------
MAX_VX = 0.15  # was 0.20 - see manual_control.py's comment on this constant (kept in sync). Cuts the
               # stride disturbance further to give real margin under the 25deg abort line.
MAX_TURN = 1.0
MAX_VX_ACCEL = 0.20
MAX_TURN_ACCEL = 2.0

TURN_VX_GAIN = 0.2 * MAX_VX # see manual_control.py's comment on this constant (kept in sync). With
                            # TROT_PERIOD=1.0 (below), forward walk / rotation / back walk now all clear
                            # cleanly - that was the fix for the plain-walking falls. combined
                            # linear+angular is now the only failing phase, and it fails fast (pitch
                            # oscillates -10/+14/-14deg within ~1.8s then aborts) - still too much stacked
                            # vx+turn disturbance for the pitch loop to damp. Cut further to 0.2*MAX_VX.

target_vx = [0.0]
target_turn = [0.0]
desired_vx = [0.0]
desired_turn = [0.0]

def foot_target_for_leg(leg, t, vx):
    local_phase = leg_phase_frac(leg, t)
    A = 0.5 * abs(vx) * STANCE_DUTY * TROT_PERIOD
    direction = 1.0 if vx >= 0.0 else -1.0
    if local_phase < SWING_DUTY:
        swing_frac = local_phase / SWING_DUTY
        s = _smoothstep(swing_frac)
        fx = direction * (-A + 2 * A * s)
        fz = STANCE_FZ + SWING_HEIGHT * math.sin(math.pi * swing_frac)
    else:
        stance_frac = (local_phase - SWING_DUTY) / STANCE_DUTY
        fx = direction * (A - 2 * A * stance_frac)
        fz = STANCE_FZ
    return clamp_fx(fx), fz

# --- reactive pitch/roll balance correction ----------------------------------------------------------
PITCH_SIGN = 1.0
CORRECTION_FRACTION = 0.25 # was 0.4 - kept in sync with manual_control.py. A fresh manual run fell
                           # during plain continuous forward walking (turn=0 the whole time), pitch
                           # oscillating with growing/non-decaying amplitude almost from the start,
                           # crossing the 25deg abort after ~18s of walking - the same signature
                           # trot_demo.py had for its post-gait oscillation, fixed there by cutting this
                           # same gain 0.4->0.25. See manual_control.py's comment for the full run
                           # evidence.
MAX_CORRECTION_RAD = 0.45  # was 0.35 - see manual_control.py's comment on this constant (kept in sync).
                           # Several aborts showed pitch climbing steadily right up to the limit, meaning
                           # the correction was maxed out on its clamp and still losing - raising the clamp
                           # gives it more room to actually arrest a lean before the safety abort fires.
PITCH_RATE_DAMPING = 0.15  # REVERTED - see manual_control.py's comment on this constant (kept in sync).
                           # Briefly doubled to 0.3 to fix an apparent underdamped oscillation, but 3
                           # straight runs after that change failed immediately in forward walk, one
                           # violently (roll spiking to 10.6deg, avg_yaw_rate=45.7 deg/s within under a
                           # second) - the noisy, lightly-filtered pitch-rate derivative was likely getting
                           # amplified into real torque kicks right at the crouch-to-gait transition,
                           # not adding clean damping. Back to 0.15.
ROLL_ABAD_FRACTION = 0.3    # kept in sync with manual_control.py - restored for an A/B baseline run
                            # against the ROLL_ABAD_FRACTION=0.0 test. See manual_control.py's comment
                            # above "abad = abad_geo + roll_term" for the narrowing-stance-width theory.
MAX_ABAD_ROLL_CORR = 0.15

SPEED_SETTLE_THRESHOLD = 0.06   # m/s - ported from trot_demo.py's post-gait-fall fix / kept in sync
                                 # with manual_control.py. Treat the body as "stopped enough" once real
                                 # MEASURED speed is at/below this, not just once the rate-limited
                                 # target_vx/turn have decayed to ~0 or a fixed timer expires - see
                                 # settle() and the shutdown sequence at the bottom of the main sequence.

running = [True]
FLIP_LIMIT_DEG = 25.0
FLIP_LIMIT_RAD = math.radians(FLIP_LIMIT_DEG)
aborted = [False]
gait_active = [False]

def check_abort():
    if aborted[0]:
        return True
    if abs(latest_pitch[0]) > FLIP_LIMIT_RAD or abs(latest_roll[0]) > FLIP_LIMIT_RAD:
        aborted[0] = True
        print(f"!!! SAFETY ABORT: |pitch|={math.degrees(latest_pitch[0]):.1f} deg  "
              f"|roll|={math.degrees(latest_roll[0]):.1f} deg exceeded {FLIP_LIMIT_DEG} deg !!!")
    return aborted[0]

CONTROL_DT = 0.02
foot_target = {leg: (0.0, STANCE_FZ) for leg in legs}
gait_start_time = [None]
last_theta_terms = {"clamped": 0.0}
last_status = {"roll_term": 0.0}
last_abad = {leg: 0.0 for leg in legs}

MOVE_EPS_VX = 0.005
MOVE_EPS_TURN = 0.02

FOOT_RATE_LIMIT = 1.0

def _rate_limit(current, target, max_rate, dt):
    step = max_rate * dt
    return current + max(-step, min(step, target - current))

def control_loop():
    last_t = time.time()
    while running[0]:
        now = time.time()
        dt = max(1e-4, now - last_t)
        last_t = now

        if check_abort():
            break

        target_vx[0] = _rate_limit(target_vx[0], desired_vx[0], MAX_VX_ACCEL, dt)
        target_turn[0] = _rate_limit(target_turn[0], desired_turn[0], MAX_TURN_ACCEL, dt)

        is_moving = (abs(target_vx[0]) > MOVE_EPS_VX) or (abs(target_turn[0]) > MOVE_EPS_TURN)

        if gait_active[0] and is_moving:
            if gait_start_time[0] is None:
                gait_start_time[0] = now
            t = now - gait_start_time[0]
            turn_vx = TURN_VX_GAIN * target_turn[0]
            for leg in legs:
                leg_vx = target_vx[0] + LEG_LR[leg] * turn_vx
                raw_fx, raw_fz = foot_target_for_leg(leg, t, leg_vx)
                cur_fx, cur_fz = foot_target[leg]
                fx = _rate_limit(cur_fx, raw_fx, FOOT_RATE_LIMIT, dt)
                fz = _rate_limit(cur_fz, raw_fz, FOOT_RATE_LIMIT, dt)
                foot_target[leg] = (fx, fz)
        else:
            gait_start_time[0] = None
            for leg in legs:
                cur_fx, cur_fz = foot_target[leg]
                fx = _rate_limit(cur_fx, 0.0, FOOT_RATE_LIMIT, dt)
                fz = _rate_limit(cur_fz, STANCE_FZ, FOOT_RATE_LIMIT, dt)
                foot_target[leg] = (fx, fz)

        theta_p = PITCH_SIGN * CORRECTION_FRACTION * latest_pitch[0]
        theta_d = PITCH_SIGN * PITCH_RATE_DAMPING * latest_pitch_rate[0]
        theta = max(-MAX_CORRECTION_RAD, min(MAX_CORRECTION_RAD, theta_p + theta_d))
        last_theta_terms["clamped"] = theta

        roll_term = ROLL_ABAD_FRACTION * latest_roll[0]
        roll_term = max(-MAX_ABAD_ROLL_CORR, min(MAX_ABAD_ROLL_CORR, roll_term))
        last_status["roll_term"] = roll_term

        for leg in legs:
            fx, fz = foot_target[leg]
            fx_c, fz_c = rotate(fx, fz, theta)
            abad_geo, hip, knee = leg_ik_3d(fx_c, 0.0, fz_c, OY[leg], LEG_SIDE[leg])
            abad = abad_geo + roll_term
            last_abad[leg] = abad

            m0 = Double(); m0.data = abad
            pubs[f"{leg}_ABAD"].publish(m0)
            m1 = Double(); m1.data = hip
            pubs[f"{leg}_HIP"].publish(m1)
            m2 = Double(); m2.data = knee
            pubs[f"{leg}_KNEE"].publish(m2)
        time.sleep(CONTROL_DT)

t = threading.Thread(target=control_loop, daemon=True)

def move_feet_manual(deltas, duration=1.5, steps=75):
    starts = {leg: foot_target[leg] for leg in deltas}
    for i in range(1, steps + 1):
        if check_abort():
            return
        frac = i / steps
        for leg, (tx, tz) in deltas.items():
            sx, sz = starts[leg]
            foot_target[leg] = (sx + (tx - sx) * frac, sz + (tz - sz) * frac)
        time.sleep(duration / steps)
    for leg, tgt in deltas.items():
        foot_target[leg] = tgt

def status_line():
    vx, vy, vz = body_vel
    speed = math.hypot(vx, vy)
    abad_str = " ".join(f"{leg}:{math.degrees(last_abad[leg]):+.1f}" for leg in legs)
    print(f"  vx_target={target_vx[0]:+.3f}  turn_target={target_turn[0]:+.2f}  "
          f"speed_xy={speed:.3f}  body_xyz={body_xyz}  "
          f"pitch={math.degrees(latest_pitch[0]):+.1f}deg  "
          f"roll={math.degrees(latest_roll[0]):+.1f}deg  "
          f"roll_term={math.degrees(last_status['roll_term']):+.2f}deg  "
          f"yaw={math.degrees(latest_yaw[0]):+.1f}deg  "
          f"abad={{{abad_str}}}")

# ============================================================================
# scripted phase runner - this is the whole point of this file vs manual_control.py: no keyboard,
# just a fixed sequence of (vx, turn) commands run automatically so turning behavior (and the
# turn-then-fall issue) can be reproduced and measured the same way every time. -----------------
# ============================================================================
PHASE_DURATION = 4 * TROT_PERIOD   # ~6.4s of active movement per phase - long enough to get a few
                                    # full gait cycles of steady-state behavior, not just a transient.
PHASE_SETTLE = 1.5                 # seconds of zero command between phases, so each phase starts
                                    # from a clean planted stance and its own gait phase clock at t=0,
                                    # and so one phase's residual motion/lean can't bleed into the next.

failed_phase = [None]

def run_phase(label, vx, turn, duration=PHASE_DURATION):
    print(f"--- phase: {label}  (vx={vx:+.3f} m/s, turn={turn:+.2f}) ---")
    log_line(f"PHASE START: {label} vx={vx:+.3f} turn={turn:+.2f}")
    start_xyz = list(body_xyz)
    start_yaw = latest_yaw[0]
    desired_vx[0] = vx
    desired_turn[0] = turn

    phase_start = time.time()
    last_print = 0.0
    while True:
        if check_abort():
            break
        elapsed = time.time() - phase_start
        if elapsed >= duration:
            break
        now = time.time()
        if now - last_print > 0.3:
            status_line()
            last_print = now
        time.sleep(0.05)

    elapsed_actual = max(1e-3, time.time() - phase_start)
    end_xyz = list(body_xyz)
    dx = end_xyz[0] - start_xyz[0]
    dy = end_xyz[1] - start_xyz[1]
    dist = math.hypot(dx, dy)
    dyaw_deg = math.degrees(math.atan2(math.sin(latest_yaw[0] - start_yaw),
                                        math.cos(latest_yaw[0] - start_yaw)))
    summary = (f"    [{label}] dx={dx:+.3f} dy={dy:+.3f} dist={dist:.3f}m  "
               f"net_yaw={dyaw_deg:+.1f}deg  avg_speed={dist/elapsed_actual:.3f} m/s  "
               f"avg_yaw_rate={dyaw_deg/elapsed_actual:+.1f} deg/s"
               + ("  *** ABORTED (fell/flipped during this phase) ***" if aborted[0] else ""))
    print(summary)
    log_line(f"PHASE END: {label}")
    if aborted[0]:
        failed_phase[0] = label
        return True
    return False

def settle(duration=PHASE_SETTLE):
    desired_vx[0] = 0.0
    desired_turn[0] = 0.0
    deadline = time.time() + duration
    while time.time() < deadline and not aborted[0]:
        check_abort()
        time.sleep(0.05)
    # Ported from trot_demo.py's post-gait-fall fix: the fixed PHASE_SETTLE duration above says
    # nothing about whether the body has actually stopped moving, and a phase with more residual
    # momentum (combined linear+angular in particular) could still be sliding when the next phase
    # starts, contaminating its start_xyz/start_yaw baseline. Give it a little extra, speed-gated time
    # on top, capped so a genuinely-never-settling case can't hang the script.
    if not aborted[0]:
        EXTRA_MAX = 3.0
        _extra_start = time.time()
        while not aborted[0]:
            if check_abort():
                break
            speed_now = math.hypot(body_vel[0], body_vel[1])
            if speed_now <= SPEED_SETTLE_THRESHOLD or time.time() - _extra_start >= EXTRA_MAX:
                break
            time.sleep(0.05)

# ============================================================================
# main sequence
# ============================================================================
print("logging this run to run_log_turn_test.txt (same folder)")
print("waiting for the drop to settle...")
time.sleep(5.0)
print(f"landed, pitch = {math.degrees(latest_pitch[0]):.2f} deg  roll = {math.degrees(latest_roll[0]):.2f} deg")

t.start()

print("--- crouch ---")
move_feet_manual({leg: (0.0, STANCE_FZ) for leg in legs}, duration=1.5, steps=75)

gait_start_time[0] = time.time()
gait_active[0] = True

PHASES = [
    ("forward walk",              MAX_VX,  0.0),
    ("in-place rotation",         0.0,     MAX_TURN),
    ("back walk",                -MAX_VX,  0.0),
    ("combined linear+angular",   MAX_VX,  MAX_TURN),
]

print("=" * 60)
print(" scripted test: " + " -> ".join(label for label, _, _ in PHASES))
print("=" * 60)

if not aborted[0]:
    for label, vx, turn in PHASES:
        if run_phase(label, vx, turn):
            break
        settle()

print("--- stopping: ramping targets to zero ---")
desired_vx[0] = 0.0
desired_turn[0] = 0.0
stop_deadline = time.time() + 2.0
while time.time() < stop_deadline and not aborted[0]:
    if abs(target_vx[0]) < 0.005 and abs(target_turn[0]) < 0.02:
        break
    time.sleep(0.05)

gait_active[0] = False
if not aborted[0]:
    print("--- returning to a neutral stance ---")
    move_feet_manual({leg: (0.0, STANCE_FZ) for leg in legs}, duration=1.0, steps=50)

# Kept in sync with manual_control.py: keep the control thread alive - and with it the pitch/roll
# correction and flat-stance holding - until real measured speed has actually settled, instead of
# tearing it down right after the fixed-duration move above regardless of residual momentum.
if not aborted[0]:
    print(f"--- waiting for speed < {SPEED_SETTLE_THRESHOLD:.2f} m/s before stopping ---")
    END_WAIT_MIN, END_WAIT_MAX = 1.0, 5.0
    _wait_start = time.time()
    while not aborted[0]:
        if check_abort():
            break
        _elapsed_wait = time.time() - _wait_start
        speed_now = math.hypot(body_vel[0], body_vel[1])
        if (_elapsed_wait >= END_WAIT_MIN and speed_now <= SPEED_SETTLE_THRESHOLD) or _elapsed_wait >= END_WAIT_MAX:
            break
        time.sleep(0.1)

time.sleep(0.2)
running[0] = False
if aborted[0]:
    print(f"sequence stopped early (safety abort) during phase: {failed_phase[0]}")
else:
    print("all phases completed without a safety abort")
print("turn_test.py exiting")
