import gz.transport13 as transport
from gz.msgs10.double_pb2 import Double
from gz.msgs10.imu_pb2 import IMU
from gz.msgs10.pose_v_pb2 import Pose_V
import math
import threading
import time
import sys

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

# --- ground-truth body pose (diagnostics only) ------------------------------------------------------
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
TROT_PERIOD = 1.6
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

def leg_is_swinging(leg, t):
    return leg_phase_frac(leg, t) < SWING_DUTY

def current_swing_pair(t):
    if leg_is_swinging(PAIR_A[0], t):
        return "A"
    if leg_is_swinging(PAIR_B[0], t):
        return "B"
    return None

# --- live control targets, updated by the keyboard thread, read by control_loop --------------------
MAX_VX = 0.20               # m/s - forward/backward speed ceiling. Pitch/roll tipping is the main
                            # risk at higher values; FLIP_LIMIT_DEG's safety abort is the backstop.
MAX_TURN = 1.0              # dimensionless steering intensity, -1.0 (full left) to +1.0 (full right) -
                            # NOT a calibrated rad/s. There's no measured-yaw-rate feedback in this
                            # file (see header note), so this is an open-loop command, not a target
                            # the code verifies it actually achieved.
MAX_VX_ACCEL = 0.20         # m/s of target_vx change allowed per second - keeps a key-tap from
                            # injecting a sudden step change into the gait; ~0.75s to go 0->MAX_VX.
MAX_TURN_ACCEL = 2.0        # steering-units per second, same reasoning.

TURN_GAIN = 0.15            # scales steering intensity into a foot-offset command.
MAX_TURN_FX = 0.06          # ceiling on that offset, so steering can't destabilize a swinging leg's
                            # landing target.

target_vx = [0.0]           # current, rate-limited forward-speed target (m/s)
target_turn = [0.0]         # current, rate-limited steering target (-1..+1)
desired_vx = [0.0]          # what the keys are currently asking for (before rate limiting)
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
CORRECTION_FRACTION = 0.4
MAX_CORRECTION_RAD = 0.35
PITCH_RATE_DAMPING = 0.15
ROLL_ABAD_FRACTION = 0.3
MAX_ABAD_ROLL_CORR = 0.15

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
last_status = {"turn_term": 0.0}

# The trot phase only advances while there's live input above a small deadband;
# otherwise all four feet hold a static planted stance and the phase clock resets to
# 0, so movement always resumes cleanly from a full stance rather than mid-cycle.
MOVE_EPS_VX = 0.005     # m/s - below this, forward/back input counts as "released"
MOVE_EPS_TURN = 0.02    # steering units - below this, turn input counts as "released"

# Foot targets are position commands sent straight to IK every tick, with no
# smoothing of their own (the trot waveform is smooth by construction, but the
# switch between "trotting" and "static neutral stance" above is a mode change,
# not a continuous function) - rate-limit them here so that switch is a fast
# glide instead of an instant multi-cm snap.
FOOT_RATE_LIMIT = 1.0   # m/s ceiling on how fast a commanded foot position may move

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

        # rate-limit the live targets toward whatever the keyboard thread is currently asking for
        target_vx[0] = _rate_limit(target_vx[0], desired_vx[0], MAX_VX_ACCEL, dt)
        target_turn[0] = _rate_limit(target_turn[0], desired_turn[0], MAX_TURN_ACCEL, dt)

        is_moving = (abs(target_vx[0]) > MOVE_EPS_VX) or (abs(target_turn[0]) > MOVE_EPS_TURN)

        if gait_active[0] and is_moving:
            if gait_start_time[0] is None:
                gait_start_time[0] = now
            t = now - gait_start_time[0]
            for leg in legs:
                raw_fx, raw_fz = foot_target_for_leg(leg, t, target_vx[0])
                cur_fx, cur_fz = foot_target[leg]
                fx = _rate_limit(cur_fx, raw_fx, FOOT_RATE_LIMIT, dt)
                fz = _rate_limit(cur_fz, raw_fz, FOOT_RATE_LIMIT, dt)
                foot_target[leg] = (fx, fz)
            active_pair = current_swing_pair(t)
        else:
            gait_start_time[0] = None   # so the next move starts the phase clock fresh, at t=0
            for leg in legs:
                cur_fx, cur_fz = foot_target[leg]
                fx = _rate_limit(cur_fx, 0.0, FOOT_RATE_LIMIT, dt)
                fz = _rate_limit(cur_fz, STANCE_FZ, FOOT_RATE_LIMIT, dt)
                foot_target[leg] = (fx, fz)
            active_pair = None

        theta_p = PITCH_SIGN * CORRECTION_FRACTION * latest_pitch[0]
        theta_d = PITCH_SIGN * PITCH_RATE_DAMPING * latest_pitch_rate[0]
        theta = max(-MAX_CORRECTION_RAD, min(MAX_CORRECTION_RAD, theta_p + theta_d))
        last_theta_terms["clamped"] = theta

        roll_term = ROLL_ABAD_FRACTION * latest_roll[0]
        roll_term = max(-MAX_ABAD_ROLL_CORR, min(MAX_ABAD_ROLL_CORR, roll_term))

        turn_term = max(-MAX_TURN_FX, min(MAX_TURN_FX, TURN_GAIN * target_turn[0]))

        swinging_legs = PAIR_A if active_pair == "A" else (PAIR_B if active_pair == "B" else [])

        # Both legs of the swinging pair share the same raw stride fx (same phase, same
        # direction) but get +turn_term / -turn_term respectively. Shrink turn_term to
        # whatever headroom both legs can absorb without exceeding TROT_FX_LIMIT, so
        # steering authority tapers off smoothly and symmetrically as stride amplitude
        # grows, instead of one leg clipping while the other keeps the full offset.
        if swinging_legs:
            raw_fx_pair = foot_target[swinging_legs[0]][0]
            turn_headroom = max(0.0, TROT_FX_LIMIT - abs(raw_fx_pair))
            turn_term = max(-turn_headroom, min(turn_headroom, turn_term))
        last_status["turn_term"] = turn_term

        for leg in legs:
            fx, fz = foot_target[leg]
            this_turn_term = turn_term if leg in swinging_legs else 0.0
            fx = clamp_fx(fx + LEG_LR[leg] * this_turn_term)
            fx_c, fz_c = rotate(fx, fz, theta)
            abad_geo, hip, knee = leg_ik_3d(fx_c, 0.0, fz_c, OY[leg], LEG_SIDE[leg])
            abad = abad_geo + roll_term

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

# --- keyboard input - Windows only (msvcrt is stdlib on Windows, no extra install needed). To port
# to Linux/macOS, swap this thread's body for a termios/tty cbreak-mode reader or the third-party
# `keyboard` package - everything else in this file is platform-independent. ------------------------
try:
    import msvcrt
except ImportError:
    print("manual_control.py needs Windows (uses msvcrt for keyboard input) - see the comment "
          "above the import for a porting note.")
    sys.exit(1)

KEY_HOLD_TIMEOUT = 0.25   # seconds since a key was last seen before treating it as released -
                          # Windows' console key-repeat re-sends a held key every ~30-50ms, so this
                          # comfortably bridges normal repeat gaps without lagging on release.
_last_seen = {"w": 0.0, "a": 0.0, "s": 0.0, "d": 0.0}

def _held(key):
    return (time.time() - _last_seen[key]) < KEY_HOLD_TIMEOUT

def keyboard_thread():
    while running[0]:
        if msvcrt.kbhit():
            ch = msvcrt.getch()
            if ch in (b"q", b"Q", b"\x1b"):
                running[0] = False
                continue
            if ch == b" ":
                desired_vx[0] = 0.0
                desired_turn[0] = 0.0
                for k in _last_seen:
                    _last_seen[k] = 0.0
                continue
            try:
                k = ch.decode("utf-8").lower()
            except UnicodeDecodeError:
                k = None
            if k in _last_seen:
                _last_seen[k] = time.time()
        w, a, s, d = _held("w"), _held("a"), _held("s"), _held("d")
        desired_vx[0] = MAX_VX if (w and not s) else (-MAX_VX if (s and not w) else 0.0)
        desired_turn[0] = MAX_TURN if (d and not a) else (-MAX_TURN if (a and not d) else 0.0)
        time.sleep(0.01)

def status_line():
    vx, vy, vz = body_vel
    speed = math.hypot(vx, vy)
    print(f"  vx_target={target_vx[0]:+.3f}  turn_target={target_turn[0]:+.2f}  "
          f"speed_xy={speed:.3f}  pitch={math.degrees(latest_pitch[0]):+.1f}deg  "
          f"roll={math.degrees(latest_roll[0]):+.1f}deg  yaw={math.degrees(latest_yaw[0]):+.1f}deg")

# ============================================================================
# main sequence
# ============================================================================
print("waiting for the drop to settle...")
time.sleep(5.0)
print(f"landed, pitch = {math.degrees(latest_pitch[0]):.2f} deg  roll = {math.degrees(latest_roll[0]):.2f} deg")

t.start()   # started AFTER the drop-settle wait, so the balance correction doesn't fight the drop

print("--- crouch ---")
move_feet_manual({leg: (0.0, STANCE_FZ) for leg in legs}, duration=1.5, steps=75)

gait_start_time[0] = time.time()
gait_active[0] = True

kb_thread = threading.Thread(target=keyboard_thread, daemon=True)
kb_thread.start()

print("=" * 60)
print(" W/S = forward/back   A/D = turn left/right   SPACE = stop")
print(" Q or ESC = quit (ramps to a stop, then a safe stance)")
print("=" * 60)

last_print = 0.0
while running[0]:
    if check_abort():
        break
    now = time.time()
    if now - last_print > 0.3:
        status_line()
        last_print = now
    time.sleep(0.05)

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

time.sleep(0.2)
running[0] = False
print("sequence stopped early (safety abort)" if aborted[0] else "manual_control.py exiting")
