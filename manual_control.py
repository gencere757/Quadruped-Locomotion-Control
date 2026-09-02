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
# pattern as trot_demo.py's run_log_trot.txt. Console interaction during a manual run is
# transient - once it scrolls off, it's gone - so this is what lets a "it fell after turning"
# run actually be looked at afterward instead of relying on what you happened to read live. -----
#
# Archive the previous run's log before truncating it (see turn_test.py's matching comment) -
# copies go in run_log_archive/, timestamped; run_log_manual.txt itself keeps meaning "the live/
# most recent run" for anything that reads it.
_LOG_NAME = "run_log_manual.txt"
_ARCHIVE_DIR = "run_log_archive"
try:
    os.makedirs(_ARCHIVE_DIR, exist_ok=True)
    if os.path.exists(_LOG_NAME):
        _ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        shutil.copy2(_LOG_NAME, os.path.join(_ARCHIVE_DIR, f"{_ts}_{_LOG_NAME}"))
except OSError:
    pass

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
TROT_PERIOD = 1.0            # was 1.6. foot_target_for_leg's stride amplitude A = 0.5*vx*STANCE_DUTY*
                            # TROT_PERIOD scales linearly with this at a fixed commanded speed, so a
                            # shorter period means shorter, more frequent strides for the same vx instead
                            # of fewer big lunging ones - directly cuts how far (and how hard) each leg's
                            # own mass swings per step. Worth trying because the recurring falls at every
                            # p_gain/d_gain we tried (72.6 through 265) were almost always in plain
                            # forward/back walking with turn=0.00 - not a turn-logic problem, and not
                            # obviously a joint-tracking-gain problem either since that axis plateaued.
                            # The model's legs got much heavier in the latest CAD export (thigh+shank
                            # combined mass roughly tripled vs. before), so each stride now swings a lot
                            # more mass, and the body pitch-balance correction below (CORRECTION_FRACTION
                            # etc.) was tuned against the old, lighter legs and never revisited - shrinking
                            # the disturbance itself is the safer first test vs. retuning that loop blind.
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

# --- live control targets, updated by the keyboard thread, read by control_loop --------------------
MAX_VX = 0.15               # m/s - forward/backward speed ceiling. Pitch/roll tipping is the main
                            # risk at higher values; FLIP_LIMIT_DEG's safety abort is the backstop.
                            # Cut from 0.20 after TROT_PERIOD/TURN_VX_GAIN got things from "falls
                            # immediately, every phase" to "clears most phases most runs, occasionally
                            # aborts 0-2deg over the 25deg limit near the end of a phase" - the remaining
                            # failures are a stride disturbance that's still just a bit too big for this
                            # much heavier model, not a control-loop bug (the PITCH_RATE_DAMPING experiment
                            # already showed retuning that loop blind makes it worse). Same lever as
                            # TROT_PERIOD (stride amplitude A = 0.5*vx*STANCE_DUTY*TROT_PERIOD scales
                            # directly with vx too), just applied to the ceiling instead of the period -
                            # a 25% cut in top speed for a real margin under the abort line.
MAX_TURN = 1.0              # dimensionless steering intensity, -1.0 (full left) to +1.0 (full right) -
                            # NOT a calibrated rad/s. There's no measured-yaw-rate feedback in this
                            # file (see header note), so this is an open-loop command, not a target
                            # the code verifies it actually achieved.
MAX_VX_ACCEL = 0.20         # m/s of target_vx change allowed per second - keeps a key-tap from
                            # injecting a sudden step change into the gait; ~0.75s to go 0->MAX_VX.
MAX_TURN_ACCEL = 2.0        # steering-units per second, same reasoning.

TURN_VX_GAIN = 0.2 * MAX_VX # turns steering intensity into a per-side *effective forward speed* -
                            # see foot_target_for_leg / control_loop. At full steering (+-1.0) each
                            # side's leg_vx is offset by +-TURN_VX_GAIN, so a full turn command with
                            # target_vx==0 drives the left legs forward at TURN_VX_GAIN and the right
                            # legs backward at TURN_VX_GAIN (or vice versa) - genuine differential-drive
                            # rotation in place, since amplitude+direction in foot_target_for_leg both
                            # come from this per-leg speed, not a one-time touchdown offset.
                            #
                            # Was MAX_VX, then halved to 0.5*MAX_VX after run_log_turn_test.txt showed
                            # rotation working but pitch escalating cycle-over-cycle during full-strength
                            # rotation (roll stayed bounded/sign-flipping, ruling out the roll/stance-
                            # narrowing theory - it's the turn disturbance itself overpowering the pitch
                            # correction loop). That got pure rotation and translate-only phases reliably
                            # clean on the new (heavier) model, after re-tuning p_gain/d_gain up through
                            # 72.6->145->217->250->265 (282 overshot). But across many runs at all of
                            # those gains, "combined linear+angular" (full vx AND full turn at once) kept
                            # being the one phase that still fails - it's the same overpowering-the-pitch-
                            # correction mechanism, just needing translation AND turn stacked to trigger
                            # it instead of turn alone. Cut to 0.35*MAX_VX, then confirmed with
                            # TROT_PERIOD=1.0 (below) that forward walk / rotation / back walk now ALL
                            # clear cleanly (max pitch ~18deg, well under FLIP_LIMIT_DEG) - TROT_PERIOD
                            # was the real fix for the plain-walking falls. combined linear+angular is now
                            # the ONLY phase that still fails, and it fails fast: run_log_turn_test.txt
                            # showed pitch swinging -10 -> +14 -> -14deg inside ~1.8s of combined starting,
                            # then aborting at -26.5deg - an oscillation, not a slow drift, so the stacked
                            # vx+turn disturbance is still too strong for the pitch correction loop to
                            # damp before it resonates. Cut further to 0.2*MAX_VX; if combined still fails
                            # reliably at this value, the next lever is CORRECTION_FRACTION/
                            # PITCH_RATE_DAMPING (untouched since the old lighter-leg model) rather than
                            # cutting this again - a sub-0.2 TURN_VX_GAIN would barely rotate at all while
                            # walking, which stops being a fix and starts being "don't turn while moving."

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
CORRECTION_FRACTION = 0.25 # was 0.4. A fresh manual run fell during plain continuous forward walking
                           # (W held, turn=0 the whole time) - not a release/settle issue, a real fall
                           # mid-walk: pitch oscillated with growing/non-decaying amplitude almost from
                           # the start (-7.2, -10.8, -17.9, -19.5, -24.0deg) over about 18s of walking,
                           # finally crossing the 25deg abort. That's the exact same signature
                           # trot_demo.py had for its post-gait oscillation (large swinging pitch that
                           # never settles, not a slow one-directional drift), which got fixed there by
                           # cutting this same gain 0.4->0.25 - but that fix was never ported over to
                           # this file. Cutting P here too rather than raising the D term
                           # (PITCH_RATE_DAMPING) - that was already tried once for a similar-looking
                           # oscillation and made things clearly worse (derivative kick from a noisy
                           # signal, see PITCH_RATE_DAMPING's own comment below).
MAX_CORRECTION_RAD = 0.45  # was 0.35. Several aborts happened with pitch climbing steadily right up to
                           # the 25deg limit rather than oscillating - i.e. the correction was applying
                           # its full clamped authority (0.35 rad = 20deg of foot-rotation) and still
                           # losing ground to the disturbance. Raising the clamp gives it more authority to
                           # actually arrest a developing lean before the safety abort triggers, instead of
                           # capping out partway through one. Left CORRECTION_FRACTION (the gain) alone -
                           # this only raises how far it's allowed to go, not how aggressively.
PITCH_RATE_DAMPING = 0.15  # REVERTED - briefly tried 0.3 (doubled) to fix an underdamped-looking
                           # oscillation, but it made things clearly worse: 3 straight runs after that
                           # change failed immediately in forward walk, including one violent one (pitch
                           # +8.4 -> +5.0 -> abort at 26.1deg WITH roll suddenly at 10.6deg and
                           # avg_yaw_rate=45.7 deg/s - a fast tip/spin, not a slow climb) within under a
                           # second of the gait starting. latest_pitch_rate is a finite-difference
                           # derivative of a noisy signal, only lightly smoothed (PITCH_RATE_LPF_ALPHA =
                           # 0.2 - see above), so doubling its gain likely amplified noise into real
                           # torque commands right at the crouch-to-gait transition (when pitch is
                           # changing fastest and least predictably) instead of adding clean damping -
                           # classic derivative-kick. Back to 0.15. If pitch-loop retuning is worth
                           # revisiting, smooth the derivative more first (lower PITCH_RATE_LPF_ALPHA, which
                           # gives more weight to past samples) so it's trustworthy before turning its gain
                           # up again - don't just re-try a bigger D on the same noisy signal.
ROLL_ABAD_FRACTION = 0.3    # Restored to the original value for an A/B baseline run against the
                            # ROLL_ABAD_FRACTION=0.0 test - see the comment at "abad = abad_geo +
                            # roll_term" below for the narrowing-stance-width theory this is testing.
MAX_ABAD_ROLL_CORR = 0.15

SPEED_SETTLE_THRESHOLD = 0.06   # m/s - ported from trot_demo.py's post-gait-fall fix. Treat the body
                                 # as "stopped enough" once real MEASURED speed is at/below this, not
                                 # just once the rate-limited target_vx/turn have decayed to ~0 - those
                                 # say nothing about whether the body itself has actually stopped
                                 # moving. Added after a manual run where releasing all input (W/A/S/D)
                                 # made the robot jump backward and fall: the shutdown sequence used to
                                 # kill the control thread (and with it the pitch/roll correction AND the
                                 # flat-stance holding - see control_loop's else branch) on a fixed 2s
                                 # timer keyed only off the targets, regardless of real residual speed.
                                 # See the wait loop at the bottom of the main sequence below.

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
last_status = {"roll_term": 0.0}   # published for status_line/diagnostic_line to read - control_loop
                                    # runs on its own thread, so this is how the debugging output sees
                                    # the roll->ABAD correction term without recomputing it separately.
last_abad = {leg: 0.0 for leg in legs}   # final commanded ABAD angle per leg, for watching stance
                                          # width narrow/widen live (see the roll_term investigation).

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
            # Per-leg effective forward speed = the commanded translation plus a per-side
            # steering offset (+ for left legs, - for right legs, via LEG_LR). Feeding this
            # into foot_target_for_leg instead of the shared target_vx[0] means turning isn't
            # a separate touchdown-offset bolted on afterward - it's the same stride-amplitude
            # and stride-direction machinery already used for walking, just driven per side.
            # That's what lets stance legs actively sweep during a pure in-place rotation
            # (target_vx==0, target_turn!=0), and lets translate+turn compose for free.
            turn_vx = TURN_VX_GAIN * target_turn[0]
            for leg in legs:
                leg_vx = target_vx[0] + LEG_LR[leg] * turn_vx
                raw_fx, raw_fz = foot_target_for_leg(leg, t, leg_vx)
                cur_fx, cur_fz = foot_target[leg]
                fx = _rate_limit(cur_fx, raw_fx, FOOT_RATE_LIMIT, dt)
                fz = _rate_limit(cur_fz, raw_fz, FOOT_RATE_LIMIT, dt)
                foot_target[leg] = (fx, fz)
        else:
            gait_start_time[0] = None   # so the next move starts the phase clock fresh, at t=0
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
            # NOTE: roll_term is added identically to every leg here, with no per-side sign flip.
            # abad_geo's own sign is already mirrored left/right (baked into leg_ik_3d via the oy
            # term), so adding the SAME delta to all four legs doesn't symmetrically flare/narrow
            # both sides - it shoves every foot's lateral position the same physical direction,
            # which widens stance on one side and narrows it on the other. That's fine (small) noise
            # for standing/walking, but turning induces real roll from the asymmetric left/right
            # stance forces, and this term reacting to that roll can end up progressively pulling
            # one side's feet inward right when the diagonal-trot support base is already thin -
            # a plausible mechanism for "legs come too close together" after turning. If disabling
            # it above (ROLL_ABAD_FRACTION = 0.0) stops the turn-then-fall, the fix is to mirror the
            # sign per leg, e.g. `abad = abad_geo + OY[leg] * roll_term`, instead of re-enabling this
            # as-is.
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

# --- keyboard input - Windows only. Uses ctypes + GetAsyncKeyState to poll real, simultaneous
# key state directly, rather than inferring "held" from msvcrt's console key-repeat (which only
# auto-repeats one "active" key at a time - see the old KEY_HOLD_TIMEOUT approach in git history -
# so translation + rotation keys held together would starve each other out). To port to
# Linux/macOS, swap this thread's body for a termios/tty cbreak-mode reader or the third-party
# `keyboard` package - everything else in this file is platform-independent. ------------------------
if not sys.platform.startswith("win"):
    print("manual_control.py needs Windows (uses GetAsyncKeyState for keyboard input) - see the "
          "comment above this check for a porting note.")
    sys.exit(1)

import ctypes

VK_W, VK_A, VK_S, VK_D = 0x57, 0x41, 0x53, 0x44
VK_SPACE, VK_Q, VK_ESCAPE = 0x20, 0x51, 0x1B

def _key_down(vk):
    # High bit of GetAsyncKeyState's return is set iff the key is down *right now* -
    # a direct hardware-state query, independent per key, with no repeat-timing involved.
    return bool(ctypes.windll.user32.GetAsyncKeyState(vk) & 0x8000)

def keyboard_thread():
    while running[0]:
        if _key_down(VK_Q) or _key_down(VK_ESCAPE):
            running[0] = False
            continue
        if _key_down(VK_SPACE):
            desired_vx[0] = 0.0
            desired_turn[0] = 0.0
        else:
            w, a, s, d = _key_down(VK_W), _key_down(VK_A), _key_down(VK_S), _key_down(VK_D)
            desired_vx[0] = MAX_VX if (w and not s) else (-MAX_VX if (s and not w) else 0.0)
            desired_turn[0] = MAX_TURN if (d and not a) else (-MAX_TURN if (a and not d) else 0.0)
        time.sleep(0.01)

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
# main sequence
# ============================================================================
print("logging this run to run_log_manual.txt (same folder)")
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

# Keep the control thread alive - and with it the pitch/roll correction and flat-stance holding -
# until real measured speed has actually settled, instead of tearing it down right after the fixed-
# duration move above regardless of residual momentum. See SPEED_SETTLE_THRESHOLD's comment; this is
# the direct port of trot_demo.py's coast-to-stop / speed-gated final wait, applied to manual_control's
# own shutdown instead of a scripted run's end.
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
print("sequence stopped early (safety abort)" if aborted[0] else "manual_control.py exiting")
