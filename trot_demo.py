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

# Archive the previous run's log before truncating it (see turn_test.py's matching comment) -
# copies go in run_log_archive/, timestamped; run_log_trot.txt itself keeps meaning "the live/
# most recent run" for anything that reads it.
_LOG_NAME = "run_log_trot.txt"
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
FRONT_BACK = {"FL": 1, "FR": 1, "BL": -1, "BR": -1}   # used by the fy-based yaw steering below -
                                                        # +1 front, -1 back (independent of left/right).

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

# --- IMU: orientation + filtered rates ---------------------------------------------------------------
latest_pitch = [0.0]
latest_roll = [0.0]
latest_yaw = [0.0]

PITCH_RATE_LPF_ALPHA = 0.2
latest_pitch_rate = [0.0]
_pitch_rate_source = [None]
_prev_pitch_for_rate = [None]
_prev_pitch_rate_time = [None]

YAW_RATE_LPF_ALPHA = 0.2
latest_yaw_rate = [0.0]
_yaw_rate_source = [None]
_prev_yaw_for_rate = [None]
_prev_yaw_rate_time = [None]

def imu_callback(msg):
    q = msg.orientation
    sinp = max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))
    latest_pitch[0] = math.asin(sinp)
    latest_roll[0] = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    latest_yaw[0] = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    raw_rate = None
    try:
        raw_rate = msg.angular_velocity.y
        _pitch_rate_source[0] = "gyro"
    except AttributeError:
        _pitch_rate_source[0] = "fd"
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

    yaw_raw_rate = None
    try:
        yaw_raw_rate = msg.angular_velocity.z
        _yaw_rate_source[0] = "gyro"
    except AttributeError:
        _yaw_rate_source[0] = "fd"
        now2 = time.time()
        if _prev_yaw_for_rate[0] is not None and _prev_yaw_rate_time[0] is not None:
            dt2 = now2 - _prev_yaw_rate_time[0]
            if dt2 > 1e-4:
                yaw_raw_rate = (latest_yaw[0] - _prev_yaw_for_rate[0]) / dt2
        _prev_yaw_for_rate[0] = latest_yaw[0]
        _prev_yaw_rate_time[0] = now2
    if yaw_raw_rate is not None:
        latest_yaw_rate[0] = (YAW_RATE_LPF_ALPHA * yaw_raw_rate
                               + (1.0 - YAW_RATE_LPF_ALPHA) * latest_yaw_rate[0])

node.subscribe(IMU, "/model/my_quadruped/imu", imu_callback)

# --- ground-truth body pose + per-foot world z (for diagnostics only, no support-polygon math) -----
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

# --- trot gait config -------------------------------------------------------------------------------
STANCE_FZ = -0.34         # crouch/stance depth, validated for this robot's leg geometry.
SWING_HEIGHT = 0.05
TROT_FX_LIMIT = 0.12      # reach-safety clamp on commanded fx. Geometric max reach at STANCE_FZ with
                           # L1=L2=0.2 is sqrt(0.4^2-0.34^2)=~0.21m; this leaves a healthy margin below
                           # that.

TROT_PERIOD = 1.0         # seconds per full cycle. Real small quadrupeds trot at roughly 0.3-0.6s
                           # cycles; this stays well below that for stability margin.
                           # Cut from 1.6 - manual_control.py/turn_test.py hit the same recurring
                           # falls-during-plain-walking problem on this much heavier CAD export
                           # (thigh+shank mass roughly tripled), and this same fix (shorter/more-frequent
                           # strides instead of fewer big lunges - see STRIDE_HALF_AMPLITUDE below, which
                           # scales directly with this) is what got them clearing most runs instead of
                           # falling every run. DESIRED_VX here is already 0.15 (matches the other files'
                           # post-fix MAX_VX), so this brings trot_demo.py's disturbance magnitude in
                           # line with the configuration that's actually been shown to work.
STANCE_DUTY = 0.5         # fraction of TROT_PERIOD each leg spends in stance. At 0.5 there are no
                           # all-4-down double-support windows at all (pure alternating trot, one pair
                           # always swinging) - see PAIR_PHASE_OFFSET below. Below 0.5 would require an
                           # actual flight phase (running trot), so 0.5 is the floor for this mechanism.

SWING_DUTY = 1.0 - STANCE_DUTY

DESIRED_VX = 0.15         # m/s - target forward speed. Kept below the eventual 0.5-0.6 m/s goal
                           # because raising it increases STRIDE_HALF_AMPLITUDE directly, which
                           # increases the yaw disturbance per step; MAX_YAW_FX/MAX_YAW_FY below are
                           # fixed absolute caps, so a bigger stride needs more correction headroom
                           # than they currently provide.

# Derived: the stance-time relationship for a planted foot sweeping while the body moves at DESIRED_VX.
# A foot fixed on the ground during a stance phase of duration STANCE_DUTY*TROT_PERIOD, while the body
# moves DESIRED_VX*STANCE_DUTY*TROT_PERIOD forward, must sweep exactly that same distance in the hip
# frame (foot fixed, body moves = hip-frame foot position sweeps backward by the body's own excursion).
# Symmetric about the hip (+A at touchdown/leading, -A at liftoff/trailing) gives a full sweep of 2A:
STRIDE_HALF_AMPLITUDE = 0.5 * DESIRED_VX * STANCE_DUTY * TROT_PERIOD

# current_target_vx drives the live stride amplitude (see foot_target_for_leg below) - starts equal
# to DESIRED_VX (identical behavior to the old fixed STRIDE_HALF_AMPLITUDE) and is only ever changed
# during the new deceleration ramp at the end of the run (see DECEL_DURATION near the main sequence).
# Why: every post-gait fall we've seen (across several runs, both before and after retuning the
# pitch-correction gain) happens AFTER "trot complete" - not during active walking - while genuinely
# at rest with real leftover speed_xy (up to ~0.48 m/s) that just keeps oscillating instead of
# decaying. The crouch phase, BEFORE any gait ever ran, is rock solid (pitch/roll pinned near 0.00deg
# for 10+ seconds, zero drift) - which rules out a generic joint-holding-stiffness problem at these
# gains and points squarely at leftover momentum from cutting a full-speed trot off abruptly: stride
# amplitude (and therefore the body's own commanded motion) used to go from full DESIRED_VX straight
# to a dead stop the instant N_CYCLES elapsed, with nothing to actively arrest whatever real velocity
# the body had built up. Ramping stride amplitude down to 0 over the last DECEL_DURATION seconds,
# using the gait's own normal swing/stance mechanics, lets the legs actually brake the body the way a
# real walking animal slows down - a few progressively shorter strides - instead of freezing mid-
# momentum and leaving the pitch-correction loop to fight a disturbance it has no way to actually stop.
current_target_vx = [DESIRED_VX]

ENABLE_RAIBERT_FEEDBACK = False   # When True, blends in a live-velocity-based correction (Raibert
                                   # 1986) on top of the fixed STRIDE_HALF_AMPLITUDE above, which
                                   # passively resists speed disturbances (too fast -> touchdown moves
                                   # further forward -> next stance decelerates more; too slow -> the
                                   # reverse).
KV_RAIBERT = 0.03                 # meters of touchdown-position shift per (m/s) of velocity error.
                                   # Starting small deliberately - this is unvalidated.

# KNOWN BUG, left disabled because of it: A_dyn's dominant term below scales by vx_meas (the
# actual measured speed) instead of DESIRED_VX (the target speed). That makes the feedback mostly
# recompute the stride to match whatever speed the robot is ALREADY doing rather than correct
# toward the target - if it's undershooting, this shrinks the stride to match the shortfall
# instead of pushing back against it. KV_RAIBERT's error-correction term is too small to outweigh
# that. The fix is using DESIRED_VX (i.e. STRIDE_HALF_AMPLITUDE) as the base and adding only the
# measured-error term on top, matching actual Raibert foot-placement control.

def _smoothstep(x):
    x = max(0.0, min(1.0, x))
    return x * x * (3 - 2 * x)

def clamp_fx(v):
    return max(-TROT_FX_LIMIT, min(TROT_FX_LIMIT, v))

# Diagonal pairing - classic trot: front-left+back-right together, front-right+back-left together.
# Phase offsets 0.0 and 0.5 put pair A's swing window and pair B's swing window exactly half a
# cycle apart. At STANCE_DUTY=0.5 (SWING_DUTY=0.5) the two windows exactly tile the cycle with no
# gap, so one pair is always swinging and at most 2 legs are ever airborne at once. A STANCE_DUTY
# above 0.5 would open real all-4-down double-support windows between the two pairs' swings.
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

def foot_target_for_leg(leg, t, vx_meas):
    local_phase = leg_phase_frac(leg, t)
    if ENABLE_RAIBERT_FEEDBACK:
        A_dyn = 0.5 * vx_meas * STANCE_DUTY * TROT_PERIOD + KV_RAIBERT * (vx_meas - DESIRED_VX)
        A = 0.5 * A_dyn + 0.5 * STRIDE_HALF_AMPLITUDE   # blended, not raw, to damp noisy vx_meas
    else:
        A = 0.5 * current_target_vx[0] * STANCE_DUTY * TROT_PERIOD

    if local_phase < SWING_DUTY:
        swing_frac = local_phase / SWING_DUTY
        s = _smoothstep(swing_frac)
        fx = -A + 2 * A * s
        fz = STANCE_FZ + SWING_HEIGHT * math.sin(math.pi * swing_frac)
    else:
        stance_frac = (local_phase - SWING_DUTY) / STANCE_DUTY
        fx = A - 2 * A * stance_frac
        fz = STANCE_FZ
    return clamp_fx(fx), fz

# --- reactive pitch/roll correction ------------------------------------------------------------------
PITCH_SIGN = 1.0
CORRECTION_FRACTION = 0.25  # was 0.4. The last two runs (after TROT_PERIOD=1.0 fixed the ACTIVE-
                            # walking falls) aborted during "post-gait settle"/"end of loop" instead -
                            # i.e. after the trot ended and gait_active went False, with all 4 legs
                            # just holding a static stance. theta swung between roughly -20 and +18deg
                            # (near MAX_CORRECTION_RAD's clamp both ways) while pitch oscillated
                            # -10/-6/-7/-18/-3/+3/-10/-15/-7/-1/-12/-22deg over ~5-6 seconds with no
                            # sign of decaying, right up to the 25deg abort - a growing/sustained
                            # oscillation while nominally at rest, not a disturbance still working
                            # itself out. This loop computes theta unconditionally every tick
                            # regardless of gait_active, so it's just as live during "standing still"
                            # as during active walking; a symmetric 4-leg stance should be passively
                            # stable at rest without needing an aggressive active correction, and the
                            # heavier legs' slower response to a commanded foot-rotation means the same
                            # P-gain that was fine before now has more effective loop delay behind it -
                            # a classic recipe for this loop's own correction ringing rather than
                            # damping. We already tried raising the D-term (PITCH_RATE_DAMPING) for a
                            # similar-looking oscillation during WALKING and it made things worse
                            # (amplified noise in the pitch-rate derivative into real torque kicks) - so
                            # this time cutting P instead, the standard alternative fix for a loop
                            # that's oscillating rather than cleanly settling.
MAX_CORRECTION_RAD = 0.35
PITCH_RATE_DAMPING = 0.15
ROLL_ABAD_FRACTION = 0.3
MAX_ABAD_ROLL_CORR = 0.15

# --- yaw correction, applied only to the currently-swinging pair (nudging an airborne foot's
# landing spot is free; pushing on a loaded stance foot is not) -------------------------------------
YAW_FX_GAIN = 0.15        # proportional gain turning yaw error into a fore-aft foot-offset correction.
MAX_YAW_FX = 0.06          # kept deliberately BELOW STRIDE_HALF_AMPLITUDE (0.078) so the correction
                           # term can never exceed the nominal stride itself and destabilize the gait
                           # on its own.

# Additive lateral (fy) yaw steering: splits front and back legs apart sideways instead of fore-aft -
# front legs get +fy_correction, back legs get -fy_correction (FRONT_BACK sign above), continuously,
# on all 4 legs (stance and swing alike). This doesn't create a front/back fx mismatch on a loaded
# diagonal stance pair, so it doesn't couple into pitch the way the fx correction can.
YAW_FY_GAIN = -0.07        # negative: increasing fy on a stance foot pushes the body the opposite way.
MAX_YAW_FY = 0.08

# Integral term: a pure-proportional correction against a persistent one-directional disturbance
# settles at a nonzero steady-state error by design - it never fully cancels a constant bias. The
# integral term keeps growing its contribution for as long as the error persists, canceling that
# residual bias instead of just leaving it.
YAW_KI = 0.3
YAW_INTEGRAL_LIMIT = 0.3  # anti-windup clamp on the integral term itself, so it stays a trim
                           # (max contribution 0.3*0.3=0.09 rad, ~5deg) rather than a stuck bias able
                           # to dominate the proportional term on its own.
yaw_integral = [0.0]

gait_start_yaw = [None]
gait_start_time = [None]
coast_to_stop = [False]  # True once the run's fixed duration has elapsed and we're just waiting for
                          # measured speed to actually settle before handing off to the static post-gait
                          # stance (see MAX_EXTRA_DECEL/SPEED_SETTLE_THRESHOLD in the main sequence).
                          # Added after a run where the wait period ran its full extra-grace window
                          # without ever reaching the speed threshold (0.166 m/s left at the cutoff,
                          # oscillating rather than decaying: 0.156 -> 0.251 -> 0.166) and then tipped
                          # during "end of loop" a few seconds later. Two things were still actively
                          # working against it settling during that wait: (1) the gait phase was still
                          # cycling swing/stance at zero stride amplitude, meaning only 2 feet were ever
                          # planted at once - a needlessly weak support base for something that's
                          # supposed to just be coasting to a stop; (2) yaw_integral was still pinned at
                          # its -0.300 clamp from the walk, and the yaw-correction block kept feeding
                          # that stale, saturated bias back in every tick since it only checks
                          # gait_active. Neither has anything to do with actually arresting momentum.

foot_target = {leg: (0.0, STANCE_FZ) for leg in legs}

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

SPEED_SETTLE_THRESHOLD = 0.06   # m/s - treat the body as "stopped enough" to hand off to the fixed
                           # post-gait stance / call the run done, once real measured speed is at/below
                           # this - not just once the commanded stride amplitude has reached zero or a
                           # fixed timer has expired. Used both by the coast-to-stop wait at the end of
                           # the trot loop and by the final "end of loop" tail below.

CONTROL_DT = 0.02
last_theta_terms = {"p": 0.0, "d": 0.0, "clamped": 0.0}
last_yaw_terms = {"err": 0.0, "fx_term": 0.0, "fy_term": 0.0}
last_active_pair = [None]

def control_loop():
    while running[0]:
        check_abort()

        active_pair = None
        if gait_active[0] and gait_start_time[0] is not None:
            if coast_to_stop[0]:
                # Plant all 4 feet flat immediately instead of continuing to cycle swing/stance at
                # zero amplitude - see coast_to_stop's comment above.
                for leg in legs:
                    foot_target[leg] = (0.0, STANCE_FZ)
            else:
                t = time.time() - gait_start_time[0]
                active_pair = current_swing_pair(t)
                for leg in legs:
                    foot_target[leg] = foot_target_for_leg(leg, t, body_vel[0])
        last_active_pair[0] = active_pair

        theta_p = PITCH_SIGN * CORRECTION_FRACTION * latest_pitch[0]
        theta_d = PITCH_SIGN * PITCH_RATE_DAMPING * latest_pitch_rate[0]
        theta = max(-MAX_CORRECTION_RAD, min(MAX_CORRECTION_RAD, theta_p + theta_d))
        last_theta_terms["p"] = theta_p
        last_theta_terms["d"] = theta_d
        last_theta_terms["clamped"] = theta

        roll_term = ROLL_ABAD_FRACTION * latest_roll[0]
        roll_term = max(-MAX_ABAD_ROLL_CORR, min(MAX_ABAD_ROLL_CORR, roll_term))

        if gait_active[0] and not coast_to_stop[0] and gait_start_yaw[0] is not None:
            yaw_err = latest_yaw[0] - gait_start_yaw[0]
            yaw_integral[0] = max(-YAW_INTEGRAL_LIMIT, min(YAW_INTEGRAL_LIMIT,
                                                             yaw_integral[0] + yaw_err * CONTROL_DT))
        else:
            yaw_err = 0.0
            yaw_integral[0] = 0.0
        yaw_signal = yaw_err + YAW_KI * yaw_integral[0]
        yaw_correction = max(-MAX_YAW_FX, min(MAX_YAW_FX, YAW_FX_GAIN * yaw_signal))
        fy_correction = max(-MAX_YAW_FY, min(MAX_YAW_FY, YAW_FY_GAIN * yaw_signal))
        last_yaw_terms["err"] = yaw_err
        last_yaw_terms["fx_term"] = yaw_correction
        last_yaw_terms["fy_term"] = fy_correction

        # The fx yaw correction is only ever applied to the currently-swinging pair, never to planted
        # (stance) legs: a diagonal stance pair always has one front leg and one back leg with
        # opposite LEG_LR sign, so pushing both by LEG_LR[leg]*yaw_correction would shift them in
        # opposite fx directions - not pure yaw correction, but a front/back stance asymmetry that
        # couples directly into pitch. Restricting it to swing legs (never "both pairs planted"
        # either, since that would push the body directly via ground reaction force through the same
        # LEG_LR sign, another mechanism entirely) avoids that coupling.
        swinging_legs = PAIR_A if active_pair == "A" else (PAIR_B if active_pair == "B" else [])

        for leg in legs:
            fx, fz = foot_target[leg]
            this_yaw_term = yaw_correction if leg in swinging_legs else 0.0
            fx = clamp_fx(fx + LEG_LR[leg] * this_yaw_term)
            fx_c, fz_c = rotate(fx, fz, theta)
            fy_c = FRONT_BACK[leg] * fy_correction   # lateral steering, all 4 legs, always on
            abad_geo, hip, knee = leg_ik_3d(fx_c, fy_c, fz_c, OY[leg], LEG_SIDE[leg])
            abad = abad_geo + roll_term

            m0 = Double(); m0.data = abad
            pubs[f"{leg}_ABAD"].publish(m0)
            m1 = Double(); m1.data = hip
            pubs[f"{leg}_HIP"].publish(m1)
            m2 = Double(); m2.data = knee
            pubs[f"{leg}_KNEE"].publish(m2)
        time.sleep(CONTROL_DT)

# Thread object created here, but not started until after the drop has settled, so no joint
# commands are published during the chaotic drop/land.
t = threading.Thread(target=control_loop, daemon=True)

def diagnostic_line(label):
    vx, vy, vz = body_vel
    speed = math.hypot(vx, vy)
    states = {l: ("SW" if leg_is_swinging(l, (time.time() - gait_start_time[0]) if gait_start_time[0] else 0.0) else "st")
              for l in legs} if gait_active[0] else {l: "-" for l in legs}
    foot_z_str = " ".join(f"{l}:{link_z.get(l, float('nan')):+.3f}" for l in legs)
    line = (f"  [{label}] pitch: {math.degrees(latest_pitch[0]):+.2f} deg  "
            f"roll: {math.degrees(latest_roll[0]):+.2f} deg  yaw: {math.degrees(latest_yaw[0]):+.2f} deg  "
            f"body_xyz: {body_xyz}  speed_xy: {speed:.3f} (target {DESIRED_VX:.3f})  "
            f"theta: {math.degrees(last_theta_terms['clamped']):+.2f} deg  "
            f"yaw_corr(err={math.degrees(last_yaw_terms['err']):+.2f}deg I={yaw_integral[0]:+.3f} -> fx_term={last_yaw_terms['fx_term']:+.4f} fy_term={last_yaw_terms['fy_term']:+.4f})  "
            f"legs: {states}  active_pair: {last_active_pair[0]}  "
            f"foot_z: {{{foot_z_str}}}")
    log_line(line)

def move_feet_manual(deltas, duration=1.5, steps=75, label=None):
    starts = {leg: foot_target[leg] for leg in deltas}
    print_every = max(1, steps // 10)
    for i in range(1, steps + 1):
        if check_abort():
            return
        frac = i / steps
        for leg, (tx, tz) in deltas.items():
            sx, sz = starts[leg]
            foot_target[leg] = (sx + (tx - sx) * frac, sz + (tz - sz) * frac)
        if label and (i % print_every == 0 or i == 1):
            diagnostic_line(f"{label} {frac*100:3.0f}%")
        time.sleep(duration / steps)
    for leg, tgt in deltas.items():
        foot_target[leg] = tgt

def ease_into_trot(duration=3.0, steps=150):
    """Ramp foot_target from the crouch stance point to each leg's own t=0 trot target before
       gait_active flips on - avoids a hard position jump at gait start."""
    starts = {leg: foot_target[leg] for leg in legs}
    targets = {leg: foot_target_for_leg(leg, 0.0, 0.0) for leg in legs}
    for i in range(1, steps + 1):
        if check_abort():
            return
        frac = i / steps
        for leg in legs:
            sx, sz = starts[leg]
            tx, tz = targets[leg]
            foot_target[leg] = (sx + (tx - sx) * frac, sz + (tz - sz) * frac)
        if i % 15 == 0 or i == 1:
            diagnostic_line(f"easing into trot {frac*100:3.0f}%")
        time.sleep(duration / steps)
    for leg in legs:
        foot_target[leg] = targets[leg]

N_CYCLES = 6

print("waiting for the drop to settle...")
time.sleep(5.0)
print(f"landed, pitch = {math.degrees(latest_pitch[0]):.2f} deg  roll = {math.degrees(latest_roll[0]):.2f} deg")
print(f"body position: {body_xyz}")
print(f"yaw at settle (before control_loop has published a single command): {math.degrees(latest_yaw[0]):+.2f} deg")

t.start()

print("--- crouch (hip/knee only, all ABAD at 0) ---")
move_feet_manual({leg: (0.0, STANCE_FZ) for leg in legs}, duration=1.5, steps=75, label="crouch")
for _ in range(20):
    if check_abort():
        break
    diagnostic_line("crouch settle")
    time.sleep(0.1)

start_xyz = list(body_xyz)
start_yaw = latest_yaw[0]
print(f"start_yaw (reset baseline, absolute): {math.degrees(start_yaw):+.2f} deg")
print(f"TROT_PERIOD={TROT_PERIOD}s  STANCE_DUTY={STANCE_DUTY}  DESIRED_VX={DESIRED_VX} m/s  "
      f"STRIDE_HALF_AMPLITUDE={STRIDE_HALF_AMPLITUDE:.4f} m  RAIBERT={ENABLE_RAIBERT_FEEDBACK}")

if not aborted[0]:
    print("--- easing into trot's t=0 pose ---")
    ease_into_trot()
if not aborted[0]:
    gait_start_yaw[0] = latest_yaw[0]
    gait_start_time[0] = time.time()
    gait_active[0] = True
    print(f"--- trot engaged: period={TROT_PERIOD}s  target_vx={DESIRED_VX} m/s  cycles={N_CYCLES} ---")
    total_duration = N_CYCLES * TROT_PERIOD
    DECEL_DURATION = 2.5   # seconds - ramp current_target_vx down to 0 over this window at the end of
                           # the run, instead of cutting stride amplitude from full speed to zero in one
                           # tick. See current_target_vx's own comment above for the full reasoning. Was
                           # 1.5: that cut the post-gait failure rate roughly in half (3 clean / 5 runs,
                           # vs. near-total failure before) but didn't eliminate it.
                           #
                           # Went looking at what the 2.5s value actually did on its first live run and
                           # found the real bug: this only ramps the gait's COMMANDED stride amplitude -
                           # it says nothing about the body's actual measured speed. That run's log
                           # showed speed_xy still bouncing between 0.04 and 0.23 m/s (and hitting 0.377
                           # on the very last trot tick) throughout "post-gait settle", with pitch/roll
                           # themselves looking fine right up to where the log went quiet mid-settle and
                           # the robot reportedly stumbled and tipped a moment later. So a purely timed
                           # ramp can still hand off to the static neutral stance below while the body is
                           # genuinely still sliding - no amount of retuning DECEL_DURATION alone fixes
                           # that, since it's not gated on the thing that actually matters.
    MAX_EXTRA_DECEL = 5.0  # seconds of extra grace, beyond total_duration, to wait for measured speed to
                           # actually drop before forcing the handoff anyway - a safety cap so noisy
                           # velocity readings (body_vel is a raw finite difference of pose) can't stall
                           # this forever. Was 3.0: one run used the full window without ever reaching
                           # SPEED_SETTLE_THRESHOLD (last reading 0.166 m/s, oscillating rather than
                           # decaying: 0.156 -> 0.251 -> 0.166) and tipped a few seconds later during
                           # "end of loop". Raised for more margin now that coast_to_stop also plants all
                           # 4 feet and stops feeding a stale saturated yaw correction during the wait -
                           # the actual momentum-arresting mechanism should matter more now than raw time.
    decel_start = max(0.0, total_duration - DECEL_DURATION)
    run_start = time.time()
    tick = 0
    while True:
        if check_abort():
            break
        elapsed = time.time() - run_start
        if elapsed >= decel_start:
            frac = min(1.0, (elapsed - decel_start) / DECEL_DURATION)
            current_target_vx[0] = DESIRED_VX * (1.0 - frac)
        if elapsed >= total_duration:
            if not coast_to_stop[0]:
                coast_to_stop[0] = True
                print(f"--- run duration elapsed (t={elapsed:.2f}s); planting all 4 feet flat, "
                      f"waiting for speed < {SPEED_SETTLE_THRESHOLD:.2f} m/s ---")
            speed_now = math.hypot(body_vel[0], body_vel[1])
            if speed_now <= SPEED_SETTLE_THRESHOLD or elapsed >= total_duration + MAX_EXTRA_DECEL:
                break
        if tick % 15 == 0:
            tag = " (decelerating)" if elapsed >= decel_start else ""
            if elapsed >= total_duration:
                tag += f" (waiting for speed<{SPEED_SETTLE_THRESHOLD:.2f}, now {math.hypot(body_vel[0], body_vel[1]):.3f})"
            diagnostic_line(f"trot t={elapsed:.2f}{tag}")
        tick += 1
        time.sleep(CONTROL_DT)
    current_target_vx[0] = 0.0   # in case the loop exited (abort) before the ramp finished on its own

    if not aborted[0]:
        gait_active[0] = False
        print("--- trot complete, returning to a stable stance ---")
        move_feet_manual({leg: (0.0, STANCE_FZ) for leg in legs}, duration=1.5, steps=75,
                          label="post-gait settle")

end_xyz = list(body_xyz)
dx = end_xyz[0] - start_xyz[0]
dy = end_xyz[1] - start_xyz[1]
dist = math.sqrt(dx*dx + dy*dy)
elapsed_total = N_CYCLES * TROT_PERIOD
dyaw_deg = math.degrees(math.atan2(math.sin(latest_yaw[0] - start_yaw), math.cos(latest_yaw[0] - start_yaw)))
print(f"net displacement: dx={dx:.3f} dy={dy:.3f}  total distance={dist:.3f} m  "
      f"net yaw turned={dyaw_deg:+.1f} deg")
print(f"avg speed achieved: {dist/elapsed_total:.4f} m/s (target was {DESIRED_VX:.3f} m/s)")
print(f"final body z: {end_xyz[2]:.3f}  (collapsed if well below ~0.35)")
# Was a fixed 20 iterations (2.0s) regardless of state. A run showed why that's not enough: it printed
# "sequence complete" while pitch was still climbing on its very last tick (+16.97deg) and speed_xy was
# still 0.23-0.41 m/s the entire tail - body_xyz.x climbed steadily the whole 2 seconds (0.57 -> 0.80m),
# meaning the robot was still actively sliding/toppling when the script simply gave up on the clock and
# declared the run done. Gated on measured speed now, same as the coast-to-stop wait above, with a
# generous cap so a genuinely-never-settling run still can't hang the script forever - check_abort() is
# still checked every iteration in the meantime, so a real tip past 25deg is caught either way.
END_WAIT_MIN = 2.0    # seconds - always sample at least this long, even if speed happens to read low
                       # immediately (avoids calling a fluke instant "settled").
END_WAIT_MAX = 6.0     # seconds - hard cap.
_end_wait_start = time.time()
while True:
    if check_abort():
        break
    diagnostic_line("end of loop")
    _elapsed_end = time.time() - _end_wait_start
    _speed_now = math.hypot(body_vel[0], body_vel[1])
    if (_elapsed_end >= END_WAIT_MIN and _speed_now <= SPEED_SETTLE_THRESHOLD) or _elapsed_end >= END_WAIT_MAX:
        break
    time.sleep(0.1)

print("sequence stopped early (safety abort)" if aborted[0] else "sequence complete")
running[0] = False