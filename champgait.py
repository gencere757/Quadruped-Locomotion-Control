import gz.transport13 as transport
from gz.msgs10.double_pb2 import Double
from gz.msgs10.imu_pb2 import IMU
from gz.msgs10.pose_v_pb2 import Pose_V
import math
import threading
import time
import sys

# tee all print() output to a log file as well as the console, so it can be read back
# without copy/pasting - always the same filename, overwritten each run
_log_file = open("run_log.txt", "w")

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

# v6.7 fix: print() (via the Tee above) goes to BOTH console and run_log.txt - fine for the small
# number of high-level phase markers (crouch, step sequence engaged, centering CoM before lifting X,
# stepping X, safety abort, final summary), but every per-tick diagnostic_line() call was ALSO going
# through it, which is what actually bloats the console - dozens of long telemetry lines per step.
# log_line() writes ONLY to the file, bypassing the Tee'd stdout, so the console stays down to just
# the step-level narration while run_log.txt keeps 100% of the detail for after-the-fact analysis.
def log_line(text):
    print(text, file=_log_file)
    _log_file.flush()

# ============================================================================
# v5 controller - GROUND-UP REWRITE, informed by the CHAMP quadruped framework
# (https://github.com/chvmp/champ, https://github.com/chvmp/libchamp).
#
# crawlgait.py (the previous controller) drove the gait as a sequential state
# machine: one leg at a time went through shift -> lift -> swing -> place ->
# push, and while that leg moved, the OTHER three legs' foot targets were
# completely frozen except during a brief separate "shift" phase. Reading
# CHAMP's actual leg_controller.h / trajectory_planner.h showed that isn't
# how a real continuous gait works: CHAMP runs a single shared sawtooth phase
# clock every tick, drives ALL FOUR legs off it simultaneously, and a leg
# that's in stance CONTINUOUSLY sweeps its foot target backward in body frame
# for the entire time it's planted (x = (step_length/2)*(1-2*stance_phase),
# never frozen) - modeling the fact that a real planted foot doesn't stay at
# a fixed body-frame offset while the body is translating over it. That's
# the architectural gap this rewrite closes.
#
# This is NOT a port of CHAMP's C++ - it's this project's own IK (leg_ik,
# 2-link planar solve, L1=L2=0.2), IMU/ground-truth telemetry, and
# gz-transport13 plumbing, restructured around CHAMP's actual control
# pattern:
#   - one shared phase clock (leg_phase_fracs / current_swing_leg)
#   - continuous, never-frozen stance sweep (foot_offset_for_leg, stance leg)
#   - smooth swing trajectory with matched (zero) endpoint velocity, using a
#     smoothstep horizontal profile + sine vertical lift instead of CHAMP's
#     12-point Bezier (same goal - a continuous, jerk-reduced arc - simpler
#     since we don't need CHAMP's general swing-height/step-length
#     re-parameterization machinery for a single fixed gait)
#
# Gait choice: this uses a CREEP gait (one leg swinging at a time, the other
# three always planted - SWING_DUTY=0.25 exactly divides the cycle among 4
# legs with zero overlap), not CHAMP's default trot (two diagonal legs
# swinging together). A trot only ever has a 2-point diagonal line of
# support and needs active dynamic balance to not fall over between steps -
# given this robot's whole debugging history has been repeated tipping
# failures, starting from creep (which keeps a full 3-foot support triangle
# at all times, like the old gait) isolates "does the continuous-stance-sweep
# fix work at all" from "can this robot's control loop handle a trot's much
# smaller support margin" as separate questions. Trot can be a later step
# once creep is proven stable.
#
# Known carry-over fix: the previous D-term (pitch-rate damping) used the raw
# IMU angular_velocity.y unfiltered, which run_log.txt showed swinging by
# tens to hundreds of deg/s between consecutive samples even while the body
# was nearly motionless - that noise was very likely why the last crawlgait
# run failed in a NEW, more violent way (theta repeatedly saturating and
# flipping sign) right after the D-term was added. This rewrite low-pass
# filters the rate signal (gyro or finite-difference fallback, whichever is
# active) before it ever reaches the correction term - see
# PITCH_RATE_LPF_ALPHA below. Both the raw and filtered values are still
# logged every tick so this can be checked against real data, not assumed.
#
# Everything NOT related to gait generation - the leg_ik/rotate math, the
# frame-correct world-space foot tracking, the support-polygon/ZMP/capture-
# point stability checks, the safety-abort watchdog - is carried over
# unchanged from crawlgait.py. Those were validated independently (ZMP/CP
# both stayed comfortably inside during the actual tipping failures, and the
# support-polygon math itself was never in question) and aren't part of what
# broke.
#
# ----------------------------------------------------------------------------
# v6 UPDATE: explicit CoM-centered stepping, replacing the v5 continuous phase
# clock, per direct request: after a leg lands, look at whichever leg lifts
# NEXT, build the support polygon of the other three, find its centroid, move
# the body's CoM there (measured, closed-loop - not a fixed-time/fixed-
# magnitude guess like the old shift_bias), and only once actually aligned,
# lift that next leg. This is the same idea crawlgait.py's old "shift" phase
# was reaching for, done properly: geometrically computed from real foot
# positions instead of a hand-picked SHIFT_MAG constant, and gated on
# measured convergence instead of a fixed ramp time.
#
# Centering the CoM over a support triangle is inherently a 2D (fore-aft AND
# left-right) problem, but the leg model up to this point only had a true
# translation DOF in the fore-aft/vertical (fx, fz) plane - sideways leaning
# was only ever an approximate side-effect of the ABAD angle used for roll
# correction, never an exact target. Getting genuine 2D centering means the
# IK needs a real third DOF, so leg_ik_3d() below extends the leg model to
# its actual 3-joint kinematics (ABAD abduction + HIP/KNEE 2-link, matching
# model.sdf's actual joint axes - confirmed by reading the joint definitions
# directly, not assumed): ABAD rotates about the body's fore-aft (X) axis,
# pivoting at each leg's ABAD mount point, with a fixed 0.1m arm out to where
# the HIP pivot sits at zero abduction; HIP/KNEE then do the existing 2-link
# planar solve within whatever plane that abduction angle has tilted them
# into. Given a target (fx, fy, fz) relative to that ABAD pivot, the abad
# angle is solved in closed form from the target's Y-Z projection (a "reach
# a circle of radius d, then solve 2-link within the tilted plane" style
# decomposition), and the existing, unchanged leg_ik() is reused verbatim for
# the HIP/KNEE part once that plane is known - so nothing about the already-
# tuned 2-link math changes, this only adds a new joint driving *which*
# tilted plane it operates in. The whole IK/FK derivation was round-trip
# verified numerically (random targets through IK then back through forward
# kinematics, machine-precision agreement) before being put in this file -
# see the session's scratch verification, not included here. What is NOT
# independently verified is the ABAD joint's physical positive-rotation sign
# in gz-sim matching this derivation's assumed convention - the existing
# roll correction only ever sent the SAME command to all four legs, which
# doesn't disambiguate a per-side mirroring the way this lateral-shift use
# does. If the next run shows the body shifting the opposite lateral
# direction from the intended target, that is a sign flip in leg_ik_3d, not
# evidence the geometric approach is wrong.
# ----------------------------------------------------------------------------
# v6.1 FIX: the first v6 run tipped over (pitch -25.6 deg safety abort) before even finishing one
# step. Root cause, precisely diagnosed from the log: centering the CoM over {FL,FR,BL} before
# lifting BR required a real ~0.084m lateral (Y) shift, but MAX_FY_SHIFT was a flat 0.03 - a number
# checked against only ONE (fx, STANCE_FZ) point, never against what this algorithm actually needs.
# Because fy physically couldn't reach the target, shift_com_to's combined-error convergence check
# (err = hypot(ex,ey) < SHIFT_TOLERANCE) could never pass, so the loop ran the full SHIFT_TIMEOUT
# every single step, and its per-tick ACCUMULATOR (body_shift[0] += step_x, not "set toward target")
# kept adding to fx that whole time even after fx had individually converged - eventually driving fx
# to its own FX_LIMIT, at which point the combined (fx=0.09, fy=0.09-clamped) offset pushed the CoM
# outside the support triangle entirely.
#
# Two independent problems, two independent fixes:
#   1. MAX_FY_SHIFT was never grounded in the leg's actual reach budget. Numerically verified: at
#      the old STANCE_FZ=-0.384, vertical depth alone eats ~96% of the leg's 0.4m max extension,
#      leaving as little as 0.0m of safe lateral margin at fx near FX_LIMIT. No flat constant fixes
#      this - it's a function of both fx AND depth. Replaced with max_safe_fy()/clamp_fy() below: a
#      closed-form (not searched) reach-aware bound derived straight from leg_ik_3d's own geometry
#      (see max_safe_fy's docstring), applied per-leg using that leg's ACTUAL fx that tick - so fy
#      degrades gracefully leg-by-leg instead of ever asking for a target that's physically unreachable.
#   2. STANCE_FZ itself dropped from -0.384 to -0.34 (see its own comment below) purely to buy back
#      enough of that reach budget for the shifts this algorithm needs to actually be achievable, not
#      just safely clamped-and-still-too-small.
#   3. shift_com_to's accumulator now updates (and can windup on) each axis independently, freezing
#      an axis the moment ITS OWN error is within tolerance rather than continuing to nudge it off a
#      combined-error check that a stuck other axis can hold open forever - so a converged axis can
#      no longer be dragged along by one that hasn't (or can't) converge. A stall detector also cuts
#      a step short if error stops improving well before SHIFT_TIMEOUT, instead of always waiting out
#      the full 3s on every step whose target isn't perfectly reachable.
# ----------------------------------------------------------------------------
# ============================================================================

L1 = 0.2
L2 = 0.2

def leg_ik(fx, fz, s):
    """fx,fz = desired foot position relative to hip, in body frame (fz negative = below hip).
       s = +1 for front legs, -1 for back legs. Returns (hip, knee) angles."""
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
    """Rotate a foot-target vector by theta (same convention as the leg's own swing rotation)."""
    return (fx*math.cos(theta) - fz*math.sin(theta),
            fx*math.sin(theta) + fz*math.cos(theta))

# --- 3-DOF leg IK (ABAD abduction + the existing HIP/KNEE 2-link) ------------------------------
# D_ABAD: the fixed distance, in the leg's own local Y direction at zero abduction, from the ABAD
# joint's own pivot to where the HIP joint's pivot sits - read directly from model.sdf's link
# poses (FL_thigh y=0.213 minus FL_ABAD_L y=0.113 = 0.1; confirmed the same magnitude on all four
# legs). This is a fixed mechanical arm length, not a tuning constant.
D_ABAD = 0.1

# OY: genuine left(+1)/right(-1) map - which direction (in true body-frame Y) is "outward" for
# each leg at zero abduction. NOT the same thing as LEG_SIDE above (that's front/back, for the
# 2-link plane's own geometry) - reuses the same left/right assignment as LEG_LR further below
# (added for the yaw correction), since both describe the same physical fact about the robot.
OY = {"FL": 1, "FR": -1, "BL": 1, "BR": -1}

def leg_ik_3d(fx, fy, fz, oy, s):
    """fx,fy,fz = desired foot position relative to the leg's ABAD pivot, in true body-frame axes.
       fy=0 reproduces exactly the old 2-link-only target (foot at the nominal zero-abduction
       offset); fy != 0 asks the ABAD joint to lean the whole leg sideways by whatever angle puts
       the foot at that additional Y offset. oy = OY[leg] (+1 left, -1 right), s = LEG_SIDE[leg]
       (+1 front, -1 back, same as leg_ik's own convention). Returns (abad, hip, knee).
       Derivation: the ABAD joint only rotates in the body's Y-Z plane (its axis is the body's own
       X axis - confirmed from model.sdf's joint definitions), so the HIP pivot is constrained to a
       circle of radius D_ABAD around the ABAD axis in that plane; solving where on that circle the
       remaining 2-link chain can reach the target's Y-Z projection gives the abad angle, and
       reduces the rest to the existing, unchanged 2-link leg_ik() in whatever plane that leaves."""
    dy = oy * D_ABAD + fy
    dz = fz
    r = math.hypot(dy, dz)
    r = max(r, D_ABAD + 1e-6)   # target's Y-Z distance from the ABAD axis can't be less than the
                                # arm length itself - clamp rather than crash on an unreachable ask
    c = max(-1.0, min(1.0, (oy * D_ABAD) / r))
    base = math.atan2(dz, dy)
    phi_a = base + math.acos(c)
    phi_b = base - math.acos(c)
    abad = phi_a if abs(phi_a) < abs(phi_b) else phi_b   # pick the smaller-abduction solution
    w = -dy*math.sin(abad) + dz*math.cos(abad)           # effective fz within the tilted 2-link plane
    hip, knee = leg_ik(fx, w, s)
    return abad, hip, knee

LEG_SIDE = {"FL": 1, "FR": 1, "BL": -1, "BR": -1}
legs = ["FL", "FR", "BL", "BR"]

node = transport.Node()
pubs = {}
for leg in legs:
    pubs[f"{leg}_ABAD"] = node.advertise(f"/model/my_quadruped/joint/{leg}_ABAD/cmd_pos", Double)
    pubs[f"{leg}_HIP"] = node.advertise(f"/model/my_quadruped/joint/{leg}_HIP/cmd_pos", Double)
    pubs[f"{leg}_KNEE"] = node.advertise(f"/model/my_quadruped/joint/{leg}_KNEE/cmd_pos", Double)

# --- IMU: orientation (pitch/roll) + pitch RATE for the D-term below -------------------------
latest_pitch = [0.0]
latest_roll = [0.0]
latest_yaw = [0.0]   # added purely to diagnose "walks forward vs. turns in place" (which pitch/
                     # roll alone can't distinguish from body_xyz drift) - now also feeds the
                     # yaw-hold correction further below

# pitch rate: both the raw source value (gyro if available, else finite-differenced pitch) AND
# a low-pass-filtered version. Only the FILTERED value (latest_pitch_rate) feeds the D-term -
# the raw gyro reading was shown (crawlgait.py's last run) to swing by tens to hundreds of deg/s
# between consecutive samples even during near-motionless phases, which fed a chaotic, sign-
# flipping correction. latest_pitch_rate_raw is kept only so run_log.txt can show the filter is
# actually smoothing something, not assumed to be working.
PITCH_RATE_LPF_ALPHA = 0.2   # exponential low-pass filter coefficient (0-1, lower = more smoothing)
latest_pitch_rate = [0.0]
latest_pitch_rate_raw = [0.0]
_pitch_rate_source = [None]     # "gyro" or "fd"
_prev_pitch_for_rate = [None]
_prev_pitch_rate_time = [None]
_dumped_imu_fields = [False]

# v6.6 fix: yaw rate, same pattern as pitch rate above (gyro if available, else finite-difference,
# low-pass filtered). Added because the last run's yaw wasn't drifting steadily in one direction - it
# was OSCILLATING hard (+/-3 to 7 degrees between consecutive ~0.24s log samples, both growing and
# shrinking at different points), which a pure position-error (P-only) correction can't damp and can
# even fight out of phase with. The yaw-hold correction below gets a genuine D-term from this, the
# same fix pitch already needed (see PITCH_RATE_DAMPING's own history above) for the identical reason.
YAW_RATE_LPF_ALPHA = 0.2
latest_yaw_rate = [0.0]
latest_yaw_rate_raw = [0.0]
_yaw_rate_source = [None]
_prev_yaw_for_rate = [None]
_prev_yaw_rate_time = [None]

def imu_callback(msg):
    if not _dumped_imu_fields[0]:
        _dumped_imu_fields[0] = True
        try:
            log_line(f"DEBUG: IMU message fields: {[f.name for f in msg.DESCRIPTOR.fields]}")
        except Exception as e:
            log_line(f"DEBUG: could not introspect IMU message fields: {e}")

    q = msg.orientation
    sinp = max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))
    pitch = math.asin(sinp)
    latest_pitch[0] = pitch
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
                raw_rate = (pitch - _prev_pitch_for_rate[0]) / dt
        _prev_pitch_for_rate[0] = pitch
        _prev_pitch_rate_time[0] = now

    if raw_rate is not None:
        latest_pitch_rate_raw[0] = raw_rate
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
        latest_yaw_rate_raw[0] = yaw_raw_rate
        latest_yaw_rate[0] = (YAW_RATE_LPF_ALPHA * yaw_raw_rate
                               + (1.0 - YAW_RATE_LPF_ALPHA) * latest_yaw_rate[0])

node.subscribe(IMU, "/model/my_quadruped/imu", imu_callback)

# --- ground-truth body + per-foot world-frame telemetry (unchanged from crawlgait.py) --------
body_xyz = [None, None, None]
body_vel = [0.0, 0.0, 0.0]
body_accel = [0.0, 0.0, 0.0]
_prev_body_xyz = [None, None, None]
_prev_body_vel = [None, None, None]
_prev_pose_time = [None]

link_xyz = {}
_dumped_pose_names = [False]

# FRAME NOTE: Pose_V gives the MODEL's pose ("my_quadruped") relative to the world, but gives
# each LINK's pose ("FR_shank" etc.) relative to the model (local/body frame) - so per-foot
# world position needs each link offset rotated into world frame before adding to body_xyz.
def _rotate_body_to_world(dx, dy, dz, pitch, roll):
    """Rotates a vector from the model's local/body frame into world frame (yaw=0, no yaw
       sensor and this gait never yaws)."""
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    wx = cp*dx + sp*sr*dy + sp*cr*dz
    wy = cr*dy - sr*dz
    wz = -sp*dx + cp*sr*dy + cp*cr*dz
    return wx, wy, wz

foot_world_xyz = {}
foot_world_vel = {leg: [0.0, 0.0, 0.0] for leg in legs}
_prev_foot_world_xyz = {}
_prev_foot_world_time = {}

def _update_foot_world_positions(now):
    if body_xyz[0] is None:
        return
    pitch, roll = latest_pitch[0], latest_roll[0]
    for leg_name, (lx, ly, lz) in link_xyz.items():
        wx_off, wy_off, wz_off = _rotate_body_to_world(lx, ly, lz, pitch, roll)
        wx, wy, wz = body_xyz[0] + wx_off, body_xyz[1] + wy_off, body_xyz[2] + wz_off
        prev = _prev_foot_world_xyz.get(leg_name)
        prev_t = _prev_foot_world_time.get(leg_name)
        if prev is not None and prev_t is not None:
            dt = now - prev_t
            if dt > 1e-4:
                foot_world_vel[leg_name][0] = (wx - prev[0]) / dt
                foot_world_vel[leg_name][1] = (wy - prev[1]) / dt
                foot_world_vel[leg_name][2] = (wz - prev[2]) / dt
        foot_world_xyz[leg_name] = (wx, wy, wz)
        _prev_foot_world_xyz[leg_name] = (wx, wy, wz)
        _prev_foot_world_time[leg_name] = now

# --- commanded-vs-actual foot tracking, added to settle whether "clawing"/high stance-foot
# slip is real ground slip or the leg's own joints failing to hold their commanded pose under
# load (compliance - p_gain=50/d_gain=2 is fairly soft, and was tuned for the old sequential
# gait, not this continuously-loaded one). CMD_HIP_OFFSET is each leg's nominal hip-attachment
# offset from the body origin at ABAD=0 (from the SDF's thigh link poses - the point leg_ik's
# (fx,fz) target is defined relative to). Rotating (x0+fx_c, y0, fz_c) into world the same way
# _update_foot_world_positions() rotates the ACTUAL link offsets gives an apples-to-apples
# "commanded foot world position" to diff against the real one. This ignores ABAD's own small
# rotation of the leg plane (roll correction stays under ~8.6 degrees) - a known approximation,
# not exact forward kinematics, but plenty to see whether actual tracks commanded at all.
CMD_HIP_OFFSET = {"FL": (0.15, 0.213), "FR": (0.15, -0.213), "BL": (-0.15, 0.213), "BR": (-0.15, -0.213)}

cmd_foot_world_xyz = {}
cmd_foot_world_vel = {leg: [0.0, 0.0, 0.0] for leg in legs}
_prev_cmd_foot_world_xyz = {}
_prev_cmd_foot_world_time = {}

def _update_commanded_foot_world(leg, fx_c, fz_c, now):
    if body_xyz[0] is None:
        return
    x0, y0 = CMD_HIP_OFFSET[leg]
    pitch, roll = latest_pitch[0], latest_roll[0]
    wx_off, wy_off, wz_off = _rotate_body_to_world(x0 + fx_c, y0, fz_c, pitch, roll)
    wx, wy, wz = body_xyz[0] + wx_off, body_xyz[1] + wy_off, body_xyz[2] + wz_off
    prev = _prev_cmd_foot_world_xyz.get(leg)
    prev_t = _prev_cmd_foot_world_time.get(leg)
    if prev is not None and prev_t is not None:
        dt = now - prev_t
        if dt > 1e-4:
            cmd_foot_world_vel[leg][0] = (wx - prev[0]) / dt
            cmd_foot_world_vel[leg][1] = (wy - prev[1]) / dt
            cmd_foot_world_vel[leg][2] = (wz - prev[2]) / dt
    cmd_foot_world_xyz[leg] = (wx, wy, wz)
    _prev_cmd_foot_world_xyz[leg] = (wx, wy, wz)
    _prev_cmd_foot_world_time[leg] = now

def foot_tracking_error(active_leg):
    """NOTE: pos_err here is NOT a trustworthy absolute-tracking number - cmd_foot_world_xyz is
       built at the true foot-tip (per leg_ik's convention) while foot_world_xyz/link_xyz has
       always sampled the *_shank LINK's own origin (effectively the knee), which sits ~L2=0.2m
       above the true tip. That fixed offset shows up as a near-constant ~0.19-0.22m "error" on
       every leg at every tick, even standing still, and swamps any real tracking signal. Keep
       vel_err (differentiates out most of a constant offset) but do not read pos_err as real.
       Use all_foot_world_z() below for an unambiguous check of whether a foot is actually
       lifting off the ground during its swing."""
    out = {}
    for l in legs:
        if l == active_leg:
            continue
        if l in cmd_foot_world_xyz and l in foot_world_xyz:
            cx, cy, cz = cmd_foot_world_xyz[l]
            ax, ay, az = foot_world_xyz[l]
            pos_err = math.sqrt((cx - ax) ** 2 + (cy - ay) ** 2 + (cz - az) ** 2)
            cvx, cvy, _cvz = cmd_foot_world_vel[l]
            avx, avy, _avz = foot_world_vel[l]
            vel_err = math.hypot(cvx - avx, cvy - avy)
            out[l] = (pos_err, vel_err)
    return out

def all_foot_world_z():
    """{leg: z} actual measured world-frame height for every leg's shank-origin sample point
       (foot_world_xyz). Not the true foot tip (see note above) but a FIXED, consistent offset
       from it for a given leg orientation, so relative motion (does it rise during swing?) is
       trustworthy even though the absolute value isn't the ground-contact height."""
    return {l: foot_world_xyz[l][2] for l in legs if l in foot_world_xyz}

def pose_callback(msg):
    if not _dumped_pose_names[0]:
        _dumped_pose_names[0] = True
        log_line(f"DEBUG: pose entity names seen: {sorted(set(p.name for p in msg.pose))}")
    for p in msg.pose:
        if p.name == "my_quadruped":
            now = time.time()
            if _prev_pose_time[0] is not None:
                dt = now - _prev_pose_time[0]
                if dt > 1e-4:
                    new_vx = (p.position.x - _prev_body_xyz[0]) / dt
                    new_vy = (p.position.y - _prev_body_xyz[1]) / dt
                    new_vz = (p.position.z - _prev_body_xyz[2]) / dt
                    if _prev_body_vel[0] is not None:
                        body_accel[0] = (new_vx - _prev_body_vel[0]) / dt
                        body_accel[1] = (new_vy - _prev_body_vel[1]) / dt
                        body_accel[2] = (new_vz - _prev_body_vel[2]) / dt
                    _prev_body_vel[0] = new_vx
                    _prev_body_vel[1] = new_vy
                    _prev_body_vel[2] = new_vz
                    body_vel[0] = new_vx
                    body_vel[1] = new_vy
                    body_vel[2] = new_vz
            _prev_body_xyz[0] = p.position.x
            _prev_body_xyz[1] = p.position.y
            _prev_body_xyz[2] = p.position.z
            _prev_pose_time[0] = now

            body_xyz[0] = p.position.x
            body_xyz[1] = p.position.y
            body_xyz[2] = p.position.z
        else:
            for leg_name in legs:
                if p.name.endswith(f"{leg_name}_shank"):
                    link_xyz[leg_name] = (p.position.x, p.position.y, p.position.z)
                    break
    _update_foot_world_positions(time.time())

node.subscribe(Pose_V, "/world/empty/pose/info", pose_callback)

# --- stability checks against the support polygon (unchanged math from crawlgait.py) ---------
def _sign2d(p1, p2, p3):
    return (p1[0]-p3[0])*(p2[1]-p3[1]) - (p2[0]-p3[0])*(p1[1]-p3[1])

def _point_in_triangle(pt, v1, v2, v3):
    d1 = _sign2d(pt, v1, v2)
    d2 = _sign2d(pt, v2, v3)
    d3 = _sign2d(pt, v3, v1)
    has_neg = (d1 < 0) or (d2 < 0) or (d3 < 0)
    has_pos = (d1 > 0) or (d2 > 0) or (d3 > 0)
    return not (has_neg and has_pos)

def _polygon_check(point_xy, active_leg):
    """(inside, margin) for point_xy against the triangle of the three legs other than
       active_leg. Returns (None, None) if any needed foot position hasn't been seen yet."""
    support_legs = [l for l in legs if l != active_leg]
    if any(l not in foot_world_xyz for l in support_legs):
        return None, None
    pts = [(foot_world_xyz[l][0], foot_world_xyz[l][1]) for l in support_legs]
    inside = _point_in_triangle(point_xy, pts[0], pts[1], pts[2])
    def edge_dist(a, b):
        ex, ey = b[0]-a[0], b[1]-a[1]
        edge_len = math.hypot(ex, ey)
        if edge_len < 1e-6:
            return 0.0
        cross = (point_xy[0]-a[0])*ey - (point_xy[1]-a[1])*ex
        return cross / edge_len
    margins = [edge_dist(pts[0], pts[1]), edge_dist(pts[1], pts[2]), edge_dist(pts[2], pts[0])]
    return inside, min(abs(m) for m in margins)

def support_status(active_leg):
    """Static CoM-vs-support-triangle check. Weak (quasi-static only) - kept for comparison
       against ZMP/capture-point, not as the primary stability signal."""
    if body_xyz[0] is None:
        return None, None
    return _polygon_check((body_xyz[0], body_xyz[1]), active_leg)

G = 9.8

def compute_zmp():
    """x_zmp = x_com - (xdd/(zdd+g))*z_com - dropping the angular-momentum-rate term (first-order
       approximation; body mass dominates leg mass ~15:1 in this robot)."""
    if body_xyz[0] is None:
        return None
    z_com = body_xyz[2]
    xdd, ydd, zdd = body_accel
    denom = zdd + G
    if abs(denom) < 1.0:
        denom = G
    return (body_xyz[0] - (xdd/denom)*z_com, body_xyz[1] - (ydd/denom)*z_com)

def zmp_status(active_leg):
    zmp = compute_zmp()
    if zmp is None:
        return None, None, None
    inside, margin = _polygon_check(zmp, active_leg)
    return inside, margin, zmp

CAPTURE_HEIGHT = 0.40
CAPTURE_GAIN = math.sqrt(CAPTURE_HEIGHT / 9.8)   # ~0.202

def compute_capture_point():
    """x_cp = x_com + vx*sqrt(z0/g) (Pratt et al.) - where the CoM would land if every leg's
       velocity went to zero right now."""
    if body_xyz[0] is None:
        return None
    return (body_xyz[0] + body_vel[0]*CAPTURE_GAIN, body_xyz[1] + body_vel[1]*CAPTURE_GAIN)

def capture_point_status(active_leg):
    cp = compute_capture_point()
    if cp is None:
        return None, None, None
    inside, margin = _polygon_check(cp, active_leg)
    return inside, margin, cp

def stance_foot_velocities(active_leg):
    """{leg: (vx, vy)} signed world-frame velocity for each currently-planted (non-active) leg -
       a genuinely planted foot should read ~0; a sustained nonzero value means that leg's
       position-servo joints are losing the argument with the ground under load."""
    support_legs = [l for l in legs if l != active_leg]
    return {l: tuple(foot_world_vel.get(l, (0.0, 0.0, 0.0))[:2]) for l in support_legs}

def all_foot_fx():
    """{leg: current body-frame fx target} for all four legs."""
    return {l: foot_target[l][0] for l in legs}

# --- gait constants ----------------------------------------------------------------------------
# STANCE_FZ: was -0.384 ("proven settled-crouch depth from crawlgait.py") through v6. Raised
# (shallower) to -0.34 as part of the v6.1 fix - numerically, -0.384 alone consumed ~96% of the
# leg's 0.4m max extension just going down, leaving essentially zero safe lateral (fy) margin at the
# fx values a real step uses, which is what let the CoM-centering shift's ~0.084m lateral requirement
# collide with the reach limit and trigger a safety abort. -0.34 frees up to ~0.10-0.12m of lateral
# margin (see max_safe_fy below) even combined with a near-limit fx - comfortably covers what
# dropping one corner leg from the stance rectangle geometrically requires. Trade-off: less of the
# old depth's settled-crouch stability margin, un-verified until the next real run - if the robot
# looks less planted/more wobbly at rest than before, that's the lever to look at first.
STANCE_FZ = -0.34

# front legs get a shorter step and lower swing lift than back legs - carried over from
# crawlgait.py's tuning, which found front-leg loading (place->push) was specifically the moment
# pitch ran away; smaller front-leg excursions give that moment less leverage.
#
# STEP_LENGTH_BACK cut 0.10 -> 0.08: the first successful (non-falling) full run still showed the
# tilt correction swinging ~13 degrees and flipping sign within half a second on nearly every
# step, worst during the back legs' swings (BR pitch dips of -6 to -10 deg) - a real disturbance
# being fought hard, not just a jerky controller. STEP_LENGTH_BACK was the largest single number
# in this whole design and the one most repeatedly implicated, so it's the direct lever for
# "smaller disturbance per step" rather than tuning the correction gains that are load-bearing for
# not falling over.
STEP_LENGTH_FRONT = 0.06
STEP_LENGTH_BACK = 0.08

# SWING_HEIGHT_FRONT history: started at 0.044 (front legs dragged, never cleared the ground -
# confirmed by direct observation). Raised all the way to 0.08 (full match with the back legs) as
# the first fix - that run then failed MUCH worse than any prior one: not a single tipping event
# but a growing pitch oscillation (+/-2 deg -> +/-25 deg over ~2s, safety abort) starting right
# after FL's swing and continuing to build through the following leg=None gap and FR's swing. The
# only change between the last stable (non-falling) run and this one was this constant, so the
# extra front-leg swing height is the prime suspect: a taller front-foot arc is a bigger inertial
# disturbance at front-leg liftoff/landing than before. Splitting the difference (0.06) instead of
# matching the back legs exactly, so the next run can separate "does the foot clear the ground"
# (checkable directly via foot_z) from "is the disturbance still too big to handle" - though the
# v6 CoM-centering rewrite below (explicit shift-before-lift, closed-loop) targets this same
# liftoff-disturbance problem far more directly than the old fixed-magnitude bias ever did.
SWING_HEIGHT_FRONT = 0.06
SWING_HEIGHT_BACK = 0.08

FX_LIMIT = 0.09   # hard clamp - both step lengths above stay well inside this with margin

def clamp_fx(v):
    return max(-FX_LIMIT, min(FX_LIMIT, v))

# creep order: one leg lifts at a time, in this fixed sequence, repeated N_CYCLES times. There is
# no shared "phase clock" or fixed period anymore (see v6 header note) - each step now takes as
# long as it actually takes: a variable-duration CoM-centering shift, then a fixed-duration swing.
GAIT_ORDER = ["BR", "FL", "FR", "BL"]
N_CYCLES = 3

def _smoothstep(x):
    x = max(0.0, min(1.0, x))
    return x * x * (3 - 2 * x)

def swing_profile(leg, frac):
    """Same swing arc shape used since the very first version of this file (smoothstep fore-aft +
       sine vertical lift, zero-velocity endpoints) - now evaluated as a standalone, fixed-duration
       motion for one leg via do_swing() below, rather than read off a shared continuous phase
       clock. frac goes 0 (liftoff, at -step/2) to 1 (touchdown, at +step/2)."""
    is_front = leg in ("FL", "FR")
    step = STEP_LENGTH_FRONT if is_front else STEP_LENGTH_BACK
    height = SWING_HEIGHT_FRONT if is_front else SWING_HEIGHT_BACK
    s = _smoothstep(frac)
    fx = -step / 2.0 + step * s
    fz = STANCE_FZ + height * math.sin(math.pi * frac)
    return clamp_fx(fx), fz

SWING_DURATION = 0.9   # seconds - unchanged pacing from the old SWING_DUTY(0.15) * GAIT_PERIOD(6.0)
SWING_STEPS = 45       # ~50Hz worth of profile updates over SWING_DURATION

# --- CoM-centering shift (replaces the old fixed-magnitude/fixed-time shift_bias) ---------------
# body_shift = a single shared (fx, fy) offset applied identically to ALL FOUR legs' targets every
# tick (control_loop, below) - since every currently-planted foot is fixed to the ground, adding
# the SAME offset to every leg's body-frame target is exactly what makes the BODY itself translate
# under them (the same trick the old continuous stance-sweep used for fx alone; fy is new, made
# possible by leg_ik_3d). shift_com_to() closes the loop on this: it nudges body_shift a little
# every control tick and watches the REAL measured body_xyz, instead of assuming a fixed magnitude
# and a fixed ramp time gets there (the old shift_bias's actual failure mode, repeatedly - see the
# SHIFT_MAG history that used to live here).
body_shift = [0.0, 0.0]

# REACH_BUDGET: the leg's real max 2-link extension is L1+L2=0.4m exactly (a physical singularity,
# not a soft limit) - budget stays 0.01m under that so leg_ik/leg_ik_3d's own last-resort reach
# clamps (max(r, D_ABAD+eps) etc.) never actually have to engage in normal operation.
REACH_BUDGET = 0.39

def max_safe_fy(fx_c, fz_c, budget=REACH_BUDGET):
    """Closed-form (not searched) reach-aware bound on |fy|, replacing the old flat MAX_FY_SHIFT
       (v6.1 fix - see the file-header note above). Derivation: leg_ik_3d's own geometry gives, for
       either abduction side, total 2-link+abad reach = hypot(fx_c, w) where w = sqrt(r^2 - D_ABAD^2)
       and r = hypot(oy*D_ABAD + fy, fz_c). Requiring that reach <= budget for BOTH oy=+1 and oy=-1
       simultaneously (fy is one shared value applied to every leg, left and right alike) and solving
       for the largest common |fy| gives, after the D_ABAD terms cancel between the two sides:
           dy_max = sqrt((budget^2 - fx_c^2 + D_ABAD^2) - fz_c^2)
           max |fy| = max(0, dy_max - D_ABAD)
       Verified against the session's earlier binary-search reach table to 4 decimal places at every
       sampled (fx, STANCE_FZ) point. Depends on THIS TICK's actual fx_c/fz_c (after body_shift, yaw
       correction, and the theta rotation all apply) rather than a value checked at one fixed point,
       so it stays correct as those other corrections move the leg's real operating point around."""
    r_max_sq = budget * budget - fx_c * fx_c + D_ABAD * D_ABAD
    if r_max_sq <= 0.0:
        return 0.0
    dy_max_sq = r_max_sq - fz_c * fz_c
    if dy_max_sq <= 0.0:
        return 0.0
    return max(0.0, math.sqrt(dy_max_sq) - D_ABAD)

SHIFT_KP = 0.125      # proportional gain: body_shift moves toward closing the error each tick.
                      # HALVED (was 0.5) in v6.3 for the same reason it's HALVED AGAIN here (v6.8):
                      # the sign-corrected first run converged in position beautifully (err 0.12 ->
                      # 0.024) but built real momentum doing it (speed_xy peaked at 0.337 m/s, more
                      # than double the 0.15 m/s the old SHIFT_STEP_MAX ramp rate should have produced
                      # on its own - a sign the physical response was underdamped/overshooting, not
                      # just following the setpoint), and that momentum carried straight through the
                      # support triangle edge and tipped the robot. Halving it (v6.3) fixed THAT
                      # specific run, but a later run (cycle 2's first re-centering, a bigger combined
                      # fx+fy move that drove fx into its own FX_LIMIT clamp while fy was also sweeping
                      # a wide range) reproduced the identical signature: position converging fine
                      # while the gyro's raw pitch rate flipped from -104deg/s to +110deg/s in one
                      # ~0.24s sample - a real physical event (almost certainly a stance foot's grip
                      # briefly breaking under the combined load), not a smooth control response. Same
                      # lever, applied again: less peak acceleration means less chance of exceeding a
                      # stance foot's friction limit, this time for the harder, bigger combined moves
                      # v6.3's pace still weren't gentle enough for.
SHIFT_STEP_MAX = 0.00075   # meters/tick cap on how fast body_shift itself can move (~0.0375 m/s
                           # equivalent, HALVED again in v6.8 - same rationale as SHIFT_KP above)
SHIFT_TOLERANCE = 0.006  # meters - "close enough" to call the CoM aligned with the support centroid
SHIFT_VEL_TOLERANCE = 0.02   # m/s - v6.3 fix: being AT the target position isn't enough to call a
                             # shift done, the body also has to have actually stopped moving there.
                             # Previously shift_com_to only ever checked position error, so it happily
                             # declared victory while the body was still carrying real momentum -
                             # exactly what carried it past the support polygon edge last run. Combined
                             # with position tolerance, the loop now genuinely waits for the body to
                             # settle before considering a step ready.
SHIFT_TIMEOUT = 6.0      # seconds - DOUBLED in v6.8: SHIFT_KP/SHIFT_STEP_MAX were just halved again,
                         # so the body now closes any given error at roughly half the previous rate.
                         # At the old 3.0s bound, a typical 0.09-0.15m combined shift would routinely
                         # hit the timeout before actually converging - i.e. the safety bound would
                         # become the normal exit path instead of a rare fallback, silently undoing
                         # the benefit of slowing down. Doubling it keeps the same real safety margin
                         # (still a bound, not unbounded) while giving the gentler pace room to work.
SHIFT_STALL_TICKS = 80   # ~1.6s at CONTROL_DT=0.02 - also DOUBLED in v6.8, same reasoning as
                         # SHIFT_TIMEOUT above: with half the step size, the settle metric improves
                         # roughly half as much per tick, so the old 40-tick (~0.8s) window would
                         # start flagging normal, slower-but-real progress as a stall. If the metric
                         # (position error AND velocity, see SHIFT_VEL_WEIGHT) hasn't meaningfully
                         # improved in this many ticks, waiting longer won't help; stop burning the
                         # rest of SHIFT_TIMEOUT
SHIFT_STALL_EPS = 0.001  # improvement smaller than this doesn't reset the stall counter
SHIFT_VEL_WEIGHT = 0.1   # v6.3 fix: folds speed_xy into the SAME settle metric the stall detector
                         # watches (metric = pos_err + SHIFT_VEL_WEIGHT*speed_xy), so decaying momentum
                         # after position has already converged still counts as "improving" instead of
                         # being mistaken for a stall and cut short before the body actually stops

# v6.11: TARGETED fy-rate throttle, not another blanket gain cut. The cycle-2 fall (about to lift FR)
# showed FL slip hard - world-frame foot velocity spiking to -0.37 m/s, 3-10x the normal background
# level - exactly while body_shift[1] (fy) swept through a wide range. What's structurally different
# about cycle 2+ vs cycle 1: EVERY leg already carries its post-swing fx offset (+step/2, permanent
# until that leg's next swing) instead of cycle 1's mix of some legs still at fx=0.
#
# First cut of this throttle keyed on reach headroom (max_safe_fy minus |fy|), reasoning that a
# pre-loaded fx eats into the reach budget. Checked that against the actual failing run's own numbers
# before shipping it: at every point in that shift, every leg still had 3-4cm of reach headroom to
# spare (fy never got past ~0.07, max_safe_fy at those fx values was ~0.10-0.11) - nowhere near
# REACH_BUDGET's limit. So reach isn't what ran out; that version wouldn't have actually engaged for
# the incident it was built to catch. Rebuilt on the thing there IS direct evidence for instead: it's
# the pre-loaded fx state itself (cycle 2+ only) that's the difference, not how much geometric reach
# headroom happens to be left at any given fy - so key the throttle directly on that, not a reach proxy.
FY_LOADED_FX_THRESHOLD = 0.015   # meters - any leg's foot_target fx at/above this means that leg is
                                  # past its first swing and permanently carrying +step/2 (0.03 front /
                                  # 0.04 back) rather than cycle 1's still-neutral 0 - the one concrete,
                                  # confirmed structural difference between cycle 1's shifts (survived)
                                  # and the cycle 2 shift that didn't.
FY_RATE_LOADED_SCALE = 0.35      # fy's step cap once any leg is "loaded" per above: down to ~1/3 of
                                  # SHIFT_STEP_MAX for fy only - fx and cycle-1 shifts are untouched.

def fy_rate_scale():
    """1.0 (full SHIFT_STEP_MAX) unless some leg already carries a post-swing fx offset, in which case
       fy - and only fy - is throttled to FY_RATE_LOADED_SCALE. See the v6.11 note above for why this
       replaced an earlier reach-margin-based version that the actual failing run's numbers ruled out."""
    if any(abs(foot_target[leg][0]) >= FY_LOADED_FX_THRESHOLD for leg in legs):
        return FY_RATE_LOADED_SCALE
    return 1.0

def clamp_fy(v, fx_c=0.0, fz_c=None):
    """fx_c/fz_c default to a body-shift-only, untilted estimate (fz_c=STANCE_FZ) for callers like
       shift_com_to that are clamping the shared accumulator itself rather than one leg's exact,
       post-rotation target - control_loop instead passes this tick's real per-leg fx_c/fz_c."""
    if fz_c is None:
        fz_c = STANCE_FZ
    bound = max_safe_fy(fx_c, fz_c)
    return max(-bound, min(bound, v))

def support_polygon_centroid(support_legs):
    """Plain geometric centroid (average) of the given legs' current world-frame foot positions -
       NOT the robot's true center of mass, same approximation support_status()/zmp_status() above
       already make (body_xyz as the CoM proxy); this is the target we move that proxy onto."""
    xs = [foot_world_xyz[l][0] for l in support_legs]
    ys = [foot_world_xyz[l][1] for l in support_legs]
    return (sum(xs) / len(xs), sum(ys) / len(ys))

# v6.5 FIX, REVERTED: briefly added an artificial FORWARD_BIAS here (nudging every shift's target
# forward beyond the pure centroid), reasoning that the CoM-centering rule as specified has no notion
# of "forward" in it. That was wrong, and worth recording why. When a leg swings, its NEW foot
# position = wherever the body was at that moment + hip_offset + step/2 (the swing's own forward
# endpoint) - it's a permanent gain (the foot stays planted there until that leg's next turn), not a
# reset. Simulating the exact rule (idealized: perfect convergence, no falling, same GAIT_ORDER) over
# many cycles shows a genuine, consistent net-forward drift with NO bias needed - and, checked against
# the real log, that idealized model reproduces the SAME early-cycle pattern the real run showed
# (forward on dropping a back leg, a dip on dropping a front leg, summing to slightly NEGATIVE after 3
# of 4 steps) before the 4th step in a cycle (dropping the last back leg) produces the single largest
# forward gain of the whole cycle. The real robot has never yet survived to that 4th step - it always
# fell partway through the first cycle - so "no net progress in a partial cycle" was mistaken for "no
# directional drive in the design," which the math says isn't true. The actual problem is survivability
# through a full cycle, not a missing forward signal - so this bias is removed rather than tuned.

def shift_com_to(target_xy, about_to_lift=None):
    """Closed-loop: nudge body_shift toward target_xy every tick, checking the REAL body_xyz each
       time, until within SHIFT_TOLERANCE (or SHIFT_TIMEOUT expires). This runs in the MAIN thread
       (like move_feet_manual) - control_loop just keeps applying whatever body_shift currently is
       to every leg, every tick, same pattern as everything else in this file.

       v6.2 SIGN FIX: body_shift is a body-frame foot-target offset applied identically to every
       PLANTED leg. A planted foot is fixed in the world, so asking for a LARGER offset in some
       direction doesn't move the foot - it moves the BODY the OPPOSITE way to keep that foot at the
       new relative position. This is exactly what the old v5 continuous stance-sweep already relied
       on (its comment: fx = (step/2)*(1-2*phase), decreasing over stance to drive the body forward)
       - and it means the sign here must be the error's NEGATIVE, not the error itself. The first cut
       of this function got that backwards (step = +KP*error), which commanded body_shift to grow in
       exactly the direction that pushes the body away from target_xy - confirmed against a real run
       where body_xyz walked monotonically further from target_xy (error 0.12 -> 0.31) on both axes
       until fx hit FX_LIMIT. Fixed below: step = -KP*error on both axes (fy is a uniform body-frame
       Y offset across all four legs' targets exactly the same way fx is - see max_safe_fy's own
       docstring - so the identical argument applies to both)."""
    start_t = time.time()
    tick = 0
    best_metric = None
    best_tick = 0
    while True:
        if check_abort():
            return
        if body_xyz[0] is None:
            time.sleep(CONTROL_DT)
            continue
        ex = target_xy[0] - body_xyz[0]
        ey = target_xy[1] - body_xyz[1]
        err = math.hypot(ex, ey)
        speed_xy = math.hypot(body_vel[0], body_vel[1])
        metric = err + SHIFT_VEL_WEIGHT * speed_xy   # v6.3 fix - see SHIFT_VEL_WEIGHT's own comment
        if tick % 12 == 0:   # ~4x/sec at CONTROL_DT=0.02
            diagnostic_line(f"shift err={err:.4f} speed={speed_xy:.3f}", active_leg=about_to_lift)
        # v6.3 fix: position converged is no longer enough on its own - also require the body to have
        # actually stopped (speed_xy below SHIFT_VEL_TOLERANCE). See SHIFT_VEL_TOLERANCE's comment:
        # the previous run declared victory on position alone while still carrying 0.337 m/s, and that
        # momentum is what carried it through the support triangle edge a moment later.
        if abs(ex) < SHIFT_TOLERANCE and abs(ey) < SHIFT_TOLERANCE and speed_xy < SHIFT_VEL_TOLERANCE:
            break
        elapsed = time.time() - start_t
        if elapsed > SHIFT_TIMEOUT:
            log_line(f"  [shift] timed out after {SHIFT_TIMEOUT}s with err={err:.4f} speed={speed_xy:.3f} "
                     f"- lifting anyway")
            break
        # v6.1 fix (now tracking the combined position+velocity metric, v6.3): stall detection - if a
        # target axis is clamped/unreachable, or the body just won't settle, the metric stops improving
        # long before SHIFT_TIMEOUT; don't burn the rest of the timeout waiting for something that
        # isn't getting any closer. Using `metric` (not just `err`) here means decaying velocity AFTER
        # position has already converged still counts as progress, so this can't cut the settle wait
        # short just because position itself stopped changing.
        if best_metric is None or metric < best_metric - SHIFT_STALL_EPS:
            best_metric = metric
            best_tick = tick
        elif tick - best_tick > SHIFT_STALL_TICKS:
            log_line(f"  [shift] stalled at err={err:.4f} speed={speed_xy:.3f} (no improvement in "
                     f"{SHIFT_STALL_TICKS} ticks) - lifting anyway")
            break
        # v6.1 fix: update (and thus only accumulate on) an axis that HASN'T yet converged. Previously
        # both axes updated every tick off the combined error, so a permanently-stuck ey (unreachable
        # fy target) kept the loop alive long enough for ex's own per-tick accumulator to wind fx all
        # the way to its limit even after ex itself had converged - this freezes a converged axis
        # instead of letting the other one drag it along.
        if abs(ex) >= SHIFT_TOLERANCE:
            # v6.2 fix: -SHIFT_KP*ex, not +SHIFT_KP*ex - see the sign-fix docstring above.
            step_x = max(-SHIFT_STEP_MAX, min(SHIFT_STEP_MAX, -SHIFT_KP * ex))
            body_shift[0] = clamp_fx(body_shift[0] + step_x)
        if abs(ey) >= SHIFT_TOLERANCE:
            # v6.11: fy's step cap is throttled by fy_rate_scale() whenever some leg is already
            # carrying a post-swing fx offset (cycle 2+) - see FY_LOADED_FX_THRESHOLD's own comment.
            scale = fy_rate_scale()
            step_y_max = SHIFT_STEP_MAX * scale
            step_y = max(-step_y_max, min(step_y_max, -SHIFT_KP * ey))
            if tick % 12 == 0 and scale < 1.0:
                log_line(f"  [shift] fy throttled to {scale:.2f}x (leg already carrying post-swing fx)")
            body_shift[1] = clamp_fy(body_shift[1] + step_y, fx_c=body_shift[0], fz_c=STANCE_FZ)
        tick += 1
        time.sleep(CONTROL_DT)

# v6.3 fix: capture_point_status was computed and logged this whole time but never actually
# CONSULTED by anything - the only thing standing between "safe to lift" and the previous run's tip
# was a plain position-error check. The capture point already bakes in body velocity (x_cp = x_com +
# vx*sqrt(z0/g)), so it's exactly the "does this leg have enough real margin, momentum included"
# signal shift_com_to's position-only check was missing. Making it an actual gate here, rather than
# rolling its logic into shift_com_to itself, keeps shift_com_to's job scoped to "get body_shift to
# converge" and puts "is it actually safe to commit to the lift" in its own explicit, easy-to-find step.
CAPTURE_MARGIN_MIN = 0.01    # meters - minimum required capture-point margin inside the support
                             # triangle that will remain once `leg` lifts
CAPTURE_WAIT_MAX = 1.0       # seconds - bounded extra time to let momentum bleed off if the capture
                             # point isn't safely inside yet when shift_com_to returns

def wait_for_safe_lift(leg, max_wait=CAPTURE_WAIT_MAX):
    """Called after shift_com_to returns (converged, stalled, or timed out) and before do_swing(leg).
       Does NOT move body_shift at all - just holds position and re-checks, giving whatever momentum
       is still present a bounded chance to decay before actually committing to lifting `leg` out from
       under the robot. Bounded by CAPTURE_WAIT_MAX so a genuinely marginal step doesn't hang forever -
       if it's still not comfortably inside after that, logs it clearly and proceeds anyway rather than
       stalling the whole sequence."""
    start_t = time.time()
    tick = 0
    while True:
        if check_abort():
            return
        inside, margin, _cp_pt = capture_point_status(leg)
        if tick % 12 == 0:
            diagnostic_line(f"pre-lift check {leg}", active_leg=leg)
        if inside is None or (inside and margin >= CAPTURE_MARGIN_MIN):
            return
        if time.time() - start_t > max_wait:
            log_line(f"  [pre-lift] capture point still not safely inside {leg}'s support triangle "
                     f"after {max_wait}s (inside={inside} margin={margin}) - lifting anyway")
            return
        tick += 1
        time.sleep(CONTROL_DT)

# v6.4 fix: a swing is a real physical disturbance, not a clean instantaneous leg-lift - the latest
# run's log shows BR's swing alone (do_swing, ~0.9s) driving yaw from -0.9deg to +11.0deg and briefly
# pushing ZMP outside the support triangle at 93% through the arc (this matches the OLDEST log
# analysis in this file's history, which already flagged BR's swing as the single largest per-cycle
# yaw contributor - it just wasn't fatal back when the whole gait was one continuous phase clock
# instead of a hard step boundary). The step sequence launched the NEXT shift_com_to (a bigger move
# this time, and TOWARD a different 3-leg polygon than the one just used) immediately after landing,
# while that disturbance was still actively decaying (yaw was still +6-7deg and the body still moving
# when the next shift's log started) - stacking a fresh, large CoM move on top of an unresolved one
# instead of letting the robot actually recover between steps. wait_for_settle() below is the landing-
# side mirror of wait_for_safe_lift() (which already gates the LIFT side the same way): hold position,
# wait (bounded) for the body to actually stop moving, before the next shift is allowed to begin.
SETTLE_WAIT_MAX = 1.0   # seconds - bounded, same rationale as CAPTURE_WAIT_MAX

def wait_for_settle(max_wait=SETTLE_WAIT_MAX, vel_tol=SHIFT_VEL_TOLERANCE):
    """Called right after do_swing lands a leg, before the NEXT shift_com_to starts centering toward
       the following leg. Does not touch body_shift at all - just waits for body_vel to actually
       settle (same SHIFT_VEL_TOLERANCE threshold shift_com_to itself uses to call a move 'done'), so
       the next shift starts from a genuinely still robot instead of one still unwinding the last
       swing's disturbance."""
    start_t = time.time()
    tick = 0
    while True:
        if check_abort():
            return
        speed_xy = math.hypot(body_vel[0], body_vel[1])
        if tick % 12 == 0:
            diagnostic_line(f"post-swing settle speed={speed_xy:.3f}")
        if speed_xy < vel_tol:
            return
        if time.time() - start_t > max_wait:
            log_line(f"  [settle] still moving (speed={speed_xy:.3f}) after {max_wait}s - proceeding anyway")
            return
        tick += 1
        time.sleep(CONTROL_DT)

def ramp_body_shift_to_zero(duration=1.0, steps=50):
    """Mirrors move_feet_manual's linear-ramp pattern, for the two body_shift scalars - used once
       at the end of the sequence so the final settle isn't left holding a nonzero lean."""
    start = list(body_shift)
    for i in range(1, steps + 1):
        if check_abort():
            return
        frac = i / steps
        body_shift[0] = start[0] * (1 - frac)
        body_shift[1] = start[1] * (1 - frac)
        time.sleep(duration / steps)
    body_shift[0] = 0.0
    body_shift[1] = 0.0

last_abad = {leg: 0.0 for leg in legs}

# foot_target[leg] = (fx, fz) in the leg's own nominal (untilted) body frame - drives HIP/KNEE via
# leg_ik_3d (fy comes separately from the shared body_shift, not per-leg). Always set directly by
# whichever main-thread function is currently running: move_feet_manual (crouch/settle) or
# do_swing (one leg's step) - control_loop only ever reads it, never computes it itself.
foot_target = {leg: (0.0, -0.4) for leg in legs}

PITCH_SIGN = 1.0
CORRECTION_FRACTION = 0.4
MAX_CORRECTION_RAD = 0.35
# v6.9 REVERTED (v6.10): the 0.15->0.05 cut above was only compensating for model.sdf's joint
# JointPositionController gains being raised to p_gain=300/d_gain=6 to fight stance-foot slip. That
# gain jump is itself reverted now (back to the original p_gain=50/d_gain=2) - it was unrealistic
# (real quadruped joint-level position gains are usually ~20-50 N*m/rad; 300 is a 6x overshoot) and,
# given how light these leg links actually are, pushed each joint's natural frequency well past what
# a 1ms physics step can resolve cleanly - which is what produced the "instant snap"/vibration and
# the crouch-phase fall this constant was cut for. With the joints back to their original stiffness,
# this constant goes back to its original value too - no reason for it to stay compensating for a
# change that no longer exists.
PITCH_RATE_DAMPING = 0.15    # now applied to the LOW-PASS-FILTERED pitch rate, not raw gyro

ROLL_ABAD_FRACTION = 0.3
MAX_ABAD_ROLL_CORR = 0.15

# --- yaw-hold correction (NEW, untested) ------------------------------------------------------
# The last run's log (now that yaw is actually tracked) showed a real, one-directional yaw drift
# that neither pitch nor roll correction touches at all - +37 deg of accumulated turn over 18s,
# growing almost every cycle rather than randomly wandering, which is what made walking forward
# look like turning/shuffling in place: each step's forward push landed in a slightly different
# world direction than the last, so the straight-line net displacement stayed tiny even though the
# robot was clearly moving its legs and body. Per-swing breakdown pointed at BR's swing as the
# single largest contributor each cycle (the other three swings partially, not fully, cancel it).
# Fix modeled on the existing pitch/roll corrections: hold heading at whatever yaw the gait started
# at, by pushing LEFT-side and RIGHT-side stance/swing legs' fore-aft target in opposite directions
# proportional to the accumulated yaw error - the direct skid-steer equivalent of the pitch theta
# correction. LEG_LR is a genuine left(+1)/right(-1) map - NOT the same as LEG_SIDE above, which
# is a front/back distinction for the IK's own geometry, unrelated to which side the leg is on.
# Sign of YAW_FX_GAIN below is a first guess (positive yaw error -> pull left legs back / push
# right legs forward, i.e. steer right to counter a leftward drift) and has NOT been validated -
# if the next run's logged yaw drift is now WORSE or reverses in an unstable way, that is the
# signal to flip this sign rather than tune its magnitude. Kept deliberately small (comparable to
# the shift_bias magnitudes) since this is a new, unproven correction axis on top of an already
# carefully-tuned pitch/roll system.
LEG_LR = {"FL": 1, "FR": -1, "BL": 1, "BR": -1}
YAW_FX_GAIN = 0.03     # rad of accumulated yaw error -> meters of fx differential
# v6.6 fix: the last run showed yaw wasn't drifting steadily, it was OSCILLATING - +/-3 to 7 degrees
# between consecutive ~0.24s samples, growing and shrinking at different points, never once settling.
# YAW_FX_GAIN alone (checked: never got anywhere near MAX_YAW_FX while yaw swung across a 20+ degree
# range) was acting as a pure P-term with no damping - exactly the failure mode pitch already worked
# through (see PITCH_RATE_DAMPING's own history), which is why yaw gets the identical fix: a rate
# (D-term) contribution from latest_yaw_rate, not just a bigger P-gain (cranking P on an oscillating,
# under-damped system risks amplifying it further, not calming it down).
YAW_RATE_DAMPING = 0.02   # first guess, unvalidated, same status as YAW_FX_GAIN's own sign note below
MAX_YAW_FX = 0.025
gait_start_yaw = [None]

running = [True]

FLIP_LIMIT_DEG = 25.0
FLIP_LIMIT_RAD = math.radians(FLIP_LIMIT_DEG)
aborted = [False]

gait_active = [False]   # gates the yaw-hold correction (only while actually stepping)

# v6.7 fix: the last run fell DURING a swing (BL's), not during a shift - body_shift itself was
# verified frozen the whole time (fx/fy unchanged in the log), but body_xyz still moved ~0.10m at
# 0.15-0.24 m/s sustained speed over that single 0.9s swing. The leak: the yaw-hold correction is
# recomputed and applied to every leg's fx (including the three PLANTED ones) every single tick with
# no notion of "we already shifted, now just hold still while one leg swings" - it kept actively
# changing throughout the swing, and because it's a differential (left legs +, right legs -), that's
# exactly the same "offset a planted foot's target -> move the body" mechanism body_shift itself
# relies on, just uncontrolled during the one phase where nothing should be moving the stance legs at
# all. swinging_now gates it off entirely during do_swing - live during the deliberate shift (where
# steering matters), zero during the swing (where the design's whole premise is that a leg already-
# centered over 3 planted feet can swing without the body needing to keep moving underneath it).
swinging_now = [False]

def check_abort():
    if aborted[0]:
        return True
    if abs(latest_pitch[0]) > FLIP_LIMIT_RAD or abs(latest_roll[0]) > FLIP_LIMIT_RAD:
        aborted[0] = True
        print(f"!!! SAFETY ABORT: |pitch|={math.degrees(latest_pitch[0]):.1f} deg  "
              f"|roll|={math.degrees(latest_roll[0]):.1f} deg exceeded {FLIP_LIMIT_DEG} deg - "
              f"freezing in place, joint commands keep publishing (clamped) !!!")
    return aborted[0]

CONTROL_DT = 0.02   # 50Hz, matches the sleep() at the bottom of this loop

last_theta_terms = {"p": 0.0, "d": 0.0, "raw": 0.0, "clamped": 0.0}
last_yaw_terms = {"err": 0.0, "p": 0.0, "d": 0.0, "fx_term": 0.0}

def control_loop():
    """No more phase clock: foot_target[leg] and body_shift are just read every tick, whatever the
       main thread (move_feet_manual / shift_com_to / do_swing) last set them to. If the main
       thread has stopped advancing anything (abort, or between explicit steps) this loop just
       keeps re-publishing the same values - which is exactly the desired 'freeze in place'
       behavior, with no separate freeze bookkeeping needed."""
    while running[0]:
        check_abort()

        theta_p = PITCH_SIGN * CORRECTION_FRACTION * latest_pitch[0]
        theta_d = PITCH_SIGN * PITCH_RATE_DAMPING * latest_pitch_rate[0]
        theta_raw = theta_p + theta_d
        theta = max(-MAX_CORRECTION_RAD, min(MAX_CORRECTION_RAD, theta_raw))
        last_theta_terms["p"] = theta_p
        last_theta_terms["d"] = theta_d
        last_theta_terms["raw"] = theta_raw
        last_theta_terms["clamped"] = theta

        roll_term = ROLL_ABAD_FRACTION * latest_roll[0]
        roll_term = max(-MAX_ABAD_ROLL_CORR, min(MAX_ABAD_ROLL_CORR, roll_term))

        if gait_active[0] and gait_start_yaw[0] is not None:
            yaw_err = latest_yaw[0] - gait_start_yaw[0]
        else:
            yaw_err = 0.0
        yaw_p = YAW_FX_GAIN * yaw_err
        yaw_d = YAW_RATE_DAMPING * latest_yaw_rate[0]   # v6.6 fix - see YAW_RATE_DAMPING's own comment
        if swinging_now[0]:
            yaw_fx_term = 0.0   # v6.7 fix - see swinging_now's own comment: no fx nudging on any leg
        else:                   # (stance or swinging) while a leg is actually mid-swing
            yaw_fx_term = max(-MAX_YAW_FX, min(MAX_YAW_FX, yaw_p + yaw_d))
        last_yaw_terms["err"] = yaw_err
        last_yaw_terms["p"] = yaw_p
        last_yaw_terms["d"] = yaw_d
        last_yaw_terms["fx_term"] = yaw_fx_term

        now = time.time()
        for leg in legs:
            fx, fz = foot_target[leg]
            fx = clamp_fx(fx + body_shift[0] + LEG_LR[leg] * yaw_fx_term)
            fx_c, fz_c = rotate(fx, fz, theta)
            # v6.1 fix: clamp fy against THIS leg's actual post-rotation (fx_c, fz_c), not a flat
            # constant - see max_safe_fy's docstring. A leg with less reach margin left over this
            # tick gets less lateral shift than the others rather than all four being held to the
            # worst-case leg's number (or, as before, to a number that wasn't checked against the
            # real operating point at all).
            fy = clamp_fy(body_shift[1], fx_c=fx_c, fz_c=fz_c)
            abad_geo, hip, knee = leg_ik_3d(fx_c, fy, fz_c, OY[leg], LEG_SIDE[leg])
            _update_commanded_foot_world(leg, fx_c, fz_c, now)

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
t.start()

def diagnostic_line(label, active_leg=None):
    vx, vy, vz = body_vel
    ax, ay, az = body_accel
    speed = math.hypot(vx, vy)
    line = (f"  [{label}] pitch: {math.degrees(latest_pitch[0]):+.2f} deg  "
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
    if active_leg is not None:
        inside, margin = support_status(active_leg)
        if inside is not None:
            line += f"  CoM: inside={inside} margin={margin:+.4f}"
        zmp_in, zmp_margin, _zmp_pt = zmp_status(active_leg)
        if zmp_in is not None:
            line += f"  ZMP: inside={zmp_in} margin={zmp_margin:+.4f}"
        cp_in, cp_margin, _cp_pt = capture_point_status(active_leg)
        if cp_in is not None:
            line += f"  CapturePt: inside={cp_in} margin={cp_margin:+.4f}"
        vels = stance_foot_velocities(active_leg)
        line += "  stance_foot_vxy: {" + " ".join(f"{l}:({vx2:+.3f},{vy2:+.3f})" for l, (vx2, vy2) in vels.items()) + "}"
    track = foot_tracking_error(active_leg)
    if track:
        line += "  velerr: {" + " ".join(f"{l}:{ve:.3f}" for l, (_pe, ve) in track.items()) + "}"
    foot_z = all_foot_world_z()
    if foot_z:
        marker = lambda l: "*" if l == active_leg else ""
        line += "  foot_z: {" + " ".join(f"{l}{marker(l)}:{z:+.3f}" for l, z in foot_z.items()) + "}"
    fx_all = all_foot_fx()
    line += "  fx: {" + " ".join(f"{l}:{v:+.4f}" for l, v in fx_all.items()) + "}"
    line += "  abad: {" + " ".join(f"{l}:{math.degrees(v):+.2f}" for l, v in last_abad.items()) + "}"
    log_line(line)   # v6.7 fix - full telemetry to run_log.txt only, console stays step-level

def move_feet_manual(deltas, duration=1.5, steps=75, label=None):
    """Linear ramp of foot_target for the given legs - used OUTSIDE the step sequence (crouch,
       final settle). During a step, do_swing() owns the swinging leg's foot_target instead."""
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

def do_swing(leg, duration=SWING_DURATION, steps=SWING_STEPS):
    """Lift, arc, and place ONE leg - a standalone timed motion (main thread owns foot_target[leg]
       for its duration), not read off any shared clock. Lands at fx=+step/2, matching
       swing_profile's own frac=1.0 endpoint."""
    print(f"  stepping {leg}...")
    swinging_now[0] = True   # v6.7 fix - see swinging_now's own comment; freezes the yaw correction
    try:
        print_every = max(1, steps // 6)
        for i in range(1, steps + 1):
            if check_abort():
                return
            frac = i / steps
            foot_target[leg] = swing_profile(leg, frac)
            if i % print_every == 0 or i == 1:
                diagnostic_line(f"swing {leg} {frac*100:3.0f}%", active_leg=leg)
            time.sleep(duration / steps)
    finally:
        swinging_now[0] = False

print("waiting for the drop to settle...")
time.sleep(5.0)
print(f"landed, pitch = {math.degrees(latest_pitch[0]):.2f} deg  roll = {math.degrees(latest_roll[0]):.2f} deg")
print(f"body position: {body_xyz}")

print("--- crouch (hip/knee only, all ABAD at 0) ---")
move_feet_manual({leg: (0.0, STANCE_FZ) for leg in legs}, duration=1.5, steps=75, label="crouch")
for _ in range(20):
    if check_abort():
        break
    diagnostic_line("crouch settle")
    time.sleep(0.1)

start_xyz = list(body_xyz)
start_yaw = latest_yaw[0]

# --- the step sequence itself: exactly what was asked for -------------------------------------
# for each leg in turn: look at the OTHER three (the ones that will carry all the weight once this
# leg lifts), find their support polygon's centroid, move the body there (closed-loop, verified
# against real body_xyz - not assumed), and only once actually aligned, lift and swing this leg.
# No fixed period, no phase clock - each step just takes as long as its shift actually takes, plus
# the fixed SWING_DURATION for the lift itself.
if not aborted[0]:
    gait_start_yaw[0] = latest_yaw[0]
    gait_active[0] = True
    print(f"--- step sequence engaged: order={GAIT_ORDER}  cycles={N_CYCLES}  "
          f"swing_duration={SWING_DURATION}s ---")

    for cycle in range(N_CYCLES):
        if check_abort():
            break
        for leg in GAIT_ORDER:
            if check_abort():
                break
            support_legs = [l for l in legs if l != leg]
            target_xy = support_polygon_centroid(support_legs)   # pure centroid - see v6.5 revert note
            print(f"--- cycle {cycle+1}/{N_CYCLES}: centering CoM over {support_legs} before "
                  f"lifting {leg} (target={target_xy[0]:+.3f},{target_xy[1]:+.3f}) ---")
            shift_com_to(target_xy, about_to_lift=leg)
            if check_abort():
                break
            wait_for_safe_lift(leg)   # v6.3 fix - real capture-point gate, not just a position check
            if check_abort():
                break
            do_swing(leg)
            if check_abort():
                break
            wait_for_settle()   # v6.4 fix - let the swing's own disturbance decay before the NEXT
                                # shift starts centering toward a different leg's polygon

    if not aborted[0]:
        gait_active[0] = False
        print("--- step sequence stopped, returning to a stable stance ---")
        ramp_body_shift_to_zero(duration=1.0, steps=50)
        move_feet_manual({leg: (0.0, STANCE_FZ) for leg in legs}, duration=1.5, steps=75,
                          label="post-gait settle")

end_xyz = list(body_xyz)
dx = end_xyz[0] - start_xyz[0]
dy = end_xyz[1] - start_xyz[1]
dist = math.sqrt(dx*dx + dy*dy)
dyaw_deg = math.degrees(math.atan2(math.sin(latest_yaw[0] - start_yaw), math.cos(latest_yaw[0] - start_yaw)))
print(f"net displacement: dx={dx:.3f} dy={dy:.3f}  total distance={dist:.3f} m  net yaw turned={dyaw_deg:+.1f} deg")
print(f"final body z: {end_xyz[2]:.3f}  (collapsed if well below ~0.35)")
for _ in range(20):
    if check_abort():
        break
    diagnostic_line("end of loop")
    time.sleep(0.1)

print("sequence stopped early (safety abort)" if aborted[0] else "sequence complete")
running[0] = False
