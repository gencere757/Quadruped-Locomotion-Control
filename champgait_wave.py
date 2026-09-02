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

# tee all print() output to a log file as well as the console, so it can be read back
# without copy/pasting - always the same filename, overwritten each run
#
# Archive the previous run's log before truncating it (same pattern now used by manual_control.py/
# turn_test.py/trot_demo.py) - copies go in run_log_archive/, timestamped; run_log_wave.txt itself
# keeps meaning "the live/most recent run" for anything that reads it.
_LOG_NAME = "run_log_wave.txt"
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

# ============================================================================
# champgait_wave.py - FORK of champgait.py (v6.11), Arda's "continuous CoM sway" proposal.
#
# This is a DELIBERATE fork, not an edit of champgait.py, specifically so this experiment can't
# regress the discrete CoM-centering controller (champgait.py) if it doesn't pan out.
#
# What's different from champgait.py: champgait.py's v6 rewrite drove the gait as an explicit
# discrete state machine per leg - shift_com_to (converge), wait_for_safe_lift, do_swing,
# wait_for_settle - repeated once per leg. Every one of this session's failures traced back to a
# version of the same pattern: a SHIFT phase that has to move the CoM by some real amount, then a
# hard boundary into a SWING phase. The cycle-2 failures in particular (FL slipping hard, 3 times in
# a row under different tuning) always happened during a shift that needed a bigger-than-usual
# combined move, right at the discrete transition - consistent with Arda's diagnosis: stopping,
# shifting, and re-starting produces genuine transients (yaw/pitch disturbances, momentum that has to
# be built up and killed again every single step) that a smooth trajectory wouldn't.
#
# The fix proposed: never let the lateral CoM position stop moving. Model it as a continuous function
# of gait phase - y_com(t) = A*sin(omega*t) - superimposed on the ALREADY-continuous forward walking
# motion (each leg's own fore-aft stance sweep, which this project had BEFORE champgait.py's discrete
# rewrite - see champ_old.py, the direct ancestor of the phase-clock code below). Because the lateral
# sway is a smooth sinusoid tied to the same shared phase clock that sequences leg swings, weight is
# already shifting onto the next support tripod WHILE the current leg is still swinging, instead of
# "swing, then stop and figure out where to shift for the NEXT leg" - there is no discrete event for a
# sudden pivoting moment to attach to.
#
# Concretely: champ_old.py already had the continuous phase clock (leg_phase_fracs/current_swing_leg/
# foot_offset_for_leg below are lightly adapted from it, unchanged in spirit) but its ONLY weight-shift
# mechanism was a crude fixed-magnitude FORE-AFT bias (SHIFT_MAG_FRONT/BACK) applied to the other three
# legs during a swing - no genuine LATERAL (side-to-side) weight shift at all, which is almost
# certainly why it needed the discrete CoM-centering rewrite in the first place (nothing was ever
# actively moving the CoM toward the correct side of the support polygon). This fork keeps
# champgait.py's real fix for THAT gap - the 3-DOF leg_ik_3d (ABAD-driven lateral shift) and its
# reach-aware max_safe_fy/clamp_fy - and drives the same body_shift[1] (fy) those already feed,
# continuously, off the sine function instead of a converge-and-stop shift_com_to loop.
#
# Everything NOT related to gait generation - leg_ik/leg_ik_3d/rotate, IMU + ground-truth telemetry,
# support-polygon/ZMP/capture-point stability math, the pitch/roll/yaw corrections, the safety-abort
# watchdog - is carried over unchanged from champgait.py (v6.11). Only the sections between the
# "===== CONTINUOUS GAIT ====" markers below are new.
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

# --- 3-DOF leg IK (ABAD abduction + the existing HIP/KNEE 2-link) - unchanged from champgait.py ---
D_ABAD = 0.1
OY = {"FL": 1, "FR": -1, "BL": 1, "BR": -1}

def leg_ik_3d(fx, fy, fz, oy, s):
    """fx,fy,fz = desired foot position relative to the leg's ABAD pivot, in true body-frame axes.
       oy = OY[leg] (+1 left, -1 right), s = LEG_SIDE[leg] (+1 front, -1 back). Returns (abad, hip,
       knee). See champgait.py's own header note for the full geometric derivation - unchanged here."""
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
legs = ["FL", "FR", "BL", "BR"]

node = transport.Node()
pubs = {}
for leg in legs:
    pubs[f"{leg}_ABAD"] = node.advertise(f"/model/my_quadruped/joint/{leg}_ABAD/cmd_pos", Double)
    pubs[f"{leg}_HIP"] = node.advertise(f"/model/my_quadruped/joint/{leg}_HIP/cmd_pos", Double)
    pubs[f"{leg}_KNEE"] = node.advertise(f"/model/my_quadruped/joint/{leg}_KNEE/cmd_pos", Double)

# --- IMU: orientation (pitch/roll/yaw) + filtered rates - unchanged from champgait.py -----------
latest_pitch = [0.0]
latest_roll = [0.0]
latest_yaw = [0.0]

PITCH_RATE_LPF_ALPHA = 0.2
latest_pitch_rate = [0.0]
latest_pitch_rate_raw = [0.0]
_pitch_rate_source = [None]
_prev_pitch_for_rate = [None]
_prev_pitch_rate_time = [None]
_dumped_imu_fields = [False]

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

# --- ground-truth body + per-foot world-frame telemetry - unchanged from champgait.py -----------
body_xyz = [None, None, None]
body_vel = [0.0, 0.0, 0.0]
body_accel = [0.0, 0.0, 0.0]
_prev_body_xyz = [None, None, None]
_prev_body_vel = [None, None, None]
_prev_pose_time = [None]

link_xyz = {}
_dumped_pose_names = [False]

def _rotate_body_to_world(dx, dy, dz, pitch, roll):
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

# --- stability checks against the support polygon - unchanged from champgait.py -----------------
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
    if body_xyz[0] is None:
        return None, None
    return _polygon_check((body_xyz[0], body_xyz[1]), active_leg)

G = 9.8

def compute_zmp():
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
CAPTURE_GAIN = math.sqrt(CAPTURE_HEIGHT / 9.8)

def compute_capture_point():
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
    support_legs = [l for l in legs if l != active_leg]
    return {l: tuple(foot_world_vel.get(l, (0.0, 0.0, 0.0))[:2]) for l in support_legs}

def all_foot_fx():
    return {l: foot_target[l][0] for l in legs}

# --- gait constants - unchanged from champgait.py v6.11 (STANCE_FZ, step lengths, swing heights,
# reach budget - all already tuned/validated against this leg's real geometry) -------------------
#
# HEADS UP before the next run: this whole file (including PITCH_RATE_DAMPING's own comment below,
# "the joint gains this compensates for are back at their original, realistic p_gain=50/d_gain=2")
# was tuned against a MUCH lighter CAD export - the current model.sdf has since been re-exported with
# thigh+shank mass roughly tripled and the ABAD/hip unit ~7x heavier, and its p_gain/d_gain were
# re-tuned to 265/10.6 for a completely different (trot) gait's needs, not this one's. Nothing in this
# file has been touched since either of those changes. First run after picking this back up should be
# treated as a from-scratch check against the new mass+gains, not an assumption that the old tuning
# (GAIT_PERIOD, STEP_LENGTH, SWAY_AMPLITUDE, X_LEAN_AMPLITUDE, the correction-loop constants, all of
# it) still holds - see run_log_wave.txt for what actually happens before touching any of it.
STANCE_FZ = -0.34
STEP_LENGTH_FRONT = 0.06
STEP_LENGTH_BACK = 0.08
SWING_HEIGHT_FRONT = 0.06
SWING_HEIGHT_BACK = 0.08
FX_LIMIT = 0.09

def clamp_fx(v):
    return max(-FX_LIMIT, min(FX_LIMIT, v))

def _smoothstep(x):
    x = max(0.0, min(1.0, x))
    return x * x * (3 - 2 * x)

REACH_BUDGET = 0.39

def max_safe_fy(fx_c, fz_c, budget=REACH_BUDGET):
    """Unchanged from champgait.py - closed-form reach-aware bound on |fy|. See that file's own
       docstring for the full derivation. Still needed here: the continuous sway below drives fy
       through a full sine sweep every cycle, so it needs the SAME per-tick, per-leg reach clamp the
       discrete version used, not just a fixed SWAY_AMPLITUDE assumed safe everywhere."""
    r_max_sq = budget * budget - fx_c * fx_c + D_ABAD * D_ABAD
    if r_max_sq <= 0.0:
        return 0.0
    dy_max_sq = r_max_sq - fz_c * fz_c
    if dy_max_sq <= 0.0:
        return 0.0
    return max(0.0, math.sqrt(dy_max_sq) - D_ABAD)

def clamp_fy(v, fx_c=0.0, fz_c=None):
    if fz_c is None:
        fz_c = STANCE_FZ
    bound = max_safe_fy(fx_c, fz_c)
    return max(-bound, min(bound, v))

# ==================================================================================================
# ===== CONTINUOUS GAIT (new in this fork - everything below replaces champgait.py's discrete    =====
# ===== shift_com_to / wait_for_safe_lift / do_swing / wait_for_settle state machine)             =====
# ==================================================================================================

GAIT_PERIOD = 24.0      # v3 FIX (was 6.0, champ_old.py's forward-walking pace): that period made the
                         # sway's own peak rate-of-change 0.06*2*pi/3.0 = 0.126 m/s - 3.35x faster than
                         # champgait.py's own proven-safe CoM-shift rate (SHIFT_STEP_MAX/CONTROL_DT =
                         # 0.00075/0.02 = 0.0375 m/s, itself already tuned DOWN twice - see SHIFT_KP's
                         # own v6.8 comment - to stop slipping). The v2 phase fix (peak lean at swing
                         # onset, not mid-swing) still fell identically, with the SAME failure signature,
                         # which rules out timing/alignment as the cause and points straight at rate: a
                         # 9.78 m/s^2 lateral acceleration spike showed up in the ease-in phase itself -
                         # a purely quasi-static weight shift with all 4 feet still planted, no swing
                         # involved at all - meaning the shift is too fast full stop, not just badly
                         # timed relative to the swing. 24.0 puts the sway's peak rate (0.0314 m/s,
                         # computed below) comfortably under the proven-safe 0.0375 m/s cap. This makes
                         # the walk ~4x slower than champ_old.py's original pace - that's the real cost
                         # of moving this much CoM weight around without slipping; MAX_SHIFT_RATE below
                         # backstops this as a hard cap regardless.
SWING_DUTY = 0.15        # fraction of GAIT_PERIOD any ONE leg spends swinging - same as champ_old.py.
                         # 4*0.15=0.6, not 1.0, so there are real quiet windows (40% of the cycle)
                         # with all four feet planted at once - more stability margin than a duty
                         # factor that keeps exactly one leg always in the air.
GAIT_ORDER = ["BR", "FL", "FR", "BL"]   # unchanged from champgait.py - alternates right/left/right/
                                          # left, which the sway derivation below depends on.
LEG_PHASE_OFFSET = {leg: i / 4.0 for i, leg in enumerate(GAIT_ORDER)}
N_CYCLES = 3

def leg_phase_fracs(leg, t):
    global_phase = (t % GAIT_PERIOD) / GAIT_PERIOD
    return (global_phase - LEG_PHASE_OFFSET[leg]) % 1.0

def current_swing_leg(t):
    """None during the quiet (all-planted) windows, otherwise whichever leg is currently swinging."""
    for leg in legs:
        if leg_phase_fracs(leg, t) < SWING_DUTY:
            return leg
    return None

def foot_offset_for_leg(leg, t):
    """Continuous fore-aft stance sweep + swing arc, adapted from champ_old.py's version of the same
       function - this is the part of the OLD continuous gait that was never the problem (forward
       walking always worked; only the missing lateral weight shift didn't). Swing uses the same
       smoothstep-fx / sine-fz arc shape champgait.py's swing_profile used, just evaluated off the
       shared phase clock instead of a standalone per-leg timer."""
    is_front = leg in ("FL", "FR")
    step = STEP_LENGTH_FRONT if is_front else STEP_LENGTH_BACK
    height = SWING_HEIGHT_FRONT if is_front else SWING_HEIGHT_BACK
    local_phase = leg_phase_fracs(leg, t)
    if local_phase < SWING_DUTY:
        swing_frac = local_phase / SWING_DUTY
        s = _smoothstep(swing_frac)
        fx = -step / 2.0 + step * s
        fz = STANCE_FZ + height * math.sin(math.pi * swing_frac)
    else:
        stance_frac = (local_phase - SWING_DUTY) / (1.0 - SWING_DUTY)
        fx = (step / 2.0) - step * stance_frac
        fz = STANCE_FZ
    return clamp_fx(fx), fz

# --- Arda's continuous lateral CoM sway: y_com(t) = A*sin(omega*t) ------------------------------
# GAIT_ORDER=[BR,FL,FR,BL] alternates right/left/right/left, so the CoM only needs to lean fully to
# one side and back TWICE per GAIT_PERIOD (not four times) - once for the BR/FR (right-leg) pair,
# once for the FL/BL (left-leg) pair. So the sway's period is GAIT_PERIOD/2, not GAIT_PERIOD.
#
# Sign/phase convention: OY (+1=left,-1=right) and this file's established fy convention (from
# champgait.py's shift_com_to sign fix: increasing fy pushes the body toward -y, i.e. right, since a
# planted foot is fixed and a larger foot-target offset moves the BODY the opposite way) together
# mean the CoM should be pushed toward NEGATIVE fy (right) while a RIGHT leg (BR, FR) is swinging -
# so the other three (all now further left) fully support it - and toward POSITIVE fy (left) while a
# LEFT leg (FL, BL) is swinging.
#
# v2 FIX (was: peak at mid-swing, global_phase = LEG_PHASE_OFFSET[leg] + SWING_DUTY/2): that placement
# meant the lean was only cos(2*pi*(SWING_DUTY/2)/0.5) = cos(54deg) = 59% of the way to full amplitude
# at the exact instant the leg LIFTS OFF (global_phase = LEG_PHASE_OFFSET[leg], i.e. swing onset) - the
# robot was starting each swing on a still-transitioning, under-leaned CoM instead of an already-settled
# one. Confirmed directly in the failing run's own log: gait engaged with fy=-0.0353 (exactly
# -0.06*0.588), and it started diverging (pitch/yaw/x-slip) the instant BR left the ground, before the
# lean ever finished. Fix: zero the phase offset so the peak lands AT swing onset instead of mid-swing -
# the body is already fully leaned by the time a leg lifts, and spends the swing window relaxing back
# toward center/the opposite lean rather than still fighting to get there:
#     y_com(phase) = -SWAY_AMPLITUDE * cos(2*pi * phase / 0.5)
# At BR onset (global_phase=0): cos(0)=1 -> -A (full right-lean, already achieved). At FL onset
# (global_phase=0.25): cos(2*pi*0.25/0.5)=cos(pi)=-1 -> +A (full left-lean). FR onset (0.5): cos(2pi)=1
# -> -A. BL onset (0.75): cos(3pi)=-1 -> +A. All four line up on their own swing onsets automatically,
# for free, because SWING_DUTY's legs are spaced by exactly half the sway period (0.25 vs 0.5).
SWAY_AMPLITUDE = 0.06     # meters - comparable to the lateral excursions champgait.py's discrete
                          # shift_com_to was already commanding per step (~0.05-0.09m observed), well
                          # inside max_safe_fy's reach budget at typical fx (~0.10-0.11m there).
SWAY_PERIOD_FRAC = 0.5    # fraction of GAIT_PERIOD per full lateral sway cycle - see derivation above
SWAY_PHASE_OFFSET_FRAC = 0.0   # v2: peak at swing ONSET, not mid-swing - see fix note above

MAX_SHIFT_RATE = 0.0375   # v3: m/s hard cap on how fast body_shift[1] is allowed to actually move,
                           # applied in control_loop below - identical to champgait.py's proven-safe
                           # SHIFT_STEP_MAX/CONTROL_DT rate (0.00075/0.02). GAIT_PERIOD=24.0 already
                           # keeps the ideal sway's own peak rate under this, so in normal operation this
                           # should rarely bind - it exists as a backstop so body_shift[1] can never be
                           # commanded to move faster than the known-safe rate, no matter what future
                           # changes to GAIT_PERIOD/SWAY_AMPLITUDE/SWAY_PERIOD_FRAC do to the ideal curve.

def y_com_target(t):
    global_phase = (t % GAIT_PERIOD) / GAIT_PERIOD
    return -SWAY_AMPLITUDE * math.cos(2 * math.pi * (global_phase - SWAY_PHASE_OFFSET_FRAC) / SWAY_PERIOD_FRAC)

# --- v4 FIX: fore-aft (x) CoM lean - the piece the "tips backwards" reports were pointing at all along.
# champgait.py's discrete controller shifts to the support_polygon_centroid of the OTHER THREE FEET'S
# ACTUAL WORLD POSITIONS before every single lift - both x AND y. This file only ever built the y half
# (the left/right sway) and left body_shift[0] (fx) at 0.0 for the entire run. GAIT_ORDER=[BR,FL,FR,BL]
# is back,front,front,back - not alternating - so unlike the left/right sway (which got a free half-
# period symmetry because R/L legs alternate every step), BACK legs (BR, BL) are grouped together
# (onsets at phase 0 and 0.75, i.e. adjacent across the 1.0/0 seam) and FRONT legs (FL, FR) are grouped
# together (onsets at 0.25 and 0.5) - a genuine two-state condition, not a sine.
#
# v4.1 SIGN FIX: the first version of this got the sign backwards. Using CMD_HIP_OFFSET's real hip
# geometry ((+-0.15, +-0.213)) plus each stance leg's own foot_offset_for_leg(t) fx, the ACTUAL
# body-frame centroid of the remaining 3 feet at BR's onset works out to cx=+0.058 (shifted forward, as
# expected - BR is a back leg, so the remaining tripod's average position IS forward of body center).
# But the planted-foot-fixed rule already established for fy (increasing a commanded foot-target offset
# moves the BODY the OPPOSITE way, since the foot itself is fixed by friction) applies to fx exactly the
# same way: the correct command to align the body with that +0.058 centroid is fx=-0.058, not +0.058.
# The original code did `return X` (positive) at BR's onset, i.e. exactly the wrong sign - confirmed
# directly against the recording: the body was seen sliding LEFT AND BACK, when BR lifting should have
# needed a LEFT AND FORWARD correction (left was already right, from the y half; back-instead-of-forward
# is precisely what a flipped fx sign produces). Fixed by negating both branches below and recalibrating
# the magnitude from the derived per-onset values (0.042-0.058 across the four onsets - not perfectly
# symmetric since STEP_LENGTH_FRONT != STEP_LENGTH_BACK) rather than the original half-guessed 0.03.
X_LEAN_AMPLITUDE = 0.035  # v6: 0.05 matched the *static* centroid shift but pitch kept climbing
                          # continuously through ease-in AND through the swing (not just a transient
                          # spike at swing-onset), and doubling the ease duration (2.0s->4.0s) did NOT
                          # reduce it - if anything pitch-at-engagement went UP slightly. That rules out
                          # "ease ramp too fast" as the main driver and points instead at a SUSTAINED
                          # pitching bias from holding this fx lean for the entire swing window, which at
                          # GAIT_PERIOD=24.0 is ~3.6s long - far longer than champgait.py's discrete
                          # controller ever held a single CoM shift for. Trading some static margin
                          # (was +0.058-0.085, still comfortably positive at this smaller magnitude) for
                          # less sustained torque over that long hold.

def x_com_target(t):
    global_phase = (t % GAIT_PERIOD) / GAIT_PERIOD
    X = X_LEAN_AMPLITUDE
    if global_phase < 0.15:                       # BR still swinging or just landed - stay forward
        return -X
    elif global_phase < 0.25:                     # quad-stance before FL - transition forward->backward
        frac = (global_phase - 0.15) / 0.10
        return -X + 2 * X * _smoothstep(frac)
    elif global_phase < 0.65:                      # FL then FR swinging - stay backward
        return X
    elif global_phase < 0.75:                      # quad-stance before BL - transition backward->forward
        frac = (global_phase - 0.65) / 0.10
        return X - 2 * X * _smoothstep(frac)
    else:                                           # BL swinging - stay forward
        return -X

body_shift = [0.0, 0.0]   # v4: body_shift[0] (fx) is now driven too, by x_com_target(t) - see the fix
                          # note above. Both entries are overwritten every tick while gait is active,
                          # each independently rate-limited by MAX_SHIFT_RATE - kept as a 2-list (not two
                          # plain floats) only so the per-leg loop below can stay identical to
                          # champgait.py's.

last_abad = {leg: 0.0 for leg in legs}
foot_target = {leg: (0.0, -0.4) for leg in legs}

PITCH_SIGN = 1.0
CORRECTION_FRACTION = 0.4
MAX_CORRECTION_RAD = 0.35
PITCH_RATE_DAMPING = 0.15   # unchanged from champgait.py v6.11 (the joint gains this compensates
                            # for are back at their original, realistic p_gain=50/d_gain=2)

ROLL_ABAD_FRACTION = 0.3
MAX_ABAD_ROLL_CORR = 0.15

LEG_LR = {"FL": 1, "FR": -1, "BL": 1, "BR": -1}
# v9: v7's mistake was doubling YAW_FX_GAIN, YAW_RATE_DAMPING, and MAX_YAW_FX all together, which made it
# impossible to tell which one caused that regression (2 falls, and yaw itself got WORSE - +30.2deg net
# turn, worse than v8's own +9.9 to +22.5deg range across 4 clean runs). Direct log evidence pointed at the
# D-term specifically: yaw_d = YAW_RATE_DAMPING * gyro_yaw_rate produced nonzero fx_term even with zero
# actual yaw error (err=+0.00deg during ease-in, yet fx_term swung off raw gyro noise alone) - it's reacting
# to sensor jitter, not real heading drift, and amplifying that (as v7 did) injects a noise-driven
# disturbance into the delicate pitch balance rather than fixing anything.
# v9 isolates the two terms instead of scaling both: DISABLE the noise-sensitive D-term entirely (set to
# 0.0), and give the P-term - a real, validated error signal (proportional to actual accumulated yaw
# error) - a modest, single 1.5x bump (not 2x) to partially compensate for the correction only being active
# during the shorter non-swing fraction of the now much-longer GAIT_PERIOD=24 cycle. MAX_YAW_FX raised
# proportionally so the clamp doesn't defeat the bumped gain. This is a smaller, single-variable-isolated
# change specifically to avoid repeating v7's failure mode - watch the next few runs for both (a) yaw drift
# actually decreasing and (b) no new pitch/roll instability before trusting it.
YAW_FX_GAIN = 0.045
YAW_RATE_DAMPING = 0.0
MAX_YAW_FX = 0.0375
# v10: constant feedforward nudge (see the yaw_fx_term comment below for the evidence/reasoning). Small
# relative to MAX_YAW_FX (about 15% of the clamp) and in the same units as the P-term's typical values seen
# in the logs for genuine errors, so it's sized to meaningfully offset a chronic per-cycle bias without
# being able to dominate or destabilize on its own.
YAW_FEEDFORWARD_BIAS = -0.005
gait_start_yaw = [None]
gait_start_time = [None]

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
              f"|roll|={math.degrees(latest_roll[0]):.1f} deg exceeded {FLIP_LIMIT_DEG} deg - "
              f"freezing in place, joint commands keep publishing (clamped) !!!")
    return aborted[0]

CONTROL_DT = 0.02

last_theta_terms = {"p": 0.0, "d": 0.0, "raw": 0.0, "clamped": 0.0}
last_yaw_terms = {"err": 0.0, "p": 0.0, "d": 0.0, "fx_term": 0.0}
last_active_leg = [None]

def control_loop():
    """Unlike champgait.py's control_loop (which only ever READ foot_target/body_shift, set by the
       main thread's discrete state machine), THIS control_loop computes both, every tick, straight
       off the shared gait clock - there is no separate main-thread sequencer driving them anymore.
       During crouch (gait_active is False), foot_target is instead driven directly by
       move_feet_manual, exactly as in champgait.py."""
    while running[0]:
        check_abort()

        active_leg = None
        if gait_active[0] and gait_start_time[0] is not None:
            t = time.time() - gait_start_time[0]
            for leg in legs:
                foot_target[leg] = foot_offset_for_leg(leg, t)
            active_leg = current_swing_leg(t)
            # v3/v4: rate-limited toward the ideal sway targets instead of a direct assignment - still
            # continuous (never frozen, never stops to converge), just capped at MAX_SHIFT_RATE so the
            # commanded value can never move faster than champgait.py's own proven-safe shift rate.
            # Applied to both axes now - v4 added the x_com_target fore-aft lean alongside y_com_target.
            max_step = MAX_SHIFT_RATE * CONTROL_DT
            target_fx = x_com_target(t)
            body_shift[0] += max(-max_step, min(max_step, target_fx - body_shift[0]))
            target_fy = y_com_target(t)
            body_shift[1] += max(-max_step, min(max_step, target_fy - body_shift[1]))
        last_active_leg[0] = active_leg

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
        yaw_d = YAW_RATE_DAMPING * latest_yaw_rate[0]
        # v10: every run since the v8 revert (8/8 across v8 and v9) drifted yaw in the SAME direction
        # (always positive, never negative) - that's a systematic per-cycle disturbance, not symmetric
        # noise. A pure P-term only partially cancels a constant disturbance like that; it settles at a
        # nonzero steady-state offset (correction force balances the average disturbance rather than ever
        # reaching zero), which matches exactly what we've measured (bounded ~8-22deg, never near 0).
        yaw_correction = max(-MAX_YAW_FX, min(MAX_YAW_FX, yaw_p + yaw_d + YAW_FEEDFORWARD_BIAS))
        last_yaw_terms["err"] = yaw_err
        last_yaw_terms["p"] = yaw_p
        last_yaw_terms["d"] = yaw_d
        last_yaw_terms["fx_term"] = yaw_correction

        now = time.time()
        for leg in legs:
            fx, fz = foot_target[leg]
            # v11: yaw correction used to be zeroed for EVERY leg any time any leg was mid-swing (same
            # v6.7 protection champgait.py uses) - that meant it was blacked out for most of the cycle at
            # GAIT_PERIOD=24 (each swing is ~3.6s), which is the root reason it could only ever partially
            # cancel the steady drift. That blackout exists to protect the 3 STANCE legs: pushing on
            # planted feet asymmetrically (LEG_LR-signed) while only 3 are grounded can tip an already-
            # reduced support base. But that risk doesn't apply to the leg that is ITSELF swinging -
            # nothing is planted there, so nudging its own target doesn't move the body at all, it just
            # steers where that foot lands (real legged robots correct heading substantially this way).
            # So now: the swinging leg gets the full correction (continuous steering via its own
            # footfall), the other (stance) legs still get zero while someone else is mid-swing, and all
            # four get it normally during quad-stance - net effect is the correction runs "in parallel"
            # with the gait far more of the time instead of only in the brief gaps between swings.
            if leg == active_leg or active_leg is None:
                this_yaw_term = yaw_correction
            else:
                this_yaw_term = 0.0
            fx = clamp_fx(fx + body_shift[0] + LEG_LR[leg] * this_yaw_term)
            fx_c, fz_c = rotate(fx, fz, theta)
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

# v13 DIAGNOSTIC CHANGE: t.start() used to be called right here, before the model has even finished
# dropping/settling into the world. That meant control_loop was actively PD-holding all 4 legs at
# foot_target=(0.0, -0.4) (p_gain=50) through the entire physically chaotic drop/landing - whichever
# foot happened to touch down a few ms before the others would get a much bigger reaction torque than
# the rest, which is a plausible source of an uncontrolled yaw twist picked up before crouch/gait ever
# ran a single command. Isolating this: the Thread object is still created here (cheap, no side
# effects), but .start() itself has been moved to AFTER "waiting for the drop to settle" below, so no
# joint commands are published at all until the robot has already come to rest. See start_yaw's logged
# value across a few runs both ways to confirm whether this was the (or a) source of the yaw offset.
t = threading.Thread(target=control_loop, daemon=True)

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
    log_line(line)

def move_feet_manual(deltas, duration=1.5, steps=75, label=None):
    """Unchanged from champgait.py - used only OUTSIDE the continuous gait (crouch, final settle)."""
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

def ease_into_gait(duration=4.0, steps=200):
    """v6.11-wave FIX: without this, engaging the continuous gait meant control_loop's very first
       gait-active tick would DIRECTLY ASSIGN foot_offset_for_leg(leg, 0.0) / y_com_target(0.0) to
       foot_target/body_shift[1] with NO rate limit at all (unlike champgait.py's SHIFT_STEP_MAX-
       capped shift_com_to) - a real position jump, not a smooth start. Concretely: BR's own
       LEG_PHASE_OFFSET is 0, so t=0 lands at the very START of its swing (fx jumps straight from
       crouch's 0 to -STEP_LENGTH_BACK/2, ~4cm) at the SAME instant y_com_target(0) demands roughly
       -3.5cm of lateral sway (t=0 is BR's swing onset, not its mid-swing, which is where the sway
       actually peaks) - two simultaneous, un-rate-limited position jumps on a freshly-settled,
       zero-velocity crouch. First real run fell within about a second of gait start (pitch already
       -9.5deg by t=0.52s), which is exactly what stacking those two jumps at t=0 would produce - the
       "no discrete transients" premise this whole rewrite is built on was being violated right at
       the starting gun. Fix: ramp foot_target (all 4 legs) and body_shift[1] smoothly from wherever
       crouch left them to their own t=0 phase-clock values BEFORE gait_active flips on, so
       control_loop's takeover at t=0 is seamless - the phase-alignment design (sway peaks at each
       leg's mid-swing) is unchanged, only the COLD START into it is now itself continuous.

       v3 update: default duration/steps raised from 1.0s/50 to 2.0s/100 (same 0.02s per-step pacing)
       because target_fy is now the FULL -SWAY_AMPLITUDE (v2 moved the peak to swing onset, so
       y_com_target(0.0) = -0.06, not -0.0353) - at the old 1.0s duration that's a 0.06 m/s ramp rate,
       above MAX_SHIFT_RATE's 0.0375 m/s proven-safe cap. 2.0s keeps this ramp, like control_loop's own
       sway now, comfortably under that rate too.

       v4 update: also eases body_shift[0] (fx) to x_com_target(0.0) - BR is a back leg, so t=0 needs
       the full +X_LEAN_AMPLITUDE forward lean already in place before gait_active flips on, same
       reasoning as the y half.

       v5 update: duration/steps doubled again, 2.0s/100 -> 4.0s/200 (same 0.02s pacing). After the
       v4.1 fx sign fix, a run had healthy CoM/ZMP/capture-point margins throughout (+0.06 to +0.10)
       and genuine forward progress (net dx=+0.228m) - the sign fix clearly worked - but pitch still
       climbed steadily positive (forward tip), starting DURING this ease-in itself (already +4.65deg
       by the time gait engaged, before BR even lifted) and never recovered. theta's reactive pitch
       correction wasn't saturating (8deg reached against MAX_CORRECTION_RAD's ~20deg clamp), so this
       reads as the combined fx+fy ramp - both axes moving to full magnitude at once here - being a
       bigger, faster disturbance than that reactive loop can arrest in time, not a sign/logic error.
       Doubling the duration halves the rate of that combined transient without touching any gain."""
    starts = {leg: foot_target[leg] for leg in legs}
    start_fx = body_shift[0]
    start_fy = body_shift[1]
    targets = {leg: foot_offset_for_leg(leg, 0.0) for leg in legs}
    target_fx = x_com_target(0.0)
    target_fy = y_com_target(0.0)
    for i in range(1, steps + 1):
        if check_abort():
            return
        frac = i / steps
        for leg in legs:
            sx, sz = starts[leg]
            tx, tz = targets[leg]
            foot_target[leg] = (sx + (tx - sx) * frac, sz + (tz - sz) * frac)
        body_shift[0] = start_fx + (target_fx - start_fx) * frac
        body_shift[1] = start_fy + (target_fy - start_fy) * frac
        if i % 10 == 0 or i == 1:
            diagnostic_line(f"easing into gait {frac*100:3.0f}%")
        time.sleep(duration / steps)
    for leg in legs:
        foot_target[leg] = targets[leg]
    body_shift[0] = target_fx
    body_shift[1] = target_fy

print("waiting for the drop to settle...")
time.sleep(5.0)
print(f"landed, pitch = {math.degrees(latest_pitch[0]):.2f} deg  roll = {math.degrees(latest_roll[0]):.2f} deg")
print(f"body position: {body_xyz}")
yaw_at_settle = latest_yaw[0]
print(f"yaw at settle (before control_loop has published a single command): {math.degrees(yaw_at_settle):+.2f} deg")

# v13: control_loop.start() moved here (was previously called before the drop-settle wait above) -
# see the comment where the Thread object is constructed for the full reasoning. No joint commands
# have been published up to this point; the legs were free/unactuated through the entire drop.
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
# v12: start_yaw was never logged anywhere - the last two runs' "net yaw turned" (a value RELATIVE to
# start_yaw) diverged sharply from the absolute yaw shown in "[end of loop]" telemetry (+7.8/+7.5deg vs
# -11.81/-13.23deg respectively), which only makes sense if start_yaw itself was around -19 to -21deg on
# those runs instead of the ~0deg every prior run implicitly had (their two numbers always matched). That
# means either the world reset occasionally leaves some residual orientation, or something else entirely -
# can't tell without seeing the actual value, so log it directly instead of inferring it indirectly again.
print(f"start_yaw (reset baseline, absolute): {math.degrees(start_yaw):+.2f} deg")

# --- the continuous gait itself: no shift/lift/settle state machine - just let the phase clock and
# the sine sway run, and watch it via diagnostic_line + the safety-abort watchdog ------------------
if not aborted[0]:
    print("--- easing into gait's t=0 pose (avoids a discrete jump at gait start) ---")
    ease_into_gait()
if not aborted[0]:
    gait_start_yaw[0] = latest_yaw[0]
    gait_start_time[0] = time.time()
    gait_active[0] = True
    print(f"--- continuous gait engaged: period={GAIT_PERIOD}s  swing_duty={SWING_DUTY}  "
          f"sway_amplitude={SWAY_AMPLITUDE}  cycles={N_CYCLES} ---")
    total_duration = N_CYCLES * GAIT_PERIOD
    run_start = time.time()
    printed_leg = [None]
    tick = 0
    while True:
        if check_abort():
            break
        elapsed = time.time() - run_start
        if elapsed >= total_duration:
            break
        cur_leg = last_active_leg[0]
        if cur_leg != printed_leg[0]:
            if cur_leg is not None:
                print(f"  stepping {cur_leg}...")
            printed_leg[0] = cur_leg
        if tick % 25 == 0:   # ~2x/sec at CONTROL_DT=0.02
            diagnostic_line(f"gait t={elapsed:.2f}", active_leg=cur_leg)
        tick += 1
        time.sleep(CONTROL_DT)

    if not aborted[0]:
        gait_active[0] = False
        print("--- continuous gait complete, returning to a stable stance ---")
        body_shift[1] = 0.0
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