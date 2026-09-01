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
latest_yaw = [0.0]   # not used for any correction - added purely to diagnose "walks forward vs.

PITCH_RATE_LPF_ALPHA = 0.2   # exponential low-pass filter coefficient (0-1, lower = more smoothing)
latest_pitch_rate = [0.0]
latest_pitch_rate_raw = [0.0]
_pitch_rate_source = [None]     # "gyro" or "fd"
_prev_pitch_for_rate = [None]
_prev_pitch_rate_time = [None]
_dumped_imu_fields = [False]

def imu_callback(msg):
    if not _dumped_imu_fields[0]:
        _dumped_imu_fields[0] = True
        try:
            print(f"DEBUG: IMU message fields: {[f.name for f in msg.DESCRIPTOR.fields]}")
        except Exception as e:
            print(f"DEBUG: could not introspect IMU message fields: {e}")

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
    """{leg: z} actual measured world-frame height for every leg's shank-origin sample point
       (foot_world_xyz). Not the true foot tip (see note above) but a FIXED, consistent offset
       from it for a given leg orientation, so relative motion (does it rise during swing?) is
       trustworthy even though the absolute value isn't the ground-contact height."""
    return {l: foot_world_xyz[l][2] for l in legs if l in foot_world_xyz}

def pose_callback(msg):
    if not _dumped_pose_names[0]:
        _dumped_pose_names[0] = True
        print(f"DEBUG: pose entity names seen: {sorted(set(p.name for p in msg.pose))}")
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
STANCE_FZ = -0.384   # proven settled-crouch depth from crawlgait.py


STEP_LENGTH_FRONT = 0.08
STEP_LENGTH_BACK = 0.08


SWING_HEIGHT_FRONT = 0.08
SWING_HEIGHT_BACK = 0.08

FX_LIMIT = 0.09   # hard clamp - both step lengths above stay well inside this with margin

def clamp_fx(v):
    return max(-FX_LIMIT, min(FX_LIMIT, v))


GAIT_PERIOD = 6.0     # seconds per full 4-leg cycle


SWING_DUTY = 0.15

GAIT_ORDER = ["BR", "FL", "FR", "BL"]
LEG_PHASE_OFFSET = {leg: i / 4.0 for i, leg in enumerate(GAIT_ORDER)}
N_CYCLES = 3   # back to 3 now that GAIT_PERIOD is back to 6.0 (keeps total gait time ~18s)


SHIFT_MAG_FRONT = 0.03  
SHIFT_MAG_BACK = 0.03    
SHIFT_RAMP_FRAC = 0.1

def _shift_envelope(swing_frac):
    """0 at liftoff/touchdown, 1 for the middle ~60% of the swing - a smooth trapezoid so the
       bias is at full strength for nearly the whole time the leg is actually off the ground
       (unlike a peaked/sine envelope, which would be weakest exactly when it's needed most)."""
    if swing_frac < SHIFT_RAMP_FRAC:
        return _smoothstep(swing_frac / SHIFT_RAMP_FRAC)
    if swing_frac > 1.0 - SHIFT_RAMP_FRAC:
        return _smoothstep((1.0 - swing_frac) / SHIFT_RAMP_FRAC)
    return 1.0

last_shift_bias = {"leg": None, "bias": 0.0}

foot_target = {leg: (0.0, -0.4) for leg in legs}
abad_cmd = {leg: 0.0 for leg in legs}

PITCH_SIGN = 1.0
CORRECTION_FRACTION = 0.4
MAX_CORRECTION_RAD = 0.35
PITCH_RATE_DAMPING = 0.15    # now applied to the LOW-PASS-FILTERED pitch rate, not raw gyro

ROLL_ABAD_FRACTION = 0.3
MAX_ABAD_ROLL_CORR = 0.15

LEG_LR = {"FL": 1, "FR": -1, "BL": 1, "BR": -1}
YAW_FX_GAIN = 0.03     # rad of accumulated yaw error -> meters of fx differential
MAX_YAW_FX = 0.025
gait_start_yaw = [None]

running = [True]

FLIP_LIMIT_DEG = 25.0
FLIP_LIMIT_RAD = math.radians(FLIP_LIMIT_DEG)
aborted = [False]
abort_freeze_t = [None]   # gait-clock time frozen at the instant of a safety abort

gait_active = [False]
gait_start_time = [None]

def leg_phase_fracs(leg, t):
    global_phase = (t % GAIT_PERIOD) / GAIT_PERIOD
    return (global_phase - LEG_PHASE_OFFSET[leg]) % 1.0

def current_swing_leg(t):
    for leg in legs:
        if leg_phase_fracs(leg, t) < SWING_DUTY:
            return leg
    return None   # shouldn't happen with SWING_DUTY*4==1.0, but keep it defensive

def _smoothstep(x):
    x = max(0.0, min(1.0, x))
    return x * x * (3 - 2 * x)

def foot_offset_for_leg(leg, t):
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

def check_abort():
    if aborted[0]:
        return True
    if abs(latest_pitch[0]) > FLIP_LIMIT_RAD or abs(latest_roll[0]) > FLIP_LIMIT_RAD:
        aborted[0] = True
        if gait_active[0] and gait_start_time[0] is not None:
            abort_freeze_t[0] = time.time() - gait_start_time[0]
        print(f"!!! SAFETY ABORT: |pitch|={math.degrees(latest_pitch[0]):.1f} deg  "
              f"|roll|={math.degrees(latest_roll[0]):.1f} deg exceeded {FLIP_LIMIT_DEG} deg - "
              f"freezing gait clock, joint commands keep publishing (clamped) !!!")
    return aborted[0]

CONTROL_DT = 0.02   # 50Hz, matches the sleep() at the bottom of this loop

last_theta_terms = {"p": 0.0, "d": 0.0, "raw": 0.0, "clamped": 0.0}
last_yaw_terms = {"err": 0.0, "fx_term": 0.0}

def control_loop():
    while running[0]:
        check_abort()

        if gait_active[0]:
            t = abort_freeze_t[0] if aborted[0] else (time.time() - gait_start_time[0])
            for leg in legs:
                foot_target[leg] = foot_offset_for_leg(leg, t)

            active_leg = current_swing_leg(t)
            bias = 0.0
            if active_leg is not None:
                swing_frac = leg_phase_fracs(active_leg, t) / SWING_DUTY
                is_front = active_leg in ("FL", "FR")
                mag = SHIFT_MAG_FRONT if is_front else SHIFT_MAG_BACK
                sign = 1.0 if is_front else -1.0
                bias = sign * mag * _shift_envelope(swing_frac)
            last_shift_bias["leg"] = active_leg
            last_shift_bias["bias"] = bias
            if active_leg is not None:
                for leg in legs:
                    if leg == active_leg:
                        continue
                    fx, fz = foot_target[leg]
                    foot_target[leg] = (clamp_fx(fx + bias), fz)
        # else: foot_target is being driven directly by move_feet_manual() (crouch / pre-gait ramp)

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
        yaw_fx_term = max(-MAX_YAW_FX, min(MAX_YAW_FX, YAW_FX_GAIN * yaw_err))
        last_yaw_terms["err"] = yaw_err
        last_yaw_terms["fx_term"] = yaw_fx_term

        now = time.time()
        for leg in legs:
            fx, fz = foot_target[leg]
            fx = clamp_fx(fx + LEG_LR[leg] * yaw_fx_term)
            # Apply pitch correction (theta) ONLY to stance legs
            if gait_active[0] and leg == current_swing_leg(t):
                fx_c, fz_c = fx, fz
            else:
                fx_c, fz_c = rotate(fx, fz, theta)
            hip, knee = leg_ik(fx_c, fz_c, LEG_SIDE[leg])
            _update_commanded_foot_world(leg, fx_c, fz_c, now)

            abad = abad_cmd[leg] + roll_term

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
            f"shift_bias(leg={last_shift_bias['leg']} val={last_shift_bias['bias']:+.4f})  "
            f"yaw_corr(err={math.degrees(last_yaw_terms['err']):+.2f}deg fx_term={last_yaw_terms['fx_term']:+.4f})")
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
    print(line)

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

# ramp from the flat crouch pose to the phase-clock's own t=0 targets, so engaging the
# continuous gait doesn't start with a sudden, uncommanded jump in every foot's target
if not aborted[0]:
    print("--- pre-gait ramp (to phase-clock t=0 pose) ---")
    t0_targets = {leg: foot_offset_for_leg(leg, 0.0) for leg in legs}
    move_feet_manual(t0_targets, duration=1.5, steps=75, label="pre-gait ramp")

if not aborted[0]:
    gait_start_time[0] = time.time()
    gait_start_yaw[0] = latest_yaw[0]
    gait_active[0] = True
    print(f"--- continuous gait engaged: period={GAIT_PERIOD}s  swing_duty={SWING_DUTY}  "
          f"order={GAIT_ORDER}  step_length(front/back)=({STEP_LENGTH_FRONT}/{STEP_LENGTH_BACK}) ---")

    total_duration = N_CYCLES * GAIT_PERIOD
    print_interval = 0.25
    t = 0.0
    while t < total_duration:
        if check_abort():
            break
        t = time.time() - gait_start_time[0]
        active_leg = current_swing_leg(t)
        diagnostic_line(f"gait t={t:5.2f}s leg={active_leg}", active_leg=active_leg)
        time.sleep(print_interval)


    if not aborted[0]:
        gait_active[0] = False
        print("--- gait clock stopped, returning to a stable stance before finishing ---")
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
