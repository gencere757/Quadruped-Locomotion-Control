import gz.transport13 as transport
from gz.msgs10.double_pb2 import Double
from gz.msgs10.imu_pb2 import IMU
from gz.msgs10.pose_v_pb2 import Pose_V
import math
import threading
import time
import sys

_log_file = open("run_log_trot.txt", "w")

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

#Lengths of the leg parts for ik
L1 = 0.2    #Upper Leg
L2 = 0.2    #Lower Leg

def leg_ik(fx, fz, s):  #Solves inverse kinematics for a positin of the feet point relative to the hip, returns the angles for hip and knee joints
    u = s * fx  #s for  inversing target for back legs, since the legs  are mirrored, the legs will move in different directions wrt to robot frame if given same sgin command
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

def rotate(fx, fz, theta):  #Rotation matrix to be used in pitch correction later
    return (fx*math.cos(theta) - fz*math.sin(theta),
            fx*math.sin(theta) + fz*math.cos(theta))
    
D_ABAD = 0.1
OY = {"FL": 1, "FR": -1, "BL": 1, "BR": -1}
FRONT_BACK = {"FL": 1, "FR": 1, "BL": -1, "BR": -1}   # used by the fy-based yaw steering below -
                                                        # +1 front, -1 back (independent of left/right).

def leg_ik_3d(fx, fy, fz, oy, s):
    #The displacement amounts for the hip abduction/ adduction action
    dy = oy * D_ABAD + fy
    dz = fz

    #The total distancce of target from ABAD joint
    r = math.hypot(dy, dz)
    r = max(r, D_ABAD + 1e-6)
    c = max(-1.0, min(1.0, (oy * D_ABAD) / r))  #The cos of angle between the link between ABAD-to-hip joint and the r

    base = math.atan2(dz, dy)   #Angle between robot y  plane and r
    #The two angles that are to the two sides of the base, with angular displacement acos(c)
    phi_a = base + math.acos(c) 
    phi_b = base - math.acos(c)
    #Pick the smaller one  among the two
    abad = phi_a if abs(phi_a) < abs(phi_b) else phi_b  #This is how much to rotate the abad angle

    w = -dy*math.sin(abad) + dz*math.cos(abad)  #Rotate the point around the x axis by abad degrees since its the new point the hip-knee complex has  to reach
    hip, knee = leg_ik(fx, w, s)    #Solve the 2D inverse kinematics problem
    return abad, hip, knee

LEG_SIDE = {"FL": 1, "FR": 1, "BL": -1, "BR": -1}
LEG_LR = {"FL": 1, "FR": -1, "BL": 1, "BR": -1}
legs = ["FL", "FR", "BL", "BR"]

#Create a node and set the publishing setup for the actuator commands
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

TROT_PERIOD = 1.6         # seconds per full cycle. Real small quadrupeds trot at roughly 0.3-0.6s
                           # cycles; this stays well below that for stability margin.
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
        A = STRIDE_HALF_AMPLITUDE

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
CORRECTION_FRACTION = 0.4
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

CONTROL_DT = 0.02
last_theta_terms = {"p": 0.0, "d": 0.0, "clamped": 0.0}
last_yaw_terms = {"err": 0.0, "fx_term": 0.0, "fy_term": 0.0}
last_active_pair = [None]

def control_loop():
    while running[0]:
        check_abort()

        active_pair = None
        if gait_active[0] and gait_start_time[0] is not None:
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

        if gait_active[0] and gait_start_yaw[0] is not None:
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
    run_start = time.time()
    tick = 0
    while True:
        if check_abort():
            break
        elapsed = time.time() - run_start
        if elapsed >= total_duration:
            break
        if tick % 15 == 0:
            diagnostic_line(f"trot t={elapsed:.2f}")
        tick += 1
        time.sleep(CONTROL_DT)

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
for _ in range(20):
    if check_abort():
        break
    diagnostic_line("end of loop")
    time.sleep(0.1)

print("sequence stopped early (safety abort)" if aborted[0] else "sequence complete")
running[0] = False