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

# ============================================================================
# v4 controller: legs now have 3 DOF each (ABAD, HIP, KNEE). ABAD rotates the
# whole hip+knee+thigh+shank chain about the body's fore-aft (X) axis, giving
# a real lateral DOF for roll correction - replacing the old differential-
# foot-height hack (which never had a clean physical meaning, since v3 legs
# had no way to move a foot sideways at all).
#
# HIP/KNEE math (leg_ik) is UNCHANGED: it still solves the 2-link planar IK
# in the sagittal (fore-aft/vertical) plane relative to the hip attachment
# point, exactly as before. ABAD is treated as a separate, decoupled DOF -
# a small-angle direct joint command, not folded into leg_ik. This matches
# how the SDF was built: the HIP joint's rotation axis is expressed in the
# body/model frame (not the ABAD link's frame), so hip/knee swinging in the
# sagittal plane stays valid at any ABAD angle - ABAD just translates the
# "shoulder" point sideways/vertically before that plane is applied.
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

LEG_SIDE = {"FL": 1, "FR": 1, "BL": -1, "BR": -1}
legs = ["FL", "FR", "BL", "BR"]
LEFT_LEGS = {"FL", "BL"}   # Y+ side per the SDF

node = transport.Node()
pubs = {}
for leg in legs:
    pubs[f"{leg}_ABAD"] = node.advertise(f"/model/my_quadruped/joint/{leg}_ABAD/cmd_pos", Double)
    pubs[f"{leg}_HIP"] = node.advertise(f"/model/my_quadruped/joint/{leg}_HIP/cmd_pos", Double)
    pubs[f"{leg}_KNEE"] = node.advertise(f"/model/my_quadruped/joint/{leg}_KNEE/cmd_pos", Double)

latest_pitch = [0.0]
latest_roll = [0.0]

# --- pitch RATE, for the new D-term in control_loop() below (see PITCH_RATE_DAMPING). The
# recurring FR-push tip runs away in an accelerating/compounding way (pitch delta roughly doubling
# sample to sample), not a linear drift - that's the signature of a control loop reacting only to
# accumulated angle error (pure P) and always lagging a rotation that's still speeding up. A gyro
# rate is a direct, low-noise, high-bandwidth signal for this - much better than differencing our
# own quaternion-derived pitch. Preferred source: the IMU message's own angular_velocity (if the
# field exists and is actually populated - can't confirm the exact gz.msgs10 IMU field name/axis
# convention without the proto available in this environment, so this is defensive: a one-time
# DEBUG dump of the message's fields on the first callback, plus an always-computed finite-
# differenced fallback that's used whenever the gyro reads read as unavailable). Whichever source
# is used, watch the printed pitch_rate values in run_log.txt on the first run to sanity check it
# actually tracks real rotation (it should spike right as pitch runs away during FR's push).
latest_pitch_rate = [0.0]
_pitch_rate_source = [None]     # "gyro" or "fd", set once we know which one is working
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

    got_gyro = False
    try:
        latest_pitch_rate[0] = msg.angular_velocity.y
        got_gyro = True
    except AttributeError:
        pass
    if got_gyro:
        _pitch_rate_source[0] = "gyro"
    else:
        _pitch_rate_source[0] = "fd"
        now = time.time()
        if _prev_pitch_for_rate[0] is not None and _prev_pitch_rate_time[0] is not None:
            dt = now - _prev_pitch_rate_time[0]
            if dt > 1e-4:
                latest_pitch_rate[0] = (pitch - _prev_pitch_for_rate[0]) / dt
        _prev_pitch_for_rate[0] = pitch
        _prev_pitch_rate_time[0] = now

node.subscribe(IMU, "/model/my_quadruped/imu", imu_callback)

body_xyz = [None, None, None]

# --- body velocity, finite-differenced from consecutive ground-truth pose samples. This is a
# sim-only shortcut (no real robot gets free ground-truth position) but since Pose_V already
# gives us body_xyz for free, differencing it is a driftless, zero-extra-sensor way to get
# v_actual for things like the Raibert/capture-point swing-placement term - no IMU integration
# (and its drift/gravity-compensation headaches) needed. Uses wall-clock time.time() deltas
# between callback firings; fine since the world runs at real_time_factor=1.
body_vel = [0.0, 0.0, 0.0]
body_accel = [0.0, 0.0, 0.0]
_prev_body_xyz = [None, None, None]
_prev_body_vel = [None, None, None]
_prev_pose_time = [None]

# --- foot-level instrumentation: track ALL FOUR shank link positions (not just FR's), keyed by
# leg name ("FL"/"FR"/"BL"/"BR") - confirmed unscoped names from the one-time DEBUG dump below
# (e.g. 'FR_shank', not 'my_quadruped::FR_shank'). Originally added just for FR to check the
# foot-strike hypothesis (refuted); now tracking all four so we can compute the actual support
# polygon (the triangle formed by whichever three feet are planted) and check it directly,
# instead of only inferring "not enough support" from pitch trend.
link_xyz = {}
_dumped_pose_names = [False]

# FRAME NOTE: Pose_V gives the MODEL's pose ("my_quadruped") relative to the world (its parent
# in the entity tree is the world), but gives each LINK's pose ("FR_shank" etc.) relative to the
# model (its parent is the model) - i.e. link_xyz values are in the model's local frame, not
# world. (Giveaway: FR_shank_z reads around -0.19 while body_xyz[2] reads around +0.40 - if both
# were world-frame that foot would be nearly 0.6m below the body, more than the robot's own leg
# length.) So each support foot's local offset has to be rotated into world frame (using current
# pitch/roll - yaw assumed 0, since this gait never yaws and we have no yaw sensor) and added to
# body_xyz - comparing link_xyz values raw against body_xyz (as an earlier version of this file
# did) silently mixed frames and produced meaningless ~0.6-0.7m "margins", larger than the whole
# robot. Defined here (before pose_callback/node.subscribe) rather than further down, since
# pose_callback itself now calls this on every message - Python resolves names in a function
# body at CALL time, but subscription callbacks can start firing on a background thread as soon
# as node.subscribe() runs, so anything a callback depends on needs to already exist by then.
def _rotate_body_to_world(dx, dy, dz, pitch, roll):
    """Rotates a vector from the model's local/body frame into world frame, using the same
       Tait-Bryan ZYX (yaw-pitch-roll) convention imu_callback uses to extract pitch/roll,
       with yaw=0."""
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    wx = cp*dx + sp*sr*dy + sp*cr*dz
    wy = cr*dy - sr*dz
    wz = -sp*dx + cp*sr*dy + cp*cr*dz
    return wx, wy, wz

# --- per-foot WORLD-frame position + velocity, finite-differenced the same way body_vel is.
# The support-polygon check (support_status, below) already showed the CoM never leaves the
# footprint of the three planted feet - even as pitch ran away, it stayed comfortably inside.
# So the failure isn't a static/quasi-static "walked past the edge of my base" problem. The next
# thing to check: are the PLANTED feet actually staying put? These are position-servoed
# (P/D) joints, not force-controlled stance legs - if a stance leg can't supply enough torque to
# hold its commanded position once FR fully loads it during push, that foot would slip/lose grip
# in the real physics even while the code still thinks it's a solid support point. A real planted
# foot should have ~zero world-frame velocity; a spike right as pitch runs away would be the
# signature of that leg giving way under load instead of a gait/geometry problem.
foot_world_xyz = {}                                    # leg -> (wx, wy, wz), most recent
foot_world_vel = {leg: [0.0, 0.0, 0.0] for leg in legs} # leg -> [vx, vy, vz]
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
            if dt > 1e-4:   # guard against a duplicate/zero-interval callback firing
                foot_world_vel[leg_name][0] = (wx - prev[0]) / dt
                foot_world_vel[leg_name][1] = (wy - prev[1]) / dt
                foot_world_vel[leg_name][2] = (wz - prev[2]) / dt
        foot_world_xyz[leg_name] = (wx, wy, wz)
        _prev_foot_world_xyz[leg_name] = (wx, wy, wz)
        _prev_foot_world_time[leg_name] = now

def pose_callback(msg):
    if not _dumped_pose_names[0]:
        _dumped_pose_names[0] = True
        print(f"DEBUG: pose entity names seen: {sorted(set(p.name for p in msg.pose))}")
    for p in msg.pose:
        if p.name == "my_quadruped":
            now = time.time()
            if _prev_pose_time[0] is not None:
                dt = now - _prev_pose_time[0]
                if dt > 1e-4:   # guard against a duplicate/zero-interval callback firing
                    new_vx = (p.position.x - _prev_body_xyz[0]) / dt
                    new_vy = (p.position.y - _prev_body_xyz[1]) / dt
                    new_vz = (p.position.z - _prev_body_xyz[2]) / dt
                    # acceleration: finite-difference the velocity estimate itself, using the same
                    # dt - needed for the ZMP check below (x_zmp depends on the CoM's horizontal
                    # AND vertical acceleration, not just position/velocity). This is a second
                    # derivative of noisy position data, so expect more noise than body_vel has -
                    # that's inherent to the method, not a bug.
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

# --- stability checks against the support polygon (the triangle formed by the three PLANTED feet,
# i.e. everyone except whichever leg is currently lifted/swinging/placing/pushing). Three different
# criteria are tested against the SAME triangle, from weakest to strongest:
#
#   1. support_status()        - the STATIC CoM check: is the body's actual (x,y) position inside
#                                 the triangle right now? This is a quasi-static criterion only -
#                                 it says nothing about momentum or acceleration. Earlier testing
#                                 showed `inside` stayed True the entire run, even through a pitch
#                                 runaway past -25 deg - so a naive "CoM walked past the edge" read
#                                 of the tipping is refuted. But that doesn't mean geometry is
#                                 irrelevant, just that the STATIC version of the check is too weak
#                                 to catch a dynamic tip - see zmp_status() and
#                                 capture_point_status() below, which is what a legged-robot
#                                 stability analysis actually uses.
#   2. zmp_status()             - the Zero Moment Point: accounts for inertial forces (CoM
#                                 acceleration). This is the standard dynamic-tipping criterion - a
#                                 robot can have its CoM sitting inside the polygon and still be
#                                 actively tipping if the ZMP has left it.
#   3. capture_point_status()   - the Capture Point: where the CoM would land if every leg froze
#                                 right now. If this is outside the polygon, the current momentum
#                                 can't be arrested without a recovery step, even if the ZMP is
#                                 currently fine.
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
    """Returns (inside, margin) for an arbitrary (x,y) point against the triangle of the three
       legs other than active_leg, using the already-computed world-frame foot positions
       (foot_world_xyz). Shared by the CoM/ZMP/capture-point checks below - only the point being
       tested differs. `margin` is a rough (not exact perpendicular) distance-to-nearest-edge in
       meters, positive or negative, useful for trend even though its sign convention isn't
       guaranteed consistent with `inside` (triangle winding varies leg to leg). Returns
       (None, None) if any needed foot position hasn't been seen yet."""
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
    margin = min(abs(m) for m in margins)
    return inside, margin

def support_status(active_leg):
    """Returns (inside, margin) for the body's STATIC (x,y) position vs. the support triangle -
       see the caveat in the comment block above. Kept for regression/comparison against the
       ZMP/capture-point checks, not as the primary stability signal any more."""
    if body_xyz[0] is None:
        return None, None
    return _polygon_check((body_xyz[0], body_xyz[1]), active_leg)

# --- ZMP (Zero Moment Point): the actual dynamic-tipping criterion. Standard form, DROPPING the
# angular-momentum-rate term (dH_y/dt) - we don't track each leg's individual inertia/angular
# momentum, and the body itself is ~8.44kg vs ~0.55kg total for all four legs combined, so the
# linear CoM-acceleration term should dominate. This is a first-order approximation, not the full
# textbook ZMP - flagged here rather than silently presented as exact:
#
#   x_zmp = x_com - (xdd_com / (zdd_com + g)) * z_com
#   y_zmp = y_com - (ydd_com / (zdd_com + g)) * z_com
#
# Uses body_accel (a second finite difference of position via body_vel - noisier than body_vel
# itself) and body_xyz[2] as z_com (valid since the ground is flat at z=0 and the inertial-frame
# offset from body origin to true CoM is <0.2mm per the SDF, i.e. negligible).
G = 9.8

def compute_zmp():
    if body_xyz[0] is None:
        return None
    z_com = body_xyz[2]
    xdd, ydd, zdd = body_accel
    denom = zdd + G
    if abs(denom) < 1.0:   # guard against a near-zero/negative denominator from noisy zdd spikes
        denom = G
    return (body_xyz[0] - (xdd/denom)*z_com, body_xyz[1] - (ydd/denom)*z_com)

def zmp_status(active_leg):
    """Returns (inside, margin, (x_zmp,y_zmp)). If this goes False before support_status() does
       (or before pitch visibly runs away), that confirms the failure IS a dynamic/inertial tipping
       problem - one the static CoM check could never have caught."""
    zmp = compute_zmp()
    if zmp is None:
        return None, None, None
    inside, margin = _polygon_check(zmp, active_leg)
    return inside, margin, zmp

# --- Capture point: where the CoM would come to rest if every leg's velocity went to zero right
# now (Pratt et al.) - x_cp = x_com + vx * sqrt(z0/g). Reuses CAPTURE_GAIN (=sqrt(height/g), defined
# further down where the swing-placement correction is - same quantity, referenced here by name
# since Python resolves it at call time and this is only ever called well after that assignment
# runs at import time). If the capture point is already outside the polygon, the current momentum
# can't be arrested without a recovery step - i.e. a fall may already be committed even if ZMP is
# still fine right now.
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

def stance_foot_speeds(active_leg):
    """Returns {leg: horizontal world-frame speed (m/s)} for each currently-planted (non-active)
       leg. A genuinely planted foot should read ~0 - it isn't supposed to move relative to the
       ground while providing support. A spike here (especially one that lines up with pitch
       running away) points to that leg's position-servo joints failing to hold their commanded
       angle under load (losing grip / slipping), not a gait-geometry problem."""
    support_legs = [l for l in legs if l != active_leg]
    out = {}
    for l in support_legs:
        vx, vy, _vz = foot_world_vel.get(l, (0.0, 0.0, 0.0))
        out[l] = math.hypot(vx, vy)
    return out

def stance_foot_velocities(active_leg):
    """Returns {leg: (vx, vy)} SIGNED world-frame velocity components for each currently-planted
       (non-active) leg - unlike stance_foot_speeds() (magnitude only), this lets us directly check
       whether front and back stance legs are sliding in the SAME or OPPOSITE physical direction
       during a shift/push. That's the concrete, checkable version of "front/back joint axis
       convention fights itself": if it were true, front and back stance-leg vx should show
       consistently OPPOSITE sign while sliding; if the LEG_SIDE compensation in leg_ik is correct,
       there's no reason to expect a systematic sign split between front and back."""
    support_legs = [l for l in legs if l != active_leg]
    out = {}
    for l in support_legs:
        vx, vy, _vz = foot_world_vel.get(l, (0.0, 0.0, 0.0))
        out[l] = (vx, vy)
    return out

def all_foot_fx():
    """Returns {leg: current body-frame fx target} for all four legs. Checks the FX_LIMIT-adjacent
       version of the "leg near full extension" concern: rotate()'s pitch/roll correction can't
       change a foot target's reach (it's a pure rotation, magnitude-preserving - r=sqrt(fx^2+fz^2)
       is invariant under rotate() for any theta), but repeated same-direction shifts COULD walk fx
       toward the +-FX_LIMIT=0.09 clamp, where r=0.394m and cos(knee)=0.945 - meaningfully closer to
       the 0.40m singularity than the nominal cos(knee)=0.843 at fx=0. Logging fx directly settles
       whether that's actually happening instead of guessing."""
    return {l: foot_target[l][0] for l in legs}

# foot_target[leg] = (fx, fz) in the leg's own nominal (untilted) body frame - drives HIP/KNEE via leg_ik
foot_target = {leg: (0.0, -0.4) for leg in legs}
# abad_cmd[leg] = direct ABAD joint angle command (radians) - the "base" lateral trim for that leg,
# separate from any auto roll-correction term added in control_loop()
abad_cmd = {leg: 0.0 for leg in legs}

PITCH_SIGN = 1.0
CORRECTION_FRACTION = 0.4   # was 0.3 - bumped after the full-loop test showed pitch outrunning
                            # correction during FR's place/push when entering with residual tilt
MAX_CORRECTION_RAD = 0.35

# --- D-term (rate damping) added on top of the existing P-term. Motivation: the full-loop test
# with the ZMP/capture-point checks added showed FR's push-phase tip is NOT a base-of-support
# departure (CoM/ZMP/CapturePt all stayed comfortably `inside`, margins GROWING, the whole time
# pitch ran away from -4.87 to -28.7 deg). But that runaway was accelerating/compounding (delta
# roughly doubling sample to sample), not linear - the signature of a correction that only reacts
# to accumulated angle (pure P) and structurally lags a rotation that's still speeding up. This
# adds a term proportional to latest_pitch_rate (see imu_callback) so the correction responds to
# how fast the body is rotating, not just how far it's already tilted.
#
# Starting gain is a guess, not derived: during the observed runaway, pitch rate was already
# ~0.3-0.5 rad/s by the time push was a few tenths of a second in. PITCH_RATE_DAMPING=0.15 adds a
# modest correction on that order without dominating the existing P-term (which itself isn't
# saturating the MAX_CORRECTION_RAD clamp at the pitch angles seen before the old runaway started -
# there's headroom). Tune from what run_log.txt shows the theta_p/theta_d breakdown doing.
PITCH_RATE_DAMPING = 0.15

# --- roll correction via real ABAD DOF (replaces the old differential-foot-height hack) ---
# Calibrated from the per-leg ABAD sweep: ALL FOUR legs showed the same sign relationship
# (positive ABAD command -> negative roll, negative command -> positive roll), so a single
# uniform correction (same sign on every leg) is what's needed - no left/right split.
ROLL_ABAD_FRACTION = 0.3   # matches the pitch controller's gain as a starting point
MAX_ABAD_ROLL_CORR = 0.15  # radians

# NOTE: tried adding an integral term here (pitch_integral/roll_integral) to kill the steady-state
# lean left over after each step. Reverted - it wound up during BR's step (while pitch stayed
# net-positive) and was still carrying that stale positive bias into FR's place, where pitch flips
# hard negative - so the I-term fought the correction exactly when it needed to be strongest, and
# the tip started earlier (during place) instead of later (during push). Legged balance in general
# isn't a good fit for integral control anyway: it's built for holding a fixed setpoint against a
# roughly constant disturbance, not for a sequence of fast, direction-reversing per-step transients.
# Back to pure P; the real fix belongs in the gait/footfall geometry, not the orientation loop.

running = [True]

# --- safety watchdog: once the robot tips past this angle, stop advancing the gait instead of
# grinding through the rest of the phases (and the rest of the log) on a robot that's already
# flipped over. Joint commands keep publishing (they're clamped, so they stay bounded) - this
# only stops step_leg()/move_feet()/move_abad()/watch() from moving on to the next phase.
FLIP_LIMIT_DEG = 25.0
FLIP_LIMIT_RAD = math.radians(FLIP_LIMIT_DEG)
aborted = [False]

def check_abort():
    if aborted[0]:
        return True
    if abs(latest_pitch[0]) > FLIP_LIMIT_RAD or abs(latest_roll[0]) > FLIP_LIMIT_RAD:
        aborted[0] = True
        print(f"!!! SAFETY ABORT: |pitch|={math.degrees(latest_pitch[0]):.1f} deg  "
              f"|roll|={math.degrees(latest_roll[0]):.1f} deg exceeded {FLIP_LIMIT_DEG} deg - "
              f"halting gait sequence (joint commands keep publishing, clamped) !!!")
    return aborted[0]

CONTROL_DT = 0.02   # matches the sleep() at the bottom of this loop

# diagnostic breakdown of the last theta computed - P term, D term, combined (pre-clamp and
# post-clamp), so run_log.txt can show whether the D-term is doing anything sensible rather than
# just trusting it blindly. Updated every control_loop tick (50Hz) but only PRINTED at the same
# periodic checkpoints move_feet()/watch() already use, so this doesn't flood the log.
last_theta_terms = {"p": 0.0, "d": 0.0, "raw": 0.0, "clamped": 0.0}

def control_loop():
    while running[0]:
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

        for leg in legs:
            fx, fz = foot_target[leg]
            fx_c, fz_c = rotate(fx, fz, theta)
            hip, knee = leg_ik(fx_c, fz_c, LEG_SIDE[leg])

            # positive ABAD command -> negative roll (confirmed identically for all 4 legs in the
            # calibration sweep), so a positive roll_term (proportional to +roll) counteracts a
            # positive roll disturbance - applied uniformly, no per-leg sign
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

def move_feet(deltas, duration=1.5, steps=75, label=None, active_leg=None):
    """deltas: {leg: (target_fx, target_fz)}. Ramps foot_target for those legs together.
       If label is given, prints pitch/roll periodically DURING the ramp, not just after.
       active_leg: if given, also prints support-polygon status DURING the ramp - this matters
       because a safety abort often fires mid-ramp (check_abort() at the top of this loop
       returns before step_leg ever reaches its own watch() call), so this is the only place
       that can catch the support polygon at the actual moment of collapse."""
    starts = {leg: foot_target[leg] for leg in deltas}
    print_every = max(1, steps // 15)
    for i in range(1, steps + 1):
        if check_abort():
            return
        frac = i / steps
        for leg, (tx, tz) in deltas.items():
            sx, sz = starts[leg]
            foot_target[leg] = (sx + (tx-sx)*frac, sz + (tz-sz)*frac)
        if label and (i % print_every == 0 or i == 1):
            fr_z = link_xyz.get("FR", (None, None, None))[2]
            line = (f"    ({label} {frac*100:3.0f}%) pitch: {math.degrees(latest_pitch[0]):+.2f} deg  roll: {math.degrees(latest_roll[0]):+.2f} deg  FR_shank_z: {fr_z}"
                    f"  pitch_rate({_pitch_rate_source[0]}): {math.degrees(latest_pitch_rate[0]):+.2f} deg/s"
                    f"  theta(p={math.degrees(last_theta_terms['p']):+.2f} d={math.degrees(last_theta_terms['d']):+.2f} -> {math.degrees(last_theta_terms['clamped']):+.2f} deg)")
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
                speeds = stance_foot_speeds(active_leg)
                line += "  stance_foot_speed: {" + " ".join(f"{l}:{s:.3f}" for l, s in speeds.items()) + "}"
                vels = stance_foot_velocities(active_leg)
                line += "  stance_foot_vxy: {" + " ".join(f"{l}:({vx:+.3f},{vy:+.3f})" for l, (vx, vy) in vels.items()) + "}"
            fx_all = all_foot_fx()
            line += "  fx: {" + " ".join(f"{l}:{v:+.4f}" for l, v in fx_all.items()) + "}"
            print(line)
        time.sleep(duration / steps)
    for leg, tgt in deltas.items():
        foot_target[leg] = tgt

def move_abad(deltas, duration=2.0, steps=100, label=None):
    """deltas: {leg: target_abad_angle_rad}. Ramps abad_cmd for those legs together."""
    starts = {leg: abad_cmd[leg] for leg in deltas}
    print_every = max(1, steps // 15)
    for i in range(1, steps + 1):
        if check_abort():
            return
        frac = i / steps
        for leg, target in deltas.items():
            s = starts[leg]
            abad_cmd[leg] = s + (target - s) * frac
        if label and (i % print_every == 0 or i == 1):
            print(f"    ({label} {frac*100:3.0f}%) pitch: {math.degrees(latest_pitch[0]):+.2f} deg  roll: {math.degrees(latest_roll[0]):+.2f} deg")
        time.sleep(duration / steps)
    for leg, target in deltas.items():
        abad_cmd[leg] = target

def watch(seconds, label, active_leg=None):
    """active_leg: if given, also checks/prints whether the body is inside the support triangle
       formed by the other three (planted) legs' feet - see support_status() above."""
    for _ in range(int(seconds / 0.1)):
        if check_abort():
            return
        fr_z = link_xyz.get("FR", (None, None, None))[2]
        vx, vy, vz = body_vel
        ax, ay, az = body_accel
        speed = math.sqrt(vx*vx + vy*vy)
        line = (f"  [{label}] pitch: {math.degrees(latest_pitch[0]):+.2f} deg  roll: {math.degrees(latest_roll[0]):+.2f} deg  "
                f"FR_shank_z: {fr_z}  body_xyz: {body_xyz}  vel(x,y,z): ({vx:+.3f},{vy:+.3f},{vz:+.3f})  speed_xy: {speed:.3f}  "
                f"accel(x,y,z): ({ax:+.3f},{ay:+.3f},{az:+.3f})"
                f"  pitch_rate({_pitch_rate_source[0]}): {math.degrees(latest_pitch_rate[0]):+.2f} deg/s"
                f"  theta(p={math.degrees(last_theta_terms['p']):+.2f} d={math.degrees(last_theta_terms['d']):+.2f} -> {math.degrees(last_theta_terms['clamped']):+.2f} deg)")
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
            speeds = stance_foot_speeds(active_leg)
            line += "  stance_foot_speed: {" + " ".join(f"{l}:{s:.3f}" for l, s in speeds.items()) + "}"
            vels = stance_foot_velocities(active_leg)
            line += "  stance_foot_vxy: {" + " ".join(f"{l}:({vx:+.3f},{vy:+.3f})" for l, (vx, vy) in vels.items()) + "}"
        fx_all = all_foot_fx()
        line += "  fx: {" + " ".join(f"{l}:{v:+.4f}" for l, v in fx_all.items()) + "}"
        print(line)
        time.sleep(0.1)

print("waiting for the drop to settle...")
time.sleep(5.0)
print(f"landed, pitch = {math.degrees(latest_pitch[0]):.2f} deg  roll = {math.degrees(latest_roll[0]):.2f} deg")
print(f"body position: {body_xyz}")

STANCE_FZ = -0.384
LIFT_FZ = -0.304
SHIFT_MAG_FRONT = 0.05
SHIFT_MAG_BACK = 0.03
FX_LIMIT = 0.09
SHIFT_DURATION_FRONT = 2.0
LIFT_FZ_FRONT = -0.34
LIFT_DURATION_FRONT = 1.5

# SWING/PUSH split front vs back - the last full-loop run showed PITCH (not roll) blowing up
# specifically during FR's place->push transition: that's the instant a front foot goes from
# swinging free to bearing load and getting dragged backward to actually propel the body, the
# highest-force moment in the whole step. Shrinking how far the front feet reach out and drag
# back gives that moment less leverage to tip the body forward, and slowing the transfer down
# gives the pitch correction more time to react. Back legs (BR) already stepped clean, untouched.
SWING_MAG_FRONT = 0.04       # was 0.06 shared
SWING_MAG_BACK = 0.06
# PUSH_MAG_FRONT halved again (0.02 -> 0.01): the FR_shank_z trace showed no foot-strike/contact
# spike - the foot tracks its commanded path smoothly the whole time, and pitch was already at
# ~-4 deg by the end of place (support down to just FL/BL/BR) before push even starts. Push then
# adds real propulsive torque - FR's foot dragged backward under full load, the only leg actively
# driving at that instant - on top of that already-reduced margin. Less push magnitude = less
# torque asked of that one corner. Trades some forward distance per step for stability margin.
PUSH_MAG_FRONT = 0.01        # was 0.02, before that 0.035 shared
PUSH_MAG_BACK = 0.035
PLACE_DURATION_FRONT = 3.0   # was 2.5
PUSH_DURATION_FRONT = 3.0    # was 2.5

# --- Raibert heuristic + capture-point swing placement (Cheetah 3 paper, eq. 6) -------------
# foot_target_x = nominal_swing_mag + CAPTURE_GAIN * (v_actual - v_desired)
# v_desired = 0 here: this is a discrete scripted step sequence, not a continuous target-speed
# walk, so the nominal SWING_MAG constants already encode the intended stride reach. This term
# only adds a correction on top - if the body is already carrying fore-aft momentum at the
# moment a leg commits to its swing target, reach that foot further out in the direction of
# travel so it lands under/ahead of the advancing CoM (a bigger effective base of support right
# when it's needed) instead of using a fixed blind reach regardless of how the body is moving.
#
# CAPTURE_GAIN = sqrt(height / gravity). Height = the settled crouch height (~0.40 m, from the
# landed body_xyz z in run_log.txt).
CAPTURE_HEIGHT = 0.40
CAPTURE_GAIN = math.sqrt(CAPTURE_HEIGHT / 9.8)   # ~0.202

# CAUTION: the sign relationship between foot_target's local fx and world-frame body_vel[0] has
# NOT been empirically verified yet (the robot never yaws in this gait, so world x ~= body x,
# but which direction is "positive fx" in the leg_ik/rotate convention vs which way is +x world
# hasn't been checked against real data). Starting at +1. The per-leg diagnostic print below
# shows v_x and the resulting swing_mag adjustment on every run - if the correction is making
# things worse (or the sign looks backwards relative to which way the body is actually pitching
# from momentum), flip this to -1.0 rather than assuming it's right.
CAPTURE_SIGN = 1.0
MAX_CAPTURE_CORRECTION = 0.03   # clamp, same units as swing_mag (m)

def capture_point_correction():
    """Sampled at the moment a leg commits to its swing target - returns a bounded fore-aft
       foot-offset correction based on the body's current (finite-differenced, ground-truth)
       velocity. See CAPTURE_GAIN/CAPTURE_SIGN above."""
    vx = body_vel[0]
    corr = CAPTURE_SIGN * CAPTURE_GAIN * vx
    return max(-MAX_CAPTURE_CORRECTION, min(MAX_CAPTURE_CORRECTION, corr))

def clamp(v):
    return max(-FX_LIMIT, min(FX_LIMIT, v))

def step_leg(leg, settle=0.3):
    """One full step cycle (shift/lift/swing/place/push) for a single leg. ABAD roll correction
       runs independently in the background control_loop the whole time. Checks check_abort()
       between every phase so a flip stops the sequence instead of grinding through the rest of
       the gait (and padding the log) on a robot that's already down."""
    others = [l for l in legs if l != leg]
    shift_sign = -1.0 if leg in ("BL", "BR") else 1.0
    is_front = leg in ("FL", "FR")
    shift_mag = SHIFT_MAG_FRONT if is_front else SHIFT_MAG_BACK
    shift_duration = SHIFT_DURATION_FRONT if is_front else 1.0
    lift_fz = LIFT_FZ_FRONT if is_front else LIFT_FZ
    lift_duration = LIFT_DURATION_FRONT if is_front else 0.5
    swing_duration = 1.0 if is_front else 0.5
    place_duration = PLACE_DURATION_FRONT if is_front else 0.5
    push_duration = PUSH_DURATION_FRONT if is_front else 0.5
    swing_mag_nominal = SWING_MAG_FRONT if is_front else SWING_MAG_BACK
    push_mag = PUSH_MAG_FRONT if is_front else PUSH_MAG_BACK

    if check_abort(): return
    print(f"--- {leg}: shift ---")
    move_feet({l: (clamp(foot_target[l][0] + shift_sign * shift_mag), STANCE_FZ) for l in others}, duration=shift_duration, label=f"{leg} shift")
    watch(settle, f"{leg} shift", active_leg=leg)

    if check_abort(): return
    print(f"--- {leg}: lift ---")
    move_feet({leg: (foot_target[leg][0], lift_fz)}, duration=lift_duration, label=f"{leg} lift", active_leg=leg)
    watch(settle, f"{leg} lift", active_leg=leg)

    if check_abort(): return
    capture_corr = capture_point_correction()
    swing_mag = swing_mag_nominal + capture_corr
    print(f"--- {leg}: swing (v_x={body_vel[0]:+.3f}  capture_corr={capture_corr:+.4f}  "
          f"swing_mag {swing_mag_nominal:.3f} -> {swing_mag:.3f}) ---")
    move_feet({leg: (swing_mag, lift_fz)}, duration=swing_duration, label=f"{leg} swing", active_leg=leg)
    watch(settle, f"{leg} swing", active_leg=leg)

    if check_abort(): return
    print(f"--- {leg}: place ---")
    move_feet({leg: (swing_mag, STANCE_FZ)}, duration=place_duration, label=f"{leg} place", active_leg=leg)
    watch(settle, f"{leg} place", active_leg=leg)

    if check_abort(): return
    print(f"--- {leg}: push ---")
    move_feet({leg: (swing_mag - push_mag, STANCE_FZ)}, duration=push_duration, label=f"{leg} push", active_leg=leg)
    watch(settle, f"{leg} push", active_leg=leg)

print("--- crouch (hip/knee only, all ABAD at 0) ---")
move_feet({leg: (0.0, STANCE_FZ) for leg in legs}, duration=1.5)
watch(2.0, "crouch")

# ============================================================================
# FULL 4-LEG GAIT LOOP - the real test. The single-leg BR step (with the axis
# bug fixed and ABAD roll correction active) survived cleanly: roll peaked at
# +2.86 deg during lift/swing but the correction pulled it back to +0.20 deg
# by the end of the step, instead of accumulating like it used to with the
# old differential-foot-height hack. This is exactly the scenario that used
# to kill the full loop (residual roll from BR's step compounding into FR's
# push in cycle 1) - so this is the test that actually matters.
# ============================================================================
start_xyz = list(body_xyz)
GAIT_ORDER = ["BR", "FR", "BL", "FL"]
N_CYCLES = 2
INTER_LEG_SETTLE = 1.5   # NEW: extra pause between legs so pitch/roll correction can decay the
                          # residual disturbance before the next leg's demanding phases start -
                          # the last run showed FR's place/push failing specifically because it
                          # started from BR's leftover +2.4deg pitch instead of a flat baseline
for cycle in range(N_CYCLES):
    if aborted[0]:
        print(f"--- gait loop stopped early before cycle {cycle+1}: safety abort triggered ---")
        break
    print(f"=== cycle {cycle+1} ===")
    for leg in GAIT_ORDER:
        if aborted[0]:
            break
        step_leg(leg)
        watch(INTER_LEG_SETTLE, f"settle after {leg}")

end_xyz = list(body_xyz)
dx = end_xyz[0] - start_xyz[0]
dy = end_xyz[1] - start_xyz[1]
dist = math.sqrt(dx*dx + dy*dy)
print(f"net displacement: dx={dx:.3f} dy={dy:.3f}  total distance={dist:.3f} m")
print(f"final body z: {end_xyz[2]:.3f}  (collapsed if well below ~0.35)")
watch(2.0, "end of loop")

print("sequence stopped early (safety abort)" if aborted[0] else "sequence complete")
running[0] = False
