"""
Script: leg_odometry.py
Author: Arda Gencer
Shared state estimator: replaces ground-truth body position/velocity (previously read straight
from Gazebo's privileged /world/empty/pose/info topic) with an estimate built the way a real
robot would have to build it - from joint angles (via forward kinematics on whichever leg(s) are
currently in stance, assumed fixed to the ground) plus IMU orientation. Still noise-free (the IMU
sensor in model.sdf has no <noise> block, and the new joint_state topic added below carries no
sensor model either) - this is about not cheating with privileged simulator state, not yet about
robustness to real sensor imperfection.

Forward kinematics below is the algebraic inverse of leg_ik/leg_ik_3d (as used in champgait.py /
trot_demo.py / etc.), verified by round-trip testing against the existing IK over 50,000 random
physically-reachable targets (max error ~1e-16, i.e. floating-point noise) before being used here.
One real bug was caught and fixed during that verification: the "along-AH" identity is oy*D_ABAD,
not plain D_ABAD - the initial derivation missed the left/right sign flip.

Requires a joint_state_publisher plugin in model.sdf (not present before - added alongside this
file) so joint angles are actually published; previously nothing published real joint feedback at
all, and every gait file was a pure open-loop position commander with zero encoder read-back.
"""

import math
import gz.transport13 as transport
from gz.msgs10.model_pb2 import Model


def leg_fk(hip, knee, s, L1, L2):
    """Inverse of leg_ik(fx, fz, s) -> (hip, knee). Returns (fx, fz) relative to the hip pivot,
    in the leg's own 2-link plane. Verified by round-trip test against leg_ik - see module
    docstring."""
    k1 = L1 + L2 * math.cos(knee)
    k2 = L2 * math.sin(knee)
    u = k1 * math.sin(hip) + k2 * math.cos(hip)
    w = k2 * math.sin(hip) - k1 * math.cos(hip)
    return s * u, w


def leg_fk_3d(abad, hip, knee, oy, s, L1, L2, D_ABAD):
    """Inverse of leg_ik_3d(fx, fy, fz, oy, s) -> (abad, hip, knee). Returns (fx, fy, fz) relative
    to the leg's ABAD pivot, in body-frame axes - same convention leg_ik_3d's docstring uses.

    Derivation: AH (the ABAD pivot to hip pivot link) has fixed length D_ABAD and is oriented at
    angle 'abad' in the same (dy, dz) polar convention leg_ik_3d itself uses (dy = oy*D_ABAD + fy,
    dz = fz). Because the 2-link (hip/knee) mechanism only ever moves the foot perpendicular to
    AH, the foot's projection *along* AH is always exactly oy*D_ABAD (same as H's own projection,
    NOT plain D_ABAD - the oy sign flip caught by round-trip testing, see module docstring), while
    its projection perpendicular to AH is exactly leg_fk's 'w'. Un-rotating that (along, w) pair by
    -abad recovers (dy, dz), and fy = dy - oy*D_ABAD undoes the same offset leg_ik_3d added."""
    fx, w = leg_fk(hip, knee, s, L1, L2)
    dy = oy * D_ABAD * math.cos(abad) - w * math.sin(abad)
    dz = oy * D_ABAD * math.sin(abad) + w * math.cos(abad)
    fy = dy - oy * D_ABAD
    fz = dz
    return fx, fy, fz


def rotate_body_to_world(dx, dy, dz, pitch, roll):
    """Same convention as the _rotate_body_to_world helper already used for ground-truth foot
    tracking in champgait.py - rotates a body-frame vector into world frame, assuming yaw=0 (no
    absolute heading reference without a magnetometer or external tracking)."""
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    wx = cp * dx + sp * sr * dy + sp * cr * dz
    wy = cr * dy - sr * dz
    wz = -sp * dx + cp * sr * dy + cp * cr * dz
    return wx, wy, wz


class LegOdometryEstimator:
    """Drop-in-ish replacement for the ground-truth body_xyz / body_vel / body_accel state each
    gait file previously read from /world/empty/pose/info. Subscribes to the model's joint_state
    topic (added via the joint_state_publisher plugin) for real per-joint angles, and is fed
    pitch/roll from the caller's existing IMU callback each tick (no separate IMU subscription
    here, to avoid a second subscriber racing the caller's own).

    Height (z) is computed directly from stance-leg kinematics each tick (assumes flat ground at
    world z=0 under whichever foot/feet are in stance) - NOT integrated, so it does not drift.
    Horizontal position (x, y) has no such reference and is obtained by integrating the estimated
    velocity over time, exactly like a real robot without GPS or motion capture - it WILL drift
    over a long run. That is the expected, physically honest tradeoff of dropping ground truth,
    not a bug."""

    def __init__(self, model_name, world_name, legs, oy_map, side_map, hip_offset_map,
                 L1, L2, D_ABAD, control_dt):
        self.legs = legs
        self.oy_map = oy_map            # {leg: +1/-1} left/right, same as OY elsewhere
        self.side_map = side_map        # {leg: +1/-1} front/back, same as LEG_SIDE elsewhere
        self.hip_offset_map = hip_offset_map   # {leg: (x0, y0)} nominal ABAD-pivot offset from body origin
        self.L1 = L1
        self.L2 = L2
        self.D_ABAD = D_ABAD
        self.control_dt = control_dt

        self._latest_joint_pos = {}     # {joint_full_name: angle_rad}, updated by _joint_state_cb
        node = transport.Node()
        topic = f"/world/{world_name}/model/{model_name}/joint_state"
        node.subscribe(Model, topic, self._joint_state_cb)
        self._node = node   # keep a reference so the subscription isn't garbage-collected

        self.body_xyz = [0.0, 0.0, 0.0]         # estimate: x,y integrated (will drift), z direct (won't)
        self.body_vel = [0.0, 0.0, 0.0]
        self.body_accel = [0.0, 0.0, 0.0]
        self._prev_body_vel = [0.0, 0.0, 0.0]
        self._initialized_z = False

        self._prev_foot_body = {}       # {leg: (fx,fy,fz)} last tick's stance-leg foot position, body frame
        self._prev_update_time = None   # wall-clock time of the last update() call, for real dt

    def _joint_state_cb(self, msg):
        for j in msg.joint:   # each joint entry in this model's joint_state message
            self._latest_joint_pos[j.name] = j.axis1.position

    def get_leg_angles(self, leg):
        """(abad, hip, knee) for one leg from the latest joint_state message, or None if we
        haven't received a reading for all three joints of this leg yet."""
        try:
            abad = self._latest_joint_pos[f"{leg}_ABAD"]
            hip = self._latest_joint_pos[f"{leg}_HIP"]
            knee = self._latest_joint_pos[f"{leg}_KNEE"]
        except KeyError:
            return None
        return abad, hip, knee

    def foot_body_frame(self, leg):
        """This leg's current foot position relative to the body origin (ABAD-pivot-relative FK
        result plus that leg's nominal hip offset), or None if joint angles aren't available yet."""
        angles = self.get_leg_angles(leg)
        if angles is None:
            return None
        abad, hip, knee = angles
        fx, fy, fz = leg_fk_3d(abad, hip, knee, self.oy_map[leg], self.side_map[leg],
                                self.L1, self.L2, self.D_ABAD)
        x0, y0 = self.hip_offset_map[leg]
        return (fx + x0, fy + y0, fz)

    def update(self, now, pitch, roll, stance_legs):
        """Call once per control tick. stance_legs: iterable of leg names currently planted
        (i.e. NOT mid-swing) - the caller already tracks this for its own gait logic."""
        if self._prev_update_time is None:
            dt = self.control_dt   # first call - no previous timestamp to measure against
        else:
            dt = now - self._prev_update_time
            if dt <= 0.0:   # clock didn't advance (or went backwards) - fall back rather than divide badly
                dt = self.control_dt
        self._prev_update_time = now

        stance_legs = list(stance_legs)
        vel_samples = []    # world-frame (vx,vy,vz) estimate from each usable stance leg this tick
        z_samples = []       # world-frame height estimate from each stance leg this tick

        this_tick_foot_body = {}   # {leg: (fx,fy,fz)} computed once per leg this tick, reused below
        for leg in stance_legs:
            foot_now = self.foot_body_frame(leg)
            if foot_now is None:
                continue
            this_tick_foot_body[leg] = foot_now

            # height: direct from this tick's kinematics, not integrated - see class docstring
            _wx, _wy, wz = rotate_body_to_world(*foot_now, pitch=pitch, roll=roll)
            z_samples.append(-wz)

            # velocity: only if this leg was ALSO in stance last tick (need a previous sample to
            # difference against - a leg that just touched down has no valid previous reading)
            prev = self._prev_foot_body.get(leg)
            if prev is not None:
                dfx = (foot_now[0] - prev[0]) / dt
                dfy = (foot_now[1] - prev[1]) / dt
                dfz = (foot_now[2] - prev[2]) / dt
                wvx, wvy, wvz = rotate_body_to_world(dfx, dfy, dfz, pitch=pitch, roll=roll)
                # a planted foot is fixed in the world, so if it appears to move by (wvx,wvy,wvz)
                # in the body-frame-derived estimate, that's actually the body moving by -that,
                # same sign logic champgait.py's v6.2 fix already established for body_shift
                vel_samples.append((-wvx, -wvy, -wvz))

        # reset previous-foot tracking to this tick's stance set (legs that left stance since last
        # tick get dropped so they don't produce a stale finite-difference next time they land)
        self._prev_foot_body = this_tick_foot_body

        if z_samples:
            self.body_xyz[2] = sum(z_samples) / len(z_samples)
            self._initialized_z = True

        if vel_samples:
            vx = sum(v[0] for v in vel_samples) / len(vel_samples)
            vy = sum(v[1] for v in vel_samples) / len(vel_samples)
            vz = sum(v[2] for v in vel_samples) / len(vel_samples)

            self.body_accel[0] = (vx - self._prev_body_vel[0]) / dt
            self.body_accel[1] = (vy - self._prev_body_vel[1]) / dt
            self.body_accel[2] = (vz - self._prev_body_vel[2]) / dt
            self._prev_body_vel = [vx, vy, vz]

            self.body_vel[0], self.body_vel[1], self.body_vel[2] = vx, vy, vz
            # xy position: integrated, will drift - see class docstring
            self.body_xyz[0] += vx * dt
            self.body_xyz[1] += vy * dt
        else:
            # no usable stance leg this tick (e.g. all four mid-swing, or just after startup
            # before any joint_state message has arrived) - hold last known velocity's integration
            # rather than silently zeroing it out
            pass
