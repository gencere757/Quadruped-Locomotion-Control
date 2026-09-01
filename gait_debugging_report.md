# Quadruped Gait Debugging — Full Process Report

This document walks through the motion-control work on two gait controllers running in
Gazebo: a crawl gait (`champgait_wave.py`) and a trot gait (`trot_demo.py`). It starts with
the underlying concepts each controller relies on — written for anyone, not just someone who
already knows legged-robot terminology — then walks through what each version changed, why,
and what evidence justified it. The guiding discipline throughout was: **change one variable
at a time, and only after a failure gives a specific, falsifiable reason to suspect that
variable.**

---

## Part 1 — Core concepts

### Center of mass, and why a robot tips over

Every physical object has a center of mass (CoM): the single point where, if you could
balance the whole object on a fingertip, it would balance perfectly. For a robot standing on
some of its feet, whether it stays upright comes down to a simple geometric test. Look
straight down from above and draw the outline connecting every foot currently touching the
ground — this is called the **support polygon**. As long as the CoM's shadow on the ground
falls inside that outline, gravity is pulling down on a point that has something underneath it
to push back, and the robot stays up. If the shadow falls outside the outline, there's nothing
under it, and the robot tips over in that direction — the same way a table tips over if you
push down outside the footprint of its legs.

A four-legged robot standing on all four feet has a support polygon shaped roughly like a
rectangle, and its CoM normally sits comfortably near the middle, with a lot of margin on
every side. A **crawl gait** deliberately lifts only one leg at a time, so it always has at
least three feet on the ground — this makes it "statically stable" in principle, meaning it
could freeze at any instant and not fall. But the moment one foot lifts, the support polygon
shrinks from that four-cornered rectangle down to a three-cornered triangle: the corner
belonging to the lifted leg disappears. If the CoM was sitting where the rectangle's center
used to be, it can easily end up outside — or dangerously close to the edge of — the new,
smaller triangle, tipping the robot toward the missing corner.

So every crawl-gait controller has to solve the same problem: shift the robot's weight (its
CoM) away from wherever the next leg is about to lift, into the middle of the triangle the
*remaining* three feet will form, before or while that leg is in the air. This is the same
balancing act as a person shifting their hips over one leg before lifting the other foot to
take a step.

### Discrete weight-shifting vs. continuous CoM sway

There are two broad ways to implement that weight shift.

The straightforward, classic approach does it in clearly separated phases: a **shift** phase
first (move the CoM into position while standing still), then a **swing** phase (lift and move
the leg forward while the body holds its shifted position steady), then repeat for the next
leg. Watched from the outside, this looks like a distinct rock-then-step-then-rock-then-step
motion — the body visibly pauses to lean before each step.

`champgait_wave.py` uses the other approach: **continuous CoM sway**. Instead of snapping into
a shifted position and holding it, the body's lean is computed as a smooth, continuously
varying function of where the gait currently is in its cycle. It flows gradually from side to
side and front to back throughout the *entire* walking motion, timed so that by the moment any
given leg needs to lift, the body has already gently swayed enough weight onto the other
three. There's no separate "stand still and shift" step — the lean rides along with the
walking motion the whole time, like a continuous rocking rhythm rather than a series of
discrete weight-shift-then-freeze maneuvers.

**The math — driving the sway with a cosine.** Writing `t` for elapsed time, `T` for the full
four-leg gait period, and `A` for the sway's peak amplitude, the lateral (side-to-side) part of
the lean is:

```
y_com(t) = -A · cos(4πt / T)
```

The argument completes a full 2π cycle every `T/2`, not every `T`, because this robot's legs
lift in a right/left/right/left order — the body only needs to lean fully to one side and back
*twice* per full cycle (once for the pair of right legs, once for the pair of left legs), not
four separate times. Choosing the phase so the cosine peaks (value `+A` or `-A`) at exactly the
instant a leg lifts, rather than partway through its swing, means the body is already fully
leaned the moment it's needed, instead of still catching up.

The fore-aft (front/back) part of the lean, by contrast, is **not** a smooth sine wave, because
this robot's actual lift order (back, front, front, back) doesn't alternate as cleanly as
left/right does — the two back legs' lifts sit adjacent in the cycle, and so do the two front
legs'. It's a genuine two-state function instead: hold a constant forward lean while the back
legs swing, hold a constant backward lean while the front legs swing, and smoothly blend
between the two during the brief all-four-planted windows in between, using a smoothstep curve

```
s(u) = u² (3 - 2u),   u ∈ [0, 1]
```

which — unlike a straight linear ramp — has zero slope at both ends, so the lean's rate of
change never jumps abruptly at the start or end of a transition.

**Worked example — sizing the lean from the actual support triangle.** Each leg's hip sits at a
fixed, known offset from the body's center (the back-right leg's hip, for instance, is 0.15 m
behind and 0.213 m to the right of center). At the instant a given leg lifts, the *actual*
body-frame centroid of the three remaining planted feet can be computed directly — average each
of their (hip offset + current commanded foot position) coordinates. Working this out for this
robot's real geometry and step lengths gives a centroid roughly **0.058 m forward** of body
center at the moment the back-right leg lifts, which makes physical sense: losing a back leg
leaves three feet whose average position sits further forward.

Here's the subtlety that caused a real, shipped bug: since a planted foot is fixed to the
ground, commanding a *larger forward* foot-offset doesn't move that foot forward — by the same
planted-foot principle discussed under yaw correction below, it pushes the **body** backward
instead. So to actually align the body's center with that +0.058 m centroid, the correct
command is `fx = -0.058`, not `+0.058`. The first version of this lean got exactly that sign
backwards, and it showed up unambiguously in the flight data: the body was recorded sliding
backward at precisely the moments it should have been sliding forward.

**The math — testing a point against the support triangle.** Whether the CoM (or another
balance point, below) is safely inside the support triangle isn't decided by computing the
triangle's centroid and measuring distance to it — it's decided with a standard
computational-geometry test. Call the three planted feet's ground positions `v1`, `v2`, `v3`,
and the point being tested `p`. For each edge of the triangle, compute the signed area of the
triangle formed by that edge and `p`:

```
sign(p, a, b) = (p.x - b.x)(a.y - b.y) - (a.x - b.x)(p.y - b.y)
```

This is twice the z-component of the cross product between the vectors from `b` to `p` and
from `b` to `a` — its sign says which side of the line through `a` and `b` the point falls on.
Doing this once per edge (`v1v2`, `v2v3`, `v3v1`) gives three numbers; `p` is inside the
triangle exactly when all three share the same sign — it's on the "inward" side of every edge
at once.

The *margin* — how far from actually tipping over the point is — comes from the perpendicular
distance to the nearest edge, not just the sign test. For an edge from `a` to `b`, with edge
vector `e = b - a` and displacement `d = p - a`:

```
distance = (d.x · e.y - d.y · e.x) / |e|
```

again a cross product, this time divided by the edge's length to turn it into an actual
distance in meters. The reported stability margin is the smallest of the three edges' absolute
distances.

Two different points get run through this same triangle test, not just the raw CoM position,
because a *moving* robot's balance depends on where it's heading and accelerating, not just
where its mass currently sits:

- **Zero Moment Point (ZMP).** With `(x, y, z)` the CoM's position and `(ẍ, ÿ, z̈)` its measured
  acceleration, and `g = 9.8 m/s²`:

  ```
  ZMP_x = x - (ẍ / (z̈ + g)) · z          ZMP_y = y - (ÿ / (z̈ + g)) · z
  ```

  This is the standard single-rigid-body ZMP formula: the ground point about which the net
  reaction moment is zero shifts *opposite* to horizontal acceleration, by an amount that
  scales with CoM height. Lean or accelerate forward, and the effective tipping point moves
  backward relative to the CoM's actual position — the inertial reaction to that acceleration
  acts like an extra force dragging the effective balance point the other way.
- **Capture point.** With `(v_x, v_y)` the CoM's horizontal velocity and `h` its height:

  ```
  CP_x = x + v_x · √(h / g)          CP_y = y + v_y · √(h / g)
  ```

  `√(h/g)` is the characteristic time constant of an inverted pendulum balanced on a single
  point — the same math describing how fast a broom balanced on your palm tips over. The
  capture point answers "if I had to plant a foot right now to bring my current momentum to a
  dead stop, where would it need to be?" — projecting the CoM forward by however far its
  current velocity would carry it before that pendulum dynamic naturally arrests it.

The appeal is that this can look and move more fluidly, without the robot pausing to perform a
separate shifting maneuver. The cost is that it's more delicate to tune: because the lean is
spread out over a longer window instead of a short, discrete shift, *sustaining* a given amount
of lean for longer can build up more tipping momentum than the same peak lean held only
briefly. This is exactly what the first real fix to this file addressed (see `v6` below) — a
lean amplitude that was correctly sized for a brief, discrete shift turned out to be too much
once it was instead sustained continuously over this gait's longer swing window.

### Yaw, and why it drifts

**Yaw** is rotation around the vertical axis — the robot's heading. A robot walking with a
perfectly symmetric gait would walk in a straight line forever; in practice, small asymmetries
(slightly uneven friction under different feet, a tiny timing offset between legs, sensor
noise, imperfect leg geometry) mean the robot gradually rotates off its intended heading even
while trying to go straight — the legged-robot equivalent of a shopping cart with one wheel
that pulls to one side. Both gaits here measure their actual yaw using the simulated robot's
IMU (an inertial sensor that reports orientation and rotation rate, the same kind of sensor
that lets a phone know which way it's tilted), compare it against where the robot was heading
when the gait started, and feed that error into a correction.

### PID feedback control, in plain terms

Both gaits correct yaw using some combination of three classic feedback terms:

- **Proportional (P):** the correction is proportional to the *current* error — bigger drift,
  bigger correction. On its own, a P-term has a structural weakness: against a *constant*
  disturbance, it settles into a permanent nonzero error rather than ever fully cancelling it,
  because the correction shrinks to zero exactly when the error reaches zero, so there's
  nothing left pushing back against a steady bias.
- **Derivative (D):** the correction reacts to how *fast* the error is changing, meant to add
  damping and prevent overshoot. Its weakness is that it reacts just as strongly to noise as to
  real signal — a sensor's noisy, rapidly-fluctuating readings can trigger a D-term even when
  there's no real error to correct, turning noise into an active disturbance.
- **Integral (I):** the correction accumulates the error over time, which is exactly what's
  needed to cancel a small but persistent one-directional bias that a P-term alone can never
  fully remove. Its weakness is **windup**: if the accumulated error isn't capped, it can grow
  very large during a long period of uncorrected drift, then unleash an oversized, laggy
  overcorrection once it finally does act.

**The math.** Writing `e(t)` for the yaw error (measured yaw minus the yaw the robot started
the gait with), the general PID correction is

```
u(t) = Kp·e(t) + Ki·∫e(t)dt + Kd·(de/dt)
```

In code, the integral is accumulated one control tick at a time (`∫e dt ≈ Σ e·Δt`, with `Δt`
the control loop's fixed timestep) and clamped to a maximum magnitude to prevent windup; the
derivative term is applied to the robot's directly-measured yaw *rate* from its gyroscope
rather than a numerically-differentiated error, avoiding an extra layer of noise on top of an
already noisy signal.

The crawl gait uses P + D plus a small constant feedforward bias, clamped to a maximum output:

```
u = clamp( Kp·e + Kd·ω + b,  ±u_max )
```

where `ω` is the measured yaw rate and `b` is the fixed feedforward nudge (see `v10` below).
The trot gait instead uses P + I (no D), with the integral clamped separately before being
folded in:

```
I ← clamp( I + e·Δt,  ±I_max )
signal = e + Ki·I
u = clamp( K·signal,  ±u_max )
```

— and, notably, that same combined `signal` drives *two* separate output channels in the trot
gait (a fore-aft correction and a lateral one), each with its own gain and its own output
clamp.

### The "planted foot vs. swinging foot" principle

This turned out to be the single most important mechanical idea reused across both files.
When a robot's foot is on the ground bearing weight (**planted**, in "stance"), commanding
that foot's target position doesn't move the foot — friction and the robot's own weight anchor
it in place — it moves the **body** instead, because the leg acts as a lever pushing against
fixed ground. When a foot is lifted in the air (**swinging**, in "swing"), commanding its
target position only decides where that foot will land next; it has no immediate effect on the
body, because a foot with nothing under it can't generate any reaction force yet.

This matters for yaw correction specifically: a small steering nudge is safe to apply to a
*swinging* foot — worst case, it lands slightly off from ideal, which is easy to correct on the
next step. The same nudge applied to a *planted* foot creates a real, immediate force on the
whole body, which can be far more disruptive than intended, especially if it's applied
asymmetrically across multiple planted feet at once.

### Duty cycle and stride length, for the trot gait

A trot alternates two diagonal pairs of legs (front-left + back-right, then front-right +
back-left). The **duty cycle** (`STANCE_DUTY`) is the fraction of each stride a given leg
spends planted versus swinging. At a duty cycle above 0.5, there are brief windows where
*both* pairs are planted at once (extra stability margin, but "dead time" where nothing is
actively driving the stride forward); at exactly 0.5, one pair is always mid-swing with no
gap at all; below 0.5 would require an actual moment where *no* foot touches the ground (a
flight phase, as in a running gait).

The nominal **stride length** — how far a foot sweeps during its stance phase — is derived
directly from the target walking speed: over the time a foot is planted, the body needs to
travel exactly that distance for the gait to be consistent, so a faster target speed or a
longer stance duration both call for a longer stride.

**The math.** If a foot is planted for a stance phase lasting `STANCE_DUTY · T` seconds
(`T` = the full stride period), and the body needs to travel forward at speed `v` during that
time, the foot — fixed to the ground — must appear, in the body's own reference frame, to sweep
backward by exactly `v · STANCE_DUTY · T` meters over that same window (the foot isn't moving
in the world; the body is moving out from under it). Splitting that sweep symmetrically around
the hip — forward at touchdown, backward at liftoff — gives a half-amplitude of

```
A = ½ · v · STANCE_DUTY · T
```

which is exactly the formula this file uses to size its nominal stride from the target speed,
duty cycle, and period.

### Raibert-style speed regulation

A gait can drive its stride from a *fixed* target speed (open-loop — it always commands the
same nominal stride regardless of what the robot is actually doing), or it can measure the
robot's actual speed and adjust the stride length in response (closed-loop). The classic
approach for the latter, from Marc Raibert's hopping-robot work in the 1980s, adjusts where a
foot touches down based on the *error* between measured and desired speed: too fast, and
placing the foot further forward creates more braking on the next stance phase; too slow, and
placing it less far forward (or trailing) lets the next stance phase accelerate the body more.
The key design detail is that this correction should be built around the *error*, layered on
top of the *target* speed's nominal stride — not around the *measured* speed directly (see
`trot_demo.py`'s `v13`/`v14` below for what goes wrong if it's built the second way).

**The math, and exactly where the trot gait's implementation went wrong.** A correct
Raibert-style correction keeps the target-speed-based nominal amplitude (the `A` formula just
above, evaluated at the *desired* speed) as its base, and adds only a small term proportional
to the *velocity error* on top:

```
A_corrected = A_nominal(v_desired) + k · (v_measured - v_desired)
```

The trot gait's implementation instead rebuilt the entire nominal term from the *measured*
speed:

```
A_dyn = ½ · v_measured · STANCE_DUTY · T + k · (v_measured - v_desired)
```

— substituting `v_measured` for `v_desired` in the dominant first term, not just in the
intended correction term. Since that first term's coefficient (`½ · STANCE_DUTY · T ≈ 0.4`) is
roughly ten times larger than the correction gain `k` (`0.03`), the formula is overwhelmingly
dominated by "recompute the stride from whatever speed the robot happens to already be doing,"
with only a token nudge from the actual error term — which is exactly why turning it on made
the existing speed shortfall worse instead of correcting it.

---

## Part 2 — The crawl gait (`champgait_wave.py`)

This file is a fork of a separately-maintained, stable crawl-gait controller
(`champgait.py`, never modified). It uses continuous CoM sway (see Part 1) rather than the
discrete weight-shift the original crawl gait uses, which is what made it more prone to falls
and yaw drift.

### v6 — the stability breakthrough
**Change:** `X_LEAN_AMPLITUDE` (the CoM sway's peak lean) reduced from `0.05` to `0.035`.
**Why:** 0.05 matched the *static* centroid shift needed for stability, but pitch kept
climbing over this gait's longer swing window — as explained in Part 1, sustaining that much
lean continuously over a longer window builds up more tipping momentum than the same lean held
briefly in a discrete shift.
**Result:** First fully successful run — all 3 gait cycles completed, no fall.

### v7 — a regression, and a lesson
**Change:** Doubled all three yaw-correction gains at once — `YAW_FX_GAIN` (0.03→0.06,
the P-term), `YAW_RATE_DAMPING` (0.02→0.04, the D-term), and `MAX_YAW_FX` (0.025→0.05, the
correction's ceiling).
**Result:** Two consecutive falls (pitch aborts at -25.1° and -26.6°), and yaw drift got
*worse* (+30.2° on the second run).
**Diagnosis:** The D-term reacts to noisy gyro readings even with no real error to correct
(see Part 1); doubling it amplified sensor noise into a destabilizing disturbance. Changing
three variables at once made this hard to isolate at first, which became the standing rule for
the rest of both files: **one variable per test.**

### v8 — revert, and re-baseline
**Change:** Reverted to the original v6 gains.
**Result:** Five consecutive clean, stable runs, with yaw drift consistently positive:
+9.9°, +20.7°, +12.7°, +22.5°, +16.6°.

### v9 — isolating the noise-sensitive term
**Change:** `YAW_RATE_DAMPING` (the D-term) set to `0.0`, fully disabled. `YAW_FX_GAIN` (the
P-term) raised only 1.5× (to 0.045, not 2×), with `MAX_YAW_FX` scaled proportionally.
**Why:** Since v8's runs all drifted the *same* direction, more P-gain — not more D-gain — was
the reasonable next single variable, with the noise-sensitive term removed rather than scaled.
**Result:** Three stable runs, yaw drift +17.9°, +8.3°, +12.4° — better, still positive.

### v10 — accounting for a constant bias
**Change:** Added `YAW_FEEDFORWARD_BIAS = -0.005`, a small constant offset applied alongside
the P and D terms.
**Why:** All eight runs so far drifted in the same direction — as explained in Part 1, a P-term
alone mathematically cannot fully cancel a *constant* bias, it settles at a permanent nonzero
error. A small fixed feedforward nudge, tuned against the observed average drift, is the
standard fix for exactly that failure mode.

### v11 — the key structural insight
**Change:** Yaw correction used to be zeroed out for *every* leg whenever *any* leg was
mid-swing. Restructured so it only zeroes out for the *planted* legs — the currently swinging
leg (or all legs, when none are swinging) still gets the full correction.
**Why:** This is a direct application of the planted-vs-swinging principle from Part 1: pushing
a swinging foot's landing target costs nothing, since nothing is anchored there yet. The old
code was suppressing correction on a leg that had nothing to lose by receiving it.
**Result:** This distinction turned out to be reusable almost verbatim in the trot gait's own
design (see Part 3).

### v12 — instrumentation, not logic
**Change:** Added a single log line printing `start_yaw` — the absolute yaw right after the
world reset, before any gait logic runs at all.
**Why:** Two consecutive v11 runs showed a confusing ~20° mismatch between the "net yaw
turned" metric (measured relative to a start-of-run baseline) and the absolute final yaw
printed at the end of the log — this looked at first like a bug v11 might have introduced.
**Result:** The new logging showed `start_yaw` was itself around -21° to -22° on the affected
runs — the robot wasn't spawning at a consistent heading after a world reset. This was a
pre-existing spawn/reset quirk, unrelated to v11's correctness, and it confirmed the "net yaw
turned" (relative) metric had been correct the whole time.

### v13 — fixing the actual spawn-yaw variance
**Change:** The control loop used to start commanding joints immediately after being created,
*before* the robot had finished physically dropping and settling onto the ground. Moved that
start to *after* the drop-settle wait instead.
**Why:** During the chaotic physical drop, the control loop was already actively holding all
four legs at a fixed target. Whichever foot happened to touch down a few milliseconds before
the others would receive an outsized reaction torque relative to the rest — a plausible,
physically real source of an uncontrolled yaw twist picked up before the gait logic ever issued
a single deliberate command.
**Result:** The very next run showed `start_yaw` of only -2.53° (versus the earlier -21° to
-22° swings), and completed cleanly with `net yaw turned = +7.4°` — strong evidence this was
the real source of the spawn-yaw non-determinism.

---

## Part 3 — The trot gait (`trot_demo.py`)

This is a separate, from-scratch controller using a classic diagonal-pair trot (front-left +
back-right together, front-right + back-left together — see Part 1's duty-cycle explanation).
Two of the crawl gait's hard-won lessons were deliberately carried over into its design from
the start: the delayed control-loop start (crawl gait's `v13`, above), and the
planted-vs-swinging correction principle (crawl gait's `v11`, above), generalized here to a
diagonal *pair* instead of a single leg.

### Starting point
An integral-windup bug (see Part 1's PID explanation) had already been found and partially
fixed before this: a `YAW_KI=0.3` integral term had been added to the yaw correction, but its
cap (`YAW_INTEGRAL_LIMIT`) was set to `2.0`, letting the integral term's contribution reach
roughly 34° — bigger than most of the raw yaw errors it was meant to correct. That cap had
been reduced to `0.3` (capping the contribution at about 5°), and correction was already
restricted to the actively swinging pair only. Yaw was still drifting despite that fix.

### v7 — a sign flip, following the code's own diagnostic
**Finding:** A run completed without falling, but with `net yaw turned = +62.5°`, and the log
showed both correction terms pinned at their positive maximum continuously for 5+ seconds
while yaw climbed the entire time, instead of shrinking.
**Why this pointed at a sign bug:** Full corrective authority applied continuously in what
should have been the opposing direction, with yaw accelerating anyway, is the signature of
**positive feedback** — the correction reinforcing the error instead of opposing it — not
insufficient gain (more gain on a backwards sign only makes it worse, faster).
**Change:** Flipped the sign of the lateral steering gain (`YAW_FY_GAIN`, +0.07 → -0.07).
Left the other correction term untouched.

### v8 — the actual root cause
**Finding:** Two runs under v7 still didn't converge — one swung to a sustained -42° error
(integral pinned at its clamp for 3+ seconds) before landing at -19.6° net yaw; the very next
run, on *identical* code, landed at +37.9° instead. That much variance on unchanged code meant
the problem was bigger than a sign error.
**Diagnosis:** The correction-gating logic applied the correction to *all four legs*
simultaneously whenever both diagonal pairs were planted at once — which happens on roughly a
third of every cycle's samples. Per Part 1's planted-vs-swinging principle, that's a
fundamentally different (and far more disruptive) mechanism than nudging an unloaded swinging
foot: with all four feet planted, the correction pushes directly against the ground and moves
the body, using a steering-direction convention that had only ever been validated for the
crawl gait's leg geometry and timing.
**Change:** Correction now only ever applies to a leg that is actually mid-swing — the
double-stance case was dropped entirely.
**Result:** Confirmed clean and repeatable across nine consecutive runs — net yaw consistently
within roughly ±12° (several single-digit, alternating sign, no runaway, no falls). This is
the fix that actually resolved the trot gait's yaw drift.

### v9 — trimming the double-support "dead time"
**Context:** With yaw stable, average forward speed was consistently only ~50% of target, and
visually the gait looked like it was stepping, pausing, stepping again.
**Diagnosis:** At a duty cycle of 0.65, each diagonal pair swings for only 35% of the cycle,
with two explicit all-four-feet-planted "dead" windows (15% each, 30% total) contributing zero
forward-driving motion.
**Change:** Duty cycle 0.65 → 0.55 (halving that dead-window margin, not eliminating it).
**Result:** Average speed rose meaningfully, yaw remained excellent (best run yet: -0.7° net
yaw, nearly dead-straight path).

### v10 — reaching the floor of that lever
**Change:** Duty cycle 0.55 → 0.5, removing the double-support dead windows entirely (one pair
is always mid-swing, no gap).
**Result:** Speed barely moved. As explained in Part 1, the nominal stride length is itself
*derived from* the duty cycle — shrinking the dead-window margin also shrinks the commanded
stride, and the two effects roughly cancelled out. This lever was exhausted.

### v11 — raising the actual speed target (and a real tradeoff)
**Change:** Target walking speed raised from 0.15 to 0.20 m/s — which, per Part 1, directly
scales the commanded stride length rather than trading it against duty cycle.
**Result:** Speed did improve meaningfully. But yaw drift got noticeably and repeatably worse
across seven consecutive runs (roughly -16° to +10°, versus the well-behaved band at the lower
speed) — still bounded, no falls, but a clear regression. Likely cause: the correction's fixed
output ceiling was sized against the smaller stride, and the bigger stride produces a
proportionally bigger yaw disturbance per step that the same ceiling can no longer fully
arrest.

### v12 — reverting the speed/yaw tradeoff
**Change:** Target speed reverted 0.20 → 0.15 m/s, given a confirmed, repeatable regression.
**Result:** Confirmed back in the well-behaved yaw range across several subsequent runs.

### v13 — trying the file's own built-in speed regulator
**Context:** Even at the confirmed-stable 0.15 m/s target, actual average speed consistently
landed around 70-73% of target — a persistent shortfall the fixed, open-loop stride (see Part
1) has no way to notice or correct, since it computes stride length once from the target speed
and never checks whether that target is actually being achieved.
**Change:** Enabled the file's built-in Raibert-style speed feedback (see Part 1).
**Result:** Made speed *worse*, not better (dropping further below the already-short average),
though yaw was unaffected.

### v14 — diagnosing why, and reverting
**Diagnosis:** The feedback formula's dominant term rescaled the nominal stride using the
*actual measured* speed instead of the *target* speed. As Part 1's Raibert explanation notes,
that inverts the intended design: if the robot is already running slow, that term mostly just
recomputes an even shorter stride to match the existing slowness, and the genuinely
error-correcting piece layered on top was far too small in comparison to outweigh it. Net
effect: the feature reinforced whatever speed the robot already happened to be doing, instead
of correcting toward the target.
**Change:** Reverted to the fixed, open-loop stride. The correct fix — rebuilding that
dominant term around the target speed instead of the measured speed — was identified but not
attempted, since it's new, untested logic rather than a simple revert.

---

## Where things stand now

**Crawl gait (`champgait_wave.py`):** Stable and walking (v6), with yaw drift brought under
control through v9-v11's correction redesign, and the remaining spawn-yaw non-determinism
resolved by v13's delayed control-loop start.

**Trot gait (`trot_demo.py`):** Yaw drift is solved — the v8 fix (restricting correction to
truly swinging legs) is confirmed clean and repeatable across many runs at the 0.15 m/s target
speed and 0.5 duty cycle. Average forward speed sits around 70-73% of that target; pushing the
target higher (v11) recovers real speed but currently costs back some of that yaw stability,
and the built-in speed regulator has a diagnosed formula bug that leaves it disabled for now.
The next open thread is either accepting the current speed/yaw balance, or implementing the
corrected speed-regulation formula (built around the target speed, with only the error term
based on measured speed) as a fresh, isolated test.
