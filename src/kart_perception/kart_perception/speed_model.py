"""Forward speed estimated from how fast detected cones approach the kart.

Why this exists
---------------
The kart has no speed sensor. The motor hall sensors would be the direct answer,
but their pins are taken on the current PCB, so nothing measures speed today.
Every speed figure in the stack is currently invented: cone_follower publishes a
throttle fraction dressed as m/s, and the Stanley controller runs on a fixed
`stanley_assumed_speed` parameter.

The ZED SDK's visual-inertial odometry used to fill this role and was turned off
in commit 02eda4d (19 April 2026) because it ran on the GPU that YOLO needs.

Cones do not move. So the rate at which a detected cone's distance shrinks is
caused entirely by the kart's own motion, and YOLO already finds those cones every
frame for the steering controller. Reading a speed out of them costs a nearest
neighbour match and some arithmetic on a few dozen points, all on the CPU.

The geometry, and why yaw rate does not appear
----------------------------------------------
Take a cone at position (x, y) in the kart's frame, x forward and y left, at range
r = sqrt(x^2 + y^2). The kart moves forward at speed v and turns at yaw rate w.
Because the cone is fixed to the ground, its apparent velocity in the kart's frame
is the negative of the kart's own motion:

    dx/dt = -v + w*y
    dy/dt =      -w*x

Differentiating r = sqrt(x^2 + y^2):

    dr/dt = (x*dx/dt + y*dy/dt) / r
          = (x*(-v + w*y) + y*(-w*x)) / r
          = (-v*x + w*x*y - w*x*y) / r
          = -v * (x / r)

The two yaw terms cancel exactly. Range rate depends only on forward speed and on
x/r, the cosine of the cone's bearing. This matters a lot in practice: it means
the estimate needs no yaw rate input, and stays correct through corners, which is
exactly where a naive "how much did the forward coordinate change" method breaks.

Rearranged, every matched cone gives an independent estimate of speed:

    v = -(dr/dt) / (x / r)

A cone straight ahead has x/r near 1 and gives a well conditioned estimate. A cone
abreast of the kart has x/r near 0: it barely changes range no matter how fast the
kart goes, so dividing by that number amplifies noise without limit. Cones below
MIN_BEARING_COS are therefore dropped rather than trusted.

Combining the cones, and the noise estimate
-------------------------------------------
Per-cone estimates are combined with a median, not a mean. The failure mode that
matters is a wrong match — cone A this frame paired with cone B last frame — which
produces an arbitrarily large bogus speed. A mean would follow it; a median ignores
it as long as most matches are right.

The spread of the per-cone estimates then becomes the measurement noise handed to
the filter, which is what makes the whole thing self-tuning. When cones are close,
plentiful and dead ahead, the estimates agree, the spread is small and the filter
trusts the frame. When they are distant, few, or off to the side, the estimates
disagree, the spread is large and the filter mostly ignores the frame and coasts.
No hand-tuned mode switch decides which situation is which.

What the median does NOT protect against
----------------------------------------
The median rejects errors that affect one cone. It is powerless against errors
every cone shares, because then there is no minority to outvote. All of the
following were found by simulation on 2026-08-10 and are real, not hypothetical:

  * Sideways motion is read as forward motion. The honest relation is
    dr/dt = -(vx*x + vy*y)/r; this module assumes vy is zero. Sliding sideways at
    2 m/s registered as about 0.15 m/s of forward speed. It stays small because the
    bearing cutoff already discards the cones most sensitive to it, and a symmetric
    view cancels much of the rest — but a one-sided view, such as a corner where
    only the inside boundary is visible, does not cancel.

  * Related, and unavoidable without a yaw rate input: the cancellation of w in the
    derivation assumes the kart turns about the camera. It does not — the ZED sits
    ahead of the axle, so cornering swings the camera sideways and that shows up as
    the lateral term above.

  * Anything that moves is believed. Cones drifting towards a stationary kart at
    1 m/s produced a confident 1.000 m/s with a tiny spread. A person or another
    kart misclassified as a cone feeds in directly.

  * Depth noise biases the result rather than just widening it. With 0.2 m of noise
    per axis, a true 5 m/s came out at 7.9 m/s. The reported uncertainty is
    optimistic in the same situation, because MAD/sqrt(n) assumes the per-cone
    errors are independent, and a depth scale or extrinsics error is common to all
    of them.

Accuracy is NOT established
---------------------------
Nobody has checked this output against a known speed on the real kart. Given the
bias above, treat the number as indicative until it has been measured against a GPS
trace or a timed run over a known distance. Do not let a controller close a loop on
it before then.
"""

from __future__ import annotations

import math

# A cone must be at least this far off the bearing limit for its range rate to say
# anything useful about forward speed. cos = 0.35 is a bearing of about 70 degrees
# off the nose; beyond that, dividing by x/r amplifies depth noise more than the
# measurement is worth.
MIN_BEARING_COS = 0.35

# Cones nearer than this are usually clipped by the image edge and their centroid
# jumps around; cones beyond it have depth noise large enough to swamp the signal.
MIN_RANGE_M = 1.5
MAX_RANGE_M = 20.0

# Largest distance a cone may appear to have moved between two frames and still be
# considered the same cone. Cones on a Formula Student track sit metres apart, and
# at 15 m/s over a 50 ms frame gap the kart covers 0.75 m, so a 1.5 m gate matches
# reliably without ever being wide enough to reach a neighbouring cone.
MATCH_GATE_M = 1.5

# Frames further apart than this are not compared. This has to be small enough that
# the kart cannot travel further than MATCH_GATE_M between two frames: if it does,
# a cone can be matched to its own NEIGHBOUR rather than to itself, and because
# every cone in view makes the same mistake at once, the median cannot reject it.
# The result would be a confident wrong speed rather than a rejected frame. At the
# kart's 12.5 m/s top speed, 0.12 s covers 1.5 m, which is the gate.
MAX_FRAME_GAP_S = 0.12

# A cone must be ahead of the kart, not behind it. This is a guard against being fed
# camera optical coordinates (x right, y down, z forward) where the kart's frame
# (x forward, y left) is expected. That mix-up does not fail loudly: the optical x
# is symmetric about zero, so the ranges look plausible, every cone's apparent
# approach cancels out, and the estimator reports a rock-steady 0.00 m/s with a
# small spread while the kart is doing 5 m/s. A confident false reading is worse
# than a crash, so refuse to produce one when most cones are behind the kart.
MIN_FRACTION_AHEAD = 0.6

# Speeds outside this band are rejected as bad matches rather than believed. The
# kart tops out around 45 km/h (12.5 m/s) and cannot reverse under autonomous
# control.
MIN_PLAUSIBLE_SPEED = -2.0
MAX_PLAUSIBLE_SPEED = 20.0

# A single cone's estimate is never precise. Even when every cone in view agrees
# exactly, the frame is not treated as better than this, so the filter can never be
# talked into near-zero uncertainty by a lucky frame.
MIN_MEASUREMENT_STDDEV = 0.15  # m/s


def bearing_cosine(forward: float, left: float) -> float:
    """Return x/r for a cone, the cosine of its bearing off the kart's nose.

    1.0 is straight ahead, 0.0 is directly abreast. This is the factor that links a
    cone's range rate to forward speed, derived in the module docstring.
    """
    r = math.hypot(forward, left)
    if r <= 0.0:
        return 0.0
    return forward / r


def cone_range(forward: float, left: float) -> float:
    """Straight-line distance from the kart to a cone."""
    return math.hypot(forward, left)


def median(values: list[float]) -> float:
    """Middle value of a list, averaging the two middle ones for an even count."""
    if not values:
        raise ValueError("median of an empty list")
    ordered = sorted(values)
    n = len(ordered)
    mid = n // 2
    if n % 2 == 1:
        return ordered[mid]
    return 0.5 * (ordered[mid - 1] + ordered[mid])


def median_absolute_deviation(values: list[float], centre: float) -> float:
    """Spread of values about a centre, measured in a way outliers cannot inflate.

    Scaled by 1.4826 so that for normally distributed data this matches the standard
    deviation, which lets the result be used directly as a measurement noise figure.
    """
    if not values:
        return 0.0
    return 1.4826 * median([abs(v - centre) for v in values])


class ConeSpeedMeasurement:
    """One frame's worth of speed evidence: a value, its noise, and how it was made.

    `n_cones` and `rejected` are carried so a caller can log or display why a frame
    was weak, instead of only seeing that the noise came out large.
    """

    def __init__(self, speed: float, stddev: float, n_cones: int, rejected: int):
        self.speed = speed
        self.stddev = stddev
        self.n_cones = n_cones
        self.rejected = rejected

    def __repr__(self) -> str:
        return (
            f"ConeSpeedMeasurement(speed={self.speed:.2f}, stddev={self.stddev:.2f}, "
            f"n_cones={self.n_cones}, rejected={self.rejected})"
        )


class ConeTracker:
    """Matches cones between consecutive frames and turns the matches into a speed.

    Association is nearest neighbour within MATCH_GATE_M, restricted to cones of the
    same class. This is enough here only because cones are sparse: they sit metres
    apart, so the nearest candidate is either the right one or nothing. The same
    approach on dense generic image features would fail, which is why general visual
    odometry needs far heavier machinery than this.
    """

    def __init__(self):
        self._prev: list[tuple[str, float, float]] = []
        self._prev_time: float | None = None

    def reset(self) -> None:
        """Forget the previous frame, so the next one starts a fresh comparison."""
        self._prev = []
        self._prev_time = None

    def update(
        self, cones: list[tuple[str, float, float]], stamp: float
    ) -> ConeSpeedMeasurement | None:
        """Compare this frame against the previous one and estimate forward speed.

        @param cones List of (class_id, forward_m, left_m) in the kart's frame.
        @param stamp Time of this frame in seconds.
        @return A measurement, or None when this frame cannot support one — no
                previous frame, too long a gap, or too few usable matches. None
                means "no evidence", never "speed is zero".
        """
        # Every cone is kept for matching, but only those in the usable range band
        # produce an estimate. Filtering before storing would drop a cone from the
        # previous frame the moment it crossed either boundary, breaking the match
        # for a frame rather than merely excluding that one cone's estimate.
        in_band = [
            c for c in cones if MIN_RANGE_M <= cone_range(c[1], c[2]) <= MAX_RANGE_M
        ]

        prev, prev_time = self._prev, self._prev_time
        self._prev, self._prev_time = list(cones), stamp

        if prev_time is None:
            return None
        dt = stamp - prev_time
        if dt <= 0.0 or dt > MAX_FRAME_GAP_S:
            return None
        if not self._mostly_ahead(in_band):
            return None

        estimates: list[float] = []
        rejected = 0
        claimed: set[int] = set()
        for cls, fwd, left in in_band:
            match = self._nearest(prev, cls, fwd, left, claimed)
            if match is None:
                continue
            idx, p_fwd, p_left = match
            claimed.add(idx)

            r_now = cone_range(fwd, left)
            r_prev = cone_range(p_fwd, p_left)

            # Average the two frames' bearings. Over one frame the cone has barely
            # moved, so either would do, but the midpoint is the better estimate of
            # the bearing that applied across the interval.
            cos_b = 0.5 * (bearing_cosine(fwd, left) + bearing_cosine(p_fwd, p_left))
            if cos_b < MIN_BEARING_COS:
                rejected += 1
                continue

            speed = -((r_now - r_prev) / dt) / cos_b
            if not (MIN_PLAUSIBLE_SPEED <= speed <= MAX_PLAUSIBLE_SPEED):
                rejected += 1
                continue
            estimates.append(speed)

        # Two cones cannot show disagreement in a meaningful way — the spread of a
        # pair is just half their difference — and a median needs a majority to be
        # robust. Three is the smallest count where a single bad match is outvoted.
        if len(estimates) < 3:
            return None

        centre = median(estimates)
        spread = median_absolute_deviation(estimates, centre)
        stddev = max(MIN_MEASUREMENT_STDDEV, spread / math.sqrt(len(estimates)))
        return ConeSpeedMeasurement(centre, stddev, len(estimates), rejected)

    @staticmethod
    def _mostly_ahead(cones: list[tuple[str, float, float]]) -> bool:
        """Whether enough cones sit in front of the kart for the frame to make sense.

        See MIN_FRACTION_AHEAD — this is the guard against being handed camera
        optical coordinates instead of the kart's forward/left frame.
        """
        if not cones:
            return False
        ahead = sum(1 for _, fwd, _ in cones if fwd > 0.0)
        return ahead >= MIN_FRACTION_AHEAD * len(cones)

    @staticmethod
    def _nearest(
        prev: list[tuple[str, float, float]],
        cls: str,
        fwd: float,
        left: float,
        claimed: set[int],
    ) -> tuple[int, float, float] | None:
        """Closest unclaimed same-class cone in the previous frame, with its index.

        Claimed cones are skipped so a previous-frame cone can only be matched once.
        Without that, two cones close together can both match the same predecessor
        and one of them contributes a fabricated range change.
        """
        best = None
        best_d = MATCH_GATE_M
        for i, p in enumerate(prev):
            if i in claimed or p[0] != cls:
                continue
            d = math.hypot(p[1] - fwd, p[2] - left)
            if d < best_d:
                best_d = d
                best = (i, p[1], p[2])
        return best


class SpeedFilter:
    """A one-state Kalman filter carrying forward speed.

    One state, not several. A filter that also estimated acceleration bias would need
    the ZED IMU, and removing gravity from that IMU's reading needs the kart's pitch
    to better than about a degree — over a bumpy circuit that error alone would inject
    more drift than it removed. Cone detections arrive at tens of hertz, fast enough
    that there is little gap for an acceleration term to fill. So the process model is
    simply "speed persists", with process noise standing in for real acceleration.

    What the filter earns over a plain rolling average is the handling of gaps and of
    varying frame quality. Cone measurements drop out whenever perception loses sight
    of the track, and a rolling average has no way to express that its output is now
    stale. Here uncertainty grows while nothing arrives, good frames pull the estimate
    hard, weak frames barely move it, and `is_valid` states plainly when the estimate
    has decayed into a guess.
    """

    def __init__(
        self,
        process_accel: float = 3.0,
        initial_stddev: float = 5.0,
        max_valid_stddev: float = 2.0,
    ):
        """
        @param process_accel How hard the kart is assumed able to accelerate, m/s^2.
               This sets how fast uncertainty grows between measurements. It is not a
               limit on the estimate, only on how confidently the filter coasts.
        @param initial_stddev Starting uncertainty before any measurement, m/s.
        @param max_valid_stddev Above this uncertainty the estimate is reported as
               invalid rather than as a number, so a stale guess never reaches the
               dashboard looking like a reading.
        """
        self.speed = 0.0
        self.variance = initial_stddev**2
        self.process_accel = process_accel
        self.max_valid_variance = max_valid_stddev**2

    @property
    def stddev(self) -> float:
        """Current uncertainty in the speed estimate, m/s."""
        return math.sqrt(self.variance)

    @property
    def is_valid(self) -> bool:
        """Whether the estimate is currently backed by evidence rather than coasting."""
        return self.variance <= self.max_valid_variance

    def predict(self, dt: float) -> None:
        """Advance the estimate by dt with no new evidence, growing its uncertainty.

        Uncertainty is grown as a standard deviation rather than as a variance:
        after t seconds without evidence, the speed could plausibly have changed by
        process_accel * t, so that is how much is added. Adding to the variance
        instead — the textbook step for a filter whose process noise is independent
        between steps — is wrong here, because it makes the growth depend on how
        often this is called. Two seconds of coasting would then look far more
        certain when sampled at 100 Hz than at 10 Hz, and the node would silently
        become more confident the faster its timer ran.

        This treats acceleration as fully correlated across the gap, which is the
        pessimistic reading. That is the right direction to err for a figure whose
        job is deciding when the estimate has decayed into a guess.
        """
        if dt <= 0.0:
            return
        self.variance = (self.stddev + self.process_accel * dt) ** 2

    def update(self, measured_speed: float, stddev: float) -> None:
        """Fold in one speed measurement with its own noise figure."""
        r = max(stddev, MIN_MEASUREMENT_STDDEV) ** 2
        gain = self.variance / (self.variance + r)
        self.speed += gain * (measured_speed - self.speed)
        self.variance *= 1.0 - gain

    def update_stationary(self, stddev: float = 0.05) -> None:
        """Apply a zero-speed measurement.

        Used when the kart is known to be stopped — no throttle, and the cones in view
        are not changing range. This is the cheapest correction available and the one
        that stops a long run of coasting from wandering, because it is the only time
        the true speed is known exactly without measuring anything.
        """
        self.update(0.0, stddev)
