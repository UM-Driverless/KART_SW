"""Tests for the cone-based speed estimator in kart_perception/speed_model.py.

Pure Python — no ROS, no camera — so these run on a laptop as well as the Orin.

The cones are simulated by driving a virtual kart through a field of fixed world
points and computing what it would see, which is the only way to test the claim
that actually matters: that the estimate stays correct while cornering, because
the yaw rate cancels out of the range-rate geometry. A test built from
hand-written cone positions could not tell a correct derivation from one where
that cancellation was assumed rather than real.
"""

import math

import pytest

from kart_perception.speed_model import (
    MIN_BEARING_COS,
    ConeTracker,
    SpeedFilter,
    bearing_cosine,
    median,
    median_absolute_deviation,
)


class VirtualKart:
    """A kart at a known pose in a world of fixed cones, reporting what it sees."""

    def __init__(self, cones_world, x=0.0, y=0.0, heading=0.0):
        self.cones_world = cones_world
        self.x = x
        self.y = y
        self.heading = heading

    def visible_cones(self):
        """Cone positions in the kart's own frame, as (class, forward, left)."""
        out = []
        for cx, cy in self.cones_world:
            dx = cx - self.x
            dy = cy - self.y
            fwd = dx * math.cos(self.heading) + dy * math.sin(self.heading)
            left = -dx * math.sin(self.heading) + dy * math.cos(self.heading)
            out.append(("blue_cone", fwd, left))
        return out

    def drive(self, speed, yaw_rate, dt):
        """Advance the kart along its heading, turning at yaw_rate (positive = left)."""
        self.x += speed * math.cos(self.heading) * dt
        self.y += speed * math.sin(self.heading) * dt
        self.heading += yaw_rate * dt


def cone_field(rows=8, spacing=4.0, half_width=2.0):
    """A corridor of cone pairs ahead of the origin, like a track's boundaries."""
    cones = []
    for i in range(rows):
        ahead = 4.0 + i * spacing
        cones.append((ahead, half_width))
        cones.append((ahead, -half_width))
    return cones


def run_tracker(kart, tracker, speed, yaw_rate, dt=0.05, steps=6):
    """Drive the kart and feed each frame to the tracker; return the measurements."""
    results = []
    t = 0.0
    for _ in range(steps):
        m = tracker.update(kart.visible_cones(), t)
        if m is not None:
            results.append(m)
        kart.drive(speed, yaw_rate, dt)
        t += dt
    return results


# ── the geometry ──────────────────────────────────────────────────────


def test_recovers_speed_driving_straight():
    kart = VirtualKart(cone_field())
    results = run_tracker(kart, ConeTracker(), speed=8.0, yaw_rate=0.0)

    assert results, "straight-line driving produced no measurement at all"
    for m in results:
        assert m.speed == pytest.approx(8.0, abs=0.05)


def test_recovers_speed_while_cornering():
    """The point of the range-rate formulation: yaw rate must not corrupt the speed.

    A method that measured how much a cone's forward coordinate changed would report
    badly wrong speeds here, because turning moves cones in the frame without the
    kart approaching them.
    """
    kart = VirtualKart(cone_field())
    results = run_tracker(kart, ConeTracker(), speed=8.0, yaw_rate=0.6)

    assert results, "cornering produced no measurement at all"
    for m in results:
        assert m.speed == pytest.approx(8.0, abs=0.1)


def test_speed_is_positive_when_approaching():
    """Sign check: closing on cones is positive speed, not negative."""
    kart = VirtualKart(cone_field())
    results = run_tracker(kart, ConeTracker(), speed=5.0, yaw_rate=0.0)
    assert all(m.speed > 0 for m in results)


def test_reports_zero_when_stationary():
    kart = VirtualKart(cone_field())
    results = run_tracker(kart, ConeTracker(), speed=0.0, yaw_rate=0.0)
    for m in results:
        assert m.speed == pytest.approx(0.0, abs=0.05)


def test_stationary_while_spinning_still_reads_zero():
    """Turning on the spot covers no ground, so the speed must stay zero."""
    kart = VirtualKart(cone_field())
    results = run_tracker(kart, ConeTracker(), speed=0.0, yaw_rate=1.0)
    for m in results:
        assert m.speed == pytest.approx(0.0, abs=0.1)


def test_recovers_speed_across_a_range_of_speeds():
    for truth in (1.0, 3.0, 6.0, 11.0):
        kart = VirtualKart(cone_field())
        results = run_tracker(kart, ConeTracker(), speed=truth, yaw_rate=0.0)
        assert results, f"no measurement at {truth} m/s"
        assert results[-1].speed == pytest.approx(truth, abs=0.05)


# ── robustness ────────────────────────────────────────────────────────


def test_one_wrong_match_does_not_move_the_estimate():
    """A single bogus cone must be outvoted, since bad matches are the real risk."""
    tracker = ConeTracker()
    kart = VirtualKart(cone_field())

    tracker.update(kart.visible_cones(), 0.0)
    kart.drive(8.0, 0.0, 0.05)

    cones = kart.visible_cones()
    # A cone that teleported towards the kart — what a mismatched pair looks like.
    cones.append(("blue_cone", 3.0, 0.2))
    m = tracker.update(cones, 0.05)

    assert m is not None
    assert m.speed == pytest.approx(8.0, abs=0.2)


def test_no_measurement_from_the_first_frame():
    """With nothing to compare against, the answer is no evidence, not zero."""
    tracker = ConeTracker()
    kart = VirtualKart(cone_field())
    assert tracker.update(kart.visible_cones(), 0.0) is None


def test_no_measurement_from_too_few_cones():
    tracker = ConeTracker()
    tracker.update([("blue_cone", 5.0, 0.0), ("blue_cone", 9.0, 1.0)], 0.0)
    m = tracker.update([("blue_cone", 4.6, 0.0), ("blue_cone", 8.6, 1.0)], 0.05)
    assert m is None


def test_no_measurement_across_a_long_gap():
    """After a long silence the match gate means nothing, so refuse to guess."""
    tracker = ConeTracker()
    kart = VirtualKart(cone_field())
    tracker.update(kart.visible_cones(), 0.0)
    kart.drive(8.0, 0.0, 2.0)
    assert tracker.update(kart.visible_cones(), 2.0) is None


def test_cones_abreast_of_the_kart_are_not_trusted():
    """A cone at 90 degrees barely changes range, so its estimate is noise."""
    assert bearing_cosine(0.0, 5.0) == pytest.approx(0.0)
    assert bearing_cosine(5.0, 0.0) == pytest.approx(1.0)

    tracker = ConeTracker()
    abreast = [("blue_cone", 0.3, 4.0), ("blue_cone", 0.2, -4.0), ("blue_cone", 0.1, 5.0)]
    tracker.update(abreast, 0.0)
    m = tracker.update(abreast, 0.05)
    assert m is None


def test_camera_optical_coordinates_are_refused_not_answered():
    """The worst failure mode found in review: a confident, wrong, near-zero speed.

    Detections arrive in the camera optical frame (x right, y down, z forward) and
    the caller must convert. Passing them through unconverted used to yield a steady
    0.00 m/s at a true 5 m/s, with many cones and a tight spread, so it looked like a
    healthy "stopped" reading rather than a fault.
    """
    tracker = ConeTracker()
    kart = VirtualKart(cone_field())

    def as_optical(cones):
        # x right, z forward — fed positionally into a (forward, left) signature.
        return [(c, -left, fwd) for c, fwd, left in cones]

    tracker.update(as_optical(kart.visible_cones()), 0.0)
    kart.drive(5.0, 0.0, 0.05)
    assert tracker.update(as_optical(kart.visible_cones()), 0.05) is None


def test_frame_gap_cannot_exceed_the_match_gate():
    """Guard the constants against each other, not just their individual values.

    If the kart can travel further between frames than the match gate, a cone can be
    paired with its neighbour instead of itself. Every cone makes that error at once,
    so the median endorses it and the output is confidently wrong.
    """
    from kart_perception.speed_model import (
        MATCH_GATE_M,
        MAX_FRAME_GAP_S,
        MAX_PLAUSIBLE_SPEED,
    )

    kart_top_speed = 12.5  # m/s, about 45 km/h
    assert kart_top_speed * MAX_FRAME_GAP_S <= MATCH_GATE_M
    assert MAX_PLAUSIBLE_SPEED > kart_top_speed


def test_one_previous_cone_cannot_be_matched_twice():
    """Two cones close together must not both claim the same predecessor."""
    tracker = ConeTracker()
    tracker.update([("blue_cone", 6.0, 0.0)], 0.0)
    m = tracker.update(
        [("blue_cone", 5.6, 0.0), ("blue_cone", 5.7, 0.1), ("blue_cone", 5.8, 0.2)],
        0.05,
    )
    # Only one of the three can match, which is below the three-cone minimum.
    assert m is None


def test_cone_leaving_the_range_band_does_not_break_the_frame():
    """A cone crossing the near limit must drop its own estimate, not the whole frame."""
    tracker = ConeTracker()
    kart = VirtualKart(cone_field())
    results = run_tracker(kart, tracker, speed=6.0, yaw_rate=0.0, dt=0.05, steps=12)
    assert len(results) >= 8, "frames were being lost as cones crossed the band edge"


def test_reset_forgets_the_previous_frame():
    tracker = ConeTracker()
    kart = VirtualKart(cone_field())
    tracker.update(kart.visible_cones(), 0.0)
    tracker.reset()
    kart.drive(8.0, 0.0, 0.05)
    assert tracker.update(kart.visible_cones(), 0.05) is None


def test_agreeing_cones_give_a_tighter_noise_figure_than_disagreeing_ones():
    """The noise figure is what makes the filter self-tuning, so it must track reality."""
    kart = VirtualKart(cone_field())
    clean = run_tracker(kart, ConeTracker(), speed=8.0, yaw_rate=0.0)

    tracker = ConeTracker()
    kart2 = VirtualKart(cone_field())
    tracker.update(kart2.visible_cones(), 0.0)
    kart2.drive(8.0, 0.0, 0.05)
    scattered = kart2.visible_cones()
    # Jitter the cones so their individual estimates disagree.
    scattered = [
        (c, f + 0.1 * ((i % 3) - 1), l) for i, (c, f, l) in enumerate(scattered)
    ]
    noisy = tracker.update(scattered, 0.05)

    assert noisy is not None
    assert noisy.stddev > clean[-1].stddev


# ── helpers ───────────────────────────────────────────────────────────


def test_median_handles_both_list_lengths():
    assert median([3.0, 1.0, 2.0]) == 2.0
    assert median([4.0, 1.0, 3.0, 2.0]) == 2.5
    with pytest.raises(ValueError):
        median([])


def test_median_absolute_deviation_ignores_an_outlier():
    tight = [1.0, 1.0, 1.0, 1.0, 1.0]
    assert median_absolute_deviation(tight, 1.0) == pytest.approx(0.0)
    with_outlier = tight + [50.0]
    assert median_absolute_deviation(with_outlier, 1.0) < 1.0


def test_bearing_cutoff_is_a_usable_angle():
    """Guard the constant itself: it should be a wide cone, not a needle."""
    angle_deg = math.degrees(math.acos(MIN_BEARING_COS))
    assert 55.0 < angle_deg < 80.0


# ── the filter ────────────────────────────────────────────────────────


def test_filter_converges_on_a_repeated_measurement():
    f = SpeedFilter()
    for _ in range(20):
        f.predict(0.05)
        f.update(7.0, 0.2)
    assert f.speed == pytest.approx(7.0, abs=0.1)
    assert f.is_valid


def test_filter_starts_invalid():
    """Before any evidence there is no estimate, only a default that must not escape."""
    assert not SpeedFilter().is_valid


def test_filter_goes_invalid_after_measurements_stop():
    f = SpeedFilter()
    for _ in range(20):
        f.predict(0.05)
        f.update(7.0, 0.2)
    assert f.is_valid

    for _ in range(40):
        f.predict(0.05)
    assert not f.is_valid, "a stale estimate must stop claiming to be a reading"


def test_filter_keeps_its_last_value_while_coasting():
    """Losing evidence must not slam the speed to zero — it just stops being published."""
    f = SpeedFilter()
    for _ in range(20):
        f.predict(0.05)
        f.update(7.0, 0.2)
    for _ in range(40):
        f.predict(0.05)
    assert f.speed == pytest.approx(7.0, abs=0.1)


def test_a_confident_measurement_moves_the_estimate_more_than_a_vague_one():
    """This is the whole reason the tracker reports a noise figure per frame."""
    confident = SpeedFilter()
    vague = SpeedFilter()
    for _ in range(30):
        confident.predict(0.05)
        confident.update(5.0, 0.2)
        vague.predict(0.05)
        vague.update(5.0, 0.2)

    confident.predict(0.05)
    confident.update(9.0, 0.15)
    vague.predict(0.05)
    vague.update(9.0, 3.0)

    assert confident.speed > vague.speed


def test_zero_speed_update_pulls_a_drifted_estimate_back():
    f = SpeedFilter()
    for _ in range(20):
        f.predict(0.05)
        f.update(6.0, 0.3)

    for _ in range(20):
        f.predict(0.05)
        f.update_stationary()
    assert f.speed == pytest.approx(0.0, abs=0.1)


def test_uncertainty_grows_faster_for_a_more_agile_kart():
    slow = SpeedFilter(process_accel=1.0)
    quick = SpeedFilter(process_accel=8.0)
    slow.predict(0.5)
    quick.predict(0.5)
    assert quick.stddev > slow.stddev
