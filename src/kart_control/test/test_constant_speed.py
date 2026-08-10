"""Tests for the constant_speed throttle loop in cone_follower_node.py.

The loop closes on /kart/speed, which comes from a cone-based estimator that has
never been checked against a real speed. These tests are therefore mostly about the
containment rather than the control: that a wrong or missing reading cannot produce
more throttle than the open-loop mode it replaces, and cannot produce any at all
when the reading has gone stale.

The node itself needs ROS, so this exercises the arithmetic through a stand-in that
carries only the fields the loop touches, keeping the tests runnable on a laptop.
"""

import pytest

# Mirrors the defaults declared in cone_follower_node.py.
MAX_SPEED = 2.625  # throttle ceiling, in the same fake-m/s units cmd_vel uses
TARGET = 2.0
KP = 0.6
KI = 0.4
STALE_TIMEOUT = 0.4


class FakeLoop:
    """The constant_speed arithmetic, lifted out of the ROS node.

    Kept deliberately close to the original: any change to the real loop that this
    no longer mirrors should be copied here, or these tests stop meaning anything.
    """

    def __init__(self, target=TARGET, kp=KP, ki=KI):
        self.target_speed = target
        self.speed_kp = kp
        self.speed_ki = ki
        self.max_speed = MAX_SPEED
        self.speed_stale_timeout = STALE_TIMEOUT
        self._speed_integral = 0.0
        self._speed_pid_time = None
        self._actual_speed = 0.0
        self._speed_age = None  # seconds since the last reading; None = never

    def step(self, measured, age, dt):
        self._actual_speed = measured
        self._speed_age = age

        if age is None or age > self.speed_stale_timeout:
            self._speed_integral = 0.0
            return 0.0
        if dt <= 0.0 or dt > 0.5:
            return 0.0

        error = self.target_speed - self._actual_speed
        proportional = self.speed_kp * error
        self._speed_integral += self.speed_ki * error * dt
        self._speed_integral = max(0.0, min(self.max_speed, self._speed_integral))
        return max(0.0, min(self.max_speed, proportional + self._speed_integral))


# ── containment: the parts that make this safe on an unvalidated estimate ──


def test_throttle_never_exceeds_the_open_loop_ceiling():
    """The whole safety argument: worst case is the mode being replaced.

    constant_throttle commands max_speed outright. If the closed loop can never beat
    that, then an estimate that is wrong in the dangerous direction — reading low
    while the kart is fast — can at most reproduce today's behaviour.
    """
    loop = FakeLoop()
    for _ in range(200):
        out = loop.step(measured=0.0, age=0.0, dt=0.05)  # estimate stuck at zero
        assert out <= MAX_SPEED


def test_no_throttle_when_the_reading_is_stale():
    loop = FakeLoop()
    for _ in range(20):
        loop.step(measured=1.0, age=0.0, dt=0.05)
    assert loop.step(measured=1.0, age=0.9, dt=0.05) == 0.0


def test_no_throttle_before_any_reading_arrives():
    """Silence means no measurement, never zero speed."""
    assert FakeLoop().step(measured=0.0, age=None, dt=0.05) == 0.0


def test_integral_is_dropped_while_blind():
    """Nothing may accumulate during a dropout and slam in when cones return."""
    loop = FakeLoop()
    for _ in range(40):
        loop.step(measured=0.0, age=0.0, dt=0.05)
    assert loop._speed_integral > 0.0

    loop.step(measured=0.0, age=1.0, dt=0.05)
    assert loop._speed_integral == 0.0


def test_integral_cannot_wind_up_past_the_ceiling():
    """A kart held back — on a slope, or chocked — must not store full throttle."""
    loop = FakeLoop()
    for _ in range(500):
        loop.step(measured=0.0, age=0.0, dt=0.05)
    assert loop._speed_integral <= MAX_SPEED


def test_throttle_is_never_negative():
    """Overspeed coasts down; it does not reach for the brake actuator."""
    loop = FakeLoop()
    for _ in range(50):
        assert loop.step(measured=8.0, age=0.0, dt=0.05) >= 0.0


def test_a_long_gap_restarts_rather_than_integrating_through_it():
    loop = FakeLoop()
    assert loop.step(measured=0.0, age=0.0, dt=3.0) == 0.0


# ── control: it should actually hold the target ──


def test_settles_on_the_target_speed():
    """Simulated kart where throttle maps to speed with first-order lag."""
    loop = FakeLoop()
    speed = 0.0
    dt = 0.05
    for _ in range(600):
        throttle = loop.step(measured=speed, age=0.0, dt=dt)
        # 1 unit of throttle command settles at about 1 m/s, with a 1 s time constant.
        speed += (throttle - speed) * dt / 1.0
    assert speed == pytest.approx(TARGET, abs=0.15)


def test_opens_the_throttle_when_below_target_and_shuts_it_when_above():
    loop = FakeLoop()
    below = loop.step(measured=TARGET - 1.0, age=0.0, dt=0.05)
    loop2 = FakeLoop()
    above = loop2.step(measured=TARGET + 1.0, age=0.0, dt=0.05)
    assert below > 0.0
    assert above == 0.0


def test_a_slower_reading_asks_for_more_throttle():
    a, b = FakeLoop(), FakeLoop()
    slow = a.step(measured=0.5, age=0.0, dt=0.05)
    fast = b.step(measured=1.8, age=0.0, dt=0.05)
    assert slow > fast
