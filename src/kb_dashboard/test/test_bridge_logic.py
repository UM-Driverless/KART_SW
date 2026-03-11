"""Unit tests for cmd_vel_bridge and esp32_sim_node logic (no ROS needed).

Tests the pure computation: speed→effort normalization, steering clamping,
simulated telemetry encoding, and edge cases.
"""
import math
import unittest

from kb_dashboard.protocol import (
    decode_steering,
    decode_speed,
    decode_accel,
    decode_braking,
    decode_throttle,
    encode_steering,
    encode_throttle,
    encode_braking,
    encode_act_steering,
    encode_act_speed,
    encode_act_accel,
    encode_act_braking,
    encode_act_throttle,
)


# ---------------------------------------------------------------------------
# Extracted logic from cmd_vel_bridge_node.py (lines 46-59)
# ---------------------------------------------------------------------------

def cmd_vel_to_efforts(speed: float, steer: float,
                       max_speed: float, max_steer: float):
    """Pure function replicating CmdVelBridgeNode._on_cmd logic."""
    if speed >= 0:
        throttle = min(1.0, speed / max_speed)
        brake = 0.0
    else:
        throttle = 0.0
        brake = min(1.0, -speed / max_speed)
    steer_rad = max(-max_steer, min(max_steer, steer))
    return throttle, brake, steer_rad


# ---------------------------------------------------------------------------
# Extracted logic from esp32_sim_node.py (lines 69-80)
# ---------------------------------------------------------------------------

def sim_cmd_vel_to_actuators(speed: float):
    """Pure function replicating Esp32SimNode._on_cmd_vel throttle/brake."""
    if speed > 0.1:
        throttle = min(1.0, speed / 10.0)
        brake = 0.0
    elif speed < -0.1:
        throttle = 0.0
        brake = min(1.0, abs(speed) / 10.0)
    else:
        throttle = 0.0
        brake = 0.0
    return throttle, brake


def sim_odom_accel(speed: float, prev_speed: float, dt: float,
                   yaw_rate: float):
    """Pure function replicating Esp32SimNode._on_odom acceleration calc."""
    if dt > 0.001:
        accel_lon = (speed - prev_speed) / dt
    else:
        accel_lon = 0.0
    accel_lat = speed * yaw_rate
    return accel_lat, accel_lon


# ===================================================================
# CmdVelBridge tests
# ===================================================================

class TestCmdVelThrottleBrake(unittest.TestCase):
    """Speed→effort normalization (max_speed=5.0)."""

    def test_zero_speed(self):
        t, b, _ = cmd_vel_to_efforts(0.0, 0.0, 5.0, 0.5)
        self.assertAlmostEqual(t, 0.0)
        self.assertAlmostEqual(b, 0.0)

    def test_positive_speed(self):
        t, b, _ = cmd_vel_to_efforts(2.5, 0.0, 5.0, 0.5)
        self.assertAlmostEqual(t, 0.5)
        self.assertAlmostEqual(b, 0.0)

    def test_max_speed(self):
        t, b, _ = cmd_vel_to_efforts(5.0, 0.0, 5.0, 0.5)
        self.assertAlmostEqual(t, 1.0)
        self.assertAlmostEqual(b, 0.0)

    def test_over_max_speed_clamps_to_one(self):
        t, b, _ = cmd_vel_to_efforts(100.0, 0.0, 5.0, 0.5)
        self.assertAlmostEqual(t, 1.0)
        self.assertAlmostEqual(b, 0.0)

    def test_negative_speed_brakes(self):
        t, b, _ = cmd_vel_to_efforts(-2.5, 0.0, 5.0, 0.5)
        self.assertAlmostEqual(t, 0.0)
        self.assertAlmostEqual(b, 0.5)

    def test_large_negative_speed_clamps(self):
        t, b, _ = cmd_vel_to_efforts(-999.0, 0.0, 5.0, 0.5)
        self.assertAlmostEqual(t, 0.0)
        self.assertAlmostEqual(b, 1.0)

    def test_tiny_positive_speed(self):
        t, b, _ = cmd_vel_to_efforts(0.01, 0.0, 5.0, 0.5)
        self.assertAlmostEqual(t, 0.002)
        self.assertAlmostEqual(b, 0.0)

    def test_throttle_and_brake_never_both_nonzero(self):
        for speed in [-10, -1, -0.01, 0, 0.01, 1, 10]:
            t, b, _ = cmd_vel_to_efforts(speed, 0.0, 5.0, 0.5)
            self.assertTrue(t == 0.0 or b == 0.0,
                            f"speed={speed}: throttle={t}, brake={b}")


class TestCmdVelSteeringClamp(unittest.TestCase):
    """Steering clamping to ±max_steer."""

    def test_within_range(self):
        _, _, s = cmd_vel_to_efforts(0.0, 0.3, 5.0, 0.5)
        self.assertAlmostEqual(s, 0.3)

    def test_positive_clamp(self):
        _, _, s = cmd_vel_to_efforts(0.0, 1.0, 5.0, 0.5)
        self.assertAlmostEqual(s, 0.5)

    def test_negative_clamp(self):
        _, _, s = cmd_vel_to_efforts(0.0, -1.0, 5.0, 0.5)
        self.assertAlmostEqual(s, -0.5)

    def test_exact_max(self):
        _, _, s = cmd_vel_to_efforts(0.0, 0.5, 5.0, 0.5)
        self.assertAlmostEqual(s, 0.5)

    def test_exact_neg_max(self):
        _, _, s = cmd_vel_to_efforts(0.0, -0.5, 5.0, 0.5)
        self.assertAlmostEqual(s, -0.5)

    def test_zero_steer(self):
        _, _, s = cmd_vel_to_efforts(0.0, 0.0, 5.0, 0.5)
        self.assertAlmostEqual(s, 0.0)


class TestCmdVelEdgeCases(unittest.TestCase):
    """Edge cases: division by zero, inf, etc."""

    def test_max_speed_zero_divides(self):
        """max_speed=0 → division by zero. Should we guard against it?"""
        with self.assertRaises(ZeroDivisionError):
            cmd_vel_to_efforts(1.0, 0.0, 0.0, 0.5)

    def test_max_steer_zero(self):
        """max_steer=0 → steer is always clamped to 0."""
        _, _, s = cmd_vel_to_efforts(0.0, 0.3, 5.0, 0.0)
        self.assertAlmostEqual(s, 0.0)

    def test_inf_speed(self):
        t, b, _ = cmd_vel_to_efforts(float('inf'), 0.0, 5.0, 0.5)
        # min(1.0, inf/5) → 1.0
        self.assertAlmostEqual(t, 1.0)
        self.assertAlmostEqual(b, 0.0)

    def test_nan_speed_causes_full_brake(self):
        """NaN >= 0 is False → brake branch. min(1.0, NaN) → 1.0 in Python.
        This means NaN input silently applies full braking — document it."""
        t, b, _ = cmd_vel_to_efforts(float('nan'), 0.0, 5.0, 0.5)
        self.assertAlmostEqual(t, 0.0)
        self.assertAlmostEqual(b, 1.0)


class TestCmdVelProtobufRoundtrip(unittest.TestCase):
    """Full encode→decode roundtrip through protobuf."""

    def test_throttle_roundtrip(self):
        t, b, s = cmd_vel_to_efforts(2.5, 0.3, 5.0, 0.5)
        self.assertAlmostEqual(decode_throttle(encode_throttle(t)), t, places=5)
        self.assertAlmostEqual(decode_braking(encode_braking(b)), b, places=5)
        self.assertAlmostEqual(decode_steering(encode_steering(s)), s, places=5)

    def test_brake_roundtrip(self):
        t, b, s = cmd_vel_to_efforts(-3.0, -0.2, 5.0, 0.5)
        self.assertAlmostEqual(decode_throttle(encode_throttle(t)), t, places=5)
        self.assertAlmostEqual(decode_braking(encode_braking(b)), b, places=5)
        self.assertAlmostEqual(decode_steering(encode_steering(s)), s, places=5)


# ===================================================================
# Esp32SimNode tests
# ===================================================================

class TestSimCmdVelToActuators(unittest.TestCase):
    """Sim node speed→throttle/brake split."""

    def test_zero_speed_deadband(self):
        t, b = sim_cmd_vel_to_actuators(0.0)
        self.assertEqual(t, 0.0)
        self.assertEqual(b, 0.0)

    def test_within_deadband_positive(self):
        t, b = sim_cmd_vel_to_actuators(0.05)
        self.assertEqual(t, 0.0)
        self.assertEqual(b, 0.0)

    def test_within_deadband_negative(self):
        t, b = sim_cmd_vel_to_actuators(-0.05)
        self.assertEqual(t, 0.0)
        self.assertEqual(b, 0.0)

    def test_above_deadband(self):
        t, b = sim_cmd_vel_to_actuators(5.0)
        self.assertAlmostEqual(t, 0.5)
        self.assertEqual(b, 0.0)

    def test_below_deadband(self):
        t, b = sim_cmd_vel_to_actuators(-5.0)
        self.assertEqual(t, 0.0)
        self.assertAlmostEqual(b, 0.5)

    def test_max_throttle(self):
        t, b = sim_cmd_vel_to_actuators(10.0)
        self.assertAlmostEqual(t, 1.0)
        self.assertEqual(b, 0.0)

    def test_over_max_throttle_clamps(self):
        t, b = sim_cmd_vel_to_actuators(20.0)
        self.assertAlmostEqual(t, 1.0)
        self.assertEqual(b, 0.0)

    def test_max_brake(self):
        t, b = sim_cmd_vel_to_actuators(-10.0)
        self.assertEqual(t, 0.0)
        self.assertAlmostEqual(b, 1.0)

    def test_over_max_brake_clamps(self):
        t, b = sim_cmd_vel_to_actuators(-50.0)
        self.assertEqual(t, 0.0)
        self.assertAlmostEqual(b, 1.0)

    def test_deadband_boundary_positive(self):
        """Exactly 0.1 — not > 0.1, so should be in deadband."""
        t, b = sim_cmd_vel_to_actuators(0.1)
        self.assertEqual(t, 0.0)
        self.assertEqual(b, 0.0)

    def test_deadband_boundary_negative(self):
        """Exactly -0.1 — not < -0.1, so should be in deadband."""
        t, b = sim_cmd_vel_to_actuators(-0.1)
        self.assertEqual(t, 0.0)
        self.assertEqual(b, 0.0)


class TestSimOdomAccel(unittest.TestCase):
    """Sim node acceleration from odometry."""

    def test_longitudinal_accel(self):
        lat, lon = sim_odom_accel(5.0, 3.0, 0.1, 0.0)
        self.assertAlmostEqual(lon, 20.0)  # (5-3)/0.1
        self.assertAlmostEqual(lat, 0.0)

    def test_lateral_accel(self):
        lat, lon = sim_odom_accel(5.0, 5.0, 0.1, 0.5)
        self.assertAlmostEqual(lon, 0.0)
        self.assertAlmostEqual(lat, 2.5)  # 5.0 * 0.5

    def test_tiny_dt_returns_zero_lon(self):
        """dt ≤ 0.001 → lon accel is 0 to avoid division by near-zero."""
        lat, lon = sim_odom_accel(10.0, 0.0, 0.0005, 0.0)
        self.assertAlmostEqual(lon, 0.0)

    def test_zero_dt_returns_zero_lon(self):
        lat, lon = sim_odom_accel(10.0, 0.0, 0.0, 0.0)
        self.assertAlmostEqual(lon, 0.0)

    def test_negative_dt_returns_zero_lon(self):
        lat, lon = sim_odom_accel(10.0, 0.0, -1.0, 0.0)
        self.assertAlmostEqual(lon, 0.0)

    def test_deceleration(self):
        lat, lon = sim_odom_accel(2.0, 5.0, 0.1, 0.0)
        self.assertAlmostEqual(lon, -30.0)  # (2-5)/0.1


class TestSimTelemetryRoundtrip(unittest.TestCase):
    """Sim node telemetry encode→decode roundtrip."""

    def test_steering_with_raw_encoder(self):
        angle = 0.25
        raw = 2048 + int(angle * 650)
        payload = encode_act_steering(angle, raw)
        decoded_angle = decode_steering(payload)
        self.assertAlmostEqual(decoded_angle, angle, places=5)

    def test_speed(self):
        payload = encode_act_speed(3.5)
        self.assertAlmostEqual(decode_speed(payload), 3.5, places=5)

    def test_accel(self):
        payload = encode_act_accel(1.2, -0.8)
        lat, lon = decode_accel(payload)
        self.assertAlmostEqual(lat, 1.2, places=5)
        self.assertAlmostEqual(lon, -0.8, places=5)

    def test_throttle(self):
        payload = encode_act_throttle(0.7)
        self.assertAlmostEqual(decode_throttle(payload), 0.7, places=5)

    def test_braking(self):
        payload = encode_act_braking(0.3)
        self.assertAlmostEqual(decode_braking(payload), 0.3, places=5)

    def test_negative_speed(self):
        payload = encode_act_speed(-2.0)
        self.assertAlmostEqual(decode_speed(payload), -2.0, places=5)

    def test_zero_everything(self):
        self.assertAlmostEqual(decode_speed(encode_act_speed(0.0)), 0.0)
        self.assertAlmostEqual(decode_throttle(encode_act_throttle(0.0)), 0.0)
        self.assertAlmostEqual(decode_braking(encode_act_braking(0.0)), 0.0)
        self.assertAlmostEqual(decode_steering(encode_act_steering(0.0)), 0.0)


if __name__ == "__main__":
    unittest.main()
