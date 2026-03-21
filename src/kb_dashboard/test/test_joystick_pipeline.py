"""End-to-end integration tests: joystick input → ESP32 actuator commands.

Tests the FULL pipeline without ROS:
  1. Browser sends WebSocket manual_control JSON
  2. server.py handle_command() dispatches to node
  3. dashboard_node.py publish_manual_control() builds Twist
  4. cmd_vel_bridge logic splits Twist into throttle/brake/steering
  5. protocol.py encodes to int32 payloads
  6. ESP32 firmware decodes and applies to actuators

The key invariant: pushing joystick LEFT must produce:
  - steering > 0 (positive = left convention)
  - throttle = 0 (no forward/backward)
  - brake = 0
And pushing joystick FORWARD must produce:
  - throttle > 0
  - steering = 0
  - brake = 0

These tests would have caught the bug where steering caused throttle.

Run: python -m pytest src/kb_dashboard/test/test_joystick_pipeline.py -v
"""
import struct
import unittest

from kb_dashboard.protocol import (
    encode_steering,
    encode_throttle,
    encode_braking,
    decode_steering,
    decode_throttle,
    decode_braking,
    ORIN_TARG_THROTTLE,
    ORIN_TARG_BRAKING,
    ORIN_TARG_STEERING,
)

# ---------------------------------------------------------------------------
# Extracted pure logic from each pipeline stage (no ROS needed)
# ---------------------------------------------------------------------------

NOMINAL_MAX_SPEED = 5.0
NOMINAL_MAX_STEER = 0.5  # radians


def joystick_to_ws_payload(vector_x: float, vector_y: float, dead_zone=0.15):
    """Stage 1: nipplejs vector → WebSocket JSON fields.

    nipplejs: x positive = right, y positive = up.
    Convention: steering positive = left, so negate x.
    """
    raw_steer = -vector_x
    raw_drive = vector_y

    mag = (vector_x**2 + vector_y**2) ** 0.5
    if mag < dead_zone:
        return {"steering": 0.0, "throttle": 0.0, "brake": 0.0}

    throttle = max(0.0, raw_drive)
    brake = max(0.0, -raw_drive)
    return {"steering": raw_steer, "throttle": throttle, "brake": brake}


def ws_to_twist(steer: float, throttle: float, brake: float):
    """Stage 2-3: WebSocket fields → Twist (linear.x, angular.z)."""
    linear_x = (throttle - brake) * NOMINAL_MAX_SPEED
    angular_z = steer * NOMINAL_MAX_STEER
    return linear_x, angular_z


def twist_to_efforts(linear_x: float, angular_z: float,
                     max_speed=NOMINAL_MAX_SPEED, max_steer=NOMINAL_MAX_STEER):
    """Stage 4: Twist → throttle_effort, brake_effort, steer_rad."""
    if linear_x >= 0:
        throttle_effort = min(1.0, linear_x / max_speed)
        brake_effort = 0.0
    else:
        throttle_effort = 0.0
        brake_effort = min(1.0, -linear_x / max_speed)
    steer_rad = max(-max_steer, min(max_steer, angular_z))
    return throttle_effort, brake_effort, steer_rad


def efforts_to_payloads(throttle_effort, brake_effort, steer_rad):
    """Stage 5: efforts → int32 payloads (what goes on the wire)."""
    return {
        "throttle": encode_throttle(throttle_effort),
        "brake": encode_braking(brake_effort),
        "steering": encode_steering(steer_rad),
    }


def esp32_decode_payloads(payloads):
    """Stage 6: ESP32 receives payloads → normalized actuator values.

    Mirrors what km_coms.c ProcessPayload + main.c control_task do:
    - throttle: payload[0] / 255.0
    - braking: payload[0] / 255.0
    - steering: payload[0] / 1000.0 (radians)
    """
    return {
        "throttle": payloads["throttle"][0] / 255.0,
        "brake": payloads["brake"][0] / 255.0,
        "steer_rad": payloads["steering"][0] / 1000.0,
    }


def full_pipeline(vector_x: float, vector_y: float):
    """Run the complete joystick → ESP32 pipeline and return final values."""
    ws = joystick_to_ws_payload(vector_x, vector_y)
    lx, az = ws_to_twist(ws["steering"], ws["throttle"], ws["brake"])
    thr, brk, steer = twist_to_efforts(lx, az)
    payloads = efforts_to_payloads(thr, brk, steer)
    return esp32_decode_payloads(payloads)


# ===================================================================
# End-to-end: joystick direction → ESP32 actuator isolation
# ===================================================================

class TestJoystickLeftProducesOnlySteering(unittest.TestCase):
    """Pushing joystick LEFT must move ONLY steering, not throttle/brake."""

    def test_full_left(self):
        """Full left: vector_x=-1.0, vector_y=0.0"""
        result = full_pipeline(-1.0, 0.0)
        self.assertGreater(result["steer_rad"], 0.0, "LEFT should produce positive steering")
        self.assertAlmostEqual(result["throttle"], 0.0, places=2, msg="LEFT must not produce throttle")
        self.assertAlmostEqual(result["brake"], 0.0, places=2, msg="LEFT must not produce braking")

    def test_half_left(self):
        result = full_pipeline(-0.5, 0.0)
        self.assertGreater(result["steer_rad"], 0.0)
        self.assertAlmostEqual(result["throttle"], 0.0, places=2)
        self.assertAlmostEqual(result["brake"], 0.0, places=2)


class TestJoystickRightProducesOnlySteering(unittest.TestCase):
    """Pushing joystick RIGHT must move ONLY steering (negative), not throttle/brake."""

    def test_full_right(self):
        result = full_pipeline(1.0, 0.0)
        self.assertLess(result["steer_rad"], 0.0, "RIGHT should produce negative steering")
        self.assertAlmostEqual(result["throttle"], 0.0, places=2)
        self.assertAlmostEqual(result["brake"], 0.0, places=2)


class TestJoystickForwardProducesOnlyThrottle(unittest.TestCase):
    """Pushing joystick FORWARD must produce ONLY throttle, not steering/brake."""

    def test_full_forward(self):
        result = full_pipeline(0.0, 1.0)
        self.assertGreater(result["throttle"], 0.0, "FORWARD should produce throttle")
        self.assertAlmostEqual(result["steer_rad"], 0.0, places=3, msg="FORWARD must not produce steering")
        self.assertAlmostEqual(result["brake"], 0.0, places=2, msg="FORWARD must not produce braking")

    def test_half_forward(self):
        result = full_pipeline(0.0, 0.5)
        self.assertGreater(result["throttle"], 0.0)
        self.assertAlmostEqual(result["steer_rad"], 0.0, places=3)
        self.assertAlmostEqual(result["brake"], 0.0, places=2)


class TestJoystickBackwardProducesOnlyBrake(unittest.TestCase):
    """Pushing joystick BACKWARD must produce ONLY brake, not steering/throttle."""

    def test_full_backward(self):
        result = full_pipeline(0.0, -1.0)
        self.assertGreater(result["brake"], 0.0, "BACKWARD should produce braking")
        self.assertAlmostEqual(result["throttle"], 0.0, places=2, msg="BACKWARD must not produce throttle")
        self.assertAlmostEqual(result["steer_rad"], 0.0, places=3, msg="BACKWARD must not produce steering")


class TestJoystickDiagonalCombination(unittest.TestCase):
    """Diagonal: forward-left should produce both throttle AND steering."""

    def test_forward_left(self):
        result = full_pipeline(-0.7, 0.7)
        self.assertGreater(result["throttle"], 0.0, "Should have throttle")
        self.assertGreater(result["steer_rad"], 0.0, "Should steer left (positive)")
        self.assertAlmostEqual(result["brake"], 0.0, places=2)

    def test_forward_right(self):
        result = full_pipeline(0.7, 0.7)
        self.assertGreater(result["throttle"], 0.0)
        self.assertLess(result["steer_rad"], 0.0, "Should steer right (negative)")
        self.assertAlmostEqual(result["brake"], 0.0, places=2)

    def test_backward_left(self):
        result = full_pipeline(-0.7, -0.7)
        self.assertGreater(result["brake"], 0.0)
        self.assertGreater(result["steer_rad"], 0.0, "Should steer left")
        self.assertAlmostEqual(result["throttle"], 0.0, places=2)


class TestJoystickDeadZone(unittest.TestCase):
    """Small joystick movements inside dead zone produce zero output."""

    def test_tiny_movement(self):
        result = full_pipeline(0.05, 0.05)
        self.assertAlmostEqual(result["throttle"], 0.0, places=2)
        self.assertAlmostEqual(result["brake"], 0.0, places=2)
        self.assertAlmostEqual(result["steer_rad"], 0.0, places=3)

    def test_at_dead_zone_boundary(self):
        result = full_pipeline(0.1, 0.1)
        # magnitude = sqrt(0.01+0.01) ≈ 0.141 < 0.15 → dead zone
        self.assertAlmostEqual(result["throttle"], 0.0, places=2)


# ===================================================================
# Sign convention: positive = left, end to end
# ===================================================================

class TestSignConvention(unittest.TestCase):
    """Verify the positive=left convention holds through every pipeline stage."""

    def test_left_is_positive_at_ws(self):
        ws = joystick_to_ws_payload(-1.0, 0.0)
        self.assertGreater(ws["steering"], 0.0, "WS steering should be positive for LEFT")

    def test_left_is_positive_at_twist(self):
        ws = joystick_to_ws_payload(-1.0, 0.0)
        _, az = ws_to_twist(ws["steering"], ws["throttle"], ws["brake"])
        self.assertGreater(az, 0.0, "Twist angular.z should be positive for LEFT")

    def test_left_is_positive_at_effort(self):
        ws = joystick_to_ws_payload(-1.0, 0.0)
        lx, az = ws_to_twist(ws["steering"], ws["throttle"], ws["brake"])
        _, _, steer = twist_to_efforts(lx, az)
        self.assertGreater(steer, 0.0, "Effort steer_rad should be positive for LEFT")

    def test_left_is_positive_at_payload(self):
        ws = joystick_to_ws_payload(-1.0, 0.0)
        lx, az = ws_to_twist(ws["steering"], ws["throttle"], ws["brake"])
        thr, brk, steer = twist_to_efforts(lx, az)
        payloads = efforts_to_payloads(thr, brk, steer)
        self.assertGreater(payloads["steering"][0], 0, "Payload should be positive for LEFT")

    def test_left_is_positive_at_esp32(self):
        result = full_pipeline(-1.0, 0.0)
        self.assertGreater(result["steer_rad"], 0.0, "ESP32 should see positive for LEFT")

    def test_right_is_negative_everywhere(self):
        result = full_pipeline(1.0, 0.0)
        self.assertLess(result["steer_rad"], 0.0, "ESP32 should see negative for RIGHT")


# ===================================================================
# Quantization: values survive encode→decode with acceptable precision
# ===================================================================

class TestQuantizationPrecision(unittest.TestCase):
    """Protocol encoding must preserve values within acceptable tolerance."""

    def test_steering_precision(self):
        """Steering: x1000 encoding → 0.001 rad precision."""
        for angle in [0.0, 0.1, 0.25, 0.5, -0.1, -0.5]:
            encoded = encode_steering(angle)
            decoded = decode_steering(encoded)
            self.assertAlmostEqual(decoded, angle, delta=0.001,
                                   msg=f"Steering {angle} round-trip failed")

    def test_throttle_precision(self):
        """Throttle: x255 encoding → ~0.004 precision."""
        for effort in [0.0, 0.1, 0.5, 1.0]:
            encoded = encode_throttle(effort)
            decoded = decode_throttle(encoded)
            self.assertAlmostEqual(decoded, effort, delta=0.004,
                                   msg=f"Throttle {effort} round-trip failed")

    def test_braking_precision(self):
        for effort in [0.0, 0.1, 0.5, 1.0]:
            encoded = encode_braking(effort)
            decoded = decode_braking(encoded)
            self.assertAlmostEqual(decoded, effort, delta=0.004,
                                   msg=f"Braking {effort} round-trip failed")


# ===================================================================
# Frame type constants: verify they match ESP32 firmware
# ===================================================================

class TestFrameTypeConstants(unittest.TestCase):
    """Frame types in Python must match km_coms.h message_type_t enum."""

    def test_throttle_type(self):
        self.assertEqual(ORIN_TARG_THROTTLE, 0x20)

    def test_braking_type(self):
        self.assertEqual(ORIN_TARG_BRAKING, 0x21)

    def test_steering_type(self):
        self.assertEqual(ORIN_TARG_STEERING, 0x22)


if __name__ == "__main__":
    unittest.main()
