"""Round-trip tests for int32 binary encode/decode protocol functions."""

from kb_dashboard.protocol import (
    decode_steering,
    decode_steering_raw,
    decode_speed,
    decode_accel,
    decode_braking,
    decode_throttle,
    decode_health,
    decode_heartbeat,
    encode_steering,
    encode_throttle,
    encode_braking,
    encode_act_steering,
    encode_act_speed,
    encode_act_accel,
    encode_act_braking,
    encode_act_throttle,
    encode_heartbeat,
    encode_health,
    ESP_ACT_SPEED,
    ESP_ACT_ACCELERATION,
    ESP_ACT_BRAKING,
    ESP_ACT_STEERING,
    ESP_HEARTBEAT,
    ESP_HEALTH_STATUS,
    ORIN_TARG_THROTTLE,
    ORIN_TARG_BRAKING,
    ORIN_TARG_STEERING,
)

# Precision: steering uses x1000 (0.001 quant), throttle/brake uses x255 (~0.004 quant)
STEER_TOL = 0.001
EFFORT_TOL = 1.0 / 255 + 1e-9


# ── Steering round-trip (Orin → ESP32: TargSteering) ────────────────

class TestSteeringCommandRoundTrip:
    def _roundtrip(self, value):
        encoded = encode_steering(value)
        return decode_steering(encoded)

    def test_zero(self):
        assert self._roundtrip(0.0) == 0.0

    def test_positive(self):
        assert abs(self._roundtrip(0.25) - 0.25) < STEER_TOL

    def test_negative(self):
        assert abs(self._roundtrip(-0.5) - (-0.5)) < STEER_TOL

    def test_small(self):
        assert abs(self._roundtrip(0.001) - 0.001) < STEER_TOL

    def test_large(self):
        assert abs(self._roundtrip(1.5) - 1.5) < STEER_TOL


# ── Steering feedback (ESP32 → Orin: ActSteering) ──────────────────

class TestSteeringFeedbackRoundTrip:
    def test_angle_only(self):
        payload = encode_act_steering(0.3)
        angle, raw, _pwm = decode_steering_raw(payload)
        assert abs(angle - 0.3) < STEER_TOL
        assert raw == 0

    def test_with_raw_encoder(self):
        angle = 0.3
        raw_encoder = 2048 + int(angle * 650)
        payload = encode_act_steering(angle, raw_encoder)
        decoded_angle, decoded_raw, _pwm = decode_steering_raw(payload)
        assert abs(decoded_angle - angle) < STEER_TOL
        assert decoded_raw == raw_encoder

    def test_negative_angle(self):
        payload = encode_act_steering(-0.15, 1900)
        angle, raw, _pwm = decode_steering_raw(payload)
        assert abs(angle - (-0.15)) < STEER_TOL
        assert raw == 1900

    def test_decode_steering_returns_angle(self):
        payload = encode_act_steering(0.42, 2500)
        assert abs(decode_steering(payload) - 0.42) < STEER_TOL


# ── Speed round-trip ─────────────────────────────────────────────────

class TestSpeedRoundTrip:
    def test_zero(self):
        assert decode_speed(encode_act_speed(0.0)) == 0.0

    def test_positive(self):
        assert abs(decode_speed(encode_act_speed(5.0)) - 5.0) < STEER_TOL

    def test_negative(self):
        assert abs(decode_speed(encode_act_speed(-2.5)) - (-2.5)) < STEER_TOL


# ── Acceleration round-trip ──────────────────────────────────────────

class TestAccelerationRoundTrip:
    def test_lat_lon(self):
        lat, lon = 1.5, -0.8
        payload = encode_act_accel(lat, lon)
        d_lat, d_lon = decode_accel(payload)
        assert abs(d_lat - lat) < STEER_TOL
        assert abs(d_lon - lon) < STEER_TOL

    def test_zero_accel(self):
        d_lat, d_lon = decode_accel(encode_act_accel(0.0, 0.0))
        assert d_lat == 0.0
        assert d_lon == 0.0


# ── Throttle/Braking round-trip ──────────────────────────────────────

class TestThrottleRoundTrip:
    def _roundtrip(self, value):
        return decode_throttle(encode_throttle(value))

    def test_zero(self):
        assert self._roundtrip(0.0) == 0.0

    def test_full(self):
        assert abs(self._roundtrip(1.0) - 1.0) < EFFORT_TOL

    def test_mid(self):
        assert abs(self._roundtrip(0.5) - 0.5) < EFFORT_TOL

    def test_quarter(self):
        assert abs(self._roundtrip(0.25) - 0.25) < EFFORT_TOL


class TestBrakingRoundTrip:
    def _roundtrip(self, value):
        return decode_braking(encode_braking(value))

    def test_zero(self):
        assert self._roundtrip(0.0) == 0.0

    def test_full(self):
        assert abs(self._roundtrip(1.0) - 1.0) < EFFORT_TOL

    def test_mid(self):
        assert abs(self._roundtrip(0.5) - 0.5) < EFFORT_TOL


class TestActThrottleBrakingRoundTrip:
    """ESP32 → Orin feedback for throttle/braking."""

    def test_throttle_feedback(self):
        assert abs(decode_throttle(encode_act_throttle(0.75)) - 0.75) < EFFORT_TOL

    def test_braking_feedback(self):
        assert abs(decode_braking(encode_act_braking(0.3)) - 0.3) < EFFORT_TOL


# ── Heartbeat round-trip ─────────────────────────────────────────────

class TestHeartbeatRoundTrip:
    def test_zero(self):
        assert decode_heartbeat(encode_heartbeat(0)) == 0

    def test_nonzero(self):
        assert decode_heartbeat(encode_heartbeat(12345)) == 12345


# ── Health round-trip ────────────────────────────────────────────────

class TestHealthRoundTrip:
    def test_all_ok(self):
        payload = encode_health(True, True, True, 50, 200, 0)
        result = decode_health(payload)
        assert result["health_magnet_ok"] is True
        assert result["health_i2c_ok"] is True
        assert result["health_heap_ok"] is True
        assert result["health_agc"] == 50
        assert result["health_heap_kb"] == 200
        assert result["health_i2c_errors"] == 0

    def test_all_bad(self):
        payload = encode_health(False, False, False, 0, 0, 255)
        result = decode_health(payload)
        assert result["health_magnet_ok"] is False
        assert result["health_i2c_ok"] is False
        assert result["health_heap_ok"] is False
        assert result["health_agc"] == 0
        assert result["health_heap_kb"] == 0
        assert result["health_i2c_errors"] == 255

    def test_mixed_flags(self):
        payload = encode_health(True, False, True, 100, 1024, 3)
        result = decode_health(payload)
        assert result["health_magnet_ok"] is True
        assert result["health_i2c_ok"] is False
        assert result["health_heap_ok"] is True
        assert result["health_agc"] == 100
        assert result["health_heap_kb"] == 1024
        assert result["health_i2c_errors"] == 3

    def test_large_heap(self):
        payload = encode_health(True, True, True, 50, 65535, 0)
        result = decode_health(payload)
        assert result["health_heap_kb"] == 65535


# ── Int32 precision ──────────────────────────────────────────────────

class TestInt32Precision:
    """Int32 x1000 encoding gives 0.001 quantization for steering/speed,
    and x255 gives ~0.004 quantization for throttle/braking."""

    def test_steering_quantization(self):
        val = 0.1234
        decoded = decode_steering(encode_steering(val))
        assert abs(decoded - val) < STEER_TOL

    def test_range(self):
        for val in [0.001, 0.5, 1.0, 10.0, -10.0, 32.0]:
            assert abs(decode_steering(encode_steering(val)) - val) < STEER_TOL


# ── Frame type constants ─────────────────────────────────────────────

class TestFrameTypeConstants:
    def test_esp_types_in_range(self):
        esp_types = [ESP_ACT_SPEED, ESP_ACT_ACCELERATION, ESP_ACT_BRAKING,
                     ESP_ACT_STEERING, ESP_HEARTBEAT, ESP_HEALTH_STATUS]
        for t in esp_types:
            assert 0x01 <= t <= 0x1F, f"ESP type 0x{t:02X} not in 0x01-0x1F"

    def test_orin_types_in_range(self):
        orin_types = [ORIN_TARG_THROTTLE, ORIN_TARG_BRAKING, ORIN_TARG_STEERING]
        for t in orin_types:
            assert 0x20 <= t <= 0x3F, f"Orin type 0x{t:02X} not in 0x20-0x3F"

    def test_no_overlap(self):
        esp_types = {ESP_ACT_SPEED, ESP_ACT_ACCELERATION, ESP_ACT_BRAKING,
                     ESP_ACT_STEERING, ESP_HEARTBEAT, ESP_HEALTH_STATUS}
        orin_types = {ORIN_TARG_THROTTLE, ORIN_TARG_BRAKING, ORIN_TARG_STEERING}
        assert esp_types.isdisjoint(orin_types)
