"""Round-trip tests for encode/decode protocol functions."""
import struct

from kb_dashboard.protocol import (
    decode_steering,
    decode_u8,
    decode_health,
    encode_int16_be,
    encode_u8,
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


# ── encode_int16_be ──────────────────────────────────────────────────

class TestEncodeInt16Be:
    def test_zero(self):
        assert encode_int16_be(0.0) == [0, 0]

    def test_positive(self):
        result = encode_int16_be(0.25)
        assert result == list(struct.pack(">h", 250))

    def test_negative(self):
        result = encode_int16_be(-0.5)
        assert result == list(struct.pack(">h", -500))

    def test_clamp_high(self):
        result = encode_int16_be(100.0)
        assert result == list(struct.pack(">h", 32767))

    def test_clamp_low(self):
        result = encode_int16_be(-100.0)
        assert result == list(struct.pack(">h", -32768))

    def test_two_bytes(self):
        assert len(encode_int16_be(1.0)) == 2


# ── encode_u8 ────────────────────────────────────────────────────────

class TestEncodeU8:
    def test_zero(self):
        assert encode_u8(0.0) == [0]

    def test_one(self):
        assert encode_u8(1.0) == [255]

    def test_mid(self):
        assert encode_u8(0.5) == [127]

    def test_clamp_high(self):
        assert encode_u8(2.0) == [255]

    def test_clamp_low(self):
        assert encode_u8(-1.0) == [0]


# ── Steering round-trip ──────────────────────────────────────────────

class TestSteeringRoundTrip:
    def _roundtrip(self, value):
        encoded = encode_int16_be(value)
        return decode_steering(encoded)

    def test_zero(self):
        assert self._roundtrip(0.0) == 0.0

    def test_positive(self):
        assert abs(self._roundtrip(0.25) - 0.25) < 0.001

    def test_negative(self):
        assert abs(self._roundtrip(-0.5) - (-0.5)) < 0.001

    def test_small(self):
        assert abs(self._roundtrip(0.001) - 0.001) < 0.001

    def test_large(self):
        assert abs(self._roundtrip(1.5) - 1.5) < 0.001

    def test_with_raw_encoder(self):
        """Full 4-byte steering payload: int16 angle + uint16 raw encoder."""
        angle = 0.3
        raw_encoder = 2048 + int(angle * 650)
        payload = encode_int16_be(angle) + [(raw_encoder >> 8) & 0xFF, raw_encoder & 0xFF]
        assert len(payload) == 4
        decoded_angle = decode_steering(payload)
        assert abs(decoded_angle - angle) < 0.001


# ── Speed round-trip ─────────────────────────────────────────────────

class TestSpeedRoundTrip:
    def test_zero(self):
        assert decode_steering(encode_int16_be(0.0)) == 0.0

    def test_positive(self):
        assert abs(decode_steering(encode_int16_be(5.0)) - 5.0) < 0.001

    def test_negative(self):
        assert abs(decode_steering(encode_int16_be(-2.5)) - (-2.5)) < 0.001


# ── Acceleration round-trip ──────────────────────────────────────────

class TestAccelerationRoundTrip:
    def test_lat_lon_payload(self):
        lat, lon = 1.5, -0.8
        payload = encode_int16_be(lat) + encode_int16_be(lon)
        assert len(payload) == 4
        decoded_lat = decode_steering(payload[:2])
        decoded_lon = decode_steering(payload[2:4])
        assert abs(decoded_lat - lat) < 0.001
        assert abs(decoded_lon - lon) < 0.001

    def test_zero_accel(self):
        payload = encode_int16_be(0.0) + encode_int16_be(0.0)
        assert decode_steering(payload[:2]) == 0.0
        assert decode_steering(payload[2:4]) == 0.0


# ── Throttle/Braking round-trip ──────────────────────────────────────

class TestThrottleBrakingRoundTrip:
    def _roundtrip(self, value):
        encoded = encode_u8(value)
        return decode_u8(encoded) / 255.0

    def test_zero(self):
        assert self._roundtrip(0.0) == 0.0

    def test_full(self):
        assert self._roundtrip(1.0) == 1.0

    def test_mid(self):
        assert abs(self._roundtrip(0.5) - 0.5) < 0.005

    def test_quarter(self):
        assert abs(self._roundtrip(0.25) - 0.25) < 0.005


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
