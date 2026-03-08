"""Unit tests for payload decoding and DashboardState."""
import time
import threading

from kb_dashboard.protocol import (
    decode_steering,
    decode_steering_raw,
    decode_speed,
    decode_accel,
    decode_throttle,
    decode_braking,
    decode_health,
    encode_steering,
    encode_act_steering,
    encode_act_speed,
    encode_act_accel,
    encode_throttle,
    encode_braking,
    encode_health,
    DashboardState,
    MISSIONS,
)


# ── decode_steering ───────────────────────────────────────────────────

class TestDecodeSteering:
    def test_zero(self):
        # Empty proto (all defaults) → 0.0
        assert decode_steering(encode_steering(0.0)) == 0.0

    def test_positive(self):
        assert abs(decode_steering(encode_steering(0.25)) - 0.25) < 1e-6

    def test_negative(self):
        assert abs(decode_steering(encode_steering(-0.5)) - (-0.5)) < 1e-6

    def test_empty_payload(self):
        # Empty bytes = all defaults
        assert decode_steering([]) == 0.0
        assert decode_steering(b"") == 0.0

    def test_act_steering_angle(self):
        """decode_steering works on ActSteering payloads too (reads angle_rad field 1)."""
        payload = encode_act_steering(0.42, 2500)
        # ActSteering uses field 1 for angle_rad, same as TargSteering
        # But they're different message types — decode_steering uses ActSteering
        angle = decode_steering(payload)
        assert abs(angle - 0.42) < 1e-6


# ── decode with raw encoder ──────────────────────────────────────────

class TestDecodeSteeringRaw:
    def test_with_encoder(self):
        payload = encode_act_steering(0.3, 2243)
        angle, raw = decode_steering_raw(payload)
        assert abs(angle - 0.3) < 1e-6
        assert raw == 2243

    def test_without_encoder(self):
        payload = encode_act_steering(0.1)
        angle, raw = decode_steering_raw(payload)
        assert abs(angle - 0.1) < 1e-6
        assert raw == 0


# ── decode_speed ─────────────────────────────────────────────────────

class TestDecodeSpeed:
    def test_positive(self):
        assert abs(decode_speed(encode_act_speed(3.5)) - 3.5) < 1e-6

    def test_zero(self):
        assert decode_speed(encode_act_speed(0.0)) == 0.0


# ── decode_accel ─────────────────────────────────────────────────────

class TestDecodeAccel:
    def test_values(self):
        lat, lon = decode_accel(encode_act_accel(1.5, -0.8))
        assert abs(lat - 1.5) < 1e-6
        assert abs(lon - (-0.8)) < 1e-6


# ── decode_throttle / decode_braking ─────────────────────────────────

class TestDecodeEffort:
    def test_throttle(self):
        assert abs(decode_throttle(encode_throttle(0.75)) - 0.75) < 1e-6

    def test_braking(self):
        assert abs(decode_braking(encode_braking(0.3)) - 0.3) < 1e-6

    def test_zero(self):
        assert decode_throttle(encode_throttle(0.0)) == 0.0
        assert decode_braking(encode_braking(0.0)) == 0.0


# ── DashboardState ────────────────────────────────────────────────────

class TestDashboardState:
    def test_initial_values(self):
        s = DashboardState()
        snap = s.snapshot()
        assert snap["esp32_heartbeat"] is False
        assert snap["esp32_heartbeat_age"] == -1.0
        assert snap["esp32_steering_rad"] == 0.0
        assert snap["mission"] == "manual"
        assert snap["state"] == "idle"

    def test_update(self):
        s = DashboardState()
        s.update("esp32_steering_rad", 0.123)
        assert s.snapshot()["esp32_steering_rad"] == 0.123

    def test_heartbeat_sets_alive(self):
        s = DashboardState()
        s.heartbeat()
        snap = s.snapshot()
        assert snap["esp32_heartbeat"] is True
        assert 0.0 <= snap["esp32_heartbeat_age"] < 1.0

    def test_heartbeat_age_stale(self):
        s = DashboardState()
        s.heartbeat()
        # Simulate time passing by backdating the heartbeat
        with s.lock:
            s._heartbeat_time = time.time() - 5.0
        snap = s.snapshot()
        assert snap["esp32_heartbeat"] is False
        assert snap["esp32_heartbeat_age"] >= 4.0

    def test_snapshot_is_copy(self):
        s = DashboardState()
        snap = s.snapshot()
        snap["mission"] = "trackdrive"
        assert s.snapshot()["mission"] == "manual"

    def test_thread_safety(self):
        s = DashboardState()
        errors = []

        def writer():
            try:
                for i in range(100):
                    s.update("esp32_steering_rad", float(i))
                    s.heartbeat()
            except Exception as e:
                errors.append(e)

        def reader():
            try:
                for _ in range(100):
                    snap = s.snapshot()
                    assert isinstance(snap, dict)
            except Exception as e:
                errors.append(e)

        threads = [threading.Thread(target=writer), threading.Thread(target=reader)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        assert errors == []


# ── MISSIONS dict ─────────────────────────────────────────────────────

class TestMissions:
    def test_manual_is_zero(self):
        assert MISSIONS["manual"] == 0

    def test_all_unique_ids(self):
        ids = list(MISSIONS.values())
        assert len(ids) == len(set(ids))

    def test_expected_missions(self):
        expected = {"manual", "acceleration", "skidpad", "autocross", "trackdrive", "ebs_test", "inspection"}
        assert set(MISSIONS.keys()) == expected
