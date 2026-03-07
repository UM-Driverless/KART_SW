"""Unit tests for payload decoding and DashboardState."""
import struct
import time
import threading

from kb_dashboard.protocol import (
    decode_steering,
    decode_u8,
    DashboardState,
    MISSIONS,
)


# ── decode_steering ───────────────────────────────────────────────────

class TestDecodeSteering:
    def test_zero(self):
        assert decode_steering([0, 0]) == 0.0

    def test_positive(self):
        # 0.25 rad → 250 → 0x00, 0xFA
        payload = list(struct.pack(">h", 250))
        assert abs(decode_steering(payload) - 0.25) < 1e-6

    def test_negative(self):
        # -0.5 rad → -500
        payload = list(struct.pack(">h", -500))
        assert abs(decode_steering(payload) - (-0.5)) < 1e-6

    def test_max_positive(self):
        payload = list(struct.pack(">h", 32767))
        assert abs(decode_steering(payload) - 32.767) < 1e-6

    def test_max_negative(self):
        payload = list(struct.pack(">h", -32768))
        assert abs(decode_steering(payload) - (-32.768)) < 1e-6

    def test_empty_payload(self):
        assert decode_steering([]) == 0.0

    def test_single_byte(self):
        assert decode_steering([0x01]) == 0.0

    def test_extra_bytes_ignored(self):
        payload = list(struct.pack(">h", 1000)) + [0xFF, 0xFF]
        assert abs(decode_steering(payload) - 1.0) < 1e-6


# ── decode_u8 ─────────────────────────────────────────────────────────

class TestDecodeU8:
    def test_zero(self):
        assert decode_u8([0]) == 0

    def test_max(self):
        assert decode_u8([255]) == 255

    def test_mid(self):
        assert decode_u8([128]) == 128

    def test_empty(self):
        assert decode_u8([]) == 0

    def test_extra_bytes(self):
        assert decode_u8([42, 99, 200]) == 42


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
