"""Unit tests for payload decoding and DashboardState.

Precision: int32 x1000 gives 0.001 quant for steering/speed,
int32 x255 gives ~0.004 quant for throttle/braking.
"""
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
    decode_pneumatic,
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
        assert decode_steering(encode_steering(0.0)) == 0.0

    def test_positive(self):
        assert abs(decode_steering(encode_steering(0.25)) - 0.25) < 0.001

    def test_negative(self):
        assert abs(decode_steering(encode_steering(-0.5)) - (-0.5)) < 0.001

    def test_empty_payload(self):
        assert decode_steering([]) == 0.0

    def test_act_steering_angle(self):
        payload = encode_act_steering(0.42, 2500)
        angle = decode_steering(payload)
        assert abs(angle - 0.42) < 0.001


# ── decode with raw encoder ──────────────────────────────────────────

class TestDecodeSteeringRaw:
    def test_with_encoder(self):
        payload = encode_act_steering(0.3, 2243)
        angle, raw, pid_pwm, _valid = decode_steering_raw(payload)
        assert abs(angle - 0.3) < 0.001
        assert raw == 2243
        # encode_act_steering emits two elements, so the PID term is absent and reads 0.0
        assert pid_pwm == 0.0

    def test_without_encoder(self):
        payload = encode_act_steering(0.1)
        angle, raw, pid_pwm, _valid = decode_steering_raw(payload)
        assert abs(angle - 0.1) < 0.001
        assert raw == 0
        assert pid_pwm == 0.0

    def test_with_pid_term(self):
        # what the real firmware sends: [angle x1000, raw encoder, pid_out x1000]
        angle, raw, pid_pwm, _valid = decode_steering_raw([300, 2243, -750])
        assert abs(angle - 0.3) < 0.001
        assert raw == 2243
        assert abs(pid_pwm - (-0.75)) < 0.001

    def test_valid_flag_set(self):
        # 4-field frame from the MT6701 firmware, sensor reading normally
        _angle, _raw, _pid, valid = decode_steering_raw([1084, 1543, 0, 1])
        assert valid is True

    def test_invalid_flag_clears_valid(self):
        # firmware has no angle: flag 0 and the angle field is the INT32_MIN sentinel
        angle, _raw, _pid, valid = decode_steering_raw([-(2 ** 31), -1, 0, 0])
        assert valid is False
        # the angle is still decoded, but it is nonsense by construction — the
        # caller is expected to gate on `valid`, not to sanity-check the number
        assert angle < -1e6

    def test_sentinel_alone_is_enough(self):
        # defence in depth: sentinel angle with a mistakenly-set flag is still invalid
        _angle, _raw, _pid, valid = decode_steering_raw([-(2 ** 31), -1, 0, 1])
        assert valid is False

    def test_three_field_frame_is_treated_as_valid(self):
        # firmware predating the validity field had no way to say "no angle"
        _angle, _raw, _pid, valid = decode_steering_raw([300, 2243, -750])
        assert valid is True


# ── decode_speed ─────────────────────────────────────────────────────

class TestDecodeSpeed:
    def test_positive(self):
        assert abs(decode_speed(encode_act_speed(3.5)) - 3.5) < 0.001

    def test_zero(self):
        assert decode_speed(encode_act_speed(0.0)) == 0.0


# ── decode_accel ─────────────────────────────────────────────────────

class TestDecodeAccel:
    def test_values(self):
        lat, lon = decode_accel(encode_act_accel(1.5, -0.8))
        assert abs(lat - 1.5) < 0.001
        assert abs(lon - (-0.8)) < 0.001


# ── decode_throttle / decode_braking ─────────────────────────────────

class TestDecodeEffort:
    def test_throttle(self):
        assert abs(decode_throttle(encode_throttle(0.75)) - 0.75) < 0.004

    def test_braking(self):
        assert abs(decode_braking(encode_braking(0.3)) - 0.3) < 0.004

    def test_zero(self):
        assert decode_throttle(encode_throttle(0.0)) == 0.0
        assert decode_braking(encode_braking(0.0)) == 0.0


# ── decode_pneumatic ─────────────────────────────────────────────────

class TestDecodePneumatic:
    def test_compressor_on(self):
        # verified calibration: bar = 3.0 * (raw/4095 * 3.3); 1 bar ≈ 413.6 ADC.
        # duty 153 → compressor on.
        fields = decode_pneumatic([414, 153])
        assert abs(fields["pneu_tank_bar"] - 1.0) < 0.02
        assert fields["esp32_compressor_on"] is True
        assert fields["esp32_compressor_duty"] == 153

    def test_full_scale(self):
        # raw 4095 → ~9.9 bar (near the sensor's 10 bar top of range)
        fields = decode_pneumatic([4095, 0])
        assert abs(fields["pneu_tank_bar"] - 9.9) < 0.1

    def test_compressor_off(self):
        fields = decode_pneumatic([1365, 0])
        assert fields["esp32_compressor_on"] is False
        assert fields["esp32_compressor_duty"] == 0

    def test_short_payload(self):
        fields = decode_pneumatic([100])
        assert fields["pneu_tank_bar"] is None
        assert fields["esp32_compressor_on"] is False


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
        expected = {"manual", "remote_control", "autonomous", "acceleration", "skidpad", "autocross", "trackdrive", "ebs_test", "inspection"}
        assert set(MISSIONS.keys()) == expected
