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
    encode_compressor_disable,
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
    def _frame(self, mv1=None, duty=0, mv2=None, state=0, sdc=0):
        """8-field frame plus the two calibrated millivolt fields."""
        f = [0, duty, 0, state, 100, 0, 0, sdc]
        if mv1 is not None:
            f.append(mv1)
            f.append(mv2 if mv2 is not None else 0)
        return f

    def test_bar_is_three_times_pin_volts(self):
        # The whole conversion: SDE5 gives 1 V/bar, the board's three equal 10k
        # resistors (R11/R12/R13) divide by three, so bar = 3 * V_pin. 7 bar puts
        # 2333 mV on the pin, 8 bar puts 2667 mV.
        assert abs(decode_pneumatic(self._frame(mv1=2333))["pneu_tank_bar"] - 7.0) < 0.01
        assert abs(decode_pneumatic(self._frame(mv1=2667))["pneu_tank_bar"] - 8.0) < 0.01
        assert abs(decode_pneumatic(self._frame(mv1=1000))["pneu_tank_bar"] - 3.0) < 0.01

    def test_calibrated_flag_tracks_the_mv_fields(self):
        assert decode_pneumatic(self._frame(mv1=2333))["pneu_calibrated"] is True
        assert decode_pneumatic([2500, 0])["pneu_calibrated"] is False

    def test_saturated_pin_is_not_a_pressure(self):
        # At/above the 11 dB ceiling the ADC is pegged, and a pegged channel cannot
        # be told from a fault. It must read "--", never a confident number. This
        # also means the top of the sensor's span is unreachable: the divider maps
        # 10 bar to 3333 mV, well past the ceiling.
        assert decode_pneumatic(self._frame(mv1=2900))["pneu_tank_bar"] is None
        assert decode_pneumatic(self._frame(mv1=3100))["pneu_tank_bar"] is None
        assert decode_pneumatic(self._frame(mv1=2899))["pneu_tank_bar"] is not None

    def test_compressor_on(self):
        fields = decode_pneumatic(self._frame(mv1=1000, duty=153))
        assert fields["esp32_compressor_on"] is True
        assert fields["esp32_compressor_duty"] == 153

    def test_raw_fallback_for_old_firmware(self):
        # Pre-2026-07-27 firmware sends no millivolts, so the decoder falls back to
        # the assumed 3.3 V full scale. Approximate by construction -- asserted here
        # so the fallback is known to work, not because the number is trusted.
        fields = decode_pneumatic([2500, 0])
        assert abs(fields["pneu_tank_bar"] - 6.04) < 0.02
        assert fields["pneu_calibrated"] is False

    def test_full_scale_raw_is_none(self):
        assert decode_pneumatic([4095, 0])["pneu_tank_bar"] is None

    def test_compressor_off(self):
        fields = decode_pneumatic(self._frame(mv1=1365, duty=0))
        assert fields["esp32_compressor_on"] is False
        assert fields["esp32_compressor_duty"] == 0

    def test_short_payload(self):
        fields = decode_pneumatic([100])
        assert fields["pneu_tank_bar"] is None
        assert fields["esp32_compressor_on"] is False
        assert fields["esp32_sdc_closed"] is None

    def test_pump_stall_state(self):
        # 4 = a full burst raised no pressure, so pumping is latched off and the
        # shutdown circuit is held open. Note mv1=0 is a LEGITIMATE empty tank --
        # the SDE5's characteristic starts at 0 V -- so an empty tank and a dead
        # sensor cannot be told apart by voltage. This state is how they are
        # distinguished: by whether pumping actually achieves anything.
        f = decode_pneumatic(self._frame(mv1=0, state=4))
        assert f["esp32_compressor_state"] == 4
        assert f["esp32_compressor_on"] is False
        assert f["pneu_tank_bar"] == 0.0     # 0 bar is a real reading, not an error

    def test_over_range_state(self):
        f = decode_pneumatic(self._frame(mv1=2950, state=5))
        assert f["esp32_compressor_state"] == 5
        assert f["pneu_tank_bar"] is None    # pegged is not a pressure

    def test_compressor_disabled_state(self):
        # state 3 = operator latch. Duty is 0, same as idle and cooldown, so the
        # state field is the only thing that distinguishes them.
        fields = decode_pneumatic([2500, 0, 0, 3, 100, 0, 0, 0])
        assert fields["esp32_compressor_state"] == 3
        assert fields["esp32_compressor_on"] is False

    def test_sdc_closed_and_open(self):
        closed = decode_pneumatic([2858, 0, 0, 0, 100, 0, 0, 1])
        assert closed["esp32_sdc_closed"] is True
        opened = decode_pneumatic([2858, 0, 0, 3, 100, 0, 0, 0])
        assert opened["esp32_sdc_closed"] is False

    def test_sdc_absent_is_not_open(self):
        # Seven-field frame from firmware predating the SDC readback. A missing
        # field must read as "no data", never as an open chain — those are
        # different conditions and only one of them is an emergency.
        fields = decode_pneumatic([2858, 0, 0, 1, 100, 255, 0])
        assert fields["esp32_sdc_closed"] is None

    def test_sdc_minus_one_reads_as_no_data(self):
        # -1 is the firmware's "this build has no SDC pin" sentinel. Both current
        # targets do define one, so nothing sends it today — it exists so that a
        # build without the pin degrades to "no data" instead of to "chain open".
        fields = decode_pneumatic([2858, 0, 0, 0, 100, 0, 0, -1])
        assert fields["esp32_sdc_closed"] is None


# ── encode_compressor_disable ────────────────────────────────────────

class TestEncodeCompressorDisable:
    def test_polarity(self):
        # 1 = disabled. The firmware's object store zero-initialises, so 0 must be
        # the "compressor runs normally" case — see COMPRESSOR_DISABLED in
        # kart-medulla's km_objects.h.
        assert encode_compressor_disable(True) == [1]
        assert encode_compressor_disable(False) == [0]


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
