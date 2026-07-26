"""Pure protocol helpers — no ROS dependencies.

Payload encoding: int32 arrays. Each value is a signed 32-bit integer.
Floats are scaled to int32 before transmission:
  - Steering angle: radians x 1000  (milliradians)
  - Throttle/braking effort: x 255  (0-255 range)
  - Speed: m/s x 1000
  - Acceleration: m/s^2 x 1000

The framing (SOF/LEN/TYPE/CRC) is unchanged — only payload encoding changed.
"""
import threading
import time

# Frame type constants (from kb_interfaces/msg/Frame)
ESP_ACT_SPEED = 0x01
ESP_ACT_ACCELERATION = 0x02
ESP_ACT_BRAKING = 0x03
ESP_ACT_STEERING = 0x04
ESP_HEARTBEAT = 0x08
ESP_HEALTH_STATUS = 0x0B
ORIN_TARG_THROTTLE = 0x20
ORIN_TARG_BRAKING = 0x21
ORIN_TARG_STEERING = 0x22
ORIN_STEER_MODE = 0x29
ORIN_COMPRESSOR_DISABLE = 0x2A

MISSIONS = {
    "manual": 0,
    "remote_control": 7,
    "autonomous": 8,
    "acceleration": 1,
    "skidpad": 2,
    "autocross": 3,
    "trackdrive": 4,
    "ebs_test": 5,
    "inspection": 6,
}


# ── Decoders (ESP32 → Orin) ─────────────────────────────────────────

def decode_steering(payload) -> float:
    """@brief Decode steering payload (1 int32: angle_rad x 1000) to float radians.

    @param payload List of int32 values from the Frame.
    @return Steering angle in radians.
    """
    if len(payload) < 1:
        return 0.0
    return payload[0] / 1000.0


def decode_steering_raw(payload) -> tuple:
    """@brief Decode steering payload to (angle_rad, raw_encoder, pid_pwm).

    Payload: [angle_rad x 1000, raw_encoder, pid_out x 1000, valid].

    The 4th field was appended when the steering sensor moved from an AS5600 on
    I2C to an MT6701 read over PWM: 1 = the angle is a real measurement, 0 = the
    firmware has no angle and the first field is a sentinel, not a reading.

    @param payload List of int32 values from the Frame.
    @return Tuple of (angle in radians, raw 12-bit encoder value,
            PID output -1.0 to 1.0, angle-is-valid bool).
    """
    if len(payload) < 1:
        return 0.0, 0, 0.0, False
    angle_rad = payload[0] / 1000.0
    raw_encoder = payload[1] if len(payload) >= 2 else 0
    pid_pwm = payload[2] / 1000.0 if len(payload) >= 3 else 0.0
    # Field 4 is the firmware's own verdict on whether the angle is real. When it
    # says no, the angle field carries INT32_MIN rather than a plausible number,
    # so both checks below catch it. Firmware predating the field sends 3 values;
    # treat that as valid, since it had no way to say otherwise.
    if len(payload) >= 4:
        valid = bool(payload[3]) and payload[0] != -(2 ** 31)
    else:
        valid = payload[0] != -(2 ** 31)
    return angle_rad, raw_encoder, pid_pwm, valid


def decode_speed(payload) -> float:
    """@brief Decode speed payload (1 int32: speed_mps x 1000) to float m/s.

    @param payload List of int32 values from the Frame.
    @return Speed in m/s.
    """
    if len(payload) < 1:
        return 0.0
    return payload[0] / 1000.0


def decode_accel(payload) -> tuple:
    """@brief Decode acceleration payload to (lateral, longitudinal) in m/s^2.

    Payload: [lateral x 1000, longitudinal x 1000].

    @param payload List of int32 values from the Frame.
    @return Tuple of (lateral, longitudinal) acceleration in m/s^2.
    """
    if len(payload) < 2:
        return 0.0, 0.0
    return payload[0] / 1000.0, payload[1] / 1000.0


def decode_braking(payload) -> float:
    """@brief Decode braking payload (1 int32: effort x 255) to float 0.0-1.0.

    @param payload List of int32 values from the Frame.
    @return Braking effort normalized to 0.0-1.0.
    """
    if len(payload) < 1:
        return 0.0
    return payload[0] / 255.0


def decode_throttle(payload) -> float:
    """@brief Decode throttle payload (1 int32: effort x 255) to float 0.0-1.0.

    @param payload List of int32 values from the Frame.
    @return Throttle effort normalized to 0.0-1.0.
    """
    if len(payload) < 1:
        return 0.0
    return payload[0] / 255.0


def decode_health_flags(payload) -> dict:
    """@brief Decode the flags-only health payload from /esp32/health/flags.

    kb_coms_micro splits the ESP32's health frame across two topics: this one
    carries a single int32 of flag bits, and /esp32/health/data carries the
    numbers. Decoding either half with decode_health() does not work — it needs
    4 fields and returns all-False for anything shorter, which is exactly how the
    dashboard's health pills silently read "bad" while the ESP32 was healthy.

    @param payload List with one int32 of flag bits.
    @return Dict of the boolean health fields only.
    """
    flags = payload[0] if len(payload) >= 1 else 0
    return {
        "health_magnet_ok": bool(flags & 0x01),
        "health_i2c_ok": bool(flags & 0x02),
        "health_heap_ok": bool(flags & 0x04),
        "health_steer_ok": bool(flags & 0x08),
        "health_steer_trip": bool(flags & 0x10),
    }


def decode_health_data(payload) -> dict:
    """@brief Decode the numeric health payload from /esp32/health/data.

    kb_coms_micro currently forwards only [agc, heap_kb, i2c_errors]; the
    firmware's appended steering frame counters are dropped before they get
    here, so those are not decodable from this topic yet.

    @param payload List of int32: [agc, heap_kb, i2c_errors].
    @return Dict of the numeric health fields present.
    """
    out = {}
    if len(payload) >= 1:
        out["health_agc"] = payload[0]
    if len(payload) >= 2:
        out["health_heap_kb"] = payload[1]
    if len(payload) >= 3:
        out["health_i2c_errors"] = payload[2]
    return out


def decode_health(payload) -> dict:
    """@brief Decode health payload to dict.

    Payload: [flags, agc, heap_kb, i2c_errors, steer_frames, steer_rejects].
    Flags: bit0=magnet_ok, bit1=i2c_ok, bit2=heap_ok, bit3=steer_ok,
           bit4=steer_trip.

    bit0/bit1 and `agc` are AS5600-over-I2C facts. On the ESP32-S3 kart board the
    firmware no longer polls an AS5600 — the sensor is an MT6701 read over PWM —
    so those two bits are permanently False there and must NOT be used as
    "is the steering sensor working". bit3 is that answer. bit4 means the
    steering fault latched: the EBS fired and throttle is refused until reboot.

    steer_frames/steer_rejects separate the failure modes: frames flat at 0 means
    no signal edges at all, rejects climbing with frames flat means edges are
    arriving at the wrong rate.

    @param payload List of int32 values from the Frame.
    @return Dict with health_magnet_ok, health_i2c_ok, health_heap_ok, etc.
    """
    if len(payload) < 4:
        return {
            "health_magnet_ok": False,
            "health_i2c_ok": False,
            "health_heap_ok": False,
            "health_steer_ok": False,
            "health_steer_trip": False,
            "health_agc": 0,
            "health_heap_kb": 0,
            "health_i2c_errors": 0,
            "health_steer_frames": 0,
            "health_steer_rejects": 0,
        }
    flags = payload[0]
    return {
        "health_magnet_ok": bool(flags & 0x01),
        "health_i2c_ok": bool(flags & 0x02),
        "health_heap_ok": bool(flags & 0x04),
        "health_steer_ok": bool(flags & 0x08),
        "health_steer_trip": bool(flags & 0x10),
        "health_agc": payload[1],
        "health_heap_kb": payload[2],
        "health_i2c_errors": payload[3],
        "health_steer_frames": payload[4] if len(payload) >= 5 else 0,
        "health_steer_rejects": payload[5] if len(payload) >= 6 else 0,
    }


# Tank pressure calibration. Tank sensor = Festo SDE5-D10, part 567465, on PRESSURE_1
# (CN7.1 -> GPIO 6). The firmware sends the raw 12-bit ADC count; this converts it to
# bar with one gauge-anchored factor: on 2026-07-18 the mechanical tank gauge read
# 7.5 bar while this channel read ADC 2679, hence 7.5 / 2679 bar per count.
#
# CHANGED 2026-07-26. The previous comment here derived bar from the datasheet chain
# and explicitly warned "do not sync the two" against main.c's trip points. It was
# wrong on two counts at once, which is why its answer was ~16% low and why the
# dashboard drew the firmware's 7-and-8-bar hysteresis band as ~6.0 and ~6.9 bar:
#
#   1. ADC full scale is 2900 mV, not 3300. ESP32-S3 datasheet Table 5-6 ("ADC
#      Calibration Results"), copy in ~/dv/datasheets/esp32-s3_espressif_datasheet.pdf:
#      ATTEN3 has an effective measurement range of 0~2900 mV. ATTEN3 is
#      ADC_ATTEN_DB_11, which is what km_gpio.c sets for this channel.
#   2. The divider is very unlikely to be 3:1. The SDE5-D10 datasheet (567465, copy
#      in ~/dv/datasheets/) gives 0-10 bar -> 0-10 V, i.e. 1 V/bar with a 0 V zero
#      offset. Note the gauge reading fixes only the PRODUCT full_scale x ratio,
#      which is 0.0027995 * 4095 = 11.46 V — it cannot separate the two. So each
#      candidate ratio implies a required full scale:
#          3.00:1 -> 3.82 V     3.47:1 -> 3.30 V     3.95:1 -> 2.90 V
#      The S3's widest attenuation gives 2900 mV and the pin cannot exceed VDDA
#      (~3.3 V), so no configuration produces 3.82 V: 3:1 is inconsistent with the
#      gauge under any real ADC setup, while 3.95 lands exactly on the datasheet
#      figure. 4:1 is also the sane design choice — 10 bar -> 2.5 V, inside the
#      range with margin, where 3:1 would clip a 10 bar tank at 8.7 bar.
#
#      This inference is only as good as the single gauge reading behind it. If that
#      gauge is off by ~30%, 3:1 becomes viable again. Hence "measure R11/R12/R13"
#      is still an open task rather than a settled fact.
#
# Those two corrections together give 2.9 * 4 / 4095 = 0.0028327 bar/count, which
# agrees with the gauge anchor's 0.0027995 to within 1.2%. Two independent datasheets
# and one mechanical gauge landing within ~1% of each other is the real reason to
# trust this number — not the gauge alone.
#
# STILL UNVERIFIED, in rough order of how much they could move the number: nobody has
# measured R11/R12/R13 to confirm 4:1 (predicted, not observed); the anchor is a single
# point, so an offset would tilt the whole scale; and the SDE5 itself is only +/-3 %FS.
# Under this factor ADC full scale computes to 11.46 bar, past the sensor's 10 bar
# span, so the top of the range is extrapolation. The settling measurement is a meter
# on the ADC pin read against the gauge and the raw count at the same instant, at two
# well-separated pressures — that separates divider, sensor and gauge in one pass.
# Open in tasks.md.
#
# Recalibrating means editing this constant AND main.c's ADC_PRESSURE_LOW/HIGH
# together: they are two expressions of the same calibration in different repos.
ADC_FULL_SCALE = 4095.0     # 12-bit
BAR_PER_ADC_COUNT = 7.5 / 2679.0   # gauge anchor, 2026-07-18 (see above)


def decode_pneumatic(payload) -> dict:
    """@brief Decode pneumatics telemetry to dict.

    Payload (8 fields): [pres1_adc, compressor_duty, pres2_adc, compressor_state,
    control_iters, ledc_readback, gpio_init_err, sdc_level]. pres1_adc is
    PRESSURE_1, the tank sensor. pres2_adc is PRESSURE_2, the piston/brake-line
    sensor. Both are raw 12-bit ADC (0-4095). compressor_duty is 0-255 with
    0 = MOSFET off. compressor_state is 0 idle / 1 running / 2 cooldown /
    3 disabled by the operator. sdc_level is the shutdown-circuit pin read back
    off the pin itself: 1 = chain closed, 0 = emergency asserted, -1 = no such
    pin on that build.

    Fields are only ever appended, so shorter payloads from older firmware still
    decode. Anything absent comes back None (or 0 for compressor_state) so it
    renders as "--" rather than as a confident wrong value — a missing sdc_level
    must not display as "chain open", which is a real and different condition.

    PRESSURE_3 exists on the board (GPIO 1) but is shared with the steering
    sensor's PWM-angle input and has no sensor fitted, so it is deliberately not sent.

    @param payload List of int32 values from the Frame.
    @return Dict with pneu_tank_bar, pneu_piston_bar (floats or None),
            esp32_compressor_on (bool), esp32_compressor_duty (int 0-255),
            esp32_compressor_state (int) and esp32_sdc_closed (bool or None).
    """
    if len(payload) < 2:
        return {
            "pneu_tank_bar": None,
            "pneu_piston_bar": None,
            "esp32_compressor_on": False,
            "esp32_compressor_duty": 0,
            "esp32_compressor_state": 0,
            "esp32_sdc_closed": None,
        }
    pressure_adc, comp_duty = payload[0], payload[1]
    # PRESSURE_2 is assumed to sit behind the same ÷3 divider as PRESSURE_1 and
    # is converted with the same map. UNVERIFIED — PRESSURE_1's factor was
    # anchored to a gauge reading on 2026-07-12, PRESSURE_2 has never been
    # checked against anything. Treat the piston number as indicative until it
    # is calibrated against a gauge the same way.
    # A channel pinned at full scale is NOT a reading. An unconnected ADC input floats to the
    # rail, and converting that gives a confident-looking 9.9 bar - which is what PRESSURE_2
    # displayed on 2026-07-25 with no sensor fitted. A genuine sensor pegged at full scale is
    # equally out of range and equally untrustworthy, so both collapse to None and the panel
    # shows "-- bar". Never emit a number that cannot be distinguished from a fault.
    pres2_adc = payload[2] if len(payload) >= 3 else None
    piston_bar = (
        round(BAR_PER_ADC_COUNT * pres2_adc, 2)
        if pres2_adc is not None and pres2_adc < ADC_FULL_SCALE
        else None
    )
    # -1 means the build has no SDC pin; anything else is the measured level.
    sdc_raw = payload[7] if len(payload) >= 8 else None
    return {
        "pneu_tank_bar": round(BAR_PER_ADC_COUNT * pressure_adc, 2),
        "pneu_piston_bar": piston_bar,
        "esp32_compressor_on": comp_duty > 0,
        "esp32_compressor_duty": comp_duty,
        "esp32_compressor_state": payload[3] if len(payload) >= 4 else 0,
        "esp32_sdc_closed": (sdc_raw == 1) if sdc_raw is not None and sdc_raw >= 0 else None,
    }


def decode_heartbeat(payload) -> int:
    """@brief Decode heartbeat payload to uptime in milliseconds.

    @param payload List of int32 values from the Frame.
    @return Uptime in milliseconds.
    """
    if len(payload) < 1:
        return 0
    return payload[0]


# ── Encoders (Orin → ESP32) ─────────────────────────────────────────

def encode_steering(angle_rad: float) -> list:
    """@brief Encode steering target as [angle_rad x 1000].

    @param angle_rad Steering angle in radians.
    @return List with one int32 element (milliradians).
    """
    return [int(angle_rad * 1000)]


def encode_throttle(effort: float) -> list:
    """@brief Encode throttle target as [effort x 255].

    @param effort Throttle effort 0.0-1.0.
    @return List with one int32 element (0-255).
    """
    return [int(effort * 255)]


def encode_braking(effort: float) -> list:
    """@brief Encode braking target as [effort x 255].

    @param effort Braking effort 0.0-1.0.
    @return List with one int32 element (0-255).
    """
    return [int(effort * 255)]


def encode_steer_mode(mode: int) -> list:
    """@brief Encode steering mode as [mode].

    @param mode 0=PID (angle target), 1=direct PWM.
    @return List with one int32 element.
    """
    return [int(mode)]


def encode_compressor_disable(disabled: bool) -> list:
    """@brief Encode the EBS compressor operator latch as [disabled].

    Sent as "disabled" rather than "enabled" to match the firmware's object
    store, where every value starts at 0 and 0 therefore has to mean "compressor
    runs normally" (see COMPRESSOR_DISABLED in kart-medulla's km_objects.h).

    @param disabled True to stop the compressor and force the shutdown circuit open.
    @return List with one int32 element (0 = run normally, 1 = disabled).
    """
    return [1 if disabled else 0]


# ── Encoders (ESP32 → Orin, used by sim node) ───────────────────────

def encode_act_steering(angle_rad: float, raw_encoder: int = 0) -> list:
    """@brief Encode steering feedback as [angle_rad x 1000, raw_encoder].

    @param angle_rad Steering angle in radians.
    @param raw_encoder Raw AS5600 encoder value (12-bit).
    @return List of two int32 elements.
    """
    return [int(angle_rad * 1000), raw_encoder]


def encode_act_speed(speed_mps: float) -> list:
    """@brief Encode speed feedback as [speed_mps x 1000].

    @param speed_mps Speed in meters per second.
    @return List with one int32 element.
    """
    return [int(speed_mps * 1000)]


def encode_act_accel(lateral: float, longitudinal: float) -> list:
    """@brief Encode acceleration feedback as [lateral x 1000, longitudinal x 1000].

    @param lateral Lateral acceleration in m/s^2.
    @param longitudinal Longitudinal acceleration in m/s^2.
    @return List of two int32 elements.
    """
    return [int(lateral * 1000), int(longitudinal * 1000)]


def encode_act_braking(effort: float) -> list:
    """@brief Encode braking feedback as [effort x 255].

    @param effort Braking effort 0.0-1.0.
    @return List with one int32 element (0-255).
    """
    return [int(effort * 255)]


def encode_act_throttle(effort: float) -> list:
    """@brief Encode throttle feedback as [effort x 255].

    @param effort Throttle effort 0.0-1.0.
    @return List with one int32 element (0-255).
    """
    return [int(effort * 255)]


def encode_heartbeat(uptime_ms: int = 0) -> list:
    """@brief Encode heartbeat as [uptime_ms].

    @param uptime_ms Uptime in milliseconds.
    @return List with one int32 element.
    """
    return [uptime_ms]


def encode_health(magnet_ok, i2c_ok, heap_ok, agc, heap_kb, i2c_errors) -> list:
    """@brief Encode health status as [flags, agc, heap_kb, i2c_errors].

    @param magnet_ok Whether the AS5600 magnet is detected.
    @param i2c_ok Whether I2C communication is healthy.
    @param heap_ok Whether free heap is above threshold.
    @param agc AS5600 automatic gain control value.
    @param heap_kb Free heap in kilobytes.
    @param i2c_errors Cumulative I2C error count.
    @return List of four int32 elements.
    """
    flags = 0
    if magnet_ok:
        flags |= 0x01
    if i2c_ok:
        flags |= 0x02
    if heap_ok:
        flags |= 0x04
    return [flags, int(agc), int(heap_kb), int(i2c_errors)]


class DashboardState:
    """Thread-safe telemetry state."""

    def __init__(self):
        """@brief Initialize with default telemetry values and a threading lock."""
        self.lock = threading.Lock()
        self.data = {
            "esp32_heartbeat": False,
            "esp32_heartbeat_age": -1.0,
            "esp32_steering_rad": 0.0,
            "esp32_speed": 0.0,
            "esp32_accel_lat": 0.0,   # lateral acceleration (m/s^2), positive = right
            "esp32_accel_lon": 0.0,   # longitudinal acceleration (m/s^2), positive = forward
            "esp32_throttle": 0.0,    # throttle pedal 0.0-1.0
            "esp32_braking": 0.0,     # brake pedal 0.0-1.0
            "orin_cmd_throttle": 0.0, # target throttle 0.0-1.0
            "orin_cmd_brake": 0.0,    # target brake 0.0-1.0
            "esp32_steering_raw": 0,
            "esp32_steering_valid": False,
            "orin_cmd_steering_rad": 0.0,
            "health_magnet_ok": False,
            "health_i2c_ok": False,
            "health_heap_ok": False,
            "health_steer_ok": False,   # firmware's own verdict on the steering sensor
            "health_steer_trip": False, # steering fault latched: EBS fired, throttle refused
            "health_agc": 0,
            "health_heap_kb": 0,
            "health_i2c_errors": 0,
            "health_steer_frames": 0,
            "health_steer_rejects": 0,
            "pneu_tank_bar": None,          # tank pressure (bar); None → dial shows "-- bar"
            "pneu_piston_bar": None,        # piston/brake-line pressure (bar); None → "-- bar"
            "esp32_compressor_on": False,   # EBS compressor MOSFET on/off
            "esp32_compressor_duty": 0,     # compressor PWM duty 0-255 (soft-start ramp)
            "esp32_compressor_state": 0,    # 0 idle / 1 running / 2 cooldown / 3 operator-disabled
            "compressor_disabled": False,   # the dashboard's own latch, echoed back to the UI
            # Shutdown circuit read back off the ESP32's Q3 gate pin: True = chain closed,
            # False = emergency asserted, None = firmware too old to report it. Not wired to
            # anything downstream yet (2026-07-26), so this reports the firmware's intent and
            # the pin's real level, and nothing about whether the kart would actually brake.
            "esp32_sdc_closed": None,
            # EBS (emergency brake system) — no signal reaches the Orin yet, nothing publishes
            # these. None means "not wired" and the dashboard renders it as NOT WIRED in grey.
            # Keep None as the default rather than a boolean: a safety indicator that reads
            # healthy because a field defaulted to False is worse than one that reads unknown.
            "ebs_state": None,              # None | "unavailable" | "armed" | "activated" (FS 2026 T 14.8)
            "ebs_valve_on": None,           # None | bool — electrovalve energised, i.e. brakes held off
            "yolo_fps": 0.0,
            "esp_fps": 0.0,
            "cones_3d_ground": [],  # [{"x": float, "z": float, "c": str}, ...] from /perception/cones_3d_ground
            "mission": "manual",
            "state": "idle",  # idle | running | ebs
            "steer_mode": "pid",  # "pid" or "pwm"
            "controller_type": "geometric",  # geometric | pure_pursuit | neural_v2 | mpc
            "speed_controller_type": "curve_factor",  # curve_factor | constant | neural_v2
        }
        self._heartbeat_time = 0.0

    def update(self, key, value):
        """@brief Thread-safe update of a single telemetry field.

        @param key Data key name (e.g. "esp32_speed").
        @param value New value to store.
        """
        with self.lock:
            self.data[key] = value

    def heartbeat(self):
        """@brief Record a heartbeat reception, updating the timestamp."""
        with self.lock:
            self._heartbeat_time = time.time()
            self.data["esp32_heartbeat"] = True

    def snapshot(self) -> dict:
        """@brief Return a thread-safe copy of all telemetry data with computed heartbeat age.

        @return Dict with all telemetry fields plus esp32_heartbeat_age.
        """
        with self.lock:
            d = dict(self.data)
            if self._heartbeat_time > 0:
                d["esp32_heartbeat_age"] = round(time.time() - self._heartbeat_time, 1)
                d["esp32_heartbeat"] = d["esp32_heartbeat_age"] < 3.0
            return d
