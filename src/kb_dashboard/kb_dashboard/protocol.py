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

    Payload: [angle_rad x 1000, raw_encoder, pid_out x 1000].

    @param payload List of int32 values from the Frame.
    @return Tuple of (angle in radians, raw AS5600 encoder value, PID output -1.0 to 1.0).
    """
    if len(payload) < 1:
        return 0.0, 0, 0.0
    angle_rad = payload[0] / 1000.0
    raw_encoder = payload[1] if len(payload) >= 2 else 0
    pid_pwm = payload[2] / 1000.0 if len(payload) >= 3 else 0.0
    return angle_rad, raw_encoder, pid_pwm


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


def decode_health(payload) -> dict:
    """@brief Decode health payload to dict.

    Payload: [flags, agc, heap_kb, i2c_errors].
    Flags: bit0=magnet_ok, bit1=i2c_ok, bit2=heap_ok.

    @param payload List of int32 values from the Frame.
    @return Dict with health_magnet_ok, health_i2c_ok, health_heap_ok, etc.
    """
    if len(payload) < 4:
        return {
            "health_magnet_ok": False,
            "health_i2c_ok": False,
            "health_heap_ok": False,
            "health_agc": 0,
            "health_heap_kb": 0,
            "health_i2c_errors": 0,
        }
    flags = payload[0]
    return {
        "health_magnet_ok": bool(flags & 0x01),
        "health_i2c_ok": bool(flags & 0x02),
        "health_heap_ok": bool(flags & 0x04),
        "health_agc": payload[1],
        "health_heap_kb": payload[2],
        "health_i2c_errors": payload[3],
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
            "orin_cmd_steering_rad": 0.0,
            "health_magnet_ok": False,
            "health_i2c_ok": False,
            "health_heap_ok": False,
            "health_agc": 0,
            "health_heap_kb": 0,
            "health_i2c_errors": 0,
            "yolo_fps": 0.0,
            "mission": "manual",
            "state": "idle",  # idle | running | ebs
            "steer_mode": "pid",  # "pid" or "pwm"
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
