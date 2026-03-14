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

MISSIONS = {
    "manual": 0,
    "remote_control": 7,
    "acceleration": 1,
    "skidpad": 2,
    "autocross": 3,
    "trackdrive": 4,
    "ebs_test": 5,
    "inspection": 6,
}


# ── Decoders (ESP32 → Orin) ─────────────────────────────────────────

def decode_steering(payload) -> float:
    """Decode steering payload (1 int32: angle_rad x 1000) to float radians."""
    if len(payload) < 1:
        return 0.0
    return payload[0] / 1000.0


def decode_steering_raw(payload) -> tuple:
    """Decode steering payload to (angle_rad, raw_encoder).
    Payload: [angle_rad x 1000, raw_encoder]
    """
    if len(payload) < 1:
        return 0.0, 0
    angle_rad = payload[0] / 1000.0
    raw_encoder = payload[1] if len(payload) >= 2 else 0
    return angle_rad, raw_encoder


def decode_speed(payload) -> float:
    """Decode speed payload (1 int32: speed_mps x 1000) to float m/s."""
    if len(payload) < 1:
        return 0.0
    return payload[0] / 1000.0


def decode_accel(payload) -> tuple:
    """Decode acceleration payload to (lateral, longitudinal) m/s^2.
    Payload: [lateral x 1000, longitudinal x 1000]
    """
    if len(payload) < 2:
        return 0.0, 0.0
    return payload[0] / 1000.0, payload[1] / 1000.0


def decode_braking(payload) -> float:
    """Decode braking payload (1 int32: effort x 255) to float 0.0-1.0."""
    if len(payload) < 1:
        return 0.0
    return payload[0] / 255.0


def decode_throttle(payload) -> float:
    """Decode throttle payload (1 int32: effort x 255) to float 0.0-1.0."""
    if len(payload) < 1:
        return 0.0
    return payload[0] / 255.0


def decode_health(payload) -> dict:
    """Decode health payload to dict.
    Payload: [flags, agc, heap_kb, i2c_errors]
    flags: bit0=magnet_ok, bit1=i2c_ok, bit2=heap_ok
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
    """Decode heartbeat payload to uptime_ms."""
    if len(payload) < 1:
        return 0
    return payload[0]


# ── Encoders (Orin → ESP32) ─────────────────────────────────────────

def encode_steering(angle_rad: float) -> list:
    """Encode steering target as [angle_rad x 1000]."""
    return [int(angle_rad * 1000)]


def encode_throttle(effort: float) -> list:
    """Encode throttle target as [effort x 255]."""
    return [int(effort * 255)]


def encode_braking(effort: float) -> list:
    """Encode braking target as [effort x 255]."""
    return [int(effort * 255)]


# ── Encoders (ESP32 → Orin, used by sim node) ───────────────────────

def encode_act_steering(angle_rad: float, raw_encoder: int = 0) -> list:
    """Encode steering feedback as [angle_rad x 1000, raw_encoder]."""
    return [int(angle_rad * 1000), raw_encoder]


def encode_act_speed(speed_mps: float) -> list:
    """Encode speed as [speed_mps x 1000]."""
    return [int(speed_mps * 1000)]


def encode_act_accel(lateral: float, longitudinal: float) -> list:
    """Encode acceleration as [lateral x 1000, longitudinal x 1000]."""
    return [int(lateral * 1000), int(longitudinal * 1000)]


def encode_act_braking(effort: float) -> list:
    """Encode braking feedback as [effort x 255]."""
    return [int(effort * 255)]


def encode_act_throttle(effort: float) -> list:
    """Encode throttle feedback as [effort x 255]."""
    return [int(effort * 255)]


def encode_heartbeat(uptime_ms: int = 0) -> list:
    """Encode heartbeat as [uptime_ms]."""
    return [uptime_ms]


def encode_health(magnet_ok, i2c_ok, heap_ok, agc, heap_kb, i2c_errors) -> list:
    """Encode health status as [flags, agc, heap_kb, i2c_errors]."""
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
        }
        self._heartbeat_time = 0.0

    def update(self, key, value):
        with self.lock:
            self.data[key] = value

    def heartbeat(self):
        with self.lock:
            self._heartbeat_time = time.time()
            self.data["esp32_heartbeat"] = True

    def snapshot(self) -> dict:
        with self.lock:
            d = dict(self.data)
            if self._heartbeat_time > 0:
                d["esp32_heartbeat_age"] = round(time.time() - self._heartbeat_time, 1)
                d["esp32_heartbeat"] = d["esp32_heartbeat_age"] < 3.0
            return d
