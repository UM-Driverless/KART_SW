"""Pure protocol helpers — no ROS dependencies.

Encoding uses protobuf (nanopb on ESP32, standard protobuf here).
The framing (SOF/LEN/TYPE/CRC) is unchanged — only payload encoding changed.
"""
import threading
import time

from .generated.kart_msgs_pb2 import (
    ActAcceleration,
    ActBraking,
    ActSpeed,
    ActSteering,
    Heartbeat,
    HealthStatus,
    TargBraking,
    TargSteering,
    TargThrottle,
)

# Frame type constants (from kb_interfaces/msg/Frame)
ESP_ACT_SPEED = 0x01
ESP_ACT_ACCELERATION = 0x02
ESP_ACT_BRAKING = 0x03
ESP_ACT_STEERING = 0x04
ESP_HEARTBEAT = 0x08
ORIN_TARG_THROTTLE = 0x20
ORIN_TARG_BRAKING = 0x21
ORIN_TARG_STEERING = 0x22
ESP_HEALTH_STATUS = 0x0B

MISSIONS = {
    "manual": 0,
    "acceleration": 1,
    "skidpad": 2,
    "autocross": 3,
    "trackdrive": 4,
    "ebs_test": 5,
    "inspection": 6,
}


# ── Decoders (ESP32 → Orin) ─────────────────────────────────────────

def decode_steering(payload) -> float:
    """Decode ActSteering protobuf payload to float radians."""
    msg = ActSteering()
    msg.ParseFromString(bytes(payload))
    return msg.angle_rad


def decode_steering_raw(payload) -> tuple:
    """Decode ActSteering protobuf payload to (angle_rad, raw_encoder)."""
    msg = ActSteering()
    msg.ParseFromString(bytes(payload))
    return msg.angle_rad, msg.raw_encoder


def decode_speed(payload) -> float:
    """Decode ActSpeed protobuf payload to float m/s."""
    msg = ActSpeed()
    msg.ParseFromString(bytes(payload))
    return msg.speed_mps


def decode_accel(payload) -> tuple:
    """Decode ActAcceleration protobuf payload to (lateral, longitudinal)."""
    msg = ActAcceleration()
    msg.ParseFromString(bytes(payload))
    return msg.lateral_mps2, msg.longitudinal_mps2


def decode_braking(payload) -> float:
    """Decode ActBraking protobuf payload to float effort 0.0-1.0."""
    msg = ActBraking()
    msg.ParseFromString(bytes(payload))
    return msg.effort


def decode_throttle(payload) -> float:
    """Decode TargThrottle protobuf payload to float effort 0.0-1.0."""
    msg = TargThrottle()
    msg.ParseFromString(bytes(payload))
    return msg.effort


def decode_health(payload) -> dict:
    """Decode HealthStatus protobuf payload to dict."""
    msg = HealthStatus()
    msg.ParseFromString(bytes(payload))
    return {
        "health_magnet_ok": msg.magnet_ok,
        "health_i2c_ok": msg.i2c_ok,
        "health_heap_ok": msg.heap_ok,
        "health_agc": msg.agc,
        "health_heap_kb": msg.heap_kb,
        "health_i2c_errors": msg.i2c_errors,
        "stack_comms": msg.stack_comms,
        "stack_control": msg.stack_control,
        "stack_heartbeat": msg.stack_heartbeat,
        "stack_health": msg.stack_health,
    }


def decode_heartbeat(payload) -> int:
    """Decode Heartbeat protobuf payload to uptime_ms."""
    msg = Heartbeat()
    msg.ParseFromString(bytes(payload))
    return msg.uptime_ms


# ── Encoders (Orin → ESP32) ─────────────────────────────────────────

def encode_steering(angle_rad: float) -> list:
    """Encode steering target as TargSteering protobuf bytes."""
    return list(TargSteering(angle_rad=angle_rad).SerializeToString())


def encode_throttle(effort: float) -> list:
    """Encode throttle target as TargThrottle protobuf bytes."""
    return list(TargThrottle(effort=effort).SerializeToString())


def encode_braking(effort: float) -> list:
    """Encode braking target as TargBraking protobuf bytes."""
    return list(TargBraking(effort=effort).SerializeToString())


# ── Encoders (ESP32 → Orin, used by sim node) ───────────────────────

def encode_act_steering(angle_rad: float, raw_encoder: int = 0) -> list:
    """Encode steering feedback as ActSteering protobuf bytes."""
    return list(ActSteering(angle_rad=angle_rad, raw_encoder=raw_encoder).SerializeToString())


def encode_act_speed(speed_mps: float) -> list:
    """Encode speed as ActSpeed protobuf bytes."""
    return list(ActSpeed(speed_mps=speed_mps).SerializeToString())


def encode_act_accel(lateral: float, longitudinal: float) -> list:
    """Encode acceleration as ActAcceleration protobuf bytes."""
    return list(ActAcceleration(lateral_mps2=lateral, longitudinal_mps2=longitudinal).SerializeToString())


def encode_act_braking(effort: float) -> list:
    """Encode braking feedback as ActBraking protobuf bytes."""
    return list(ActBraking(effort=effort).SerializeToString())


def encode_act_throttle(effort: float) -> list:
    """Encode throttle feedback as TargThrottle protobuf bytes (sim uses same msg)."""
    return list(TargThrottle(effort=effort).SerializeToString())


def encode_heartbeat(uptime_ms: int = 0) -> list:
    """Encode heartbeat as Heartbeat protobuf bytes."""
    return list(Heartbeat(uptime_ms=uptime_ms).SerializeToString())


def encode_health(magnet_ok, i2c_ok, heap_ok, agc, heap_kb, i2c_errors) -> list:
    """Encode health status as HealthStatus protobuf bytes."""
    return list(HealthStatus(
        magnet_ok=magnet_ok, i2c_ok=i2c_ok, heap_ok=heap_ok,
        agc=agc, heap_kb=heap_kb, i2c_errors=i2c_errors,
    ).SerializeToString())


class DashboardState:
    """Thread-safe telemetry state."""

    def __init__(self):
        self.lock = threading.Lock()
        self.data = {
            "esp32_heartbeat": False,
            "esp32_heartbeat_age": -1.0,
            "esp32_steering_rad": 0.0,
            "esp32_speed": 0.0,
            "esp32_accel_lat": 0.0,   # lateral acceleration (m/s²), positive = right
            "esp32_accel_lon": 0.0,   # longitudinal acceleration (m/s²), positive = forward
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
