"""Pure protocol helpers — no ROS dependencies."""
import struct
import threading
import time

# Frame type constants (from kb_interfaces/msg/Frame)
ESP_ACT_SPEED = 0x01
ESP_ACT_ACCELERATION = 0x02
ESP_ACT_BRAKING = 0x03
ESP_ACT_STEERING = 0x04
ESP_HEARTBEAT = 0x08
ORIN_TARG_THROTTLE = 0x20
ORIN_TARG_BRAKING = 0x21
ORIN_TARG_STEERING = 0x22

MISSIONS = {
    "manual": 0,
    "acceleration": 1,
    "skidpad": 2,
    "autocross": 3,
    "trackdrive": 4,
    "ebs_test": 5,
    "inspection": 6,
}


def decode_steering(payload) -> float:
    """Decode int16 big-endian steering (rad * 1000) to float radians."""
    if len(payload) >= 2:
        raw = struct.unpack(">h", bytes(payload[:2]))[0]
        return raw / 1000.0
    return 0.0


def decode_u8(payload) -> int:
    return payload[0] if payload else 0


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
            "orin_cmd_throttle": 0,   # target throttle 0-255
            "orin_cmd_brake": 0,      # target brake 0-255
            "orin_cmd_steering_rad": 0.0,
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
