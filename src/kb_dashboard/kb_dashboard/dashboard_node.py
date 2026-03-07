#!/usr/bin/env python3
"""
kb_dashboard — Phone dashboard for kart telemetry and mission control.

Runs a WebSocket server alongside a ROS2 node. Any phone/browser on the
same network can open http://<orin-ip>:8080 to see live sensor values
and send commands (mission select, start/stop, EBS).
"""

import asyncio
import json
import struct
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from kb_interfaces.msg import Frame
from std_msgs.msg import String

# Frame type constants (from kb_interfaces/msg/Frame)
ESP_ACT_SPEED = 0x01
ESP_ACT_ACCELERATION = 0x02
ESP_ACT_BRAKING = 0x03
ESP_ACT_STEERING = 0x04
ESP_HEARTBEAT = 0x08
ORIN_TARG_THROTTLE = 0x20
ORIN_TARG_BRAKING = 0x21
ORIN_TARG_STEERING = 0x22

# Mission IDs
MISSIONS = {
    "manual": 0,
    "acceleration": 1,
    "skidpad": 2,
    "autocross": 3,
    "trackdrive": 4,
    "ebs_test": 5,
    "inspection": 6,
}

HTML_PATH = Path(__file__).parent / "index.html"


def decode_steering(payload: list[int]) -> float:
    """Decode int16 big-endian steering (rad × 1000) → float radians."""
    if len(payload) >= 2:
        raw = struct.unpack(">h", bytes(payload[:2]))[0]
        return raw / 1000.0
    return 0.0


def decode_u8(payload: list[int]) -> int:
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
            "esp32_acceleration": 0.0,
            "esp32_braking": 0,
            "orin_cmd_throttle": 0,
            "orin_cmd_brake": 0,
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


class DashboardNode(Node):
    def __init__(self, state: DashboardState):
        super().__init__("kb_dashboard")
        self.state = state
        self.declare_parameter("port", 8080)
        self.port = self.get_parameter("port").value

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # ESP32 → Orin telemetry
        self.create_subscription(Frame, "/esp32/heartbeat", self._on_heartbeat, qos)
        self.create_subscription(Frame, "/esp32/steering", self._on_esp_steering, qos)
        self.create_subscription(Frame, "/esp32/speed", self._on_esp_speed, qos)
        self.create_subscription(Frame, "/esp32/acceleration", self._on_esp_accel, qos)
        self.create_subscription(Frame, "/esp32/braking", self._on_esp_braking, qos)

        # Orin → ESP32 commands (to show what we're sending)
        self.create_subscription(Frame, "/orin/throttle", self._on_orin_throttle, qos)
        self.create_subscription(Frame, "/orin/brake", self._on_orin_brake, qos)
        self.create_subscription(Frame, "/orin/steering", self._on_orin_steering, qos)

        # Publishers for mission commands
        self.mission_pub = self.create_publisher(String, "/dashboard/mission", 10)

        self.get_logger().info(f"Dashboard node started, web UI on port {self.port}")

    def _on_heartbeat(self, msg: Frame):
        self.state.heartbeat()

    def _on_esp_steering(self, msg: Frame):
        self.state.update("esp32_steering_rad", decode_steering(list(msg.payload)))

    def _on_esp_speed(self, msg: Frame):
        if msg.payload:
            self.state.update("esp32_speed", decode_steering(list(msg.payload)))

    def _on_esp_accel(self, msg: Frame):
        if msg.payload:
            self.state.update("esp32_acceleration", decode_steering(list(msg.payload)))

    def _on_esp_braking(self, msg: Frame):
        self.state.update("esp32_braking", decode_u8(list(msg.payload)))

    def _on_orin_throttle(self, msg: Frame):
        self.state.update("orin_cmd_throttle", decode_u8(list(msg.payload)))

    def _on_orin_brake(self, msg: Frame):
        self.state.update("orin_cmd_brake", decode_u8(list(msg.payload)))

    def _on_orin_steering(self, msg: Frame):
        self.state.update("orin_cmd_steering_rad", decode_steering(list(msg.payload)))

    def publish_mission(self, mission: str):
        msg = String()
        msg.data = mission
        self.mission_pub.publish(msg)
        self.state.update("mission", mission)
        self.get_logger().info(f"Mission set: {mission}")


# ── Web server (asyncio) ──────────────────────────────────────────────

async def run_websocket_server(state: DashboardState, node: DashboardNode, port: int):
    """Minimal HTTP + WebSocket server using only the stdlib + asyncio."""
    import hashlib
    import base64

    clients: set[asyncio.StreamWriter] = set()

    async def ws_accept(reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
        # Read HTTP request
        request = b""
        while True:
            line = await reader.readline()
            request += line
            if line == b"\r\n":
                break

        request_str = request.decode(errors="replace")
        first_line = request_str.split("\r\n")[0]
        path = first_line.split(" ")[1] if len(first_line.split(" ")) > 1 else "/"

        # Parse headers
        headers = {}
        for line in request_str.split("\r\n")[1:]:
            if ": " in line:
                k, v = line.split(": ", 1)
                headers[k.lower()] = v

        # WebSocket upgrade
        if "upgrade" in headers.get("connection", "").lower() and "websocket" in headers.get("upgrade", "").lower():
            key = headers.get("sec-websocket-key", "")
            accept = base64.b64encode(
                hashlib.sha1((key + "258EAFA5-E914-47DA-95CA-5AB5A4085B64").encode()).digest()
            ).decode()
            writer.write(
                f"HTTP/1.1 101 Switching Protocols\r\n"
                f"Upgrade: websocket\r\n"
                f"Connection: Upgrade\r\n"
                f"Sec-WebSocket-Accept: {accept}\r\n\r\n".encode()
            )
            await writer.drain()
            clients.add(writer)
            try:
                await handle_ws(reader, writer, state, node)
            finally:
                clients.discard(writer)
            return

        # Serve HTML
        if path == "/" or path == "/index.html":
            html = HTML_PATH.read_text()
            writer.write(
                f"HTTP/1.1 200 OK\r\n"
                f"Content-Type: text/html; charset=utf-8\r\n"
                f"Content-Length: {len(html.encode())}\r\n"
                f"Cache-Control: no-cache\r\n\r\n".encode() + html.encode()
            )
        else:
            writer.write(b"HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\n\r\n")
        await writer.drain()
        writer.close()

    async def handle_ws(reader, writer, state, node):
        """Handle incoming WebSocket frames (commands from browser)."""
        while True:
            try:
                header = await asyncio.wait_for(reader.readexactly(2), timeout=30.0)
            except (asyncio.TimeoutError, asyncio.IncompleteReadError, ConnectionError):
                break

            opcode = header[0] & 0x0F
            if opcode == 0x8:  # close
                break
            if opcode == 0x9:  # ping → pong
                ws_send(writer, b"", opcode=0xA)
                continue

            masked = bool(header[1] & 0x80)
            length = header[1] & 0x7F
            if length == 126:
                length = int.from_bytes(await reader.readexactly(2), "big")
            elif length == 127:
                length = int.from_bytes(await reader.readexactly(8), "big")

            if masked:
                mask = await reader.readexactly(4)
                raw = await reader.readexactly(length)
                data = bytes(b ^ mask[i % 4] for i, b in enumerate(raw))
            else:
                data = await reader.readexactly(length)

            if opcode == 0x1:  # text
                try:
                    cmd = json.loads(data.decode())
                    handle_command(cmd, state, node)
                except Exception:
                    pass

    def handle_command(cmd: dict, state: DashboardState, node: DashboardNode):
        action = cmd.get("action")
        if action == "set_mission":
            mission = cmd.get("mission", "manual")
            if mission in MISSIONS:
                node.publish_mission(mission)
        elif action == "set_state":
            new_state = cmd.get("state", "idle")
            if new_state in ("idle", "running", "ebs"):
                state.update("state", new_state)
                node.get_logger().info(f"State set: {new_state}")

    def ws_send(writer: asyncio.StreamWriter, data: bytes, opcode=0x1):
        frame = bytearray()
        frame.append(0x80 | opcode)
        if len(data) < 126:
            frame.append(len(data))
        elif len(data) < 65536:
            frame.append(126)
            frame.extend(len(data).to_bytes(2, "big"))
        else:
            frame.append(127)
            frame.extend(len(data).to_bytes(8, "big"))
        frame.extend(data)
        try:
            writer.write(bytes(frame))
        except Exception:
            clients.discard(writer)

    async def broadcast_loop():
        while True:
            await asyncio.sleep(0.1)  # 10 Hz
            if not clients:
                continue
            snapshot = json.dumps(state.snapshot()).encode()
            dead = set()
            for w in list(clients):
                try:
                    ws_send(w, snapshot)
                    await w.drain()
                except Exception:
                    dead.add(w)
            clients -= dead

    server = await asyncio.start_server(ws_accept, "0.0.0.0", port)
    node.get_logger().info(f"Web server listening on 0.0.0.0:{port}")

    await asyncio.gather(server.serve_forever(), broadcast_loop())


# ── Entrypoint ─────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    state = DashboardState()
    node = DashboardNode(state)

    # Run ROS spinning in a background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Run the async web server in the main thread
    try:
        asyncio.run(run_websocket_server(state, node, node.port))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
