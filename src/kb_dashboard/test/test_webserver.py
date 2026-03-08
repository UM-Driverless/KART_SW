"""Tests for the HTTP/WebSocket server (no ROS dependency)."""
import asyncio
import base64
import hashlib
import json
import os
import socket
import threading
import time

import pytest

from kb_dashboard.protocol import DashboardState
from kb_dashboard.server import run_websocket_server


def _find_free_port():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("", 0))
        return s.getsockname()[1]


class FakeNode:
    """Minimal mock replacing DashboardNode for server tests."""

    def __init__(self):
        self.published_missions = []

    def publish_mission(self, mission):
        self.published_missions.append(mission)

    def get_logger(self):
        return self

    def info(self, msg):
        pass


class ServerFixture:
    """Manages a server running in a background thread."""

    def __init__(self):
        self.port = _find_free_port()
        self.state = DashboardState()
        self.node = FakeNode()
        self._loop = asyncio.new_event_loop()
        self._task = None
        self._ready = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()
        if not self._ready.wait(timeout=5):
            raise RuntimeError("Server did not start in time")

    def stop(self):
        if self._task:
            self._loop.call_soon_threadsafe(self._task.cancel)

    def _run(self):
        asyncio.set_event_loop(self._loop)

        async def _start():
            await run_websocket_server(self.state, self.node, self.port, ready_callback=self._ready.set)

        self._task = self._loop.create_task(_start())
        try:
            self._loop.run_until_complete(self._task)
        except asyncio.CancelledError:
            pass


@pytest.fixture()
def srv():
    s = ServerFixture()
    s.start()
    yield s
    s.stop()


def _blocking_http_get(port, path="/"):
    """Plain blocking HTTP GET using raw sockets."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(5)
    s.connect(("127.0.0.1", port))
    s.sendall(f"GET {path} HTTP/1.1\r\nHost: localhost\r\nConnection: close\r\n\r\n".encode())
    data = b""
    while True:
        try:
            chunk = s.recv(65536)
            if not chunk:
                break
            data += chunk
        except socket.timeout:
            break
    s.close()
    return data.decode(errors="replace")


def _blocking_ws_connect(port):
    """Blocking WebSocket handshake, returns the raw socket."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(5)
    s.connect(("127.0.0.1", port))
    key = "dGhlIHNhbXBsZSBub25jZQ=="
    req = (
        f"GET /ws HTTP/1.1\r\n"
        f"Host: localhost:{port}\r\n"
        f"Upgrade: websocket\r\n"
        f"Connection: Upgrade\r\n"
        f"Sec-WebSocket-Key: {key}\r\n"
        f"Sec-WebSocket-Version: 13\r\n"
        f"\r\n"
    )
    s.sendall(req.encode())
    resp = s.recv(1024)
    assert b"101 Switching Protocols" in resp, f"Bad handshake: {resp}"

    expected_accept = base64.b64encode(
        hashlib.sha1((key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode()).digest()
    ).decode()
    assert expected_accept.encode() in resp
    return s


def _ws_read_frame(sock):
    """Read one WebSocket frame from a blocking socket, return payload bytes."""
    header = sock.recv(2)
    if len(header) < 2:
        return None
    length = header[1] & 0x7F
    if length == 126:
        length = int.from_bytes(sock.recv(2), "big")
    elif length == 127:
        length = int.from_bytes(sock.recv(8), "big")
    payload = b""
    while len(payload) < length:
        chunk = sock.recv(length - len(payload))
        if not chunk:
            break
        payload += chunk
    return payload


def _ws_send_text(sock, text: str):
    """Send a masked WebSocket text frame."""
    data = text.encode()
    mask = os.urandom(4)
    masked = bytes(b ^ mask[i % 4] for i, b in enumerate(data))
    frame = bytearray([0x81, 0x80 | len(data)])
    frame.extend(mask)
    frame.extend(masked)
    sock.sendall(bytes(frame))


# ── HTTP Tests ─────────────────────────────────────────────────────────

class TestHTTP:
    def test_serves_html(self, srv):
        resp = _blocking_http_get(srv.port)
        assert "HTTP/1.1 200 OK" in resp
        assert "UM Driverless" in resp
        assert "text/html" in resp

    def test_404_for_unknown_path(self, srv):
        resp = _blocking_http_get(srv.port, "/nonexistent")
        assert "404" in resp


# ── WebSocket Tests ────────────────────────────────────────────────────

class TestWebSocket:
    def test_handshake(self, srv):
        s = _blocking_ws_connect(srv.port)
        s.close()

    def test_receives_telemetry(self, srv):
        srv.state.update("esp32_steering_rad", 0.314)
        s = _blocking_ws_connect(srv.port)
        payload = _ws_read_frame(s)
        assert payload is not None
        msg = json.loads(payload)
        assert abs(msg["esp32_steering_rad"] - 0.314) < 1e-6
        assert msg["mission"] == "manual"
        assert msg["state"] == "idle"
        s.close()

    def test_mission_command(self, srv):
        s = _blocking_ws_connect(srv.port)
        _ws_send_text(s, json.dumps({"action": "set_mission", "mission": "autocross"}))
        time.sleep(0.3)
        assert "autocross" in srv.node.published_missions
        assert srv.state.snapshot()["mission"] == "autocross"
        s.close()

    def test_state_command(self, srv):
        s = _blocking_ws_connect(srv.port)
        _ws_send_text(s, json.dumps({"action": "set_state", "state": "running"}))
        time.sleep(0.3)
        assert srv.state.snapshot()["state"] == "running"
        s.close()

    def test_invalid_mission_ignored(self, srv):
        s = _blocking_ws_connect(srv.port)
        _ws_send_text(s, json.dumps({"action": "set_mission", "mission": "nonexistent"}))
        time.sleep(0.2)
        assert srv.node.published_missions == []
        assert srv.state.snapshot()["mission"] == "manual"
        s.close()

    def test_invalid_state_ignored(self, srv):
        s = _blocking_ws_connect(srv.port)
        _ws_send_text(s, json.dumps({"action": "set_state", "state": "invalid"}))
        time.sleep(0.2)
        assert srv.state.snapshot()["state"] == "idle"
        s.close()
