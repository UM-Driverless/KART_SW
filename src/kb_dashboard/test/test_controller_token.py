"""End-to-end tests for the dashboard WebSocket controller token system.

Tests verify that only one browser can send manual_control at a time,
tokens can be stolen via take_control, and tokens are released on
disconnect or mission change.
"""

import asyncio
import json
import os
import socket
import threading
import time
import base64
import hashlib

import pytest

from kb_dashboard.protocol import DashboardState
from kb_dashboard.server import run_websocket_server


# ── Helpers ───────────────────────────────────────────────────────────


def _find_free_port():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("", 0))
        return s.getsockname()[1]


class FakeNode:
    """Minimal mock replacing DashboardNode for server tests."""

    def __init__(self):
        self.published_missions = []
        self.manual_control_calls = []
        self.state_cmd_calls = []

    def publish_mission(self, mission):
        self.published_missions.append(mission)

    def publish_manual_control(self, *, steer, steer_type, throttle, brake):
        self.manual_control_calls.append(
            {"steer": steer, "steer_type": steer_type, "throttle": throttle, "brake": brake}
        )

    def publish_state_cmd(self, cmd):
        self.state_cmd_calls.append(cmd)

    def get_logger(self):
        return self

    def info(self, msg):
        pass

    def warn(self, msg):
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
            await run_websocket_server(
                self.state, self.node, self.port, ready_callback=self._ready.set
            )

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
    return s


def _ws_read_frame(sock):
    """Read one WebSocket frame from a blocking socket, return (opcode, payload)."""
    header = sock.recv(2)
    if len(header) < 2:
        return None, None
    opcode = header[0] & 0x0F
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
    return opcode, payload


def _ws_read_text(sock):
    """Read frames until we get a text frame, return parsed JSON dict."""
    while True:
        opcode, payload = _ws_read_frame(sock)
        if opcode is None:
            return None
        if opcode == 0x1:  # text
            return json.loads(payload)


def _ws_read_welcome(sock):
    """Read the welcome message containing your_id, skipping broadcast snapshots."""
    # The welcome is sent immediately on connect, but a broadcast snapshot
    # might arrive first if the broadcast loop fires between handshake and welcome read.
    for _ in range(10):
        msg = _ws_read_text(sock)
        if msg is None:
            return None
        if "your_id" in msg:
            return msg
    return None


def _ws_send_text(sock, text: str):
    """Send a masked WebSocket text frame."""
    data = text.encode()
    mask = os.urandom(4)
    masked = bytes(b ^ mask[i % 4] for i, b in enumerate(data))
    frame = bytearray([0x81, 0x80 | len(data)])
    frame.extend(mask)
    frame.extend(masked)
    sock.sendall(bytes(frame))


def _ws_close(sock):
    """Send a WebSocket close frame and close the socket."""
    mask = os.urandom(4)
    frame = bytearray([0x88, 0x80 | 0])  # close frame, 0 payload, masked
    frame.extend(mask)
    try:
        sock.sendall(bytes(frame))
    except OSError:
        pass
    sock.close()


def _send_manual_control(sock, steering=0.5, throttle=0.3):
    """Send a manual_control command."""
    _ws_send_text(
        sock,
        json.dumps({
            "action": "manual_control",
            "steering": steering,
            "steer_type": "pwm",
            "throttle": throttle,
            "brake": 0.0,
        }),
    )


# ── Tests ─────────────────────────────────────────────────────────────


class TestControllerTokenAutoAcquire:
    """First manual_control from any client auto-acquires the token."""

    def test_first_manual_control_auto_acquires(self, srv):
        s = _blocking_ws_connect(srv.port)
        # Read the welcome message
        welcome = _ws_read_text(s)
        assert "your_id" in welcome

        _send_manual_control(s, steering=0.7, throttle=0.4)
        time.sleep(0.3)

        assert len(srv.node.manual_control_calls) == 1
        call = srv.node.manual_control_calls[0]
        assert call["steer"] == 0.7
        assert call["throttle"] == 0.4

        _ws_close(s)


class TestControllerTokenExclusion:
    """Second client's manual_control is ignored when first client holds the token."""

    def test_second_client_manual_control_ignored(self, srv):
        # Client A connects and auto-acquires token
        a = _blocking_ws_connect(srv.port)
        _ws_read_text(a)  # welcome
        _send_manual_control(a, steering=0.1)
        time.sleep(0.2)
        assert len(srv.node.manual_control_calls) == 1

        # Client B connects and tries to send manual_control
        b = _blocking_ws_connect(srv.port)
        _ws_read_text(b)  # welcome
        _send_manual_control(b, steering=0.9)
        time.sleep(0.2)

        # Only client A's command should have been forwarded
        assert len(srv.node.manual_control_calls) == 1
        assert srv.node.manual_control_calls[0]["steer"] == 0.1

        _ws_close(a)
        _ws_close(b)

    def test_holder_can_keep_sending(self, srv):
        """The token holder can send multiple manual_control commands."""
        a = _blocking_ws_connect(srv.port)
        _ws_read_text(a)  # welcome
        _send_manual_control(a, steering=0.1)
        time.sleep(0.1)
        _send_manual_control(a, steering=0.2)
        time.sleep(0.1)
        _send_manual_control(a, steering=0.3)
        time.sleep(0.2)

        assert len(srv.node.manual_control_calls) == 3
        steers = [c["steer"] for c in srv.node.manual_control_calls]
        assert steers == [0.1, 0.2, 0.3]

        _ws_close(a)


class TestTakeControl:
    """take_control action steals the token from the current holder."""

    def test_take_control_steals_token(self, srv):
        # Client A auto-acquires
        a = _blocking_ws_connect(srv.port)
        _ws_read_text(a)
        _send_manual_control(a, steering=0.1)
        time.sleep(0.2)
        assert len(srv.node.manual_control_calls) == 1

        # Client B connects and takes control
        b = _blocking_ws_connect(srv.port)
        _ws_read_text(b)
        _ws_send_text(b, json.dumps({"action": "take_control"}))
        time.sleep(0.2)

        # Now B's manual_control should work
        _send_manual_control(b, steering=0.8)
        time.sleep(0.2)
        assert len(srv.node.manual_control_calls) == 2
        assert srv.node.manual_control_calls[1]["steer"] == 0.8

        # A's manual_control should now be ignored
        _send_manual_control(a, steering=0.5)
        time.sleep(0.2)
        assert len(srv.node.manual_control_calls) == 2  # unchanged

        _ws_close(a)
        _ws_close(b)


class TestTokenReleasedOnDisconnect:
    """Token is released when the holder disconnects."""

    def test_disconnect_releases_token(self, srv):
        # Client A acquires token
        a = _blocking_ws_connect(srv.port)
        _ws_read_text(a)
        _send_manual_control(a, steering=0.1)
        time.sleep(0.2)
        assert len(srv.node.manual_control_calls) == 1

        # Client B is connected but doesn't have the token
        b = _blocking_ws_connect(srv.port)
        _ws_read_text(b)

        # A disconnects — token should be released
        _ws_close(a)
        time.sleep(0.3)

        # Now B can auto-acquire via manual_control
        _send_manual_control(b, steering=0.6)
        time.sleep(0.2)
        assert len(srv.node.manual_control_calls) == 2
        assert srv.node.manual_control_calls[1]["steer"] == 0.6

        _ws_close(b)


class TestTokenReleasedOnMissionChange:
    """Token is released when holder switches away from remote_control."""

    def test_mission_change_releases_token(self, srv):
        # Client A acquires token
        a = _blocking_ws_connect(srv.port)
        _ws_read_text(a)
        _send_manual_control(a, steering=0.1)
        time.sleep(0.2)
        assert len(srv.node.manual_control_calls) == 1

        # Client B is connected
        b = _blocking_ws_connect(srv.port)
        _ws_read_text(b)

        # A switches mission to "autocross" (not remote_control) — releases token
        _ws_send_text(a, json.dumps({"action": "set_mission", "mission": "autocross"}))
        time.sleep(0.2)

        # Now B can auto-acquire via manual_control
        _send_manual_control(b, steering=0.7)
        time.sleep(0.2)
        assert len(srv.node.manual_control_calls) == 2
        assert srv.node.manual_control_calls[1]["steer"] == 0.7

        _ws_close(a)
        _ws_close(b)

    def test_switching_to_remote_control_keeps_token(self, srv):
        """Switching TO remote_control does NOT release the token."""
        a = _blocking_ws_connect(srv.port)
        _ws_read_text(a)
        _send_manual_control(a, steering=0.1)
        time.sleep(0.2)

        # Switch to remote_control — token should remain with A
        _ws_send_text(a, json.dumps({"action": "set_mission", "mission": "remote_control"}))
        time.sleep(0.2)

        # A can still send
        _send_manual_control(a, steering=0.2)
        time.sleep(0.2)
        assert len(srv.node.manual_control_calls) == 2

        # B cannot
        b = _blocking_ws_connect(srv.port)
        _ws_read_text(b)
        _send_manual_control(b, steering=0.9)
        time.sleep(0.2)
        assert len(srv.node.manual_control_calls) == 2  # unchanged

        _ws_close(a)
        _ws_close(b)


class TestControllerInBroadcast:
    """Controller ID is included in broadcast snapshots."""

    def test_broadcast_includes_controller_none(self, srv):
        """Before anyone takes control, controller is null."""
        s = _blocking_ws_connect(srv.port)
        welcome = _ws_read_text(s)

        # Wait for a broadcast snapshot (the broadcast loop runs at 10 Hz)
        snap = _ws_read_text(s)
        assert snap is not None
        assert "controller" in snap
        assert snap["controller"] is None

        _ws_close(s)

    def test_broadcast_includes_controller_id(self, srv):
        """After acquiring control, controller shows the holder's ID."""
        a = _blocking_ws_connect(srv.port)
        welcome_a = _ws_read_welcome(a)
        assert welcome_a is not None, "Did not receive welcome message"
        client_id_a = welcome_a["your_id"]

        # Auto-acquire token
        _send_manual_control(a, steering=0.1)
        time.sleep(0.2)

        # Read broadcast — controller should be A's ID
        snap = _ws_read_text(a)
        assert snap is not None
        assert snap["controller"] == client_id_a

        _ws_close(a)

    def test_broadcast_updates_after_take_control(self, srv):
        """After B takes control, broadcast shows B's ID."""
        a = _blocking_ws_connect(srv.port)
        _ws_read_welcome(a)

        _send_manual_control(a, steering=0.1)
        time.sleep(0.2)

        b = _blocking_ws_connect(srv.port)
        welcome_b = _ws_read_welcome(b)
        assert welcome_b is not None, "Did not receive welcome message"
        client_id_b = welcome_b["your_id"]

        _ws_send_text(b, json.dumps({"action": "take_control"}))
        time.sleep(0.2)

        # Read a broadcast from B's connection
        snap = _ws_read_text(b)
        assert snap is not None
        assert snap["controller"] == client_id_b

        _ws_close(a)
        _ws_close(b)


class TestReleaseControl:
    """Explicit release_control action releases the token."""

    def test_release_control(self, srv):
        a = _blocking_ws_connect(srv.port)
        _ws_read_text(a)
        _send_manual_control(a, steering=0.1)
        time.sleep(0.2)
        assert len(srv.node.manual_control_calls) == 1

        # Release control
        _ws_send_text(a, json.dumps({"action": "release_control"}))
        time.sleep(0.2)

        # B can now auto-acquire
        b = _blocking_ws_connect(srv.port)
        _ws_read_text(b)
        _send_manual_control(b, steering=0.5)
        time.sleep(0.2)
        assert len(srv.node.manual_control_calls) == 2
        assert srv.node.manual_control_calls[1]["steer"] == 0.5

        _ws_close(a)
        _ws_close(b)

    def test_release_by_non_holder_is_noop(self, srv):
        """A client that doesn't hold the token can't release it."""
        a = _blocking_ws_connect(srv.port)
        _ws_read_text(a)
        _send_manual_control(a, steering=0.1)
        time.sleep(0.2)

        b = _blocking_ws_connect(srv.port)
        _ws_read_text(b)
        _ws_send_text(b, json.dumps({"action": "release_control"}))
        time.sleep(0.2)

        # A should still hold the token
        _send_manual_control(a, steering=0.2)
        time.sleep(0.2)
        assert len(srv.node.manual_control_calls) == 2

        # B should still be blocked
        _send_manual_control(b, steering=0.9)
        time.sleep(0.2)
        assert len(srv.node.manual_control_calls) == 2  # unchanged

        _ws_close(a)
        _ws_close(b)
