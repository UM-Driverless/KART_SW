"""HTTP + WebSocket server — no ROS dependencies.

Features:
- Single controller token: only one browser can send manual_control at a time.
  Others see who has control and can "Take Control" to steal it.
- Non-blocking broadcast: slow clients don't block the event loop.
"""

import asyncio
import base64
import hashlib
import json
from pathlib import Path

from kb_dashboard.protocol import DashboardState, MISSIONS

HTML_PATH = Path(__file__).parent / "index.html"


async def run_websocket_server(
    state: DashboardState, node, port: int, ready_callback=None
):
    """@brief HTTP + WebSocket server for the dashboard.

    @param state Shared DashboardState for telemetry snapshots.
    @param node ROS node with publish_mission(), publish_state_cmd(), publish_manual_control().
    @param port TCP port to listen on.
    @param ready_callback Optional callable invoked once the server is listening.
    """
    clients: dict[asyncio.StreamWriter, str] = {}  # writer → client_id
    controller: dict = {"holder": None, "id": None}  # who has manual control
    _next_id = [0]

    def _make_id():
        _next_id[0] += 1
        return f"browser-{_next_id[0]}"

    async def ws_accept(reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
        request = b""
        while True:
            line = await reader.readline()
            if not line:
                writer.close()
                return
            request += line
            if line == b"\r\n":
                break

        request_str = request.decode(errors="replace")
        first_line = request_str.split("\r\n")[0]
        path = first_line.split(" ")[1] if len(first_line.split(" ")) > 1 else "/"

        headers = {}
        for line_str in request_str.split("\r\n")[1:]:
            if ": " in line_str:
                k, v = line_str.split(": ", 1)
                headers[k.lower()] = v.strip()

        conn_header = headers.get("connection", "").lower()
        upgrade_header = headers.get("upgrade", "").lower()
        if "upgrade" in conn_header and "websocket" in upgrade_header:
            key = headers.get("sec-websocket-key", "").strip()
            accept = base64.b64encode(
                hashlib.sha1(
                    (key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode()
                ).digest()
            ).decode()
            writer.write(
                f"HTTP/1.1 101 Switching Protocols\r\n"
                f"Upgrade: websocket\r\n"
                f"Connection: Upgrade\r\n"
                f"Sec-WebSocket-Accept: {accept}\r\n\r\n".encode()
            )
            await writer.drain()
            client_id = _make_id()
            clients[writer] = client_id
            node.get_logger().info(f"WS connected: {client_id}")
            # Send welcome with client ID
            ws_send(writer, json.dumps({"your_id": client_id}).encode())
            try:
                await handle_ws(reader, writer, client_id, state, node)
            finally:
                clients.pop(writer, None)
                if controller["holder"] is writer:
                    controller["holder"] = None
                    controller["id"] = None
                    node.get_logger().info(f"Controller released: {client_id} disconnected")
                node.get_logger().info(f"WS disconnected: {client_id}")
            return

        # Serve HTML
        if path == "/" or path == "/index.html":
            body = HTML_PATH.read_bytes()
            header = (
                f"HTTP/1.1 200 OK\r\n"
                f"Content-Type: text/html; charset=utf-8\r\n"
                f"Content-Length: {len(body)}\r\n"
                f"Connection: close\r\n"
                f"Cache-Control: no-cache\r\n\r\n"
            ).encode()
            writer.write(header + body)
        else:
            writer.write(
                b"HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\nConnection: close\r\n\r\n"
            )
        await writer.drain()
        writer.close()
        await writer.wait_closed()

    async def handle_ws(reader, writer, client_id, state, node):
        while True:
            try:
                header = await reader.readexactly(2)
            except (asyncio.IncompleteReadError, ConnectionError, OSError):
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
                    handle_command(cmd, writer, client_id, state, node)
                except Exception as e:
                    node.get_logger().warn(f"WS cmd error from {client_id}: {e}")

    def handle_command(cmd: dict, writer, client_id, state, node):
        action = cmd.get("action")
        if action == "set_mission":
            mission = cmd.get("mission", "manual")
            if mission in MISSIONS:
                state.update("mission", mission)
                node.publish_mission(mission)
                # Release control token when switching away from remote_control
                if mission != "remote_control" and controller["holder"] is writer:
                    controller["holder"] = None
                    controller["id"] = None
        elif action == "set_state":
            new_state = cmd.get("state", "idle")
            if new_state in ("idle", "running", "ebs"):
                state.update("state", new_state)
                cmd_map = {"idle": "stop", "running": "start", "ebs": "ebs"}
                if hasattr(node, "publish_state_cmd"):
                    node.publish_state_cmd(cmd_map[new_state])
        elif action == "take_control":
            old_id = controller["id"]
            controller["holder"] = writer
            controller["id"] = client_id
            if old_id and old_id != client_id:
                node.get_logger().info(f"Control taken by {client_id} (was {old_id})")
            else:
                node.get_logger().info(f"Control acquired by {client_id}")
        elif action == "release_control":
            if controller["holder"] is writer:
                controller["holder"] = None
                controller["id"] = None
                node.get_logger().info(f"Control released by {client_id}")
        elif action == "set_controller":
            ctrl_type = cmd.get("type", "geometric")
            if ctrl_type in ("geometric", "pure_pursuit", "neural_v2"):
                state.update("controller_type", ctrl_type)
                if hasattr(node, "publish_controller_type"):
                    node.publish_controller_type(ctrl_type)
        elif action == "set_steer_mode":
            mode = cmd.get("mode", "pid")  # "pid" or "pwm"
            if hasattr(node, "publish_steer_mode"):
                node.publish_steer_mode(1 if mode == "pwm" else 0)
        elif action == "manual_control":
            # Auto-acquire control on first manual_control if nobody has it
            if controller["holder"] is None:
                controller["holder"] = writer
                controller["id"] = client_id
                node.get_logger().info(f"Control auto-acquired by {client_id}")
            # Only the controller can send manual commands
            if controller["holder"] is writer:
                if hasattr(node, "publish_manual_control"):
                    node.publish_manual_control(
                        steer=float(cmd.get("steering", 0.0)),
                        steer_type=cmd.get("steer_type", "angle"),
                        throttle=float(cmd.get("throttle", 0.0)),
                        brake=float(cmd.get("brake", 0.0)),
                    )

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
            clients.pop(writer, None)

    async def broadcast_loop():
        frame_counter = 0
        while True:
            await asyncio.sleep(0.1)  # 10 Hz
            if not clients:
                continue

            # Build snapshot with controller info
            snap = state.snapshot()
            snap["controller"] = controller["id"]

            snapshot_bytes = json.dumps(snap).encode()
            dead = set()
            for w in list(clients):
                try:
                    ws_send(w, snapshot_bytes)
                    # Non-blocking drain with timeout to prevent slow clients from stalling
                    await asyncio.wait_for(w.drain(), timeout=0.5)
                except Exception:
                    dead.add(w)
            for w in dead:
                clients.pop(w, None)
                if controller["holder"] is w:
                    controller["holder"] = None
                    controller["id"] = None

            # HUD JPEG binary (every 3rd tick ≈ 3.3 Hz)
            frame_counter += 1
            if frame_counter % 3 == 0 and hasattr(node, "get_hud_jpeg"):
                jpeg = node.get_hud_jpeg()
                if jpeg:
                    for w in list(clients):
                        try:
                            ws_send(w, jpeg, opcode=0x2)
                            await asyncio.wait_for(w.drain(), timeout=0.5)
                        except Exception:
                            clients.pop(w, None)

    import socket

    server = await asyncio.start_server(
        ws_accept,
        "0.0.0.0",
        port,
        reuse_address=True,
        start_serving=False,
    )
    for s in server.sockets:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    await server.start_serving()
    node.get_logger().info(f"Web server listening on 0.0.0.0:{port}")
    if ready_callback:
        ready_callback()

    await asyncio.gather(server.serve_forever(), broadcast_loop())
