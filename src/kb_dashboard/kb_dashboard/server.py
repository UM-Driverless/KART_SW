"""HTTP + WebSocket server — no ROS dependencies."""

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
    """Minimal HTTP + WebSocket server using only the stdlib + asyncio.

    `node` must have .publish_mission(str) and .get_logger().info(str).
    `ready_callback` is called (no args) once the server is listening.
    """
    clients: set[asyncio.StreamWriter] = set()

    async def ws_accept(reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
        # Read HTTP request
        request = b""
        while True:
            line = await reader.readline()
            if not line:  # EOF — client disconnected
                writer.close()
                return
            request += line
            if line == b"\r\n":
                break

        request_str = request.decode(errors="replace")
        first_line = request_str.split("\r\n")[0]
        path = first_line.split(" ")[1] if len(first_line.split(" ")) > 1 else "/"

        # Parse headers
        headers = {}
        for line_str in request_str.split("\r\n")[1:]:
            if ": " in line_str:
                k, v = line_str.split(": ", 1)
                headers[k.lower()] = v.strip()

        # WebSocket upgrade
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
            clients.add(writer)
            try:
                await handle_ws(reader, writer, state, node)
            finally:
                clients.discard(writer)
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

    async def handle_ws(reader, writer, state, node):
        """Handle incoming WebSocket frames (commands from browser)."""
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
                    handle_command(cmd, state, node)
                except Exception:
                    pass

    def handle_command(cmd: dict, state, node):
        action = cmd.get("action")
        if action == "set_mission":
            mission = cmd.get("mission", "manual")
            if mission in MISSIONS:
                state.update("mission", mission)
                node.publish_mission(mission)
        elif action == "set_state":
            new_state = cmd.get("state", "idle")
            if new_state in ("idle", "running", "ebs"):
                state.update("state", new_state)
                node.get_logger().info(f"State set: {new_state}")
        elif action == "manual_control":
            if hasattr(node, "publish_manual_control"):
                node.publish_manual_control(
                    steer=float(cmd.get("steering", 0.0)),
                    steer_type=cmd.get("steer_type", "angle"),
                    throttle=float(cmd.get("throttle", 0.0)),
                    brake=float(cmd.get("brake", 0.0)),
                )

    def ws_send(writer: asyncio.StreamWriter, data: bytes, opcode=0x1):
        nonlocal clients
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
        nonlocal clients
        frame_counter = 0
        while True:
            await asyncio.sleep(0.1)  # 10 Hz
            if not clients:
                continue
            # Telemetry JSON (every tick = 10 Hz)
            snapshot = json.dumps(state.snapshot()).encode()
            dead = set()
            for w in list(clients):
                try:
                    ws_send(w, snapshot)
                    await w.drain()
                except Exception:
                    dead.add(w)
            clients -= dead
            # HUD JPEG binary (every 3rd tick ≈ 3.3 Hz to save bandwidth)
            frame_counter += 1
            if frame_counter % 3 == 0 and hasattr(node, "get_hud_jpeg"):
                jpeg = node.get_hud_jpeg()
                if jpeg:
                    for w in list(clients):
                        try:
                            ws_send(w, jpeg, opcode=0x2)  # binary frame
                            await w.drain()
                        except Exception:
                            clients.discard(w)

    import socket

    server = await asyncio.start_server(
        ws_accept,
        "0.0.0.0",
        port,
        reuse_address=True,
        start_serving=False,
    )
    # SO_REUSEADDR is set by reuse_address, but ensure SO_REUSEPORT too
    for s in server.sockets:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    await server.start_serving()
    node.get_logger().info(f"Web server listening on 0.0.0.0:{port}")
    if ready_callback:
        ready_callback()

    await asyncio.gather(server.serve_forever(), broadcast_loop())
