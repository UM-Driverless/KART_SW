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
import secrets
from pathlib import Path
from http.cookies import SimpleCookie

from kb_dashboard.protocol import DashboardState, MISSIONS

HTML_PATH = Path(__file__).parent / "index.html"

LOGIN_HTML = """\
<!DOCTYPE html>
<html lang="en"><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Kart Dashboard — Login</title>
<style>
  body { background: #0a0a0f; color: #e0e0e0; font-family: Inter, sans-serif;
         display: flex; justify-content: center; align-items: center; min-height: 100vh; }
  form { background: #14141f; padding: 32px; border-radius: 12px; text-align: center;
         box-shadow: 0 4px 24px rgba(0,0,0,0.5); }
  h1 { font-size: 18px; margin-bottom: 20px; }
  input { background: #1e1e2e; color: #fff; border: 1px solid #333; border-radius: 8px;
          padding: 10px 14px; font-size: 16px; width: 160px; text-align: center; }
  button { background: #2563eb; color: #fff; border: none; border-radius: 8px;
           padding: 10px 24px; font-size: 16px; cursor: pointer; margin-top: 12px; }
  button:active { background: #1d4ed8; }
  .err { color: #f87171; font-size: 13px; margin-top: 8px; }
</style>
</head><body>
<form method="POST" action="/login" autocomplete="on">
  <h1>Kart Dashboard</h1>
  <input type="text" name="username" id="username" value="kart" autocomplete="username"
         style="position:absolute;left:-9999px;width:1px;height:1px;overflow:hidden" tabindex="-1" aria-hidden="true">
  <input type="password" name="password" id="password" placeholder="Password" autocomplete="current-password" autofocus><br>
  <button type="submit">Enter</button>
  $error$
</form>
</body></html>
"""


def _parse_cookies(header_str: str) -> dict[str, str]:
    """Parse a Cookie header into a dict."""
    cookies = {}
    for item in header_str.split(";"):
        if "=" in item:
            k, v = item.strip().split("=", 1)
            cookies[k.strip()] = v.strip()
    return cookies


async def run_websocket_server(
    state: DashboardState, node, port: int, ready_callback=None, password: str = ""
):
    """@brief HTTP + WebSocket server for the dashboard.

    @param state Shared DashboardState for telemetry snapshots.
    @param node ROS node with publish_mission(), publish_state_cmd(), publish_manual_control().
    @param port TCP port to listen on.
    @param ready_callback Optional callable invoked once the server is listening.
    @param password If non-empty, require this password to access the dashboard.
    """
    # Derive session token from password so cookies survive server restarts.
    auth_token = hashlib.sha256(password.encode()).hexdigest()[:32]

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
        parts = first_line.split(" ")
        method = parts[0] if parts else "GET"
        path = parts[1] if len(parts) > 1 else "/"

        headers = {}
        for line_str in request_str.split("\r\n")[1:]:
            if ": " in line_str:
                k, v = line_str.split(": ", 1)
                headers[k.lower()] = v.strip()

        # --- Auth check ---
        def _is_authenticated() -> bool:
            if not password:
                return True
            cookies = _parse_cookies(headers.get("cookie", ""))
            return cookies.get("kart_session") == auth_token

        # Handle login POST
        if path == "/login" and method == "POST":
            # Read POST body (Content-Length)
            body_len = int(headers.get("content-length", "0"))
            body_data = b""
            if body_len > 0:
                body_data = await reader.readexactly(body_len)
            # Parse form: password=xxx
            form = {}
            for pair in body_data.decode(errors="replace").split("&"):
                if "=" in pair:
                    fk, fv = pair.split("=", 1)
                    form[fk] = fv
            if form.get("password") == password:
                resp = (
                    f"HTTP/1.1 303 See Other\r\n"
                    f"Location: /\r\n"
                    f"Set-Cookie: kart_session={auth_token}; Path=/; HttpOnly; SameSite=Lax; Max-Age=31536000; Secure\r\n"
                    f"Connection: close\r\n\r\n"
                ).encode()
            else:
                err_body = LOGIN_HTML.replace("$error$",'<p class="err">Wrong password</p>').encode()
                resp = (
                    f"HTTP/1.1 200 OK\r\n"
                    f"Content-Type: text/html; charset=utf-8\r\n"
                    f"Content-Length: {len(err_body)}\r\n"
                    f"Connection: close\r\n\r\n"
                ).encode() + err_body
            writer.write(resp)
            await writer.drain()
            writer.close()
            await writer.wait_closed()
            return

        conn_header = headers.get("connection", "").lower()
        upgrade_header = headers.get("upgrade", "").lower()
        if "upgrade" in conn_header and "websocket" in upgrade_header:
            # Reject unauthenticated WebSocket upgrades
            if not _is_authenticated():
                writer.write(b"HTTP/1.1 403 Forbidden\r\nConnection: close\r\n\r\n")
                await writer.drain()
                writer.close()
                await writer.wait_closed()
                return
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

        # Serve login page if not authenticated
        if not _is_authenticated():
            body = LOGIN_HTML.replace("$error$","").encode()
            header = (
                f"HTTP/1.1 200 OK\r\n"
                f"Content-Type: text/html; charset=utf-8\r\n"
                f"Content-Length: {len(body)}\r\n"
                f"Connection: close\r\n"
                f"Cache-Control: no-cache\r\n\r\n"
            ).encode()
            writer.write(header + body)
            await writer.drain()
            writer.close()
            await writer.wait_closed()
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
            state.update("controller_type", ctrl_type)
            if hasattr(node, "publish_controller_type"):
                node.publish_controller_type(ctrl_type)
        elif action == "set_speed_controller":
            speed_type = cmd.get("type", "curve_factor")
            state.update("speed_controller_type", speed_type)
            if hasattr(node, "publish_speed_controller_type"):
                node.publish_speed_controller_type(speed_type)
        elif action == "set_steer_mode":
            mode = cmd.get("mode", "pid")  # "pid" or "pwm"
            if hasattr(node, "publish_steer_mode"):
                node.publish_steer_mode(1 if mode == "pwm" else 0)
        elif action == "list_svo":
            import glob as _glob
            svo_dir = Path.home() / "kart_brain" / "data" / "svo"
            files = sorted(p.name for p in svo_dir.glob("*.svo")) if svo_dir.is_dir() else []
            resp = json.dumps({"svo_files": files}).encode()
            ws_send(writer, resp)
        elif action == "set_svo":
            svo_name = cmd.get("file", "live")
            import subprocess as _sp
            # Kill any running SVO player
            _sp.Popen("pkill -f 'image_source'", shell=True,
                       stdout=_sp.DEVNULL, stderr=_sp.DEVNULL)
            if svo_name != "live":
                svo_full = str(Path.home() / "kart_brain" / "data" / "svo" / svo_name)
                # Launch SVO player publishing to the ZED image topic
                _sp.Popen(
                    f"source /opt/ros/humble/setup.bash && "
                    f"source ~/kart_brain/install/setup.bash && "
                    f"ros2 run kart_perception image_source "
                    f"--ros-args -p source:={svo_full} "
                    f"-p image_topic:=/zed/zed_node/rgb/image_rect_color "
                    f"-p publish_rate:=30.0 -p loop:=true",
                    shell=True, executable="/bin/bash",
                    stdout=_sp.DEVNULL, stderr=_sp.DEVNULL,
                )
            state.update("svo_file", svo_name)
            node.get_logger().info(f"SVO set to: {svo_name}")
        elif action == "restart":
            node.get_logger().warn(f"Restart requested by {client_id}")
            import subprocess
            subprocess.Popen("echo 0 | sudo -S systemctl restart kart-brain",
                             shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
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
                        # Skip frame if client's write buffer is backed up (> 2 frames)
                        transport = w.transport
                        if transport and transport.get_write_buffer_size() > 2 * len(jpeg):
                            continue
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

    try:
        await asyncio.gather(server.serve_forever(), broadcast_loop())
    except asyncio.CancelledError:
        pass
    finally:
        server.close()
        await server.wait_closed()
