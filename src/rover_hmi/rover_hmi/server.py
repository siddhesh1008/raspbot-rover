"""Tiny web server for the rover HMI. Standard library only, no ROS (the node wires it to ROS).

    GET  /          the control page (index.html)
    GET  /status    {"distance_m": ..., "max_linear": ..., "max_angular": ...}
    POST /cmd       {"vx": m/s, "vy": m/s, "wz": rad/s, "seq": n}  -> clamped, on_cmd(cmd, seq)
    POST /stop      {"seq": n}                                    -> on_cmd((0, 0, 0), seq)

`seq` is an increasing request number from the page. Requests can arrive out of order (each
one may use its own connection), so the receiver drops any request older than the newest it
has handled: a stop is never undone by a drive command that was sent before it.

The page re-sends /cmd about 20 times a second while a key is held. The node stops the rover
when commands stop arriving, so a closed tab or lost Wi-Fi also stops it.
"""

from __future__ import annotations

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Callable

Command = tuple[float, float, float]


def clamp_command(data: dict, max_linear: float, max_angular: float) -> Command:
    """Validate and clamp a JSON command. Missing or invalid numbers count as 0."""

    def num(key: str, limit: float) -> float:
        try:
            value = float(data.get(key, 0.0))
        except (TypeError, ValueError):
            return 0.0
        if value != value:  # NaN
            return 0.0
        return max(-limit, min(limit, value))

    return (num("vx", max_linear), num("vy", max_linear), num("wz", max_angular))


def parse_seq(data: dict) -> int | None:
    seq = data.get("seq")
    return seq if isinstance(seq, int) and not isinstance(seq, bool) else None


class HmiServer:
    def __init__(
        self,
        page: Path,
        on_cmd: Callable[[Command, int | None], None],
        get_status: Callable[[], dict],
        host: str = "0.0.0.0",
        port: int = 8080,
        max_linear: float = 0.6,
        max_angular: float = 3.0,
    ) -> None:
        self.page = page
        self.on_cmd = on_cmd
        self.get_status = get_status
        self.max_linear = max_linear
        self.max_angular = max_angular
        self.httpd = ThreadingHTTPServer((host, port), self._handler_class())
        self.httpd.daemon_threads = True
        self.thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)

    @property
    def port(self) -> int:
        return self.httpd.server_address[1]

    def start(self) -> None:
        self.thread.start()

    def stop(self) -> None:
        self.httpd.shutdown()
        self.httpd.server_close()

    def status(self) -> dict:
        return {"max_linear": self.max_linear, "max_angular": self.max_angular, **self.get_status()}

    def _handler_class(self):
        server = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args) -> None:  # keep the ROS log clean
                pass

            def do_GET(self) -> None:
                if self.path in ("/", "/index.html"):
                    self._send(200, server.page.read_bytes(), "text/html; charset=utf-8")
                elif self.path == "/status":
                    self._json(server.status())
                else:
                    self._send(404, b"not found", "text/plain")

            def do_POST(self) -> None:
                if self.path not in ("/cmd", "/stop"):
                    self._send(404, b"not found", "text/plain")
                    return
                try:
                    length = min(int(self.headers.get("Content-Length", 0)), 4096)
                    data = json.loads(self.rfile.read(length) or b"{}")
                    if not isinstance(data, dict):
                        raise ValueError
                except ValueError:
                    if self.path == "/stop":      # a stop always works, even with a broken body
                        server.on_cmd((0.0, 0.0, 0.0), None)
                        self._json(server.status())
                    else:
                        self._send(400, b"bad json", "text/plain")
                    return
                if self.path == "/stop":
                    cmd: Command = (0.0, 0.0, 0.0)
                else:
                    cmd = clamp_command(data, server.max_linear, server.max_angular)
                server.on_cmd(cmd, parse_seq(data))
                self._json(server.status())

            def _json(self, obj: dict) -> None:
                self._send(200, json.dumps(obj).encode(), "application/json")

            def _send(self, code: int, body: bytes, content_type: str) -> None:
                self.send_response(code)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

        return Handler
