"""Tiny web server for the rover HMI. Standard library only, no ROS (the node wires it to ROS).

    GET  /          the control page (index.html)
    GET  /status    {"distance_m": ..., "max_linear": ..., "max_angular": ...}
    POST /cmd       {"vx": m/s, "vy": m/s, "wz": rad/s}  -> clamped, passed to on_cmd
    POST /stop      same as a zero command

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


class HmiServer:
    def __init__(
        self,
        page: Path,
        on_cmd: Callable[[Command], None],
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
                if self.path == "/stop":
                    server.on_cmd((0.0, 0.0, 0.0))
                    self._json(server.status())
                    return
                if self.path != "/cmd":
                    self._send(404, b"not found", "text/plain")
                    return
                try:
                    length = min(int(self.headers.get("Content-Length", 0)), 4096)
                    data = json.loads(self.rfile.read(length) or b"{}")
                    if not isinstance(data, dict):
                        raise ValueError
                except ValueError:
                    self._send(400, b"bad json", "text/plain")
                    return
                server.on_cmd(clamp_command(data, server.max_linear, server.max_angular))
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
