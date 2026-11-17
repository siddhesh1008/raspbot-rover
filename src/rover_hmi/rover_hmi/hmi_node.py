"""rover_hmi: browser control page for the rover. Open http://rover.local:8080

Publishes:  /cmd_vel     geometry_msgs/Twist  (20 Hz while the browser is sending commands)
Subscribes: /ultrasonic  sensor_msgs/Range    (shown on the page)

If no command arrives from the browser for `cmd_timeout` seconds (tab closed, Wi-Fi gone),
the node publishes one stop and then goes quiet, so other drivers (e.g. the WASD terminal)
are not overridden while the page is idle.
"""

from __future__ import annotations

import threading
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Range

from .server import Command, HmiServer

ZERO: Command = (0.0, 0.0, 0.0)


class HmiNode(Node):
    def __init__(self) -> None:
        super().__init__("rover_hmi")
        port = self.declare_parameter("port", 8080).value
        self.cmd_timeout = self.declare_parameter("cmd_timeout", 0.3).value
        max_linear = self.declare_parameter("max_linear", 0.6).value
        max_angular = self.declare_parameter("max_angular", 3.0).value

        self.lock = threading.Lock()
        self.command: Command = ZERO
        self.last_cmd_time = 0.0
        self.active = False
        self.distance_m: float | None = None

        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Range, "ultrasonic", self.on_range, 10)
        self.create_timer(0.05, self.tick)

        page = Path(get_package_share_directory("rover_hmi")) / "web" / "index.html"
        try:
            self.server = HmiServer(page, self.on_browser_cmd, self.status, port=port,
                                    max_linear=max_linear, max_angular=max_angular)
        except OSError as error:
            raise SystemExit(f"rover_hmi: can't use port {port} ({error.strerror}). "
                             "Is another HMI already running? Stop it or pass -p port:=8081.")
        self.server.start()
        self.get_logger().info(f"HMI running: open http://rover.local:{self.server.port} in a browser")

    def on_browser_cmd(self, cmd: Command) -> None:  # called from the web server thread
        with self.lock:
            self.command = cmd
            self.last_cmd_time = time.monotonic()

    def status(self) -> dict:
        with self.lock:
            return {"distance_m": self.distance_m}

    def on_range(self, msg: Range) -> None:
        with self.lock:
            self.distance_m = round(msg.range, 3)

    def tick(self) -> None:
        with self.lock:
            fresh = time.monotonic() - self.last_cmd_time < self.cmd_timeout
            cmd = self.command if fresh else ZERO
        if fresh and cmd != ZERO:
            self.publish(cmd)
            self.active = True
        elif self.active:          # just stopped (released, STOP pressed, or browser gone)
            self.publish(ZERO)
            self.active = False

    def publish(self, cmd: Command) -> None:
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = cmd
        self.pub.publish(msg)

    def shutdown(self) -> None:
        self.publish(ZERO)
        self.server.stop()


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = HmiNode()
    except SystemExit as error:
        print(error, flush=True)
        rclpy.shutdown()
        raise
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
