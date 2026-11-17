"""WASD keyboard driving: publishes geometry_msgs/Twist on /cmd_vel.

    ros2 run rover_teleop wasd

A terminal can't tell when a key is released, only when it is pressed (and the keyboard's
auto-repeat while held). So the rover keeps moving while key presses keep arriving and stops
`hold_timeout` seconds after the last one. The command is re-published at 20 Hz so the rover's
own 0.5 s watchdog never fires while you are driving.
"""

from __future__ import annotations

import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from .keys import FASTER, HELP, SLOWER, Speeds, command_for_key


class WasdTeleop(Node):
    def __init__(self) -> None:
        super().__init__("rover_teleop")
        self.speeds = Speeds(
            linear=self.declare_parameter("speed", 0.2).value,
            angular=self.declare_parameter("turn", 1.0).value,
        )
        # must be longer than the keyboard's delay before auto-repeat starts (often ~0.5 s)
        self.hold_timeout = self.declare_parameter("hold_timeout", 0.6).value
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.command = (0.0, 0.0, 0.0)
        self.last_key_time = 0.0

    def handle_key(self, key: str) -> None:
        if key in FASTER:
            self.speeds.faster()
            self.show_speed()
        elif key in SLOWER:
            self.speeds.slower()
            self.show_speed()
        else:
            cmd = command_for_key(key, self.speeds)
            if cmd is not None:
                self.command = cmd
                self.last_key_time = time.monotonic()

    def tick(self) -> None:
        if self.command != (0.0, 0.0, 0.0) and time.monotonic() - self.last_key_time > self.hold_timeout:
            self.command = (0.0, 0.0, 0.0)
        self.publish(self.command)

    def publish(self, cmd: tuple[float, float, float]) -> None:
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = cmd
        self.pub.publish(msg)

    def show_speed(self) -> None:
        print(f"\r  speed {self.speeds.linear:.2f} m/s   turn {self.speeds.angular:.2f} rad/s   ", end="", flush=True)


def read_key(timeout: float) -> str | None:
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    return sys.stdin.read(1) if ready else None


def main(args=None) -> None:
    if not sys.stdin.isatty():
        print("rover_teleop needs an interactive terminal.", file=sys.stderr)
        sys.exit(1)
    rclpy.init(args=args)
    node = WasdTeleop()
    print(HELP)
    node.show_speed()
    saved = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            key = read_key(0.05)  # 20 Hz loop
            if key == "\x03":     # Ctrl-C
                break
            if key:
                node.handle_key(key)
            node.tick()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, saved)
        for _ in range(3):  # make sure the stop gets through
            node.publish((0.0, 0.0, 0.0))
            time.sleep(0.05)
        print("\nstopped.")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
