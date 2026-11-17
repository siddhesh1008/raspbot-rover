"""rover_base: the ROS 2 node that drives the real rover.

Subscribes:  /cmd_vel   geometry_msgs/Twist   x forward, y left (m/s), angular.z (rad/s)
Publishes:   /ultrasonic sensor_msgs/Range     distance ahead, 10 Hz

Safety: motors stop when no /cmd_vel arrives for `cmd_timeout` seconds, and on shutdown.
With `use_fake_bus: true` it runs anywhere (WSL, CI) and only logs what it would send.
"""

from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Range

from rover_driver import Board, FakeBus

from .mecanum import Geometry, MotorLimits, body_to_wheels, wheels_to_pwm


class RoverBase(Node):
    def __init__(self, board: Board | None = None) -> None:
        super().__init__("rover_base")
        p = self.declare_parameter
        self.use_fake_bus = p("use_fake_bus", False).value
        self.cmd_timeout = p("cmd_timeout", 0.5).value
        self.geometry = Geometry(p("half_wheelbase", 0.08).value, p("half_track", 0.085).value)
        self.limits = MotorLimits(
            max_wheel_speed=p("max_wheel_speed", 0.5).value,
            max_pwm=p("max_pwm", 160).value,
            min_pwm=p("min_pwm", 35).value,
        )
        self.publish_ultrasonic = p("publish_ultrasonic", True).value

        if board is not None:
            self.board = board
        elif self.use_fake_bus:
            self.board = Board(FakeBus())
        else:
            self.board = Board.open()
        self.board.stop()

        self.last_cmd_time = -1e9
        self.current_pwm = (0, 0, 0, 0)

        self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)
        self.create_timer(0.05, self.on_watchdog)  # 20 Hz

        if self.publish_ultrasonic:
            self.board.enable_ultrasonic(True)
            self.range_pub = self.create_publisher(Range, "ultrasonic", 10)
            self.create_timer(0.1, self.on_ultrasonic)  # 10 Hz

        self.get_logger().info(
            f"rover_base ready ({'FAKE bus' if self.use_fake_bus else 'I2C 0x2B'}), "
            f"max_pwm={self.limits.max_pwm}, timeout={self.cmd_timeout}s"
        )

    # --- driving ---------------------------------------------------------------------------

    def on_cmd_vel(self, msg: Twist) -> None:
        wheels = body_to_wheels(msg.linear.x, msg.linear.y, msg.angular.z, self.geometry)
        self.set_pwm(wheels_to_pwm(wheels, self.limits))
        self.last_cmd_time = time.monotonic()

    def on_watchdog(self) -> None:
        if self.current_pwm != (0, 0, 0, 0) and time.monotonic() - self.last_cmd_time > self.cmd_timeout:
            self.get_logger().info("no /cmd_vel recently: stopping motors")
            self.set_pwm((0, 0, 0, 0))

    def set_pwm(self, pwm: tuple[int, int, int, int]) -> None:
        if pwm == self.current_pwm:
            return
        self.board.set_motors(*pwm)
        self.current_pwm = pwm
        if self.use_fake_bus:
            self.get_logger().info(f"motors FL/RL/FR/RR = {pwm}")

    # --- sensors ---------------------------------------------------------------------------

    def on_ultrasonic(self) -> None:
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "ultrasonic_link"
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.26  # ~15 deg, typical for this kind of sensor
        msg.min_range = 0.02
        msg.max_range = 4.0
        msg.range = self.board.distance_mm() / 1000.0
        self.range_pub.publish(msg)

    # --- shutdown --------------------------------------------------------------------------

    def shutdown(self) -> None:
        try:
            self.board.stop()
            if self.publish_ultrasonic:
                self.board.enable_ultrasonic(False)
        finally:
            self.board.close()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RoverBase()
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
