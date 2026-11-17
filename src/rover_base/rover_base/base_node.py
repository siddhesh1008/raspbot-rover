"""rover_base: the ROS 2 node that drives the real rover.

Subscribes:  /cmd_vel   geometry_msgs/Twist   x forward, y left (m/s), angular.z (rad/s)
Publishes:   /ultrasonic sensor_msgs/Range     distance ahead, 10 Hz (best effort, never blocks)

Safety:
- The motor board keeps the last speed forever, so a separate watchdog THREAD (not a ROS timer)
  stops the motors when no /cmd_vel has arrived for `cmd_timeout` seconds. It keeps working
  even if the ROS side of this node is stuck (e.g. while the network interface is reset).
- The board can silently miss a write (no I2C error), so the same thread also re-sends the
  current motor state every 0.25 s (every 0.1 s for 1 s after a change, so a lost stop is
  corrected quickly).
- Motors also stop on start-up and shutdown.
With `use_fake_bus: true` it runs anywhere (WSL, CI) and only logs what it would send.
"""

from __future__ import annotations

import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range

from rover_driver import Board, FakeBus

from .mecanum import Geometry, MotorLimits, body_to_wheels, wheels_to_pwm

STOPPED = (0, 0, 0, 0)
REFRESH_S = 0.25  # re-send the motor state this often, in case the board missed a write


class RoverBase(Node):
    def __init__(self, board: Board | None = None, start_watchdog: bool = True) -> None:
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

        # One lock for all board access: the ROS callbacks and the watchdog thread share the bus.
        self.board_lock = threading.Lock()
        self.last_cmd_time = -1e9
        self.current_pwm = STOPPED
        self.last_change_time = self.last_write_time = time.monotonic()
        with self.board_lock:
            self.board.stop()

        self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)

        if self.publish_ultrasonic:
            with self.board_lock:
                self.board.enable_ultrasonic(True)
            self.range_pub = self.create_publisher(Range, "ultrasonic", qos_profile_sensor_data)
            self.create_timer(0.1, self.on_ultrasonic)  # 10 Hz

        self.running = True
        self.watchdog_thread = threading.Thread(target=self.watchdog_loop, daemon=True)
        if start_watchdog:
            self.watchdog_thread.start()

        self.get_logger().info(
            f"rover_base ready ({'FAKE bus' if self.use_fake_bus else 'I2C 0x2B'}), "
            f"max_pwm={self.limits.max_pwm}, timeout={self.cmd_timeout}s"
        )

    # --- driving ---------------------------------------------------------------------------

    def on_cmd_vel(self, msg: Twist) -> None:
        wheels = body_to_wheels(msg.linear.x, msg.linear.y, msg.angular.z, self.geometry)
        self.last_cmd_time = time.monotonic()
        self.set_pwm(wheels_to_pwm(wheels, self.limits))

    def watchdog_loop(self) -> None:
        """Runs in its own thread, independent of the ROS executor."""
        while self.running:
            self.check_watchdog()
            self.refresh_motors()
            time.sleep(0.05)

    def check_watchdog(self) -> None:
        if self.current_pwm != STOPPED and time.monotonic() - self.last_cmd_time > self.cmd_timeout:
            self.set_pwm(STOPPED)
            self.get_logger().warning("no /cmd_vel for %.1f s: motors stopped" % self.cmd_timeout)

    def refresh_motors(self) -> None:
        """Re-send the current motor state, because the board can silently miss a write.

        Every REFRESH_S normally, and every 0.1 s during the first second after a change
        (so a lost stop is corrected within ~0.1 s).
        """
        now = time.monotonic()
        since_change = now - self.last_change_time
        interval = 0.1 if since_change < 1.0 else REFRESH_S
        if now - self.last_write_time < interval:
            return
        with self.board_lock:
            self.board.set_motors(*self.current_pwm)
            self.last_write_time = time.monotonic()

    def set_pwm(self, pwm: tuple[int, int, int, int]) -> None:
        with self.board_lock:
            if pwm == self.current_pwm:
                return
            self.board.set_motors(*pwm)
            self.current_pwm = pwm
            self.last_change_time = self.last_write_time = time.monotonic()
        if self.use_fake_bus:
            self.get_logger().info(f"motors FL/RL/FR/RR = {pwm}")

    # --- sensors ---------------------------------------------------------------------------

    def on_ultrasonic(self) -> None:
        with self.board_lock:
            distance_mm = self.board.distance_mm()
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "ultrasonic_link"
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.26  # ~15 deg, typical for this kind of sensor
        msg.min_range = 0.02
        msg.max_range = 4.0
        msg.range = distance_mm / 1000.0
        self.range_pub.publish(msg)

    # --- shutdown --------------------------------------------------------------------------

    def shutdown(self) -> None:
        self.running = False
        try:
            with self.board_lock:
                self.board.stop()
                self.current_pwm = STOPPED
                if self.publish_ultrasonic:
                    self.board.enable_ultrasonic(False)
        finally:
            with self.board_lock:
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
