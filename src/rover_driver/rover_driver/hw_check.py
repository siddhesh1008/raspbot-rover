"""`rover-hw`: small, careful hardware checks for the expansion board, run on the robot.

Examples (on the rover, after `colcon build` and `source install/setup.bash`):
    ros2 run rover_driver rover-hw beep
    ros2 run rover_driver rover-hw lights
    ros2 run rover_driver rover-hw sensors --seconds 10
    ros2 run rover_driver rover-hw servo 1 90
    ros2 run rover_driver rover-hw motor L1 80 --seconds 1   # ROBOT ON A STAND, WHEELS OFF THE GROUND
    ros2 run rover_driver rover-hw motor all -80

Motors always stop at the end, also on Ctrl-C or an error. Speed and run time are capped.
Add --fake to any command to run it without hardware (prints the bytes it would send).
"""

from __future__ import annotations

import argparse
import sys
import time

from .board import Board
from .fake_bus import FakeBus
from .protocol import Color, Motor

MAX_TEST_SPEED = 150
MAX_TEST_SECONDS = 3.0


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    bus = FakeBus() if args.fake else None
    try:
        board = Board(bus) if bus else Board.open()
    except (FileNotFoundError, PermissionError) as error:
        print(f"Can't open the I2C bus: {error}\n"
              "On the rover: is I2C on, and is this user in the 'i2c' group?", file=sys.stderr)
        return 1

    with board:
        args.func(board, args)

    if bus:
        for _, register, data in bus.writes:
            print(f"  write reg 0x{register:02X}: {[f'0x{b:02X}' for b in data]}")
    return 0


def _beep(board: Board, args) -> None:
    board.buzzer(True)
    time.sleep(0.2)
    board.buzzer(False)


def _lights(board: Board, args) -> None:
    for color in Color:
        print(f"  {color.name.lower()}")
        board.lights_color(color)
        time.sleep(0.4)
    board.lights_off()


def _sensors(board: Board, args) -> None:
    board.enable_ultrasonic(True)
    time.sleep(0.2)
    end = time.monotonic() + args.seconds
    try:
        while time.monotonic() < end:
            line = "".join("X" if hit else "." for hit in board.line_sensors())
            print(f"  distance {board.distance_mm():5d} mm   line [{line}]   key {board.key_raw()}")
            time.sleep(0.25)
    finally:
        board.enable_ultrasonic(False)


def _servo(board: Board, args) -> None:
    board.set_servo(args.servo, args.angle)


def _motor(board: Board, args) -> None:
    speed = max(-MAX_TEST_SPEED, min(MAX_TEST_SPEED, args.speed))
    seconds = max(0.0, min(MAX_TEST_SECONDS, args.seconds))
    motors = list(Motor) if args.motor == "all" else [Motor[args.motor]]
    names = ", ".join(m.name for m in motors)
    print(f"  {names} at speed {speed} for {seconds:.1f} s")
    try:
        for motor in motors:
            board.set_motor(motor, speed)
        time.sleep(seconds)
    finally:
        board.stop()


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="rover-hw", description="Raspbot V2 hardware checks.")
    parser.add_argument("--fake", action="store_true", help="no hardware: print the bytes instead")
    sub = parser.add_subparsers(required=True, metavar="COMMAND")

    sub.add_parser("beep", help="short beep").set_defaults(func=_beep)
    sub.add_parser("lights", help="cycle the light bar through its colours").set_defaults(func=_lights)

    sensors = sub.add_parser("sensors", help="print distance, line sensors and key")
    sensors.add_argument("--seconds", type=float, default=5.0)
    sensors.set_defaults(func=_sensors)

    servo = sub.add_parser("servo", help="move a servo")
    servo.add_argument("servo", type=int, choices=[1, 2])
    servo.add_argument("angle", type=float, help="0..180")
    servo.set_defaults(func=_servo)

    motor = sub.add_parser("motor", help=f"run motor(s) briefly (max speed {MAX_TEST_SPEED}, max {MAX_TEST_SECONDS:.0f} s)")
    motor.add_argument("motor", choices=[m.name for m in Motor] + ["all"])
    motor.add_argument("speed", type=int, nargs="?", default=80, help="-255..255 (default 80)")
    motor.add_argument("--seconds", type=float, default=1.0)
    motor.set_defaults(func=_motor)
    return parser


if __name__ == "__main__":
    sys.exit(main())
