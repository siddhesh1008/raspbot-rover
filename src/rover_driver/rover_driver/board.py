"""High-level access to the Raspbot V2 expansion board.

`Board` only needs an object with smbus2-style `write_i2c_block_data` / `read_i2c_block_data`.
On the robot that is a real `smbus2.SMBus(1)`; in tests and the simulator it is `FakeBus`.
No ROS here: ROS nodes use this class, it doesn't use ROS.
"""

from __future__ import annotations

import time
from typing import Protocol

from . import protocol as p


class I2CBus(Protocol):
    def write_i2c_block_data(self, address: int, register: int, data: list[int]) -> None: ...
    def read_i2c_block_data(self, address: int, register: int, length: int) -> list[int]: ...


class Board:
    """Motors, servos, light bar, buzzer and sensors of the expansion board.

    Use it as a context manager so the motors are stopped however the program ends:

        with Board.open() as board:
            board.set_motor(Motor.L1, 80)
    """

    def __init__(self, bus: I2CBus, address: int = p.I2C_ADDRESS, clock=time.monotonic, sleep=time.sleep):
        self._bus = bus
        self._address = address
        self._clock = clock
        self._sleep = sleep
        self._last_light_write = -1e9

    @classmethod
    def open(cls, bus_number: int = p.I2C_BUS, address: int = p.I2C_ADDRESS) -> "Board":
        """Open the real I2C bus (only works on the robot)."""
        from smbus2 import SMBus  # imported here so tests run without smbus2 / hardware

        return cls(SMBus(bus_number), address)

    # --- lifecycle -------------------------------------------------------------------------

    def __enter__(self) -> "Board":
        self.stop()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def close(self) -> None:
        """Stop motors, switch everything off and release the bus."""
        try:
            self.stop()
            self.buzzer(False)
        finally:
            close = getattr(self._bus, "close", None)
            if close:
                close()

    # --- motors and servos -----------------------------------------------------------------

    def set_motor(self, motor: int, speed: int) -> None:
        """Signed speed, -255 (full backward) .. 255 (full forward), 0 = stop."""
        self._write(p.REG_MOTOR, p.motor_bytes(motor, speed))

    def set_motors(self, l1: int, l2: int, r1: int, r2: int) -> None:
        for motor, speed in zip(p.Motor, (l1, l2, r1, r2)):
            self.set_motor(motor, speed)

    def stop(self) -> None:
        self.set_motors(0, 0, 0, 0)

    def set_servo(self, servo: int, angle: float) -> None:
        """Angle 0..180 degrees."""
        self._write(p.REG_SERVO, p.servo_bytes(servo, angle))

    # --- light bar and buzzer --------------------------------------------------------------

    def lights_off(self) -> None:
        self._write_light(p.REG_LIGHTS_ALL_PRESET, p.lights_all_preset_bytes(False))

    def lights_color(self, color: int) -> None:
        self._write_light(p.REG_LIGHTS_ALL_PRESET, p.lights_all_preset_bytes(True, color))

    def lights_rgb(self, r: int, g: int, b: int) -> None:
        self._write_light(p.REG_LIGHTS_ALL_RGB, p.lights_all_rgb_bytes(r, g, b))

    def light_rgb(self, led: int, r: int, g: int, b: int) -> None:
        """One LED, 1..14."""
        self._write_light(p.REG_LIGHT_ONE_RGB, p.light_one_rgb_bytes(led, r, g, b))

    def buzzer(self, on: bool) -> None:
        self._write(p.REG_BUZZER, p.switch_bytes(on))

    # --- sensors ---------------------------------------------------------------------------

    def enable_ultrasonic(self, on: bool = True) -> None:
        self._write(p.REG_ULTRASONIC_SWITCH, p.switch_bytes(on))

    def distance_mm(self) -> int:
        """Ultrasonic distance in mm (0 while the sensor is switched off)."""
        low = self._read(p.REG_DISTANCE_LOW, 1)[0]
        high = self._read(p.REG_DISTANCE_HIGH, 1)[0]
        return p.decode_distance_mm(low, high)

    def line_sensors(self) -> tuple[bool, bool, bool, bool]:
        """(X1, X2, X3, X4) left to right, True = triggered."""
        return p.decode_line_sensors(self._read(p.REG_LINE_SENSORS, 1)[0])

    def enable_ir(self, on: bool = True) -> None:
        self._write(p.REG_IR_SWITCH, p.switch_bytes(on))

    def ir_code(self) -> int | None:
        """Last IR remote key code, or None if no key was pressed since the last read."""
        code = self._read(p.REG_IR_CODE, 1)[0]
        return None if code == p.IR_NO_KEY else code

    def key_raw(self) -> int:
        """KEY1 button register (0 or 1; which value means 'pressed' is still to verify)."""
        return self._read(p.REG_KEY, 1)[0]

    # --- low level -------------------------------------------------------------------------

    def _write(self, register: int, data: list[int]) -> None:
        self._bus.write_i2c_block_data(self._address, register, data)

    def _write_light(self, register: int, data: list[int]) -> None:
        wait = p.LIGHT_WRITE_GAP_S - (self._clock() - self._last_light_write)
        if wait > 0:
            self._sleep(wait)
        self._write(register, data)
        self._last_light_write = self._clock()

    def _read(self, register: int, length: int) -> list[int]:
        return list(self._bus.read_i2c_block_data(self._address, register, length))
