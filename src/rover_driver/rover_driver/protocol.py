"""Register numbers and byte layouts of the Raspbot V2 expansion board.

Pure data and pure functions: nothing here touches hardware, so all of it can be unit-tested
anywhere. See docs/i2c-protocol.md for the full description and the source of each value.
"""

from enum import IntEnum

I2C_BUS = 1
I2C_ADDRESS = 0x2B

# Write registers
REG_MOTOR = 0x01
REG_SERVO = 0x02
REG_LIGHTS_ALL_PRESET = 0x03
REG_LIGHT_ONE_PRESET = 0x04
REG_IR_SWITCH = 0x05
REG_BUZZER = 0x06
REG_ULTRASONIC_SWITCH = 0x07
REG_LIGHTS_ALL_RGB = 0x08
REG_LIGHT_ONE_RGB = 0x09

# Read registers
REG_LINE_SENSORS = 0x0A
REG_IR_CODE = 0x0C
REG_KEY = 0x0D
REG_DISTANCE_LOW = 0x1A
REG_DISTANCE_HIGH = 0x1B

NUM_LEDS = 14
LIGHT_WRITE_GAP_S = 0.05  # the board needs >= 50 ms between light-bar writes
MOTOR_WRITE_GAP_S = 0.005  # small gap between motor writes; back-to-back writes can be missed
IR_NO_KEY = 0xFF


class Motor(IntEnum):
    """Motor numbers used by the board (tested on the robot).

    Positive speed (direction byte 0) turns every wheel the way that drives the rover forward.
    """

    L1 = 0  # front-left
    L2 = 1  # rear-left
    R1 = 2  # front-right
    R2 = 3  # rear-right

    FRONT_LEFT = 0
    REAR_LEFT = 1
    FRONT_RIGHT = 2
    REAR_RIGHT = 3


class Servo(IntEnum):
    S1 = 1
    S2 = 2


class Color(IntEnum):
    """Preset colours for registers 0x03 / 0x04."""

    RED = 0
    GREEN = 1
    BLUE = 2
    YELLOW = 3
    PURPLE = 4
    CYAN = 5
    WHITE = 6


def clamp(value: int, low: int, high: int) -> int:
    return max(low, min(high, int(value)))


def motor_bytes(motor: int, speed: int) -> list[int]:
    """Data bytes for REG_MOTOR. `speed` is signed: -255 (full backward) .. 255 (full forward)."""
    motor = int(Motor(motor))
    speed = clamp(speed, -255, 255)
    direction = 0 if speed >= 0 else 1
    return [motor, direction, abs(speed)]


def servo_bytes(servo: int, angle: float) -> list[int]:
    return [int(Servo(servo)), clamp(round(angle), 0, 180)]


def lights_all_preset_bytes(on: bool, color: int = Color.RED) -> list[int]:
    return [1 if on else 0, int(Color(color))]


def light_one_preset_bytes(led: int, on: bool, color: int = Color.RED) -> list[int]:
    return [_led(led), 1 if on else 0, int(Color(color))]


def lights_all_rgb_bytes(r: int, g: int, b: int) -> list[int]:
    return [clamp(r, 0, 255), clamp(g, 0, 255), clamp(b, 0, 255)]


def light_one_rgb_bytes(led: int, r: int, g: int, b: int) -> list[int]:
    return [_led(led)] + lights_all_rgb_bytes(r, g, b)


def switch_bytes(on: bool) -> list[int]:
    return [1 if on else 0]


def decode_line_sensors(raw: int) -> tuple[bool, bool, bool, bool]:
    """Raw 0x0A byte -> (X1, X2, X3, X4) from left to right, True = sensor triggered.

    The protocol sheet says a triggered sensor reads 0 (its example: only X1 lit -> 0b0111).
    Still to verify on real black tape.
    """
    return tuple(((raw >> bit) & 1) == 0 for bit in (3, 2, 1, 0))  # type: ignore[return-value]


def decode_distance_mm(low: int, high: int) -> int:
    return ((high & 0xFF) << 8) | (low & 0xFF)


def _led(led: int) -> int:
    if not 1 <= int(led) <= NUM_LEDS:
        raise ValueError(f"LED number must be 1..{NUM_LEDS}, got {led}")
    return int(led)
