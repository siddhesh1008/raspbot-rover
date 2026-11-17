"""Tests for the driver, using FakeBus: no robot needed. Run with `colcon test` or `pytest`."""

import pytest

from rover_driver import Board, Color, FakeBus, Motor, Servo
from rover_driver import protocol as p
from rover_driver.hw_check import main as hw_main


class FakeClock:
    def __init__(self):
        self.now = 100.0
        self.slept = []

    def time(self):
        return self.now

    def sleep(self, seconds):
        self.slept.append(seconds)
        self.now += seconds


@pytest.fixture
def bus():
    return FakeBus()


@pytest.fixture
def clock():
    return FakeClock()


@pytest.fixture
def board(bus, clock):
    return Board(bus, clock=clock.time, sleep=clock.sleep)


# --- byte layouts from the protocol sheet ------------------------------------------------------

def test_motor_example_from_sheet(board, bus):
    board.set_motor(Motor.L1, 200)  # sheet: L1 forward 200 -> 00 00 C8
    assert bus.writes == [(0x2B, 0x01, [0x00, 0x00, 0xC8])]


def test_negative_speed_means_backward(board, bus):
    board.set_motor(Motor.R2, -100)
    assert bus.writes_to(p.REG_MOTOR) == [[3, 1, 100]]


@pytest.mark.parametrize("speed, expected", [(999, [0, 0, 255]), (-999, [0, 1, 255]), (0, [0, 0, 0])])
def test_motor_speed_is_clamped(speed, expected):
    assert p.motor_bytes(Motor.L1, speed) == expected


def test_unknown_motor_is_rejected():
    with pytest.raises(ValueError):
        p.motor_bytes(4, 10)


def test_servo_example_from_sheet(board, bus):
    board.set_servo(Servo.S1, 90)    # sheet: 01 5A
    board.set_servo(Servo.S2, 180)   # sheet: 02 B4
    assert bus.writes_to(p.REG_SERVO) == [[0x01, 0x5A], [0x02, 0xB4]]


def test_servo_angle_is_clamped():
    assert p.servo_bytes(2, 250) == [2, 180]
    assert p.servo_bytes(1, -5) == [1, 0]


def test_light_examples_from_sheet():
    assert p.lights_all_preset_bytes(True, Color.GREEN) == [0x01, 0x01]
    assert p.light_one_preset_bytes(3, True, Color.RED) == [0x03, 0x01, 0x00]
    assert p.light_one_rgb_bytes(10, 0x7F, 0, 0) == [0x0A, 0x7F, 0x00, 0x00]


@pytest.mark.parametrize("led", [0, 15])
def test_led_number_out_of_range(led):
    with pytest.raises(ValueError):
        p.light_one_rgb_bytes(led, 1, 2, 3)


def test_light_writes_are_spaced_50ms(board, clock):
    board.lights_rgb(255, 0, 0)
    board.lights_rgb(0, 255, 0)
    assert clock.slept == [pytest.approx(0.05)]


# --- sensors -----------------------------------------------------------------------------------

def test_distance_example_from_sheet(board, bus):
    bus.registers[p.REG_DISTANCE_LOW] = 0xFF
    bus.registers[p.REG_DISTANCE_HIGH] = 0x00
    assert board.distance_mm() == 255


def test_distance_uses_high_byte(board, bus):
    bus.registers[p.REG_DISTANCE_LOW] = 0x10
    bus.registers[p.REG_DISTANCE_HIGH] = 0x02
    assert board.distance_mm() == 0x0210


def test_line_sensor_example_from_sheet(board, bus):
    bus.registers[p.REG_LINE_SENSORS] = 0b0111  # sheet: only X1 (leftmost) lit
    assert board.line_sensors() == (True, False, False, False)


def test_ir_no_key_is_none(board, bus):
    bus.registers[p.REG_IR_CODE] = 0xFF
    assert board.ir_code() is None
    bus.registers[p.REG_IR_CODE] = 0x45
    assert board.ir_code() == 0x45


# --- safety ------------------------------------------------------------------------------------

def test_context_manager_stops_motors_on_entry_and_exit(bus, clock):
    with Board(bus, clock=clock.time, sleep=clock.sleep) as board:
        bus.writes.clear()
        board.set_motor(Motor.L1, 120)
    motor_writes = bus.writes_to(p.REG_MOTOR)
    assert motor_writes[0] == [0, 0, 120]
    assert motor_writes[1:] == [[m, 0, 0] for m in range(4)]
    assert bus.closed


def test_motors_stop_even_after_an_error(bus, clock):
    with pytest.raises(RuntimeError):
        with Board(bus, clock=clock.time, sleep=clock.sleep) as board:
            board.set_motor(Motor.R1, 200)
            raise RuntimeError("crash")
    assert bus.writes_to(p.REG_MOTOR)[-4:] == [[m, 0, 0] for m in range(4)]


def test_hw_check_runs_without_hardware(capsys):
    assert hw_main(["--fake", "motor", "L1", "999", "--seconds", "0"]) == 0
    out = capsys.readouterr().out
    assert "speed 150" in out                 # test speed is capped
    assert "reg 0x01: ['0x00', '0x00', '0x96']" in out
