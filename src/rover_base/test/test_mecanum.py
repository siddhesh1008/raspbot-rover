import pytest

from rover_base.mecanum import Geometry, MotorLimits, WheelSpeeds, body_to_wheels, wheels_to_body, wheels_to_pwm

G = Geometry(half_wheelbase=0.08, half_track=0.085)


def test_forward_turns_all_wheels_forward():
    assert body_to_wheels(0.2, 0, 0, G).as_tuple() == (0.2, 0.2, 0.2, 0.2)


def test_strafe_left_pattern():
    # moving left: FL and RR roll backward, RL and FR forward
    w = body_to_wheels(0, 0.2, 0, G)
    assert w.as_tuple() == (-0.2, 0.2, 0.2, -0.2)


def test_turn_left_pattern():
    w = body_to_wheels(0, 0, 1.0, G)
    assert w.front_left < 0 and w.rear_left < 0
    assert w.front_right > 0 and w.rear_right > 0


@pytest.mark.parametrize("cmd", [(0.3, 0, 0), (0, -0.2, 0), (0, 0, 1.5), (0.1, 0.2, -0.7)])
def test_forward_and_inverse_kinematics_agree(cmd):
    assert wheels_to_body(body_to_wheels(*cmd, G), G) == pytest.approx(cmd)


def test_pwm_scales_linearly():
    limits = MotorLimits(max_wheel_speed=0.5, max_pwm=255, min_pwm=0)
    assert wheels_to_pwm(WheelSpeeds(0.25, 0.25, -0.25, -0.25), limits) == (128, 128, -128, -128)


def test_pwm_cap_keeps_direction():
    limits = MotorLimits(max_wheel_speed=0.5, max_pwm=100, min_pwm=0)
    pwm = wheels_to_pwm(WheelSpeeds(1.0, 0.5, -1.0, 0.0), limits)
    assert pwm == (100, 50, -100, 0)


def test_small_commands_are_raised_to_min_pwm():
    limits = MotorLimits(max_wheel_speed=0.5, max_pwm=160, min_pwm=35)
    assert wheels_to_pwm(WheelSpeeds(0.02, -0.02, 0, 0), limits) == (35, -35, 0, 0)


def test_deadband_stops_tiny_speeds():
    assert wheels_to_pwm(WheelSpeeds(0.001, 0, 0, 0)) == (0, 0, 0, 0)
