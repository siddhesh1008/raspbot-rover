"""Mecanum-wheel kinematics: body velocity -> four wheel commands. Pure math, no ROS.

Conventions (ROS REP 103): x forward, y left, z up; positive yaw rate = turn left (counter-clockwise
seen from above). Rollers in the usual "X" layout (seen from above they form an X).

The robot has no wheel encoders, so wheel speeds are turned into motor power (PWM 0..255) with a
simple linear model: `max_wheel_speed` m/s corresponds to PWM 255. Tune it by measuring how far the
rover drives in a few seconds.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Geometry:
    half_wheelbase: float = 0.08    # m, centre to front axle (lx). Measure on the robot.
    half_track: float = 0.085       # m, centre to wheel centre sideways (ly). Measure on the robot.


@dataclass(frozen=True)
class MotorLimits:
    max_wheel_speed: float = 0.5    # m/s of a wheel at PWM 255 (estimate; tune on the robot)
    max_pwm: int = 160              # never send more than this (safety cap while testing indoors)
    min_pwm: int = 35               # below this the motors hum but don't turn; small commands are raised to it
    deadband: float = 0.01          # m/s; wheel speeds below this are treated as 0


@dataclass(frozen=True)
class WheelSpeeds:
    """Wheel surface speeds in m/s, positive = rolling forward."""

    front_left: float
    rear_left: float
    front_right: float
    rear_right: float

    def as_tuple(self) -> tuple[float, float, float, float]:
        return (self.front_left, self.rear_left, self.front_right, self.rear_right)


def body_to_wheels(vx: float, vy: float, wz: float, geometry: Geometry = Geometry()) -> WheelSpeeds:
    """Inverse kinematics. vx, vy in m/s, wz in rad/s."""
    k = geometry.half_wheelbase + geometry.half_track
    return WheelSpeeds(
        front_left=vx - vy - k * wz,
        rear_left=vx + vy - k * wz,
        front_right=vx + vy + k * wz,
        rear_right=vx - vy + k * wz,
    )


def wheels_to_body(w: WheelSpeeds, geometry: Geometry = Geometry()) -> tuple[float, float, float]:
    """Forward kinematics (for open-loop odometry later): wheel speeds -> (vx, vy, wz)."""
    k = geometry.half_wheelbase + geometry.half_track
    vx = (w.front_left + w.rear_left + w.front_right + w.rear_right) / 4
    vy = (-w.front_left + w.rear_left + w.front_right - w.rear_right) / 4
    wz = (-w.front_left - w.rear_left + w.front_right + w.rear_right) / (4 * k)
    return vx, vy, wz


def ramp_toward(current: tuple[int, ...], target: tuple[int, ...], max_step: int) -> tuple[int, ...]:
    """One step of the acceleration ramp: move each motor at most `max_step` towards its target.

    A full stop (all targets 0) is never ramped: stopping is always immediate, for safety.
    Reversing a wheel passes through 0 at the same rate, so direction changes are smooth too.
    """
    if all(t == 0 for t in target):
        return tuple(0 for _ in target)
    return tuple(c + max(-max_step, min(max_step, t - c)) for c, t in zip(current, target))


def wheels_to_pwm(w: WheelSpeeds, limits: MotorLimits = MotorLimits()) -> tuple[int, int, int, int]:
    """Wheel speeds (m/s) -> signed PWM per motor, in board order (L1 FL, L2 RL, R1 FR, R2 RR).

    If any wheel would exceed max_pwm, all four are scaled down together, so the direction of
    travel is kept and only the speed drops.
    """
    raw = [s / limits.max_wheel_speed * 255 for s in w.as_tuple()]
    biggest = max(abs(r) for r in raw)
    if biggest > limits.max_pwm:
        raw = [r * limits.max_pwm / biggest for r in raw]

    pwm = []
    for speed, r in zip(w.as_tuple(), raw):
        if abs(speed) < limits.deadband:
            pwm.append(0)
            continue
        magnitude = max(limits.min_pwm, min(limits.max_pwm, round(abs(r))))
        pwm.append(magnitude if r > 0 else -magnitude)
    return tuple(pwm)  # type: ignore[return-value]
