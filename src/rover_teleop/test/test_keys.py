import pytest

from rover_teleop.keys import Speeds, command_for_key


@pytest.mark.parametrize("key, expected", [
    ("w", (0.2, 0.0, 0.0)),
    ("s", (-0.2, 0.0, 0.0)),
    ("a", (0.0, 0.0, 1.0)),
    ("d", (0.0, 0.0, -1.0)),
    ("q", (0.0, 0.2, 0.0)),
    ("e", (0.0, -0.2, 0.0)),
    ("W", (0.2, 0.0, 0.0)),   # caps lock / shift still works
    (" ", (0.0, 0.0, 0.0)),
    ("x", (0.0, 0.0, 0.0)),
])
def test_keys(key, expected):
    assert command_for_key(key, Speeds(linear=0.2, angular=1.0)) == expected


def test_other_keys_do_nothing():
    assert command_for_key("z", Speeds()) is None


def test_speed_steps_are_capped():
    s = Speeds(linear=0.5, angular=2.9, max_linear=0.6, max_angular=3.0)
    for _ in range(10):
        s.faster()
    assert (s.linear, s.angular) == (0.6, 3.0)
    for _ in range(100):
        s.slower()
    assert s.linear >= 0.02 and s.angular >= 0.1
