"""WASD key mapping. Pure logic, no ROS and no terminal, so it is easy to test."""

from __future__ import annotations

from dataclasses import dataclass

# key -> (forward, left, turn) as -1/0/+1 directions
MOVES: dict[str, tuple[int, int, int]] = {
    "w": (1, 0, 0),    # forward
    "s": (-1, 0, 0),   # backward
    "a": (0, 0, 1),    # turn left on the spot
    "d": (0, 0, -1),   # turn right on the spot
    "q": (0, 1, 0),    # slide left (mecanum)
    "e": (0, -1, 0),   # slide right (mecanum)
}
STOP_KEYS = {" ", "x"}
FASTER, SLOWER = {"+", "="}, {"-", "_"}

HELP = """
Rover keyboard control (hold a key to keep moving)
-------------------------------------------------
        w                 q / e : slide left / right
    a   s   d             space : stop
                          + / - : faster / slower
 w/s forward/back         Ctrl-C: quit (rover stops)
 a/d turn left/right
"""


@dataclass
class Speeds:
    linear: float = 0.2    # m/s for w/s/q/e
    angular: float = 1.0   # rad/s for a/d
    max_linear: float = 0.6
    max_angular: float = 3.0
    step: float = 1.1      # +/- changes speed by 10 %

    def faster(self) -> None:
        self.linear = min(self.max_linear, self.linear * self.step)
        self.angular = min(self.max_angular, self.angular * self.step)

    def slower(self) -> None:
        self.linear = max(0.02, self.linear / self.step)
        self.angular = max(0.1, self.angular / self.step)


def command_for_key(key: str, speeds: Speeds) -> tuple[float, float, float] | None:
    """Velocity (vx, vy, wz) for a movement key, (0, 0, 0) for stop keys, None for anything else."""
    key = key.lower() if key.isalpha() else key
    if key in MOVES:
        fwd, left, turn = MOVES[key]
        return (fwd * speeds.linear, left * speeds.linear, turn * speeds.angular)
    if key in STOP_KEYS:
        return (0.0, 0.0, 0.0)
    return None
