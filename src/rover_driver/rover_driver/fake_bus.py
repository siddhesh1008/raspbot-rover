"""A stand-in for the I2C bus, for tests and for running without the robot."""

from __future__ import annotations


class FakeBus:
    """Records every write and answers reads from a register table you can set."""

    def __init__(self) -> None:
        self.writes: list[tuple[int, int, list[int]]] = []
        self.registers: dict[int, int] = {}
        self.closed = False

    def write_i2c_block_data(self, address: int, register: int, data: list[int]) -> None:
        for byte in data:
            if not 0 <= byte <= 255:
                raise ValueError(f"byte out of range: {byte}")
        self.writes.append((address, register, list(data)))

    def read_i2c_block_data(self, address: int, register: int, length: int) -> list[int]:
        return [self.registers.get(register + i, 0) for i in range(length)]

    def close(self) -> None:
        self.closed = True

    def writes_to(self, register: int) -> list[list[int]]:
        return [data for _, reg, data in self.writes if reg == register]
