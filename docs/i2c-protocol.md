# Raspbot V2 expansion board: I2C protocol

Our own reference for talking to the Yahboom Raspbot V2 expansion board (the small MCU board
that drives the motors, servos, lights, buzzer and reads the sensors). It is written from
Yahboom's published protocol sheet, *"Main control board and expansion board i2c protocol.pdf"*
(their Google Drive, folder `4.About Hardware`), and the register examples in their driver
tutorial PDF. None of Yahboom's code is used in this project.

Items marked **(verify)** are not stated clearly in the sheet and must be checked on the robot.

## Bus

| | |
|---|---|
| I2C bus | `/dev/i2c-1` (Pi 5 GPIO 2/3) |
| Device address | `0x2B` |
| Other device on the bus | `0x3C`: the OLED display (separate SSD1306-style chip, not the MCU) **(verify)** |

Every **write** is one I2C block write: the register number, then its data bytes.
In Python (`smbus2`): `bus.write_i2c_block_data(0x2B, reg, [data...])`.

Every **read** is: write the register number, then read N bytes.
In Python: `bus.read_i2c_block_data(0x2B, reg, n)`.

## Write registers (Pi → board)

| Reg | What | Data bytes | Notes |
|---|---|---|---|
| `0x01` | Motor | `[motor, direction, speed]` | `motor`: 0 = L1, 1 = L2, 2 = R1, 3 = R2. `direction`: 0 forward, 1 backward. `speed`: 0–255. One motor per write. Example: L1 forward at 200 → `[0x00, 0x00, 0xC8]`. **Tested** (wheels off the ground, speed 80): L1 = front-left, L2 = rear-left, R1 = front-right, R2 = rear-right; direction 0 spins **every** wheel the way that drives the rover forward, so no wheel needs inverting. |
| `0x02` | Servo | `[servo, angle]` | `servo`: 1 = S1, 2 = S2. `angle`: 0–180°. Example: S1 to 90° → `[0x01, 0x5A]`. **Tested:** S1 = camera **pan**, 0° = right, 180° = left, 90° = centre, full 0–180° range is free. S2 = camera **tilt**, 0° = down, 180° = up. On this robot the camera is **level at 30°** (pan level/centre: S1 = 90°). Level sits far from 90° because the servo arm is mounted off-centre on its spline, so the camera can tilt down only ~30° but up ~150°; refitting the arm would move level towards 90°. **S2 has twice stopped responding** after quick full-range moves and recovered after a slow sweep: avoid jumping S2 across its full range in one command. |
| `0x03` | Light bar, all 14 LEDs, preset colour | `[on, colour]` | `on`: 0/1. `colour`: 0 red, 1 green, 2 blue, 3 yellow, 4 purple, 5 indigo (cyan), 6 white. Wait **≥ 50 ms** after each write. **Tested:** all 7 colours correct. |
| `0x04` | Light bar, one LED, preset colour | `[led, on, colour]` | `led`: 1–14. Wait **≥ 50 ms** after each write. |
| `0x08` | Light bar, all 14 LEDs, RGB | `[r, g, b]` | 0–255 each. Wait **≥ 50 ms** after each write. |
| `0x09` | Light bar, one LED, RGB | `[led, r, g, b]` | `led`: 1–14. 50 ms wait not stated; use it anyway. |
| `0x05` | IR remote receiver on/off | `[state]` | 0 off, 1 on. Off after every power-up. Turn it off when not needed: the sheet says it "will affect other functions". |
| `0x06` | Buzzer on/off | `[state]` | 0 off, 1 on (it beeps until turned off). **Tested.** |
| `0x07` | Ultrasonic sensor on/off | `[state]` | 0 off, 1 on. Only turn on when the sensor is plugged in. Turn off when not needed. |

The LED colour list in the sheet ends with "Close"; colour numbers above 6 are not used.

## Read registers (board → Pi)

| Reg | What | Bytes | Meaning |
|---|---|---|---|
| `0x0A` | Line sensors (4 IR) | 1 | Low 4 bits: bit 3 = X1 (leftmost) … bit 0 = X4 (rightmost). The sheet's example: only X1 "lit" reads `0b0111`, so a sensor reads **0 when it is triggered**. **Open issue:** on the robot this register always read `0x00`, even while the module's own LEDs reacted to dark objects. |
| `0x1A` | Ultrasonic distance, low byte | 1 | Distance in **mm** = `(high << 8) \| low`. Reads 0 when the sensor is off. **Tested:** hand at ~5–12 cm read 51–119 mm, open room ~1040 mm. Readings move in steps of ~17 mm. |
| `0x1B` | Ultrasonic distance, high byte | 1 | |
| `0x0C` | IR remote key code | 1 | `0xFF` when no key; resets to `0xFF` after each read. IR receiver must be on (`0x05`). |
| `0x0D` | KEY1 button | 1 | 0 or 1. **Open issue:** always read `0` on the robot, also while KEY1 was held. |

## Not available from the board

- **No wheel encoders**: the board only sets motor power, it cannot report wheel speed or
  distance travelled. Odometry has to be estimated from commands (open loop).
- **No IMU, no battery voltage register** in the sheet.

## Safety rules for our driver

1. On start-up and on exit (including crashes and Ctrl-C), **stop all four motors**.
2. A **watchdog**: if no drive command arrives for ~0.5 s, stop the motors. The board itself keeps
   the last speed forever.
3. Clamp every value to its range before writing (speed 0–255, angle 0–180, LED 1–14).
4. Respect the 50 ms gap after light-bar writes.
5. First tests on real hardware: robot on a stand, wheels off the ground.
