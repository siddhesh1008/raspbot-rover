# raspbot-rover

Software for a **Yahboom Raspbot V2** (Raspberry Pi 5, mecanum wheels, pan/tilt camera),
written from scratch on **ROS 2 Jazzy**. No Yahboom software is used: we talk to the robot's
expansion board ourselves, from their published I2C protocol ([docs/i2c-protocol.md](docs/i2c-protocol.md)).

The same code runs on the real rover and in a simulated "digital twin" on the PC.

## Hardware

| Part | Notes |
|---|---|
| Raspberry Pi 5 | Ubuntu Server 24.04, hostname `rover` |
| Expansion board (I2C `0x2B`) | 4 DC motors, 2 servos, 14-LED light bar, buzzer, ultrasonic, 4-ch line sensor, IR receiver, button |
| 4 × mecanum wheels | no encoders: the board cannot report wheel speed |
| USB camera on a 2-servo pan/tilt | `/dev/video0` |
| OLED display | I2C `0x3C` |

## Repository layout

```
raspbot-rover/
├── docs/
│   └── i2c-protocol.md        register map of the expansion board
├── setup/                     one-time install scripts (ROS 2 Jazzy etc.)
│   ├── install_ros_wsl.sh     PC: WSL Ubuntu 24.04 + RViz + Gazebo
│   ├── install_ros_rover.sh   rover: Ubuntu Server 24.04 on the Pi 5
│   └── wsl_ros_network.sh     WSL fix: ROS only on the home network + localhost (see below)
└── src/                       ROS 2 workspace packages (built with colcon)
    ├── rover_driver/          our I2C driver: plain Python, no ROS, fully unit-tested
    ├── rover_base/            ROS node: /cmd_vel → mecanum wheels, /ultrasonic, safety watchdog
    ├── rover_hmi/             browser control page, http://rover.local:8080
    ├── rover_teleop/          WASD keyboard driving in a terminal (runs in WSL)
    └── rover_bringup/         launch files: everything that runs on the rover
```

Planned packages: `rover_description` (URDF model), `rover_sim` (Gazebo worlds).

## rover_base

| | Topic | Type | |
|---|---|---|---|
| in | `/cmd_vel` | `geometry_msgs/Twist` | `linear.x` forward, `linear.y` left (m/s), `angular.z` turn left (rad/s) |
| out | `/ultrasonic` | `sensor_msgs/Range` | distance ahead in m, 10 Hz |

- Motors stop if no `/cmd_vel` arrives for 0.5 s (e.g. Wi-Fi drops) and when the node exits.
- Settings: [src/rover_base/config/rover_base.yaml](src/rover_base/config/rover_base.yaml).
  `max_pwm: 160` caps the speed while testing indoors; `max_wheel_speed` and the geometry are
  estimates to tune on the robot.
- There are no wheel encoders, so speeds are open-loop estimates.

## Drive from the browser (rover_hmi)

```bash
# on the rover: base node + web page
ros2 launch rover_bringup rover.launch.py
```

Open **http://rover.local:8080** on the PC (or a phone on the same Wi-Fi). Hold **W/A/S/D** to
drive and turn, **Q/E** to slide sideways; keys combine (W+A curves). On-screen buttons work with
mouse and touch. The page shows the ultrasonic distance and has speed sliders.
The rover stops when you let go, press space or STOP, switch tabs, or the connection drops.
Opposite keys together (W+S, Q+E, A+D) stop the rover and lock driving until all keys are released.

How driving stays responsive and safe:
- **Latest wins.** The page keeps at most one request in flight and only sends the newest command;
  `/cmd_vel` has queue depth 1. Old commands are dropped, never replayed late.
- **Acceleration ramp, instant stop.** `rover_base` ramps speed up (`ramp_pwm_per_s`, about 0.15 s to
  cruising speed) but a stop is applied at once.
- **Watchdogs.** The HMI stops after 0.4 s without requests; `rover_base` has its own thread that
  stops after 0.5 s without `/cmd_vel` and re-sends the motor state (the board can miss a write).

Test without the robot (in WSL): `ros2 launch rover_bringup rover.launch.py fake:=true`, then
http://localhost:8080. Fake mode runs in the `/fake` namespace, so it can never drive the real
rover, which shares the same ROS network.

## Drive from a terminal (rover_teleop)

```bash
# on the rover
ros2 launch rover_base base.launch.py
# in WSL (a second terminal): W/S forward/back, A/D turn, Q/E slide sideways, space stop, +/- speed
ros2 run rover_teleop wasd
```

Hold a key to keep moving; the rover stops 0.6 s after you let go (a terminal can't see key
releases, so this is set just longer than the keyboard's auto-repeat delay).

Without the robot: `ros2 launch rover_base base.launch.py fake:=true` logs the motor values.

## How we work

```
 PC (Windows + WSL)              GitHub                       Rover (Pi 5)
 edit code here  ──git push──▶  raspbot-rover  ──git pull──▶  ~/raspbot-rover
 test in WSL: unit tests                                      colcon build, run on hardware
 + Gazebo simulation
```

The PC and the rover talk ROS 2 over Wi-Fi (domain 0). See `raspberry_pi_info.txt` next to this
project for logins, the WSL network setup and the firewall rule.

**WSL network note.** WSL uses "mirrored" networking, so Linux also sees Tailscale and an internal
WSL address. Fast DDS (ROS 2's default middleware) then fails to connect even between two nodes
inside WSL. [setup/wsl_ros_network.sh](setup/wsl_ros_network.sh), sourced from `~/.bashrc`,
limits it to the home-network address and localhost.

Tip: `ros2 topic echo` gives up after about a second if it hasn't found the topic yet; pass the
type to make it wait, e.g. `ros2 topic echo /ultrasonic sensor_msgs/msg/Range`.

## Build and test

Both on the rover and in WSL, from the repository folder:

```bash
colcon build --symlink-install
source install/setup.bash
colcon test && colcon test-result --verbose
```

In WSL the repository is at `/mnt/c/Users/siddh/OneDrive/Desktop/Projects/raspbot-rover`.

## First hardware checks (on the rover)

```bash
ros2 run rover_driver rover-hw beep
ros2 run rover_driver rover-hw lights
ros2 run rover_driver rover-hw sensors --seconds 10
ros2 run rover_driver rover-hw servo 1 90
ros2 run rover_driver rover-hw motor L1 80 --seconds 1    # robot on a stand, wheels off the ground
```

Add `--fake` to any command to run it without hardware; it prints the bytes it would send.
Motor tests are capped at speed 150 and 3 seconds, and the motors always stop at the end.

## Setting up from scratch

1. Rover: flash **Ubuntu Server 24.04 (64-bit)** with Raspberry Pi Imager (hostname `rover`,
   user `rover`, Wi-Fi, SSH key). Then copy `setup/` to the rover and run
   `bash setup/install_ros_rover.sh`.
2. PC: `wsl --install -d Ubuntu-24.04`, then in WSL `bash setup/install_ros_wsl.sh`.
