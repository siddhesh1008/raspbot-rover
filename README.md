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
    └── rover_base/            ROS node: /cmd_vel → mecanum wheels, /ultrasonic, safety watchdog
```

Planned packages: `rover_description` (URDF model), `rover_sim` (Gazebo worlds),
`rover_bringup` (launch files).

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

Drive the rover from the PC:

```bash
# on the rover
ros2 launch rover_base base.launch.py
# in WSL (a second terminal); hold Shift for sideways moves (mecanum)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

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
