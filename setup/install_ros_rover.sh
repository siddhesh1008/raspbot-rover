#!/usr/bin/env bash
# Rover side (Ubuntu Server 24.04 on the Pi 5): ROS 2 Jazzy base (no GUI) + hardware access.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
bash "$here/install_ros_common.sh"

echo "== ROS 2 Jazzy base, tools and hardware packages =="
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
  ros-jazzy-ros-base ros-dev-tools \
  ros-jazzy-teleop-twist-keyboard ros-jazzy-v4l2-camera ros-jazzy-image-transport-plugins \
  i2c-tools python3-smbus2 python3-pytest v4l-utils git

echo "== Let user '$USER' use I2C and the camera =="
# Before i2c-tools is installed /dev/i2c-1 belongs to group dialout; i2c-tools adds an 'i2c' group
# and a udev rule that hands the device to it. Join all of them.
getent group i2c >/dev/null || sudo groupadd --system i2c
sudo usermod -aG i2c,dialout,video "$USER"

echo "== rosdep =="
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init
rosdep update

echo "== Shell setup (~/.bashrc) =="
grep -q 'ROVER ROS SETUP' ~/.bashrc || cat >> ~/.bashrc <<'EOF'
# --- ROVER ROS SETUP ---
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0          # default domain; the PC firewall rule ROS2-DDS-LAN opens UDP 7400-7600 = domain 0 ports
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
# --- END ROVER ROS SETUP ---
EOF

echo
echo "ROS 2 Jazzy install on the rover finished: INSTALL_DONE"
echo "Rebooting in 5 seconds so the new groups and kernel updates take effect ..."
sleep 5
sudo reboot
