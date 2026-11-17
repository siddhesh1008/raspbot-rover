#!/usr/bin/env bash
# PC side (WSL Ubuntu 24.04): ROS 2 Jazzy desktop (RViz) + Gazebo Harmonic + dev tools.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
bash "$here/install_ros_common.sh"

echo "== ROS 2 Jazzy desktop, Gazebo and tools (large download, ~15-30 min) =="
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
  ros-jazzy-desktop ros-dev-tools \
  ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-teleop-twist-keyboard ros-jazzy-rqt-image-view \
  python3-pytest python3-smbus2

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
echo "ROS 2 Jazzy install on WSL finished: INSTALL_DONE"
