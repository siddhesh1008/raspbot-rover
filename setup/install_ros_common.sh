#!/usr/bin/env bash
# Shared part: adds the official ROS 2 apt repository on Ubuntu 24.04 (noble).
# Follows https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
set -euo pipefail

echo "== Ubuntu package sources: make sure noble-updates is enabled =="
# Some Ubuntu Pi images ship without the 'noble-updates' suite. Then packages already on the
# image are newer than anything apt can see, and installing ROS fails with "unmet dependencies".
src=/etc/apt/sources.list.d/ubuntu.sources
if [ -f "$src" ] && ! grep -q 'noble-updates' "$src"; then
  sudo cp "$src" "$src.before-rover"
  sudo sed -i 's/^Suites: noble$/Suites: noble noble-updates noble-backports/' "$src"
fi

echo "== Locale (UTF-8) =="
sudo apt-get update
sudo apt-get install -y locales software-properties-common curl
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

echo "== Ubuntu 'universe' repository =="
sudo add-apt-repository -y universe

echo "== ROS 2 apt repository =="
if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
  ver=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F '"tag_name"' | awk -F'"' '{print $4}')
  codename=$(. /etc/os-release && echo "$VERSION_CODENAME")
  curl -fL -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ver}/ros2-apt-source_${ver}.${codename}_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb
fi

sudo apt-get update
sudo DEBIAN_FRONTEND=noninteractive apt-get full-upgrade -y
