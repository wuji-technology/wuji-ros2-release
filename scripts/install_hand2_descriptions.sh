#!/usr/bin/env bash
# Install the Wuji Hand 2 description packages from the ROS 2 testing apt
# repository. They aren't in the main repository until the next ROS 2 Lyrical
# sync. Only these packages come from the testing repository: the script adds
# a temporary source, installs them, and removes the source on exit.

set -eo pipefail

distro="${ROS_DISTRO:?Source /opt/ros/<distro>/setup.bash first}"
ros_source=/etc/apt/sources.list.d/ros2.sources
testing_source=/etc/apt/sources.list.d/wuji-ros2-testing.sources

sudo=
if [[ "$EUID" -ne 0 ]]; then
  sudo=sudo
fi

if [[ ! -f "$ros_source" ]]; then
  echo "error: $ros_source not found. Install the ROS 2 apt source first." >&2
  exit 1
fi

cleanup() {
  $sudo rm -f "$testing_source"
  $sudo apt-get update -qq || true
}
trap cleanup EXIT

sed 's#/ros2/ubuntu#/ros2-testing/ubuntu#' "$ros_source" |
  $sudo tee "$testing_source" >/dev/null
$sudo apt-get update -qq
$sudo apt-get install -y --no-install-recommends \
  "ros-${distro}-wuji-hand2-1-description" \
  "ros-${distro}-wuji-hand2-2-description"
