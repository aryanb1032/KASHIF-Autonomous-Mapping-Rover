#!/usr/bin/env bash
set -eo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
if [[ "${1:-}" == --help ]]; then
  echo 'Usage: ./setup_pi.sh (Ubuntu 24.04, ROS 2 apt repository already configured)'; exit 0
fi
source /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 24.04 ]]; then
  echo 'Requires Ubuntu 24.04. See README.md.' >&2; exit 1
fi
if ! apt-cache show ros-jazzy-desktop >/dev/null 2>&1; then
  echo 'Configure the official ROS 2 Jazzy apt repository first:' >&2
  echo 'https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html' >&2
  exit 1
fi
sudo apt-get update
sudo apt-get install -y git python3-serial python3-scipy python3-numpy python3-yaml \
  ros-jazzy-desktop ros-dev-tools python3-colcon-common-extensions python3-rosdep \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge
LIDAR_COMMIT=34300099fadfc772965962dec837bf436706188f
LIDAR_DIR="$HERE/rover_ws/src/sllidar_ros2"
if [[ ! -d "$LIDAR_DIR" ]]; then
  git clone https://github.com/Slamtec/sllidar_ros2.git "$LIDAR_DIR"
  git -C "$LIDAR_DIR" checkout --detach "$LIDAR_COMMIT"
elif [[ "$(git -C "$LIDAR_DIR" rev-parse HEAD)" != "$LIDAR_COMMIT" ]]; then
  echo "Existing lidar checkout differs from tested commit $LIDAR_COMMIT. Preserve your changes and resolve this before building." >&2; exit 1
fi
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then sudo rosdep init; fi
rosdep update
source /opt/ros/jazzy/setup.bash
cd "$HERE/rover_ws"
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
printf '\nSource this in each terminal:\nsource /opt/ros/jazzy/setup.bash\nsource %s/rover_ws/install/setup.bash\n' "$HERE"
