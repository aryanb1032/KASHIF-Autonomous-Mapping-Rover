#!/usr/bin/env bash
set -eo pipefail
HERE="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$HERE/rover_ws/install/setup.bash"
export ROS_DOMAIN_ID=88
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
python3 -m unittest discover -s "$HERE/tests" -v
CHECK_BUILD="$(mktemp -d)"
trap 'rm -r "$CHECK_BUILD"' EXIT
g++ -std=c++17 -Wall -Wextra -Werror "$HERE/tests/test_firmware.cpp" -o "$CHECK_BUILD/parser"
"$CHECK_BUILD/parser"
g++ -std=c++17 -Wall -Wextra -Werror -I"$HERE/tests/fakes" "$HERE/tests/test_firmware_loop.cpp" -o "$CHECK_BUILD/controller"
"$CHECK_BUILD/controller"
echo 'Host firmware and Python checks passed. Use flash_esp32.sh --check for ESP32 compilation.'
