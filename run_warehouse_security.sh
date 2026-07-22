#!/usr/bin/env bash
set -eo pipefail
WS="${WS:-/ws_slam}"
if [[ ! -f "$WS/install/setup.bash" ]]; then
  WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
fi
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export TURTLEBOT3_MODEL=waffle_pi
cd "$WS"
exec ros2 launch warehouse_patrol warehouse_security.launch.py "$@"
