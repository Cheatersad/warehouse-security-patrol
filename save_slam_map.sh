#!/usr/bin/env bash
set -eo pipefail
WS="${WS:-/ws_slam}"
OUTPUT="${1:-$WS/warehouse_slam_map}"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
ros2 run nav2_map_server map_saver_cli -f "$OUTPUT" --ros-args -p save_map_timeout:=30.0
printf 'Saved map to %s.yaml and %s.pgm\n' "$OUTPUT" "$OUTPUT"
