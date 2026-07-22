#!/usr/bin/env bash
set -eo pipefail

WS="${WS:-/ws_slam}"
if [[ ! -d "$WS/src" ]]; then
  WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
fi

source /opt/ros/humble/setup.bash
if [[ -f "$WS/install/setup.bash" ]]; then
  # Reuse the TurtleBot3 packages already built in the supplied workspace.
  source "$WS/install/setup.bash"
fi

cd "$WS"
export TURTLEBOT3_MODEL=waffle_pi

python3 - <<'PY'
required = ['yaml', 'cv2']
missing = []
for module in required:
    try:
        __import__(module)
    except Exception as exc:
        missing.append(f'{module}: {exc}')
if missing:
    raise SystemExit('Missing Python dependencies:\n  ' + '\n  '.join(missing))
try:
    import ultralytics  # noqa: F401
    print('ultralytics: OK')
except Exception as exc:
    print(f'WARNING: ultralytics is unavailable ({exc}).')
    print('The project can still run with start_perception:=false.')
PY

colcon build --symlink-install --packages-select warehouse_patrol security_perception

echo
echo "Build complete. Run:"
echo "  source /opt/ros/humble/setup.bash"
echo "  source $WS/install/setup.bash"
echo "  ros2 launch warehouse_patrol warehouse_security.launch.py"
