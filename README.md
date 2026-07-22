# Autonomous Warehouse Security Patrol Robot

A ROS 2 Humble simulation of an autonomous TurtleBot3 security robot operating inside a multi-zone warehouse. The robot patrols predefined zones with Nav2, detects unattended or suspicious objects using its camera and LiDAR, displays temporary alert markers in Gazebo, logs security events, and creates a report after each completed patrol loop.

## Project Summary

This repository combines two project tasks:

- **Person 1 — Autonomous patrol:** warehouse world and occupancy map, localization with AMCL, Nav2 path planning, obstacle avoidance, zone-based waypoints, and continuous patrol loops.
- **Person 2 — Security perception:** camera-based anomaly detection, LiDAR distance estimation, live object visibility tracking, Gazebo alert markers, evidence logging, and patrol-cycle reports.

### Main Features

- Complex warehouse with six patrol zones and 64 waypoints.
- TurtleBot3 Waffle Pi simulation in Gazebo Classic.
- AMCL localization and Nav2 autonomous navigation.
- NavFn global planner and DWB local controller.
- Detection of simulated suspicious objects such as a shoe, backpack, suitcase, obstruction, spill, and restricted-zone person.
- Alert marker appears only while an object is visible to the robot and disappears when it leaves the camera view.
- Dynamic object tracking through Gazebo model states, allowing test objects to be moved during a run.
- Security alerts, evidence snapshots, text logs, JSON logs, and end-of-loop patrol reports.
- Optional YOLOv8 perception pipeline.

## Repository Structure

```text
.
├── src/
│   ├── warehouse_patrol/       # World, map, Nav2, patrol and launch files
│   └── security_perception/     # Detection, alerts, visualizer and logger
├── build_custom_packages.sh
├── run_warehouse_security.sh
├── run_warehouse_slam.sh
├── save_slam_map.sh
├── validate_warehouse_project.py
├── place_anomaly_in_front.py    # Optional dynamic detection test helper
└── README.md
```

## Requirements

The following instructions assume the user is already inside the project Docker container and the repository is located at `/ws_slam`.

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic
- TurtleBot3 packages
- Nav2 and AMCL
- Python 3, OpenCV, NumPy and PyYAML
- Ultralytics only when the optional YOLO detector is enabled

## Build

```bash
cd /ws_slam

set +u
source /opt/ros/humble/setup.bash

rosdep install --from-paths src --ignore-src -r -y

colcon build \
  --base-paths /ws_slam/src \
  --symlink-install \
  --packages-select warehouse_patrol security_perception

source /ws_slam/install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
```

The helper script may also be used:

```bash
cd /ws_slam
chmod +x build_custom_packages.sh
./build_custom_packages.sh
```

## Run the Complete Project

The recommended configuration uses the simulator anomaly detector because it reliably detects the custom Gazebo objects:

```bash
cd /ws_slam

set +u
source /opt/ros/humble/setup.bash
source /ws_slam/install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi

./run_warehouse_security.sh \
  gui:=true \
  rviz:=true \
  start_patrol:=true \
  start_perception:=false \
  start_sim_detector:=true \
  start_gazebo_alert_visualizer:=true \
  start_logger:=true \
  demo_alerts:=false
```

This starts Gazebo, RViz, Nav2, AMCL, the waypoint patrol manager, custom anomaly detection, alert visualization and event logging.

### Run With YOLOv8 as an Additional Detector

Install Ultralytics and place the required model weights in the workspace before using this mode.

```bash
./run_warehouse_security.sh \
  gui:=true \
  rviz:=true \
  start_patrol:=true \
  start_perception:=true \
  start_sim_detector:=true \
  start_gazebo_alert_visualizer:=true \
  start_logger:=true \
  demo_alerts:=false
```

## Monitor the System

Open another terminal inside the same container and source the workspace:

```bash
source /opt/ros/humble/setup.bash
source /ws_slam/install/setup.bash
```

Useful topics:

```bash
ros2 topic echo /patrol/current_zone
ros2 topic echo /patrol/status
ros2 topic echo /security_alert
ros2 topic echo /security/visibility
ros2 topic echo /security/detection_events
ros2 topic echo /security/gazebo_visual_status
```

View the event log:

```bash
tail -f /ws_slam/security_log.txt
```

Reports are generated after a complete patrol loop:

```text
/ws_slam/patrol_reports/patrol_cycle_XXX_<timestamp>.txt
/ws_slam/patrol_reports/patrol_cycle_XXX_<timestamp>.json
```

## Test Dynamic Object Detection

A suspicious object can be moved in front of the robot without manually dragging it in Gazebo:

```bash
python3 /ws_slam/place_anomaly_in_front.py shoe --distance 1.25
```

The marker should appear when the camera sees the shoe and disappear after the shoe leaves the camera view.

## SLAM Mode

To demonstrate mapping separately:

```bash
cd /ws_slam
./run_warehouse_slam.sh
```

Save the generated map with:

```bash
./save_slam_map.sh warehouse_map
```

## Output Files

Generated runtime files are intentionally excluded from Git:

- `build/`, `install/`, `log/`
- `security_events/`
- `patrol_reports/`
- `security_log.txt`
- `security_events.jsonl`

## Notes

- Do not run multiple Gazebo instances at the same time.
- Avoid dragging models in Gazebo during autonomous navigation because a simulation-time reset can clear TF buffers and interrupt Nav2. Use `place_anomaly_in_front.py` for detection tests.
- If RViz has an OpenGL problem, run it with software rendering:

```bash
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
```

## Authors

- **Person 1:** autonomous warehouse mapping, localization, navigation and patrol.
- **Person 2:** anomaly detection, security alerts, evidence logging and patrol reports.
