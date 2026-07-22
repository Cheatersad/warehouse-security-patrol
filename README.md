# Autonomous Warehouse Security Patrol Robot

**Team 9 — Three-Person Case Study Project**

A ROS 2 Humble simulation of an autonomous TurtleBot3 Waffle Pi that patrols a multi-zone warehouse, detects unattended or suspicious objects, displays live security markers in Gazebo, records evidence, and produces an incident report after a completed patrol cycle.

## Team Members and Task Division

| Person | Team member | Assigned task | Main contribution |
|---|---|---|---|
| **Person 1** | **Gaurang Chaudhary — 22509238** | Simulation, warehouse and mapping | Created the Gazebo warehouse, arranged the security zones, generated the occupancy map with Cartographer SLAM, and designed the 64-waypoint patrol route with safe robot clearance. |
| **Person 2** | **Karthik Katta — 22510526** | Localization, navigation and patrol control | Configured AMCL, TF, Nav2, A* global planning, DWB local control, costmaps, obstacle avoidance, recovery behaviour, and sequential patrol execution. |
| **Person 3** | **Suhel Khan Kareparambil Yoonus Khan — 22507454** | Perception, anomaly detection and alert management | Implemented camera-based detection, LiDAR/distance support, dynamic object tracking, visibility-based alerts, Gazebo warning markers, evidence logging, and end-of-patrol reports. |

## Project Summary

Warehouses often rely on fixed cameras and repetitive manual patrols. Fixed cameras can have blind spots, while manual patrols may be tiring, inconsistent, and expensive. This project demonstrates a mobile security robot that can:

- Navigate autonomously through a mapped warehouse.
- Patrol six security zones using 64 ordered waypoints.
- Localize itself with AMCL using a saved occupancy map.
- Plan global routes with NavFn using A* search.
- Follow paths and avoid obstacles with the DWB local controller.
- Detect unattended or suspicious objects using camera processing and distance information.
- Show a Gazebo warning marker only while an object is visible.
- Save evidence and create text/JSON reports after a complete patrol loop.

## Main Features

- Approximately **30 m × 22 m** custom warehouse.
- Six patrol zones and **64 sequential waypoints**.
- TurtleBot3 Waffle Pi with differential drive, 2D LiDAR, RGB camera, IMU and odometry.
- Cartographer SLAM for map generation.
- AMCL localization on the saved map.
- NavFn global planner with `use_astar: true`.
- DWB local controller for velocity generation and obstacle avoidance.
- Global and local costmaps with obstacle and inflation layers.
- Detection of a shoe, backpack, suitcase, restricted-zone person marker, aisle obstruction and hazardous spill.
- Dynamic object-position tracking through `/gazebo/model_states`.
- Visibility-state logic to prevent continuous or duplicate alerts.
- Temporary non-collision Gazebo beacons above visible anomalies.
- Evidence images, text logs, JSON logs and patrol-cycle reports.
- Optional YOLOv8 detector for standard object classes.

## Warehouse Zones

| Zone | Area |
|---|---|
| Zone A | Receiving |
| Zone B | Central Dispatch |
| Zone C | West Storage / Operational Section |
| Zone D | East Storage / Inspection-Aisle Section |
| Zone E | Restricted Inventory |
| Zone F | Packing |

The patrol manager covers one zone and then continues to the next until all 64 waypoints are completed.

## System Architecture

```text
Gazebo Warehouse
├── Camera ───────────────► Anomaly Detector
├── LiDAR ────────────────► Nav2 Costmaps / Distance Support
├── Odometry ─────────────► TF and AMCL
└── Simulation Clock ─────► ROS 2 nodes using simulation time

AMCL + Nav2
├── Map Server
├── NavFn A* Global Planner
├── DWB Local Controller
├── Recovery Behaviours
└── /cmd_vel ─────────────► TurtleBot3

Security Pipeline
├── /security_alert
├── /security/visibility
├── /security/detection_events
├── Gazebo Alert Visualizer
└── Event Logger ─────────► Evidence, logs and patrol reports
```

The required TF chain is:

```text
map → odom → base_footprint → base_link → sensors
```

## Repository Structure

```text
.
├── src/
│   ├── warehouse_patrol/
│   │   ├── config/              # Nav2 parameters and patrol waypoints
│   │   ├── launch/              # Simulation, SLAM and combined launch files
│   │   ├── maps/                # Saved occupancy-grid map
│   │   ├── worlds/              # Gazebo warehouse world
│   │   └── warehouse_patrol/    # Patrol manager and TF guard nodes
│   └── security_perception/
│       └── security_perception/ # Detection, alert visualizer and logger nodes
├── build_custom_packages.sh
├── run_warehouse_security.sh
├── run_warehouse_slam.sh
├── save_slam_map.sh
├── validate_warehouse_project.py
├── place_anomaly_in_front.py
└── README.md
```

## Software Environment

The following instructions assume that the user:

- Is already inside the Docker container.
- Has ROS 2 Humble and the project dependencies installed.
- Has placed the repository at `/ws_slam`.
- Knows basic Linux, ROS 2 and Colcon commands.

Main technologies:

- Ubuntu 22.04
- Docker
- ROS 2 Humble
- Gazebo Classic
- RViz2
- Nav2
- Cartographer
- AMCL
- OpenCV
- Python 3
- YAML and SDF/XML
- Optional Ultralytics YOLOv8

## Build the Project

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

## Run the Complete Three-Person Project

The recommended configuration uses the custom simulator anomaly detector because it reliably detects the project-specific Gazebo objects:

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

This starts:

- Gazebo warehouse simulation.
- TurtleBot3 Waffle Pi.
- Map server and AMCL.
- Nav2 planner, controller and recovery nodes.
- RViz visualization.
- 64-waypoint patrol manager.
- Custom anomaly detector.
- Gazebo alert visualizer.
- Security event logger and patrol report generator.

## Optional YOLOv8 Mode

YOLOv8 is useful for standard dataset classes such as `person`, `backpack` and `suitcase`. The custom simulator detector remains recommended for project-specific objects such as the shoe, spill and coloured obstruction.

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

In another terminal inside the same container:

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
ros2 topic echo /security/perception_status
ros2 topic echo /security/gazebo_visual_status
```

View the event log:

```bash
tail -f /ws_slam/security_log.txt
```

## Test Dynamic Object Detection

Move the shoe in front of the robot without manually dragging it in Gazebo:

```bash
python3 /ws_slam/place_anomaly_in_front.py shoe --distance 1.25
```

Expected behaviour:

1. The detector confirms that the shoe is inside the camera view.
2. An `UNATTENDED_SHOE` alert is published.
3. A warning beacon appears above the shoe in Gazebo.
4. The event is recorded by the logger.
5. The beacon disappears after the object is no longer visible.

## Alert Types

| Object | Alert reason |
|---|---|
| Shoe | `UNATTENDED_SHOE` |
| Backpack | `UNATTENDED_BACKPACK` |
| Suitcase | `UNATTENDED_SUITCASE` |
| Person marker | `PERSON_IN_RESTRICTED_ZONE` |
| Crate | `AISLE_OBSTRUCTION` |
| Spill | `HAZARDOUS_SPILL` |

## Patrol Reports

After all 64 waypoints are completed, the event logger creates:

```text
/ws_slam/patrol_reports/patrol_cycle_XXX_<timestamp>.txt
/ws_slam/patrol_reports/patrol_cycle_XXX_<timestamp>.json
```

Each report can include:

- Patrol cycle number.
- Object class and alert reason.
- Warehouse zone.
- Detection timestamp.
- Object map coordinates.
- Robot coordinates when the object was first seen.
- Estimated distance and confidence.
- Evidence-image path.

## SLAM and Map Generation

Run the mapping mode separately:

```bash
cd /ws_slam
./run_warehouse_slam.sh
```

Save the generated map:

```bash
./save_slam_map.sh warehouse_map
```

The saved map is used by AMCL during autonomous patrol.

## Important ROS 2 Components

| Component | Purpose |
|---|---|
| `robot_state_publisher` | Publishes robot-link transforms. |
| `map_server` | Publishes the saved occupancy map. |
| `amcl` | Estimates the robot pose on the map. |
| `planner_server` | Creates the global A* path through NavFn. |
| `controller_server` | Generates local motion using DWB. |
| `bt_navigator` | Executes navigation and recovery behaviour. |
| `warehouse_patrol_manager` | Sends the 64 ordered navigation goals. |
| `odom_tf_guard` | Publishes odometry TF only when the normal transform is missing. |
| `sim_anomaly_detector` | Detects custom suspicious objects and manages visibility state. |
| `yolo_node` | Optional YOLOv8 perception pipeline. |
| `gazebo_alert_visualizer` | Displays and removes Gazebo warning markers. |
| `security_event_logger` | Saves incidents, evidence and cycle reports. |

## Generated Files

Runtime-generated files are excluded from Git:

- `build/`
- `install/`
- `log/`
- `security_events/`
- `patrol_reports/`
- `security_log.txt`
- `security_events.jsonl`

## Troubleshooting Notes

- Do not run multiple Gazebo instances at the same time.
- All simulation nodes must use `use_sim_time:=true`.
- Avoid dragging models manually during autonomous navigation because simulation-time resets can clear TF buffers. Use `place_anomaly_in_front.py` for detection tests.
- The patrol report is generated only after all 64 waypoints are completed.
- A standard pretrained YOLO model cannot reliably detect every custom Gazebo object.
- A 2D LiDAR may miss low objects such as shoes or spills; camera detection and live simulator coordinates support these cases.
- If RViz has an OpenGL problem, use software rendering:

```bash
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
```

## Project Limitations

- Most testing was performed in Gazebo rather than on a physical robot.
- Colour segmentation can be affected by lighting and similar colours.
- Dynamic people and obstacles are simplified in simulation.
- The saved map is assumed to remain mostly unchanged.
- Real deployment would require custom model training, additional sensing and extensive safety testing.

## Future Improvements

- Train a custom YOLO model for warehouse-specific anomalies.
- Add an RGB-D camera or 3D LiDAR.
- Add semantic SLAM and dynamic human tracking.
- Add automatic charging and battery-aware patrol scheduling.
- Store reports in a database or cloud service.
- Send email or mobile notifications for high-severity incidents.
- Test the complete system on a physical TurtleBot3.

## Team 9

- **Gaurang Chaudhary — 22509238:** Simulation, warehouse design, SLAM mapping and waypoint route.
- **Karthik Katta — 22510526:** AMCL localization, Nav2 navigation, TF, costmaps and patrol control.
- **Suhel Khan Kareparambil Yoonus Khan — 22507454:** Perception, anomaly detection, alert visualization, evidence and reporting.
