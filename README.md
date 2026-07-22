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

## Run the Complete Project

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

## Patrol Reports

After all 64 waypoints are completed, the event logger creates:

```text
/ws_slam/patrol_reports/patrol_cycle_XXX_<timestamp>.txt
/ws_slam/patrol_reports/patrol_cycle_XXX_<timestamp>.json
```

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

## Team 9

- **Gaurang Chaudhary — 22509238:** Simulation, warehouse design, SLAM mapping and waypoint route.
- **Karthik Katta — 22510526:** AMCL localization, Nav2 navigation, TF, costmaps and patrol control.
- **Suhel Khan Kareparambil Yoonus Khan — 22507454:** Perception, anomaly detection, alert visualization, evidence and reporting.
