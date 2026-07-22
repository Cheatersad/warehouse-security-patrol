#!/usr/bin/env python3
"""Camera + Gazebo-model-state anomaly detector for movable simulation props."""
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
import json
import math
from pathlib import Path
import time
from typing import Any

import cv2
from cv_bridge import CvBridge
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String


@dataclass(frozen=True)
class ObjectSpec:
    marker_name: str
    gazebo_model_name: str
    object_class: str
    reason: str
    severity: str
    lower_hsv: tuple[int, int, int]
    upper_hsv: tuple[int, int, int]
    fallback_x: float
    fallback_y: float
    min_area: float
    max_area_fraction: float
    min_aspect: float
    max_aspect: float
    pixel_tolerance_fraction: float
    lidar_required: bool = True


OBJECTS = (
    ObjectSpec(
        marker_name='unattended_shoe_marker',
        gazebo_model_name='sim_anomaly_shoe',
        object_class='shoe',
        reason='UNATTENDED_SHOE',
        severity='HIGH',
        lower_hsv=(124, 120, 65),
        upper_hsv=(149, 255, 255),
        fallback_x=-6.65,
        fallback_y=-9.20,
        min_area=45.0,
        max_area_fraction=0.62,
        min_aspect=0.10,
        max_aspect=1.70,
        pixel_tolerance_fraction=0.23,
        # A low shoe can fall below the TurtleBot3 lidar plane after it is moved.
        lidar_required=False,
    ),
    ObjectSpec(
        marker_name='unattended_backpack_marker',
        gazebo_model_name='sim_anomaly_backpack',
        object_class='backpack',
        reason='UNATTENDED_BACKPACK',
        severity='HIGH',
        lower_hsv=(80, 145, 90),
        upper_hsv=(104, 255, 255),
        fallback_x=-8.90,
        fallback_y=-5.00,
        min_area=90.0,
        max_area_fraction=0.42,
        min_aspect=0.55,
        max_aspect=3.00,
        pixel_tolerance_fraction=0.16,
    ),
    ObjectSpec(
        marker_name='unattended_suitcase_marker',
        gazebo_model_name='sim_anomaly_suitcase',
        object_class='suitcase',
        reason='UNATTENDED_SUITCASE',
        severity='HIGH',
        lower_hsv=(21, 175, 130),
        upper_hsv=(34, 255, 255),
        fallback_x=8.40,
        fallback_y=-6.00,
        min_area=120.0,
        max_area_fraction=0.38,
        min_aspect=0.95,
        max_aspect=3.50,
        pixel_tolerance_fraction=0.12,
    ),
    ObjectSpec(
        marker_name='restricted_person_marker',
        gazebo_model_name='sim_anomaly_person_marker',
        object_class='person',
        reason='PERSON_IN_RESTRICTED_ZONE',
        severity='CRITICAL',
        lower_hsv=(140, 145, 90),
        upper_hsv=(177, 255, 255),
        fallback_x=13.15,
        fallback_y=0.55,
        min_area=110.0,
        max_area_fraction=0.55,
        min_aspect=0.70,
        max_aspect=5.00,
        pixel_tolerance_fraction=0.16,
        lidar_required=False,
    ),
    ObjectSpec(
        marker_name='aisle_obstruction_marker',
        gazebo_model_name='sim_anomaly_aisle_obstruction',
        object_class='box',
        reason='AISLE_OBSTRUCTION',
        severity='MEDIUM',
        lower_hsv=(5, 155, 100),
        upper_hsv=(19, 255, 255),
        fallback_x=8.20,
        fallback_y=6.90,
        min_area=90.0,
        max_area_fraction=0.48,
        min_aspect=0.45,
        max_aspect=2.20,
        pixel_tolerance_fraction=0.16,
    ),
    ObjectSpec(
        marker_name='hazardous_spill_marker',
        gazebo_model_name='sim_anomaly_hazardous_spill',
        object_class='spill',
        reason='HAZARDOUS_SPILL',
        severity='HIGH',
        lower_hsv=(39, 140, 75),
        upper_hsv=(77, 255, 255),
        fallback_x=-3.80,
        fallback_y=6.50,
        min_area=55.0,
        max_area_fraction=0.58,
        min_aspect=0.05,
        max_aspect=1.10,
        pixel_tolerance_fraction=0.19,
        lidar_required=False,
    ),
)


class SimAnomalyDetector(Node):
    def __init__(self) -> None:
        super().__init__('sim_anomaly_detector')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('model_states_topic', '/gazebo/model_states')
        self.declare_parameter('robot_model_name', 'waffle_pi')
        self.declare_parameter('camera_horizontal_fov', 1.085595)
        self.declare_parameter('lidar_sector_degrees', 8.0)
        # Kept for launch-file compatibility. Per-object values are used instead.
        self.declare_parameter('minimum_contour_area', 45.0)
        self.declare_parameter('maximum_area_fraction', 0.62)
        self.declare_parameter('process_every_n_frames', 2)
        self.declare_parameter('maximum_alert_distance_m', 7.0)
        self.declare_parameter('bearing_margin_degrees', 11.0)
        self.declare_parameter('bearing_pixel_tolerance_fraction', 0.16)
        self.declare_parameter('lidar_tolerance_m', 0.95)
        self.declare_parameter('lidar_tolerance_fraction', 0.34)
        self.declare_parameter('confirmation_frames', 2)
        self.declare_parameter('clear_frames', 2)
        self.declare_parameter('detection_publish_period_seconds', 0.35)
        self.declare_parameter('realert_cooldown_seconds', 2.0)
        self.declare_parameter('alert_cooldown_seconds', 12.0)
        self.declare_parameter('save_snapshots', True)
        self.declare_parameter('snapshot_directory', '/ws_slam/security_events')

        self.bridge = CvBridge()
        self.camera_hfov = float(self.get_parameter('camera_horizontal_fov').value)
        self.lidar_half_sector = math.radians(
            float(self.get_parameter('lidar_sector_degrees').value) / 2.0
        )
        self.process_every_n_frames = max(
            1, int(self.get_parameter('process_every_n_frames').value)
        )
        self.maximum_alert_distance = float(
            self.get_parameter('maximum_alert_distance_m').value
        )
        self.bearing_margin = math.radians(
            float(self.get_parameter('bearing_margin_degrees').value)
        )
        self.lidar_tolerance_m = float(self.get_parameter('lidar_tolerance_m').value)
        self.lidar_tolerance_fraction = float(
            self.get_parameter('lidar_tolerance_fraction').value
        )
        self.confirmation_frames = max(
            1, int(self.get_parameter('confirmation_frames').value)
        )
        self.clear_frames = max(1, int(self.get_parameter('clear_frames').value))
        self.detection_publish_period_ns = int(
            max(0.05, float(self.get_parameter('detection_publish_period_seconds').value))
            * 1e9
        )
        self.realert_cooldown_ns = int(
            max(0.0, float(self.get_parameter('realert_cooldown_seconds').value))
            * 1e9
        )
        self.save_snapshots = bool(self.get_parameter('save_snapshots').value)
        self.snapshot_directory = Path(
            str(self.get_parameter('snapshot_directory').value)
        )
        self.snapshot_directory.mkdir(parents=True, exist_ok=True)
        self.robot_model_name = str(self.get_parameter('robot_model_name').value)

        self.latest_scan: LaserScan | None = None
        self.current_patrol_zone = 'UNKNOWN'
        self.amcl_robot_pose: tuple[float, float, float] | None = None
        self.sim_robot_pose: tuple[float, float, float] | None = None
        self.object_positions: dict[str, tuple[float, float]] = {
            spec.marker_name: (spec.fallback_x, spec.fallback_y) for spec in OBJECTS
        }
        self.dynamic_position_available: dict[str, bool] = {
            spec.marker_name: False for spec in OBJECTS
        }
        self.last_model_states_wall_time = 0.0
        self.last_logged_positions = dict(self.object_positions)
        self.frame_count = 0
        self.visibility_sequence = 0
        self.image_received = False
        self.model_states_received = False
        self.alerts_published = 0
        self.detections_published = 0

        self.hit_streak = {spec.marker_name: 0 for spec in OBJECTS}
        self.miss_streak = {spec.marker_name: 0 for spec in OBJECTS}
        self.confirmed_visible = {spec.marker_name: False for spec in OBJECTS}
        self.last_payload: dict[str, dict[str, Any]] = {}
        self.last_detection_ns: dict[str, int] = {}
        self.last_alert_ns: dict[str, int] = {}

        image_topic = str(self.get_parameter('image_topic').value)
        scan_topic = str(self.get_parameter('scan_topic').value)
        model_states_topic = str(self.get_parameter('model_states_topic').value)
        self.create_subscription(Image, image_topic, self.image_callback, qos_profile_sensor_data)
        self.create_subscription(LaserScan, scan_topic, self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(ModelStates, model_states_topic, self.model_states_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.create_subscription(String, '/patrol/current_zone', self.zone_callback, 10)

        self.alert_publisher = self.create_publisher(String, '/security_alert', 10)
        self.detection_publisher = self.create_publisher(String, '/security/detections', 10)
        self.visibility_publisher = self.create_publisher(String, '/security/visibility', 10)
        self.event_publisher = self.create_publisher(String, '/security/detection_events', 10)
        self.annotated_publisher = self.create_publisher(Image, '/security/sim_annotated_image', 10)
        self.status_publisher = self.create_publisher(String, '/security/perception_status', 10)
        self.create_timer(2.0, self.publish_status)

        self.kernel = np.ones((5, 5), dtype=np.uint8)
        self.get_logger().info(
            'Dynamic visibility detector ready. Gazebo model positions are tracked, '
            'so moved anomaly objects can be detected at their new locations.'
        )

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        pose = msg.pose.pose
        self.amcl_robot_pose = (
            float(pose.position.x),
            float(pose.position.y),
            self.yaw_from_quaternion(
                float(pose.orientation.x),
                float(pose.orientation.y),
                float(pose.orientation.z),
                float(pose.orientation.w),
            ),
        )

    def zone_callback(self, msg: String) -> None:
        self.current_patrol_zone = msg.data.strip() or 'UNKNOWN'

    def model_states_callback(self, msg: ModelStates) -> None:
        indices = {name: index for index, name in enumerate(msg.name)}
        robot_index = indices.get(self.robot_model_name)
        if robot_index is not None and robot_index < len(msg.pose):
            pose = msg.pose[robot_index]
            self.sim_robot_pose = (
                float(pose.position.x),
                float(pose.position.y),
                self.yaw_from_quaternion(
                    float(pose.orientation.x),
                    float(pose.orientation.y),
                    float(pose.orientation.z),
                    float(pose.orientation.w),
                ),
            )

        for spec in OBJECTS:
            index = indices.get(spec.gazebo_model_name)
            if index is None or index >= len(msg.pose):
                continue
            pose = msg.pose[index]
            new_position = (float(pose.position.x), float(pose.position.y))
            old_position = self.object_positions[spec.marker_name]
            self.object_positions[spec.marker_name] = new_position
            self.dynamic_position_available[spec.marker_name] = True
            if math.hypot(new_position[0] - old_position[0], new_position[1] - old_position[1]) > 0.20:
                self.get_logger().info(
                    f'{spec.object_class} moved to '
                    f'({new_position[0]:.2f}, {new_position[1]:.2f}); '
                    'future detections will use the new position.'
                )
        self.model_states_received = True
        self.last_model_states_wall_time = time.monotonic()

    def robot_pose(self) -> tuple[float, float, float] | None:
        # Gazebo pose is preferred because the target positions also come from Gazebo.
        return self.sim_robot_pose or self.amcl_robot_pose

    def geometric_zone(self) -> str:
        pose = self.robot_pose()
        if pose is None:
            return self.current_patrol_zone
        x, y, _ = pose
        if 7.0 <= x <= 14.5 and -0.3 <= y <= 3.45:
            return 'Zone_E_Restricted_Inventory'
        if 6.4 <= x <= 14.6 and -10.6 <= y < -0.3:
            return 'Zone_F_Packing'
        if 0.0 <= x <= 14.6 and 3.45 < y <= 10.7:
            return 'Zone_D_East_Storage'
        if -14.6 <= x < 0.0 and 3.45 < y <= 10.7:
            return 'Zone_C_West_Storage'
        if -14.6 <= x <= -7.2 and -10.6 <= y <= 3.45:
            return 'Zone_A_Receiving'
        if -7.2 < x < 7.0 and -10.6 <= y <= 3.45:
            return 'Zone_B_Central_Dispatch'
        return self.current_patrol_zone

    def expected_geometry(self, spec: ObjectSpec) -> tuple[float, float, float, float] | None:
        pose = self.robot_pose()
        if pose is None:
            return None
        robot_x, robot_y, robot_yaw = pose
        object_x, object_y = self.object_positions[spec.marker_name]
        dx = object_x - robot_x
        dy = object_y - robot_y
        distance = math.hypot(dx, dy)
        relative_bearing = self.normalize_angle(math.atan2(dy, dx) - robot_yaw)
        return distance, relative_bearing, object_x, object_y

    def lidar_distance_for_bearing(self, target_angle: float, expected_distance: float) -> float:
        scan = self.latest_scan
        if scan is None or not scan.ranges:
            return -1.0
        candidates: list[float] = []
        for index, raw_distance in enumerate(scan.ranges):
            distance = float(raw_distance)
            if not math.isfinite(distance):
                continue
            if distance < max(float(scan.range_min), 0.02):
                continue
            if scan.range_max > 0.0 and distance > float(scan.range_max):
                continue
            angle = float(scan.angle_min) + index * float(scan.angle_increment)
            if abs(self.normalize_angle(angle - target_angle)) <= self.lidar_half_sector:
                candidates.append(distance)
        if not candidates:
            return -1.0
        return round(float(min(candidates, key=lambda value: abs(value - expected_distance))), 2)

    def should_alert(self, spec: ObjectSpec, zone: str, distance: float) -> bool:
        if distance <= 0.0 or distance > self.maximum_alert_distance:
            return False
        if spec.object_class == 'person':
            return zone == 'Zone_E_Restricted_Inventory'
        # Unattended props remain suspicious in any warehouse zone after being moved.
        return True

    @staticmethod
    def publish_json(publisher: Any, payload: dict[str, Any]) -> None:
        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        publisher.publish(message)

    def save_snapshot(self, image: np.ndarray, payload: dict[str, Any]) -> str | None:
        if not self.save_snapshots:
            return None
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        path = self.snapshot_directory / f'{stamp}_{payload["reason"]}_{payload["object_class"]}.jpg'
        if cv2.imwrite(str(path), image):
            return str(path)
        return None

    def candidate_for_spec(
        self,
        spec: ObjectSpec,
        hsv: np.ndarray,
        image_width: int,
        image_height: int,
        frame_area: float,
    ) -> dict[str, Any] | None:
        geometry = self.expected_geometry(spec)
        if geometry is None:
            return None
        expected_distance, relative_bearing, object_x, object_y = geometry
        if expected_distance > self.maximum_alert_distance:
            return None
        if abs(relative_bearing) > self.camera_hfov / 2.0 + self.bearing_margin:
            return None

        expected_x = image_width / 2.0 - (relative_bearing / self.camera_hfov) * image_width
        pixel_tolerance = max(45.0, image_width * spec.pixel_tolerance_fraction)
        mask = cv2.inRange(
            hsv,
            np.array(spec.lower_hsv, dtype=np.uint8),
            np.array(spec.upper_hsv, dtype=np.uint8),
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best: dict[str, Any] | None = None
        best_score = float('inf')
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < spec.min_area or area > frame_area * spec.max_area_fraction:
                continue
            x, y, box_width, box_height = cv2.boundingRect(contour)
            if box_width < 7 or box_height < 5:
                continue
            aspect = box_height / max(1.0, float(box_width))
            if aspect < spec.min_aspect or aspect > spec.max_aspect:
                continue
            # A shoe may be partially cropped by the bottom of the image when close.
            if spec.object_class != 'shoe':
                if y + box_height >= int(image_height * 0.985) and box_height < 24:
                    continue
                if box_width > int(image_width * 0.65) and y > int(image_height * 0.55):
                    continue

            center_x = x + box_width / 2.0
            center_y = y + box_height / 2.0
            pixel_error = abs(center_x - expected_x)
            if pixel_error > pixel_tolerance:
                continue
            score = pixel_error / image_width - min(area / frame_area, 0.06)
            if score < best_score:
                best_score = score
                best = {
                    'x': x,
                    'y': y,
                    'width': box_width,
                    'height': box_height,
                    'center_x': center_x,
                    'center_y': center_y,
                    'area': area,
                    'aspect': aspect,
                    'expected_distance': expected_distance,
                    'relative_bearing': relative_bearing,
                    'expected_x': expected_x,
                    'pixel_error': pixel_error,
                    'object_x': object_x,
                    'object_y': object_y,
                }
        if best is None:
            return None

        lidar_distance = self.lidar_distance_for_bearing(relative_bearing, expected_distance)
        tolerance = max(self.lidar_tolerance_m, expected_distance * self.lidar_tolerance_fraction)
        if spec.lidar_required:
            if lidar_distance <= 0.0 or abs(lidar_distance - expected_distance) > tolerance:
                return None
            estimated_distance = lidar_distance
        else:
            estimated_distance = (
                lidar_distance
                if lidar_distance > 0.0 and abs(lidar_distance - expected_distance) <= tolerance
                else round(expected_distance, 2)
            )
        best['lidar_distance'] = lidar_distance
        best['estimated_distance'] = estimated_distance
        return best

    def build_payload(self, spec: ObjectSpec, candidate: dict[str, Any], zone: str) -> dict[str, Any]:
        confidence = 0.82
        confidence += min(0.09, candidate['area'] / 10000.0)
        confidence += max(0.0, 0.07 * (1.0 - candidate['pixel_error'] / max(45.0, 640.0 * spec.pixel_tolerance_fraction)))
        confidence = round(min(0.98, confidence), 3)
        alert = self.should_alert(spec, zone, candidate['estimated_distance'])
        x = int(candidate['x'])
        y = int(candidate['y'])
        width = int(candidate['width'])
        height = int(candidate['height'])
        robot_pose = self.robot_pose()
        return {
            'event_type': 'SECURITY_ALERT' if alert else 'DETECTION',
            'reason': spec.reason if alert else 'SIMULATION_TARGET_DETECTED',
            'severity': spec.severity if alert else 'INFO',
            'object_class': spec.object_class,
            'marker_name': spec.marker_name,
            'gazebo_model_name': spec.gazebo_model_name,
            'confidence': confidence,
            'bounding_box': [x, y, x + width, y + height],
            'bounding_box_center': [round(float(candidate['center_x']), 1), round(float(candidate['center_y']), 1)],
            'estimated_distance_m': round(float(candidate['estimated_distance']), 2),
            'expected_map_distance_m': round(float(candidate['expected_distance']), 2),
            'lidar_distance_m': round(float(candidate['lidar_distance']), 2),
            'relative_bearing_deg': round(math.degrees(float(candidate['relative_bearing'])), 1),
            'bearing_pixel_error': round(float(candidate['pixel_error']), 1),
            'patrol_zone': zone,
            'restricted_zone': zone == 'Zone_E_Restricted_Inventory',
            'robot_map_x': round(robot_pose[0], 2) if robot_pose else None,
            'robot_map_y': round(robot_pose[1], 2) if robot_pose else None,
            'robot_yaw_rad': round(robot_pose[2], 3) if robot_pose else None,
            'object_map_x': round(float(candidate['object_x']), 2),
            'object_map_y': round(float(candidate['object_y']), 2),
            'object_position_source': 'gazebo_model_states' if self.dynamic_position_available[spec.marker_name] else 'fallback_static',
            'robot_position_source': 'gazebo_model_states' if self.sim_robot_pose else 'amcl_pose',
            'source': 'GAZEBO_CAMERA_DYNAMIC_MODEL_LIDAR_FUSION',
            'alert': alert,
            'visible': True,
        }

    def image_callback(self, msg: Image) -> None:
        self.image_received = True
        self.frame_count += 1
        if self.frame_count % self.process_every_n_frames != 0:
            return
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'cv_bridge conversion failed: {exc}')
            return

        image_height, image_width = image.shape[:2]
        frame_area = float(image_height * image_width)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        zone = self.geometric_zone()
        now_ns = self.get_clock().now().nanoseconds
        visible_payloads: list[dict[str, Any]] = []

        for spec in OBJECTS:
            key = spec.marker_name
            candidate = self.candidate_for_spec(spec, hsv, image_width, image_height, frame_area)
            if candidate is not None:
                self.hit_streak[key] += 1
                self.miss_streak[key] = 0
                payload = self.build_payload(spec, candidate, zone)
                self.last_payload[key] = payload
                if not self.confirmed_visible[key] and self.hit_streak[key] >= self.confirmation_frames:
                    self.confirmed_visible[key] = True
                    event = dict(payload)
                    event['event_type'] = 'OBJECT_BECAME_VISIBLE'
                    self.publish_json(self.event_publisher, event)
                    previous_alert = self.last_alert_ns.get(key, -10**30)
                    if payload['alert'] and now_ns - previous_alert >= self.realert_cooldown_ns:
                        snapshot = self.save_snapshot(image, payload)
                        alert_payload = dict(payload)
                        if snapshot is not None:
                            alert_payload['snapshot_path'] = snapshot
                        self.publish_json(self.alert_publisher, alert_payload)
                        self.last_alert_ns[key] = now_ns
                        self.alerts_published += 1
                        self.get_logger().warning(
                            f'{spec.reason}: {spec.object_class} became visible at dynamic '
                            f'position ({payload["object_map_x"]}, {payload["object_map_y"]}) | '
                            f'zone={zone} | distance={payload["estimated_distance_m"]}m'
                        )
                if self.confirmed_visible[key]:
                    visible_payloads.append(payload)
                    previous_detection = self.last_detection_ns.get(key, -10**30)
                    if now_ns - previous_detection >= self.detection_publish_period_ns:
                        self.publish_json(self.detection_publisher, payload)
                        self.last_detection_ns[key] = now_ns
                        self.detections_published += 1
                    x1, y1, x2, y2 = payload['bounding_box']
                    colour = (0, 0, 255) if payload['alert'] else (0, 255, 255)
                    cv2.rectangle(image, (x1, y1), (x2, y2), colour, 2)
                    cv2.putText(
                        image,
                        f'{spec.object_class} {payload["confidence"]:.2f} {payload["estimated_distance_m"]:.1f}m',
                        (x1, max(20, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.52,
                        colour,
                        2,
                        cv2.LINE_AA,
                    )
            else:
                self.hit_streak[key] = 0
                self.miss_streak[key] += 1
                if self.confirmed_visible[key] and self.miss_streak[key] >= self.clear_frames:
                    self.confirmed_visible[key] = False
                    cleared = dict(self.last_payload.get(key, {}))
                    cleared.update({'event_type': 'OBJECT_NO_LONGER_VISIBLE', 'visible': False})
                    self.publish_json(self.event_publisher, cleared)
                    self.get_logger().info(f'{spec.object_class} is no longer visible; Gazebo marker will clear.')

        self.visibility_sequence += 1
        self.publish_json(self.visibility_publisher, {
            'sequence': self.visibility_sequence,
            'visible_count': len(visible_payloads),
            'visible_objects': visible_payloads,
            'model_states_received': self.model_states_received,
            'model_states_age_wall_seconds': round(time.monotonic() - self.last_model_states_wall_time, 3) if self.model_states_received else None,
            'source': 'GAZEBO_CAMERA_DYNAMIC_VISIBILITY',
        })
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            annotated_msg.header = msg.header
            self.annotated_publisher.publish(annotated_msg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Annotated image publication failed: {exc}')

    def publish_status(self) -> None:
        visible = [spec.object_class for spec in OBJECTS if self.confirmed_visible[spec.marker_name]]
        self.publish_json(self.status_publisher, {
            'camera_received': self.image_received,
            'scan_received': self.latest_scan is not None,
            'model_states_received': self.model_states_received,
            'robot_pose_source': 'gazebo_model_states' if self.sim_robot_pose else 'amcl_pose' if self.amcl_robot_pose else 'none',
            'currently_visible': visible,
            'alerts_published': self.alerts_published,
            'detections_published': self.detections_published,
            'dynamic_positions': {
                spec.object_class: {
                    'x': round(self.object_positions[spec.marker_name][0], 2),
                    'y': round(self.object_positions[spec.marker_name][1], 2),
                    'live': self.dynamic_position_available[spec.marker_name],
                }
                for spec in OBJECTS
            },
        })


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SimAnomalyDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
