#!/usr/bin/env python3
"""YOLOv8 camera detection with LiDAR, patrol-zone and AMCL fusion."""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
from typing import Any

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String


class YoloDetectionNode(Node):
    """Detect people and unattended items and publish structured alerts."""

    def __init__(self) -> None:
        super().__init__('yolo_detection_node')

        self.declare_parameter('model_path', '/ws_slam/yolov8n.pt')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('confidence_threshold', 0.40)
        self.declare_parameter('target_classes', ['person', 'backpack', 'suitcase'])
        self.declare_parameter(
            'restricted_zones', ['Zone_E_Restricted_Inventory']
        )
        self.declare_parameter('camera_horizontal_fov', 1.085595)
        self.declare_parameter('lidar_sector_degrees', 8.0)
        self.declare_parameter('alert_cooldown_seconds', 5.0)
        self.declare_parameter('save_snapshots', True)
        self.declare_parameter('snapshot_directory', '/ws_slam/security_events')

        self.bridge = CvBridge()
        self.conf_threshold = float(
            self.get_parameter('confidence_threshold').value
        )
        self.target_classes = set(
            str(v) for v in self.get_parameter('target_classes').value
        )
        self.restricted_zones = set(
            str(v) for v in self.get_parameter('restricted_zones').value
        )
        self.camera_hfov = float(
            self.get_parameter('camera_horizontal_fov').value
        )
        self.lidar_half_sector = math.radians(
            float(self.get_parameter('lidar_sector_degrees').value) / 2.0
        )
        self.alert_cooldown_ns = int(
            float(self.get_parameter('alert_cooldown_seconds').value) * 1e9
        )
        self.save_snapshots = bool(self.get_parameter('save_snapshots').value)
        self.snapshot_directory = Path(
            str(self.get_parameter('snapshot_directory').value)
        )
        if self.save_snapshots:
            self.snapshot_directory.mkdir(parents=True, exist_ok=True)

        model_path = self._resolve_model_path(
            str(self.get_parameter('model_path').value)
        )
        self.get_logger().info(f'Loading YOLOv8 model: {model_path}')
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                'Python package "ultralytics" is missing. Install it inside '
                'the Docker container with: pip3 install ultralytics'
            ) from exc
        self.model = YOLO(str(model_path))

        self.latest_scan: LaserScan | None = None
        self.current_zone = 'UNKNOWN'
        self.robot_x: float | None = None
        self.robot_y: float | None = None
        self.last_alert_ns: dict[str, int] = {}

        image_topic = str(self.get_parameter('image_topic').value)
        scan_topic = str(self.get_parameter('scan_topic').value)
        self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            String, '/patrol/current_zone', self.zone_callback, 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10
        )

        self.alert_publisher = self.create_publisher(String, '/security_alert', 10)
        self.detection_publisher = self.create_publisher(
            String, '/security/detections', 10
        )
        self.annotated_publisher = self.create_publisher(
            Image, '/security/annotated_image', 10
        )

        self.get_logger().info(
            f'Perception ready: camera={image_topic}, lidar={scan_topic}, '
            f'targets={sorted(self.target_classes)}'
        )

    @staticmethod
    def _resolve_model_path(requested: str) -> Path:
        candidates = [
            Path(os.path.expandvars(os.path.expanduser(requested))),
            Path('/ws_slam/yolov8n.pt'),
            Path.cwd() / 'yolov8n.pt',
        ]
        for candidate in candidates:
            if candidate.is_file():
                return candidate.resolve()
        raise FileNotFoundError(
            'YOLO model not found. Checked: '
            + ', '.join(str(c) for c in candidates)
        )

    def scan_callback(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def zone_callback(self, msg: String) -> None:
        self.current_zone = msg.data.strip() or 'UNKNOWN'

    def pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        self.robot_x = float(msg.pose.pose.position.x)
        self.robot_y = float(msg.pose.pose.position.y)

    @staticmethod
    def _angular_difference(a: float, b: float) -> float:
        return math.atan2(math.sin(a - b), math.cos(a - b))

    def _distance_for_image_x(self, object_x: float, image_width: int) -> float:
        scan = self.latest_scan
        if scan is None or not scan.ranges or image_width <= 0:
            return -1.0

        # Pixel-left corresponds to positive yaw in ROS LaserScan coordinates.
        normalized_x = (object_x - image_width / 2.0) / image_width
        target_angle = -normalized_x * self.camera_hfov

        valid: list[float] = []
        for index, value in enumerate(scan.ranges):
            distance = float(value)
            if not math.isfinite(distance):
                continue
            if distance < max(0.0, float(scan.range_min)):
                continue
            if scan.range_max > 0.0 and distance > float(scan.range_max):
                continue
            angle = float(scan.angle_min) + index * float(scan.angle_increment)
            if abs(self._angular_difference(angle, target_angle)) <= self.lidar_half_sector:
                valid.append(distance)
        return round(min(valid), 2) if valid else -1.0

    def _alert_reason(self, class_name: str) -> tuple[bool, str]:
        if class_name == 'person' and self.current_zone in self.restricted_zones:
            return True, 'PERSON_IN_RESTRICTED_ZONE'
        if class_name in {'backpack', 'suitcase'}:
            return True, 'UNATTENDED_ITEM'
        return False, 'TARGET_DETECTED'

    def _cooldown_allows(self, key: str) -> bool:
        now_ns = self.get_clock().now().nanoseconds
        previous = self.last_alert_ns.get(key, -10**30)
        if now_ns - previous < self.alert_cooldown_ns:
            return False
        self.last_alert_ns[key] = now_ns
        return True

    def image_callback(self, msg: Image) -> None:
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'cv_bridge conversion failed: {exc}')
            return

        try:
            result = self.model(image, verbose=False)[0]
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'YOLO inference failed: {exc}')
            return

        height, width = image.shape[:2]
        target_count = 0
        alert_snapshots: list[tuple[str, dict[str, Any]]] = []

        for box in result.boxes:
            class_id = int(box.cls[0])
            class_name = str(self.model.names[class_id])
            confidence = float(box.conf[0])
            if confidence < self.conf_threshold or class_name not in self.target_classes:
                continue

            target_count += 1
            x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
            x1 = max(0, min(width - 1, x1))
            x2 = max(0, min(width - 1, x2))
            y1 = max(0, min(height - 1, y1))
            y2 = max(0, min(height - 1, y2))
            center_x = (x1 + x2) / 2.0
            center_y = (y1 + y2) / 2.0
            distance = self._distance_for_image_x(center_x, width)

            if center_x < 0.4 * width:
                horizontal_position = 'left'
            elif center_x > 0.6 * width:
                horizontal_position = 'right'
            else:
                horizontal_position = 'center'

            is_alert, reason = self._alert_reason(class_name)
            payload: dict[str, Any] = {
                'event_type': 'SECURITY_ALERT' if is_alert else 'DETECTION',
                'reason': reason,
                'object_class': class_name,
                'confidence': round(confidence, 3),
                'bounding_box': [x1, y1, x2, y2],
                'bounding_box_center': [round(center_x, 1), round(center_y, 1)],
                'object_position': horizontal_position,
                'estimated_distance_m': distance,
                'patrol_zone': self.current_zone,
                'restricted_zone': self.current_zone in self.restricted_zones,
                'robot_map_x': round(self.robot_x, 2) if self.robot_x is not None else None,
                'robot_map_y': round(self.robot_y, 2) if self.robot_y is not None else None,
                'source': 'YOLOV8_LIDAR_FUSION',
            }

            detection_msg = String()
            detection_msg.data = json.dumps(payload)
            self.detection_publisher.publish(detection_msg)

            label = f'{class_name} {confidence:.2f}'
            if distance > 0.0:
                label += f' {distance:.1f}m'
            if is_alert:
                label += ' ALERT'
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 0, 255) if is_alert else (0, 255, 255), 2)
            cv2.putText(
                image,
                label,
                (x1, max(18, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                (0, 0, 255) if is_alert else (0, 255, 255),
                2,
                cv2.LINE_AA,
            )

            if is_alert:
                key = f'{reason}:{class_name}:{self.current_zone}'
                if self._cooldown_allows(key):
                    alert_msg = String()
                    alert_msg.data = json.dumps(payload)
                    self.alert_publisher.publish(alert_msg)
                    alert_snapshots.append((key, payload))
                    self.get_logger().warning(
                        f'ALERT {reason}: {class_name} in {self.current_zone}, '
                        f'distance={distance}m, confidence={confidence:.2f}'
                    )

        cv2.putText(
            image,
            f'Zone: {self.current_zone}',
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.68,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        if target_count == 0:
            cv2.putText(
                image,
                'Patrolling - no target anomaly',
                (12, height - 18),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

        annotated_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        annotated_msg.header = msg.header
        self.annotated_publisher.publish(annotated_msg)

        if self.save_snapshots and alert_snapshots:
            stamp_ns = self.get_clock().now().nanoseconds
            for index, (_, payload) in enumerate(alert_snapshots):
                safe_zone = str(payload['patrol_zone']).replace('/', '_')
                filename = (
                    f'{stamp_ns}_{index}_{safe_zone}_'
                    f'{payload["object_class"]}.jpg'
                )
                try:
                    cv2.imwrite(str(self.snapshot_directory / filename), image)
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().error(f'Failed to save alert snapshot: {exc}')


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: YoloDetectionNode | None = None
    try:
        node = YoloDetectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
