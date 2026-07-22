#!/usr/bin/env python3
"""Optional deterministic alert source for testing the logger and UI."""
from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DemoAnomalyPublisher(Node):
    def __init__(self) -> None:
        super().__init__('demo_anomaly_publisher')
        self.declare_parameter('trigger_zone', 'Zone_E_Restricted_Inventory')
        self.declare_parameter('delay_seconds', 4.0)
        self.declare_parameter('repeat_seconds', 30.0)

        self.trigger_zone = str(self.get_parameter('trigger_zone').value)
        self.delay_seconds = float(self.get_parameter('delay_seconds').value)
        self.repeat_seconds = float(self.get_parameter('repeat_seconds').value)
        self.current_zone = 'UNKNOWN'
        self.zone_enter_ns: int | None = None
        self.last_publish_ns = -10**30

        self.publisher = self.create_publisher(String, '/security_alert', 10)
        self.create_subscription(String, '/patrol/current_zone', self.zone_callback, 10)
        self.create_timer(1.0, self.timer_callback)
        self.get_logger().warning(
            'Deterministic demo alerts are ENABLED. These are test events, not YOLO detections.'
        )

    def zone_callback(self, msg: String) -> None:
        new_zone = msg.data.strip() or 'UNKNOWN'
        if new_zone != self.current_zone:
            self.current_zone = new_zone
            self.zone_enter_ns = self.get_clock().now().nanoseconds

    def timer_callback(self) -> None:
        if self.current_zone != self.trigger_zone or self.zone_enter_ns is None:
            return
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.zone_enter_ns < int(self.delay_seconds * 1e9):
            return
        if now_ns - self.last_publish_ns < int(self.repeat_seconds * 1e9):
            return

        payload = {
            'event_type': 'SECURITY_ALERT',
            'reason': 'DEMO_PERSON_IN_RESTRICTED_ZONE',
            'object_class': 'person',
            'confidence': 1.0,
            'estimated_distance_m': 1.5,
            'patrol_zone': self.current_zone,
            'restricted_zone': True,
            'robot_map_x': None,
            'robot_map_y': None,
            'source': 'DETERMINISTIC_DEMO_PUBLISHER',
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.publisher.publish(msg)
        self.last_publish_ns = now_ns
        self.get_logger().warning('Published deterministic restricted-zone demo alert.')


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DemoAnomalyPublisher()
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
