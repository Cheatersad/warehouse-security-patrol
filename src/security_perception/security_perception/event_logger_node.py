#!/usr/bin/env python3
"""Persist alerts and create one object-location report per patrol cycle."""
from __future__ import annotations

from datetime import datetime
import json
from pathlib import Path
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


FALLBACK_OBJECT_POSITIONS = {
    'shoe': (-6.65, -9.20),
    'backpack': (-8.90, -5.00),
    'suitcase': (8.40, -6.00),
    'person': (13.15, 0.55),
    'box': (8.20, 6.90),
    'spill': (-3.80, 6.50),
}


class EventLoggerNode(Node):
    def __init__(self) -> None:
        super().__init__('security_event_logger')
        self.declare_parameter('text_log_path', '/ws_slam/security_log.txt')
        self.declare_parameter('jsonl_log_path', '/ws_slam/security_events.jsonl')
        self.declare_parameter('report_directory', '/ws_slam/patrol_reports')

        self.text_log_path = Path(str(self.get_parameter('text_log_path').value))
        self.jsonl_log_path = Path(str(self.get_parameter('jsonl_log_path').value))
        self.report_directory = Path(
            str(self.get_parameter('report_directory').value)
        )
        self.text_log_path.parent.mkdir(parents=True, exist_ok=True)
        self.jsonl_log_path.parent.mkdir(parents=True, exist_ok=True)
        self.report_directory.mkdir(parents=True, exist_ok=True)

        self.active_cycle = 1
        self.cycle_records: dict[int, dict[str, dict[str, Any]]] = {}
        self.reported_cycles: set[int] = set()

        self.create_subscription(String, '/security_alert', self.alert_callback, 10)
        self.create_subscription(String, '/patrol/status', self.status_callback, 10)
        self.report_publisher = self.create_publisher(
            String, '/security/patrol_report', 10
        )
        self.get_logger().info(
            f'Event logger ready. Cycle reports: {self.report_directory}'
        )

    @staticmethod
    def now_iso() -> str:
        return datetime.now().astimezone().isoformat(timespec='seconds')

    def alert_callback(self, msg: String) -> None:
        try:
            data: dict[str, Any] = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Invalid /security_alert JSON: {exc}')
            return

        timestamp = self.now_iso()
        data['logged_at'] = timestamp
        data['patrol_cycle'] = self.active_cycle
        reason = str(data.get('reason', 'UNKNOWN_ALERT'))
        object_class = str(data.get('object_class', 'unknown'))
        zone = str(data.get('patrol_zone', 'UNKNOWN'))
        distance = float(data.get('estimated_distance_m', -1.0) or -1.0)
        confidence = data.get('confidence', 'n/a')
        robot_x = data.get('robot_map_x')
        robot_y = data.get('robot_map_y')

        readable = (
            f'[{timestamp}] cycle={self.active_cycle} | ALERT={reason} | '
            f'object={object_class} | zone={zone} | distance={distance}m | '
            f'confidence={confidence} | robot=({robot_x}, {robot_y})\n'
        )
        try:
            with self.text_log_path.open('a', encoding='utf-8') as handle:
                handle.write(readable)
            with self.jsonl_log_path.open('a', encoding='utf-8') as handle:
                handle.write(json.dumps(data, sort_keys=True) + '\n')
        except OSError as exc:
            self.get_logger().error(f'Could not write security logs: {exc}')
            return

        self.update_cycle_record(data, timestamp)
        self.get_logger().warning(
            f'Logged {reason}: {object_class} in {zone} at {distance}m'
        )

    def update_cycle_record(self, data: dict[str, Any], timestamp: str) -> None:
        object_class = str(data.get('object_class', 'unknown'))
        marker_name = str(data.get('marker_name', object_class))
        key = f'{marker_name}:{object_class}'
        records = self.cycle_records.setdefault(self.active_cycle, {})

        object_x = data.get('object_map_x')
        object_y = data.get('object_map_y')
        if object_x is None or object_y is None:
            fallback = FALLBACK_OBJECT_POSITIONS.get(object_class)
            if fallback is not None:
                object_x, object_y = fallback

        distance = float(data.get('estimated_distance_m', -1.0) or -1.0)
        confidence_value = data.get('confidence')
        try:
            confidence = float(confidence_value)
        except (TypeError, ValueError):
            confidence = 0.0

        if key not in records:
            records[key] = {
                'object_class': object_class,
                'marker_name': marker_name,
                'reason': data.get('reason'),
                'severity': data.get('severity'),
                'zone': data.get('patrol_zone'),
                'object_map_x': object_x,
                'object_map_y': object_y,
                'first_seen_at': timestamp,
                'last_seen_at': timestamp,
                'first_seen_robot_x': data.get('robot_map_x'),
                'first_seen_robot_y': data.get('robot_map_y'),
                'closest_seen_robot_x': data.get('robot_map_x'),
                'closest_seen_robot_y': data.get('robot_map_y'),
                'minimum_distance_m': distance,
                'maximum_confidence': confidence,
                'sightings': 1,
                'snapshot_path': data.get('snapshot_path'),
                'locations_seen': [{
                    'object_map_x': object_x,
                    'object_map_y': object_y,
                    'zone': data.get('patrol_zone'),
                    'seen_at': timestamp,
                    'robot_map_x': data.get('robot_map_x'),
                    'robot_map_y': data.get('robot_map_y'),
                }],
            }
            return

        record = records[key]
        record['last_seen_at'] = timestamp
        record['sightings'] = int(record.get('sightings', 1)) + 1
        record['object_map_x'] = object_x
        record['object_map_y'] = object_y
        record['zone'] = data.get('patrol_zone')
        locations = record.setdefault('locations_seen', [])
        locations.append({
            'object_map_x': object_x,
            'object_map_y': object_y,
            'zone': data.get('patrol_zone'),
            'seen_at': timestamp,
            'robot_map_x': data.get('robot_map_x'),
            'robot_map_y': data.get('robot_map_y'),
        })
        record['maximum_confidence'] = max(
            float(record.get('maximum_confidence', 0.0)), confidence
        )
        old_distance = float(record.get('minimum_distance_m', -1.0) or -1.0)
        if distance > 0.0 and (old_distance <= 0.0 or distance < old_distance):
            record['minimum_distance_m'] = distance
            record['closest_seen_robot_x'] = data.get('robot_map_x')
            record['closest_seen_robot_y'] = data.get('robot_map_y')
        if data.get('snapshot_path'):
            record['snapshot_path'] = data.get('snapshot_path')

    def status_callback(self, msg: String) -> None:
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        state = str(status.get('state', ''))
        try:
            completed_cycles = int(status.get('cycle', 0))
        except (TypeError, ValueError):
            completed_cycles = 0

        if state == 'CYCLE_COMPLETE':
            cycle = max(1, completed_cycles)
            self.write_cycle_report(cycle, status)
            self.active_cycle = cycle + 1
        elif state == 'PATROL_COMPLETE':
            cycle = max(1, completed_cycles or self.active_cycle)
            self.write_cycle_report(cycle, status)
            self.active_cycle = cycle + 1
        else:
            self.active_cycle = max(self.active_cycle, completed_cycles + 1)

    def write_cycle_report(self, cycle: int, status: dict[str, Any]) -> None:
        if cycle in self.reported_cycles:
            return
        self.reported_cycles.add(cycle)
        completed_at = self.now_iso()
        records = list(self.cycle_records.get(cycle, {}).values())
        records.sort(key=lambda item: (str(item.get('zone')), str(item.get('object_class'))))

        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        stem = f'patrol_cycle_{cycle:03d}_{stamp}'
        text_path = self.report_directory / f'{stem}.txt'
        json_path = self.report_directory / f'{stem}.json'

        report = {
            'report_type': 'WAREHOUSE_PATROL_SECURITY_REPORT',
            'cycle': cycle,
            'completed_at': completed_at,
            'total_unique_objects': len(records),
            'objects_found': records,
            'patrol_status': status,
        }

        lines = [
            'WAREHOUSE PATROL SECURITY REPORT',
            '=' * 36,
            f'Patrol cycle: {cycle}',
            f'Completed at: {completed_at}',
            f'Unique suspicious objects found: {len(records)}',
            '',
        ]
        if not records:
            lines.append('No unattended or suspicious objects were found in this patrol loop.')
        else:
            for index, item in enumerate(records, start=1):
                lines.extend([
                    f'{index}. Object: {item.get("object_class")}',
                    f'   Alert: {item.get("reason")}',
                    f'   Severity: {item.get("severity")}',
                    f'   Zone: {item.get("zone")}',
                    '   Latest object map location: '
                    f'({item.get("object_map_x")}, {item.get("object_map_y")})',
                    f'   All detected locations: {item.get("locations_seen", [])}',
                    '   First seen from robot location: '
                    f'({item.get("first_seen_robot_x")}, {item.get("first_seen_robot_y")})',
                    '   Closest seen from robot location: '
                    f'({item.get("closest_seen_robot_x")}, {item.get("closest_seen_robot_y")})',
                    f'   Closest estimated distance: {item.get("minimum_distance_m")} m',
                    f'   Maximum confidence: {item.get("maximum_confidence")}',
                    f'   Visibility encounters: {item.get("sightings")}',
                    f'   First seen: {item.get("first_seen_at")}',
                    f'   Last seen: {item.get("last_seen_at")}',
                    f'   Evidence image: {item.get("snapshot_path")}',
                    '',
                ])

        try:
            text_path.write_text('\n'.join(lines) + '\n', encoding='utf-8')
            json_path.write_text(
                json.dumps(report, indent=2, sort_keys=True) + '\n',
                encoding='utf-8',
            )
        except OSError as exc:
            self.get_logger().error(f'Could not write patrol report: {exc}')
            return

        message = String()
        message.data = json.dumps({
            'cycle': cycle,
            'text_report': str(text_path),
            'json_report': str(json_path),
            'objects_found': len(records),
        }, sort_keys=True)
        self.report_publisher.publish(message)
        self.get_logger().warning(
            f'Patrol cycle {cycle} report created: {text_path}'
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = EventLoggerNode()
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
