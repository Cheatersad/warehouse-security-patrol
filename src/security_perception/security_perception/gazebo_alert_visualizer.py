#!/usr/bin/env python3
"""Display a Gazebo beacon only while an alerting object is camera-visible."""
from __future__ import annotations

import json
import math
from typing import Any

from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import DeleteEntity, SetEntityState, SpawnEntity
from geometry_msgs.msg import Pose
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class GazeboAlertVisualizer(Node):
    def __init__(self) -> None:
        super().__init__('gazebo_alert_visualizer')
        self.declare_parameter('visibility_topic', '/security/visibility')
        self.declare_parameter('visibility_timeout_seconds', 0.8)
        self.declare_parameter('reference_frame', 'world')
        self.declare_parameter('alert_topic', '/security_alert')
        self.declare_parameter('display_seconds', 10.0)

        visibility_topic = str(self.get_parameter('visibility_topic').value)
        self.timeout_ns = int(max(0.2, float(self.get_parameter('visibility_timeout_seconds').value)) * 1e9)
        self.reference_frame = str(self.get_parameter('reference_frame').value)

        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        self.set_state_client = self.create_client(SetEntityState, '/set_entity_state')
        self.status_publisher = self.create_publisher(String, '/security/gazebo_visual_status', 10)
        self.create_subscription(String, visibility_topic, self.visibility_callback, 10)
        self.create_timer(0.20, self.expiry_callback)

        self.active_last_seen_ns: dict[str, int] = {}
        self.active_payload: dict[str, dict[str, Any]] = {}
        self.spawned_models: set[str] = set()
        self.pending_spawn: set[str] = set()
        self.pending_delete: set[str] = set()
        self.last_visual_position: dict[str, tuple[float, float]] = {}
        self.get_logger().info(
            f'Gazebo alert visuals follow live camera visibility on {visibility_topic} '
            'and track moved objects.'
        )

    @staticmethod
    def safe_key(value: str) -> str:
        cleaned = ''.join(ch if ch.isalnum() else '_' for ch in value.lower())
        return cleaned.strip('_') or 'unknown'

    @staticmethod
    def colours_for_severity(severity: str) -> tuple[str, str, str]:
        severity = severity.upper()
        if severity == 'CRITICAL':
            return ('1 0 0.65 0.78', '1 0 0.65 1', '0.60 0 0.32 1')
        if severity == 'HIGH':
            return ('1 0 0 0.78', '1 0.05 0.02 1', '0.60 0 0 1')
        return ('1 0.42 0 0.78', '1 0.42 0 1', '0.60 0.18 0 1')

    def beacon_sdf(self, model_name: str, severity: str) -> str:
        translucent, bright, glow = self.colours_for_severity(severity)
        return f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{model_name}'>
    <static>true</static>
    <link name='alert_beacon'>
      <visual name='ground_halo'>
        <pose>0 0 0.035 0 0 0</pose>
        <geometry><cylinder><radius>0.55</radius><length>0.07</length></cylinder></geometry>
        <material><ambient>{translucent}</ambient><diffuse>{translucent}</diffuse><emissive>{glow}</emissive></material>
        <transparency>0.24</transparency><cast_shadows>false</cast_shadows>
      </visual>
      <visual name='vertical_beam'>
        <pose>0 0 1.15 0 0 0</pose>
        <geometry><cylinder><radius>0.075</radius><length>2.20</length></cylinder></geometry>
        <material><ambient>{translucent}</ambient><diffuse>{translucent}</diffuse><emissive>{glow}</emissive></material>
        <transparency>0.18</transparency><cast_shadows>false</cast_shadows>
      </visual>
      <visual name='warning_orb'>
        <pose>0 0 2.34 0 0 0</pose>
        <geometry><sphere><radius>0.22</radius></sphere></geometry>
        <material><ambient>{bright}</ambient><diffuse>{bright}</diffuse><emissive>{bright}</emissive></material>
        <cast_shadows>false</cast_shadows>
      </visual>
    </link>
  </model>
</sdf>"""

    def publish_status(self, payload: dict[str, Any]) -> None:
        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        self.status_publisher.publish(message)

    def visibility_callback(self, msg: String) -> None:
        try:
            frame = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        objects = frame.get('visible_objects', [])
        if not isinstance(objects, list):
            return
        now_ns = self.get_clock().now().nanoseconds
        for payload in objects:
            if not isinstance(payload, dict) or not bool(payload.get('alert', False)):
                continue
            object_class = str(payload.get('object_class', '')).lower().strip()
            if not object_class:
                continue
            try:
                x = float(payload['object_map_x'])
                y = float(payload['object_map_y'])
            except (KeyError, TypeError, ValueError):
                continue
            model_name = f'security_alert_visual_{self.safe_key(object_class)}'
            self.active_last_seen_ns[model_name] = now_ns
            self.active_payload[model_name] = dict(payload)
            self.last_visual_position[model_name] = (x, y)
            if model_name in self.spawned_models:
                self.move_visual(model_name, x, y)
            elif model_name not in self.pending_spawn and model_name not in self.pending_delete:
                self.spawn_visual(model_name, object_class, x, y, payload)

    def spawn_visual(self, model_name: str, object_class: str, x: float, y: float, payload: dict[str, Any]) -> None:
        if not self.spawn_client.wait_for_service(timeout_sec=0.05):
            return
        request = SpawnEntity.Request()
        request.name = model_name
        request.xml = self.beacon_sdf(model_name, str(payload.get('severity', 'HIGH')))
        request.robot_namespace = ''
        request.reference_frame = self.reference_frame
        request.initial_pose = Pose()
        request.initial_pose.position.x = x
        request.initial_pose.position.y = y
        request.initial_pose.orientation.w = 1.0
        self.pending_spawn.add(model_name)
        future = self.spawn_client.call_async(request)
        future.add_done_callback(
            lambda completed, name=model_name, obj=object_class, px=x, py=y, data=dict(payload):
            self.spawn_done(completed, name, obj, px, py, data)
        )

    def spawn_done(self, future: Any, model_name: str, object_class: str, x: float, y: float, payload: dict[str, Any]) -> None:
        self.pending_spawn.discard(model_name)
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Gazebo beacon spawn failed: {exc}')
            return
        if response.success or 'already exists' in response.status_message.lower():
            self.spawned_models.add(model_name)
            self.move_visual(model_name, x, y)
            self.publish_status({
                'state': 'VISIBLE',
                'model': model_name,
                'object_class': object_class,
                'reason': payload.get('reason'),
                'severity': payload.get('severity'),
                'x': x,
                'y': y,
            })
        else:
            self.get_logger().error(f'Gazebo rejected alert beacon: {response.status_message}')

    def move_visual(self, model_name: str, x: float, y: float) -> None:
        if not self.set_state_client.wait_for_service(timeout_sec=0.01):
            return
        state = EntityState()
        state.name = model_name
        state.reference_frame = self.reference_frame
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = 0.0
        state.pose.orientation.w = 1.0
        request = SetEntityState.Request()
        request.state = state
        self.set_state_client.call_async(request)

    def expiry_callback(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        expired = [
            name for name in list(self.spawned_models)
            if now_ns - self.active_last_seen_ns.get(name, -10**30) >= self.timeout_ns
            and name not in self.pending_delete
        ]
        for model_name in expired:
            if not self.delete_client.wait_for_service(timeout_sec=0.02):
                continue
            request = DeleteEntity.Request()
            request.name = model_name
            self.pending_delete.add(model_name)
            future = self.delete_client.call_async(request)
            future.add_done_callback(lambda completed, name=model_name: self.delete_done(completed, name))

    def delete_done(self, future: Any, model_name: str) -> None:
        self.pending_delete.discard(model_name)
        payload = self.active_payload.pop(model_name, {})
        self.active_last_seen_ns.pop(model_name, None)
        self.last_visual_position.pop(model_name, None)
        self.spawned_models.discard(model_name)
        try:
            response = future.result()
            if not response.success and 'does not exist' not in response.status_message.lower():
                self.get_logger().warning(f'Could not remove {model_name}: {response.status_message}')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'Gazebo beacon deletion failed: {exc}')
        self.publish_status({
            'state': 'HIDDEN',
            'model': model_name,
            'object_class': payload.get('object_class'),
            'reason': payload.get('reason'),
        })


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GazeboAlertVisualizer()
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
