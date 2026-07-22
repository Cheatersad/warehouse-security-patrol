#!/usr/bin/env python3
"""Move a Gazebo anomaly safely in front of TurtleBot3 without using the GUI."""
from __future__ import annotations

import argparse
import math
import sys

from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import GetEntityState, SetEntityState
import rclpy
from rclpy.node import Node


MODELS = {
    'shoe': 'sim_anomaly_shoe',
    'backpack': 'sim_anomaly_backpack',
    'suitcase': 'sim_anomaly_suitcase',
    'person': 'sim_anomaly_person_marker',
    'box': 'sim_anomaly_aisle_obstruction',
    'spill': 'sim_anomaly_hazardous_spill',
}


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class Mover(Node):
    def __init__(self) -> None:
        super().__init__('place_anomaly_in_front')
        self.get_client = self.create_client(GetEntityState, '/get_entity_state')
        self.set_client = self.create_client(SetEntityState, '/set_entity_state')

    def get_state(self, name: str):
        if not self.get_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('/get_entity_state is unavailable')
        request = GetEntityState.Request()
        request.name = name
        request.reference_frame = 'world'
        future = self.get_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError(f'Could not read Gazebo entity {name!r}')
        return response.state

    def set_state(self, state: EntityState) -> None:
        if not self.set_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('/set_entity_state is unavailable')
        request = SetEntityState.Request()
        request.state = state
        future = self.set_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        response = future.result()
        if response is None or not response.success:
            message = response.status_message if response is not None else 'no response'
            raise RuntimeError(f'Gazebo rejected move: {message}')


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('object', choices=sorted(MODELS))
    parser.add_argument('--distance', type=float, default=1.25)
    parser.add_argument('--lateral', type=float, default=0.0)
    args = parser.parse_args()

    rclpy.init()
    node = Mover()
    try:
        robot = node.get_state('waffle_pi')
        target = node.get_state(MODELS[args.object])
        yaw = yaw_from_quaternion(robot.pose.orientation)
        forward_x = math.cos(yaw)
        forward_y = math.sin(yaw)
        left_x = -math.sin(yaw)
        left_y = math.cos(yaw)
        target.name = MODELS[args.object]
        target.reference_frame = 'world'
        target.pose.position.x = robot.pose.position.x + args.distance * forward_x + args.lateral * left_x
        target.pose.position.y = robot.pose.position.y + args.distance * forward_y + args.lateral * left_y
        node.set_state(target)
        print(
            f'Moved {args.object} to ({target.pose.position.x:.2f}, '
            f'{target.pose.position.y:.2f}), {args.distance:.2f} m in front of the robot.'
        )
        return 0
    except Exception as exc:  # noqa: BLE001
        print(f'ERROR: {exc}', file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
