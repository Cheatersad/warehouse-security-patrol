#!/usr/bin/env python3

import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


class OdomTfGuard(Node):
    """Publish odom -> robot-base TF only when another publisher is absent."""

    def __init__(self):
        super().__init__('odom_tf_guard')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'base_footprint')
        self.declare_parameter('detection_seconds', 3.0)

        odom_topic = str(self.get_parameter('odom_topic').value)
        self.default_parent = str(
            self.get_parameter('parent_frame').value
        ).lstrip('/')
        self.default_child = str(
            self.get_parameter('child_frame').value
        ).lstrip('/')
        self.detection_seconds = float(
            self.get_parameter('detection_seconds').value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            spin_thread=False,
        )
        self.tf_broadcaster = TransformBroadcaster(self)

        self.start_wall_time = time.monotonic()
        self.mode = 'detecting'
        self.received_odom = False

        self.subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            50,
        )

        self.status_timer = self.create_timer(
            2.0,
            self.status_callback,
        )

        self.get_logger().info(
            f'Watching {odom_topic} for the '
            f'{self.default_parent} -> {self.default_child} transform.'
        )

    def existing_transform_available(self, parent, child):
        try:
            return self.tf_buffer.can_transform(
                parent,
                child,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except Exception:
            return False

    def odom_callback(self, msg):
        self.received_odom = True

        parent = (
            msg.header.frame_id or self.default_parent
        ).lstrip('/')

        child = (
            msg.child_frame_id or self.default_child
        ).lstrip('/')

        elapsed = time.monotonic() - self.start_wall_time

        if self.mode == 'detecting':
            if self.existing_transform_available(parent, child):
                self.mode = 'passive'
                self.get_logger().info(
                    f'Existing {parent} -> {child} TF detected. '
                    'Guard will not publish a duplicate transform.'
                )
                return

            if elapsed < self.detection_seconds:
                return

            self.mode = 'active'
            self.get_logger().warning(
                f'No {parent} -> {child} TF detected. '
                'Publishing it from /odom.'
            )

        if self.mode != 'active':
            return

        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = parent
        transform.child_frame_id = child

        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(transform)

    def status_callback(self):
        if not self.received_odom:
            self.get_logger().warning(
                'No /odom messages received. Check the Gazebo '
                'differential-drive plugin.'
            )


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfGuard()

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
