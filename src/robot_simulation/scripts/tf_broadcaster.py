#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from nav_msgs.msg import Odometry

class TransformBroadcasterNode(Node):
    def __init__(self):
        super().__init__('transform_broadcaster_node')

        # Create a transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # Subscribe to odometry topic to update the transform
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',  # Adjust this if your odom topic has a different name
            self.handle_odom_message,
            10)

        # Publish static transforms for the wheels
        self.publish_static_transforms()

    def publish_static_transforms(self):
        # Left wheel static transform
        left_wheel_transform = TransformStamped()
        left_wheel_transform.header.stamp = self.get_clock().now().to_msg()
        left_wheel_transform.header.frame_id = 'base_link'
        left_wheel_transform.child_frame_id = 'left_wheel'
        left_wheel_transform.transform.translation.x = 0.0  # Replace with actual values
        left_wheel_transform.transform.translation.y = 0.2  # Replace with actual values
        left_wheel_transform.transform.translation.z = 0.0  # Replace with actual values
        left_wheel_transform.transform.rotation.x = 0.0
        left_wheel_transform.transform.rotation.y = 0.0
        left_wheel_transform.transform.rotation.z = 0.0
        left_wheel_transform.transform.rotation.w = 1.0

        # Right wheel static transform
        right_wheel_transform = TransformStamped()
        right_wheel_transform.header.stamp = self.get_clock().now().to_msg()
        right_wheel_transform.header.frame_id = 'base_link'
        right_wheel_transform.child_frame_id = 'right_wheel'
        right_wheel_transform.transform.translation.x = 0.0  # Replace with actual values
        right_wheel_transform.transform.translation.y = -0.2  # Replace with actual values
        right_wheel_transform.transform.translation.z = 0.0  # Replace with actual values
        right_wheel_transform.transform.rotation.x = 0.0
        right_wheel_transform.transform.rotation.y = 0.0
        right_wheel_transform.transform.rotation.z = 0.0
        right_wheel_transform.transform.rotation.w = 1.0

        # Broadcast static transforms
        self.static_broadcaster.sendTransform([left_wheel_transform, right_wheel_transform])

    def handle_odom_message(self, msg):
        # Create a transform from the odom message
        transform = TransformStamped()

        # Set the timestamp to the time the odometry message was received
        transform.header.stamp = msg.header.stamp

        # Set the frame IDs
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'

        # Set the translation and rotation
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = TransformBroadcasterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
