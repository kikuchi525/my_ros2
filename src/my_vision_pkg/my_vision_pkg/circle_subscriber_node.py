#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from my_vision_pkg.msg import Circles  # カスタムメッセージ
from geometry_msgs.msg import Point


class CircleSubscriber(Node):
    def __init__(self):
        super().__init__('circle_subscriber')
        self.subscription = self.create_subscription(
            Circles,
            '/circle_data',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('CircleSubscriber started and listening to /circle_data')

    def listener_callback(self, msg):
        self.get_logger().info(f'--- {len(msg.circles)} circles received ---')
        for i, circle in enumerate(msg.circles):
            center = circle.center
            radius = circle.radius
            self.get_logger().info(
                f'Circle {i+1}: x={center.x:.2f}, y={center.y:.2f}, z={center.z:.2f}, radius={radius:.2f}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = CircleSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
