#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class DynamixelCurrentPublisher(Node):
    def __init__(self):
        super().__init__('dynamixel_current_publisher')
        self.publisher_ = self.create_publisher(SetCurrent, '/set_current', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.current = 100  # 10mA

    def timer_callback(self):
        msg = SetCurrent()
        msg.id = 1
        msg.current = self.current
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: ID={msg.id}, Current={msg.current}")
        self.current = (self.current + 10) % 100  # ループ

def main(args=None):
    rclpy.init(args=args)
    node = DynamixelCurrentPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
