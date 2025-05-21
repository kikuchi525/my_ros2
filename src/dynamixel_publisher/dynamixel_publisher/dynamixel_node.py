#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from dynamixel_sdk_custom_interfaces.msg import SetPosition
from dynamixel_control.dynamixel_node import DynamixelNode


class DynamixelPublisher(Node):
    def __init__(self):
        super().__init__('dynamixel_publisher')
        self.publisher_ = self.create_publisher(SetPosition, '/set_position', 10)
        self.timer_ = self.create_timer(1.0, self.timer_callback)
        self.position = 0

    def timer_callback(self):
        msg = SetPosition()
        msg.id = 1
        msg.position = self.position
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: ID={msg.id}, Position={msg.position}')
        self.position = (self.position + 100) % 1024

def main(args=None):
    rclpy.init(args=args)
    node = DynamixelPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
