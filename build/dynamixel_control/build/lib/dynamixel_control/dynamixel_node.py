#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from dynamixel_sdk import PortHandler, PacketHandler
from dynamixel_sdk_custom_interfaces.msg import SetPosition

class DynamixelNode(Node):
    def __init__(self):
        super().__init__('dynamixel_node')
        self.publisher_ = self.create_publisher(SetPosition, '/set_position', 10)
        self.timer_ = self.create_timer(1.0, self.timer_callback)
        self.port_handler = PortHandler('/dev/ttyUSB0')
        self.packet_handler = PacketHandler(2.0)

        if self.port_handler.openPort():
            self.get_logger().info('Port opened successfully.')
        else:
            self.get_logger().error('Failed to open port.')

        if self.port_handler.setBaudRate(57600):
            self.get_logger().info('Baudrate set successfully.')
        else:
            self.get_logger().error('Failed to set baudrate.')

    def timer_callback(self):
        msg = SetPosition()
        msg.id = 1
        msg.position = 512  # 例: 中央位置
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: ID={msg.id}, Position={msg.position}')

def main(args=None):
    rclpy.init(args=args)
    node = DynamixelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


