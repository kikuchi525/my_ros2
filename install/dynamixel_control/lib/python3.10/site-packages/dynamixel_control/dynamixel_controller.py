#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from dynamixel_sdk import PortHandler, PacketHandler
from dynamixel_sdk_custom_interfaces.msg import SetPosition

TORQUE_ENABLE_ADDR = 64
GOAL_POSITION_ADDR = 116
TORQUE_ENABLE = 1

class DynamixelController(Node):
    def __init__(self):
        super().__init__('dynamixel_controller')
        self.subscription = self.create_subscription(
            SetPosition,
            '/set_position',
            self.listener_callback,
            10)
        self.subscription

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

        # トルクON（ID 1 のみ例）
        dxl_id = 1
        result, error = self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, TORQUE_ENABLE_ADDR, TORQUE_ENABLE)
        if result != 0:
            self.get_logger().error(f'Torque enable comm failed: {result}')
        elif error != 0:
            self.get_logger().error(f'Torque enable error: {error}')
        else:
            self.get_logger().info('Torque enabled successfully')

    def listener_callback(self, msg):
        dxl_id = msg.id
        goal_position = msg.position
        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
            self.port_handler, dxl_id, GOAL_POSITION_ADDR, goal_position)
        if dxl_comm_result != 0:
            self.get_logger().error(f'Communication failed: {dxl_comm_result}')
        elif dxl_error != 0:
            self.get_logger().error(f'DXL error: {dxl_error}')
        else:
            self.get_logger().info(f'Set ID {dxl_id} to position {goal_position}')


def main(args=None):
    rclpy.init(args=args)
    node = DynamixelController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
