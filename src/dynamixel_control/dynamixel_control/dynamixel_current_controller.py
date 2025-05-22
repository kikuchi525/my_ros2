#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from dynamixel_sdk import PortHandler, PacketHandler


TORQUE_ENABLE_ADDR = 64
OPERATING_MODE_ADDR = 11
GOAL_CURRENT_ADDR = 102
TORQUE_ENABLE = 1


class DynamixelCurrentController(Node):
    def __init__(self):
        super().__init__('dynamixel_current_controller')

        self.port_handler = PortHandler('/dev/ttyUSB0')
        self.packet_handler = PacketHandler(2.0)

        if not self.port_handler.openPort():
            self.get_logger().error('Failed to open port.')
        elif not self.port_handler.setBaudRate(57600):
            self.get_logger().error('Failed to set baudrate.')
        else:
            self.get_logger().info('Port opened and baudrate set.')

        self.dxl_id = 1
        self.disable_torque(self.dxl_id)
        self.set_current_control_mode(self.dxl_id)
        self.enable_torque(self.dxl_id)

        # サブスクライバの作成
        self.subscription = self.create_subscription(
            Int16,
            'goal_current',
            self.goal_current_callback,
            10
        )

    def set_current_control_mode(self, dxl_id):
        result, error = self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, OPERATING_MODE_ADDR, 0)
        if result != 0 or error != 0:
            self.get_logger().error(f"Failed to set mode: {result}, error={error}")
        else:
            self.get_logger().info(f"Set ID {dxl_id} to Current Control Mode")

    def send_goal_current(self, dxl_id, current):
        result, error = self.packet_handler.write2ByteTxRx(self.port_handler, dxl_id, GOAL_CURRENT_ADDR, current)
        if result != 0 or error != 0:
            self.get_logger().error(f"Failed to send current: {result}, error={error}")
        else:
            self.get_logger().info(f"Sent goal current: {current}")

    def enable_torque(self, dxl_id):
        result, error = self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, TORQUE_ENABLE_ADDR, TORQUE_ENABLE)
        if result != 0 or error != 0:
            self.get_logger().error(f"Failed to enable torque: {result}, error={error}")
        else:
            self.get_logger().info("Torque enabled.")

    def disable_torque(self, dxl_id):
        result, error = self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, TORQUE_ENABLE_ADDR, 0)
        if result != 0 or error != 0:
            self.get_logger().error(f"❌ Failed to disable torque: result={result}, error={error}")
        else:
            self.get_logger().info("✅ Torque disabled.")

    def goal_current_callback(self, msg):
        current = msg.data
        self.send_goal_current(self.dxl_id, current)


def main(args=None):
    rclpy.init(args=args)
    node = DynamixelCurrentController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
