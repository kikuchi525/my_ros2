#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
import sys
import termios
import tty
import select

class KeyboardCurrentPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_current_publisher')
        self.publisher_ = self.create_publisher(Int16, 'goal_current', 10)
        self.current = 0
        self.step = 10  # 1ステップ = 1.0mA（10 = 1.0mA）
        self.get_logger().info("Press ↑ to increase, ↓ to decrease current, q to quit.")
        self.run()

    def get_key(self):
        """非ブロッキングで1文字取得"""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        self.settings = termios.tcgetattr(sys.stdin)
        try:
            while rclpy.ok():
                key = self.get_key()
                if key == '\x1b':  # 特殊キー（矢印）
                    key2 = self.get_key()
                    key3 = self.get_key()
                    if key3 == 'A':  # ↑
                        self.current += self.step
                    elif key3 == 'B':  # ↓
                        self.current -= self.step
                    else:
                        continue
                elif key == 'q':
                    self.get_logger().info("Exiting keyboard current control.")
                    break
                else:
                    continue

                # 値をパブリッシュ
                msg = Int16()
                msg.data = self.current
                self.publisher_.publish(msg)
                self.get_logger().info(f"Published goal current: {self.current} (={self.current/10.0:.1f} mA)")

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardCurrentPublisher()
    node.destroy_node()
    rclpy.shutdown()
