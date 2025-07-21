#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16

import sys
import select
import tty
import termios
import threading

class KeyboardCurrentPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_current_publisher')
        self.publisher_ = self.create_publisher(Int16, 'goal_current', 10)
        self.current = 0
        self.step = 10  # 0.1mA単位（10 = 1.0mA）

        self.settings = termios.tcgetattr(sys.stdin)
        self.running = True

        # 非同期スレッドでキー入力処理
        self.thread = threading.Thread(target=self.keyboard_loop)
        self.thread.start()

        self.get_logger().info("↑: +1mA / ↓: -1mA / q: quit")

    def keyboard_loop(self):
        tty.setraw(sys.stdin.fileno())
        try:
            while self.running:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    if key == '\x1b':  # escape
                        key += sys.stdin.read(2)  # read next two chars
                        if key == '\x1b[A':  # ↑
                            self.current += self.step
                        elif key == '\x1b[B':  # ↓
                            self.current -= self.step
                        else:
                            continue
                    elif key == 'q':
                        self.running = False
                        break
                    else:
                        continue

                    # 値を publish
                    msg = Int16()
                    msg.data = self.current
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"Published current: {self.current} (= {self.current/10:.1f}mA)")

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def destroy_node(self):
        self.running = False
        self.thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardCurrentPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
