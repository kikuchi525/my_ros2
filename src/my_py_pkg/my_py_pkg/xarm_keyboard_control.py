#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from xarm.wrapper import XArmAPI
import tty
import termios
import sys
import select


class XArmKeyboardControl(Node):
    def __init__(self):
        super().__init__('xarm_keyboard_control')

        # xArmのIPアドレス（変更してください）
        self.arm = XArmAPI('192.168.1.214')
        self.arm.connect()
        self.arm.clean_warn()
        self.arm.clean_error()
        self.arm.set_mode(1)   # サーボモード
        self.arm.set_state(0)  # Ready状態

        self.step_size = 10  # mm単位の移動ステップ

        self.get_logger().info("xArm ready. Use WASD keys to move, QE for up/down, ESC to quit.")

    def get_key(self):
        """非ブロッキングで1文字読み取り"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
            else:
                key = ''
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def control_loop(self):
        key_bindings = {
            'w': (0, 0, self.step_size),   # 前進（Z軸正方向）
            's': (0, 0, -self.step_size),  # 後退（Z軸負方向）
            'a': (-self.step_size, 0, 0),  # 左（X軸負方向）
            'd': (self.step_size, 0, 0),   # 右（X軸正方向）
            'q': (0, self.step_size, 0),   # 上（Y軸正方向）
            'e': (0, -self.step_size, 0),  # 下（Y軸負方向）
        }

        while rclpy.ok():
            key = self.get_key()
            if key == '\x1b':  # ESCキーで終了
                self.get_logger().info("Exiting...")
                break
            elif key in key_bindings:
                dx, dy, dz = key_bindings[key]
                self.arm.set_position(x=dx, y=dy, z=dz, relative=True, wait=True)
                self.get_logger().info(f"Moved by dx={dx}, dy={dy}, dz={dz}")
            elif key != '':
                self.get_logger().info(f"Unrecognized key: '{key}'")


def main(args=None):
    rclpy.init(args=args)
    node = XArmKeyboardControl()
    try:
        node.control_loop()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

